using System;
using System.Collections.Generic;
using System.Globalization;
using Expansions.Serenity;
using UnityEngine;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// The Unity side of conn.tower_site.tower_bubble_v1: one bubble per tower, keyed by
    /// persistentId, stepped from TowerAddon.FixedUpdate. Each step: the tower looked at
    /// (TowerBubbleWatch.Observation), the watcher stepped, the ranges it wants written into the
    /// vessel's own VesselRanges (all seven situations, as kRPC's physics_range), each event logged
    /// with the game time.
    ///
    /// THE RANGES. Vessel.vesselRanges is the vessel's own copy (Vessel.Awake: new
    /// VesselRanges(PhysicsGlobals.VesselRangesDefault), deep copy), read by Vessel.Update (load,
    /// unload: situation) and OrbitPhysicsManager.LateUpdate (pack: BestSituation; unpack:
    /// situation), both after FixedUpdate: a range written here acts in the same frame. The
    /// original ranges are copied at the first pose and written back at the release. Written again
    /// at every step (no allocation: preallocated copies); a range found changed since the last
    /// write means another writer holds this vessel (kRPC's VesselPhysicsRangeAddon re-applies a
    /// physics_range at every FixedUpdate): logged once, flagged in the status (overwritten).
    ///
    /// WHO RELEASES. A bubble is not owned by the client connection that posed it (as kRPC's
    /// physics_range): a batch parent releases the bubbles its children posed, and a client that
    /// died leaves its bubble to whoever releases it (TowerBubbleReleaseV1, by persistentId). At the
    /// end of the flight scene every bubble is forgotten and the original ranges are written back
    /// on the vessels still there (the next scene rebuilds every vessel with the stock ranges
    /// anyway): a bubble never outlives its flight scene.
    ///
    /// Main thread only (kRPC calls and FixedUpdate).
    /// </summary>
    internal static class TowerBubbles
    {
        const string Log = "[KRPC.Bridge.Tower] bubble ";

        sealed class Entry
        {
            internal uint Pid;
            internal string Name;
            internal Vessel Vessel;
            internal TowerBubbleWatch Watch;
            internal readonly VesselRanges Original = new VesselRanges ();
            internal readonly VesselRanges Written = new VesselRanges ();
            internal Vessel WrittenOn;
            internal bool OverwriteLogged;
            internal double[] Positions = new double [0];
        }

        static readonly Dictionary<uint, Entry> entries = new Dictionary<uint, Entry> ();
        // Scratch of Step (no list allocated per physics step).
        static readonly List<Entry> scratch = new List<Entry> ();

        static double Now ()
        {
            return Planetarium.GetUniversalTime ();
        }

        static string F (double x, string format)
        {
            return x.ToString (format, CultureInfo.InvariantCulture);
        }

        static string Ut (double now)
        {
            return "UT " + F (now, "F3");
        }

        static string Who (Entry e)
        {
            return e.Name + " (" + e.Pid + ")";
        }

        // ------------------------------------------------------------------ the procedures

        /// <summary>
        /// Poses a bubble of range <paramref name="range"/> (m) on the tower <paramref name="v"/>, or keeps
        /// the larger range when one is held; returns its status. A released or lost bubble is replaced.
        /// </summary>
        internal static List<double> Pose (Vessel v, double range)
        {
            var why = TowerBubbleWatch.Invalid (range);
            if (why != null)
                throw new ArgumentException (why, "rangeM");
            double now = Now ();
            Entry e;
            if (entries.TryGetValue (v.persistentId, out e) && e.Watch.Active) {
                double before = e.Watch.Range;
                if (e.Watch.Raise (range))
                    Debug.Log (Log + "range of " + Who (e) + " raised from " + F (before, "F0") + " to "
                               + F (range, "F0") + " m, " + Ut (now));
                else
                    Debug.Log (Log + "already held on " + Who (e) + " at " + F (before, "F0") + " m (asked "
                               + F (range, "F0") + " m: kept), " + Ut (now));
                e.Vessel = v;
                StepOne (e, now);
                return Status (e, now);
            }
            e = new Entry { Pid = v.persistentId, Name = v.vesselName, Vessel = v,
                            Watch = TowerBubbleWatch.Pose (range, now) };
            Copy (v.vesselRanges, e.Original);
            entries [e.Pid] = e;
            var o = e.Original.landed;
            Debug.Log (Log + "posed on " + Who (e) + ": range " + F (range, "F0") + " m, loaded first (load "
                       + F (Math.Max (TowerBubbleWatch.Hysteresis * range, o.load), "F0") + " / unload "
                       + F (Math.Max (range, o.unload), "F0") + " m, pack " + F (o.pack, "F0") + " / unpack "
                       + F (o.unpack, "F0") + " m kept, landed), unpacked once its load is finished; distance "
                       + F (Distance (v), "F0") + " m, loaded " + v.loaded + ", packed " + v.packed + ", " + Ut (now));
            StepOne (e, now);
            return Status (e, now);
        }

        /// <summary>The status of the bubble of <paramref name="v"/> (StateNone when there is none).</summary>
        internal static List<double> Read (Vessel v)
        {
            Entry e;
            if (v != null && entries.TryGetValue (v.persistentId, out e))
                return Status (e, Now ());
            return TowerBubbleWatch.NoneStatus (LoadedOf (v), PackedOf (v), Distance (v));
        }

        /// <summary>
        /// Releases the bubble of <paramref name="v"/>: its original ranges written back, state Released;
        /// returns the status. Nothing held: nothing done, the status as it is.
        /// </summary>
        internal static List<double> Release (Vessel v)
        {
            Entry e;
            if (v == null || !entries.TryGetValue (v.persistentId, out e))
                return Read (v);
            double now = Now ();
            if (e.Watch.Active) {
                e.Vessel = v;
                Copy (e.Original, v.vesselRanges);
                e.Watch.Release (now);
                var o = e.Original.landed;
                Debug.Log (Log + "released on " + Who (e) + ": original ranges written back (landed load "
                           + F (o.load, "F0") + " / unload " + F (o.unload, "F0") + " / pack " + F (o.pack, "F0")
                           + " / unpack " + F (o.unpack, "F0") + " m), loaded " + v.loaded + ", packed " + v.packed
                           + ", " + Ut (now));
            }
            return Status (e, now);
        }

        // ------------------------------------------------------------------ the physics step

        /// <summary>One physics step of every held bubble (TowerAddon.FixedUpdate).</summary>
        internal static void Step ()
        {
            if (entries.Count == 0)
                return;
            scratch.Clear ();
            foreach (var e in entries.Values)
                if (e.Watch.Active)
                    scratch.Add (e);
            if (scratch.Count == 0)
                return;
            double now = Now ();
            foreach (var e in scratch)
                StepOne (e, now);
            scratch.Clear ();
        }

        static void StepOne (Entry e, double now)
        {
            try {
                var v = Resolve (e);
                if (v != null && e.WrittenOn == v && !Same (v.vesselRanges, e.Written)) {
                    e.Watch.MarkOverwritten ();
                    if (!e.OverwriteLogged) {
                        e.OverwriteLogged = true;
                        Debug.LogWarning (Log + "ranges of " + Who (e) + " changed by another writer since the last "
                                          + "step (kRPC's physics_range on this vessel?): the bubble does not hold "
                                          + "it alone, " + Ut (now));
                    }
                }
                var o = Observe (e, v);
                int before = e.Watch.State;
                var ev = e.Watch.Step (now, Time.fixedTime, o);
                if (v != null)
                    Write (e, v);
                Report (e, ev, before, now);
            } catch (Exception exc) {
                Debug.LogError (Log + "step failed on " + Who (e) + ": " + exc);
            }
        }

        /// <summary>The vessel of the entry: the same object while it lives, else found again by persistentId.</summary>
        static Vessel Resolve (Entry e)
        {
            var v = e.Vessel;
            if (v != null && v.persistentId == e.Pid && v.state != Vessel.State.DEAD)
                return v;
            v = null;
            var vessels = FlightGlobals.Vessels;
            for (int i = 0; vessels != null && i < vessels.Count; i++) {
                var x = vessels [i];
                if (x != null && x.persistentId == e.Pid && x.state != Vessel.State.DEAD) {
                    v = x;
                    break;
                }
            }
            e.Vessel = v;
            return v;
        }

        static TowerBubbleWatch.Observation Observe (Entry e, Vessel v)
        {
            var o = new TowerBubbleWatch.Observation { Exists = v != null, Distance = Distance (v), Scale = double.NaN };
            if (v == null)
                return o;
            o.Loaded = v.loaded;
            o.Packed = v.packed;
            if (!o.Loaded)
                return o;
            var parts = v.parts;
            o.Parts = parts != null ? parts.Count : 0;
            for (int i = 0; i < o.Parts; i++) {
                var p = parts [i];
                if (p == null)
                    continue;
                if (p.started)
                    o.PartsStarted++;
                var mods = p.Modules;
                for (int k = 0; mods != null && k < mods.Count; k++) {
                    var s = mods [k] as BaseServo;
                    if (s == null)
                        continue;
                    o.Servos++;
                    if (s.ServoInitComplete)
                        o.ServosReady++;
                }
            }
            // The pose is watched only while the load is (packed, before the unpack ranges).
            int st = e.Watch.State;
            if (o.Packed && (st == TowerBubbleWatch.StateLoading || st == TowerBubbleWatch.StateSettling))
                Measure (e, v, ref o);
            return o;
        }

        /// <summary>
        /// Root-relative positions of every rigid body (a part's own, a servo's moving one), in the root's
        /// frame, the difference taken in double; Scale = the largest absolute world coordinate + the extent.
        /// </summary>
        static void Measure (Entry e, Vessel v, ref TowerBubbleWatch.Observation o)
        {
            var root = v.rootPart != null ? v.rootPart.transform : null;
            if (root == null)
                return;
            var parts = v.parts;
            int need = 0;
            for (int i = 0; i < parts.Count; i++) {
                var p = parts [i];
                if (p == null)
                    continue;
                if (p.rb != null)
                    need += 3;
                if (p.servoRb != null)
                    need += 3;
            }
            if (e.Positions.Length < need)
                e.Positions = new double [need];
            Vector3d origin = root.position;
            Quaternion inv = Quaternion.Inverse (root.rotation);
            double maxCoord = Math.Max (Math.Abs (origin.x), Math.Max (Math.Abs (origin.y), Math.Abs (origin.z)));
            double extent = 0.0;
            int n = 0;
            for (int i = 0; i < parts.Count; i++) {
                var p = parts [i];
                if (p == null)
                    continue;
                for (int b = 0; b < 2; b++) {
                    var rb = b == 0 ? p.rb : p.servoRb;
                    if (rb == null)
                        continue;
                    Vector3d w = rb.transform.position;
                    maxCoord = Math.Max (maxCoord, Math.Max (Math.Abs (w.x), Math.Max (Math.Abs (w.y), Math.Abs (w.z))));
                    Vector3 rel = inv * (Vector3) (w - origin);
                    e.Positions [n] = rel.x;
                    e.Positions [n + 1] = rel.y;
                    e.Positions [n + 2] = rel.z;
                    extent = Math.Max (extent, rel.magnitude);
                    n += 3;
                }
            }
            o.Positions = e.Positions;
            o.Count = n;
            o.Scale = maxCoord + extent;
        }

        static void Write (Entry e, Vessel v)
        {
            var mode = e.Watch.Wanted;
            if (mode == TowerBubbleWatch.Ranges.None)
                return;
            var r = v.vesselRanges;
            var o = e.Original;
            float range = (float) e.Watch.Range;
            Set (r.prelaunch, o.prelaunch, range, mode);
            Set (r.landed, o.landed, range, mode);
            Set (r.splashed, o.splashed, range, mode);
            Set (r.flying, o.flying, range, mode);
            Set (r.subOrbital, o.subOrbital, range, mode);
            Set (r.orbit, o.orbit, range, mode);
            Set (r.escaping, o.escaping, range, mode);
            Copy (r, e.Written);
            e.WrittenOn = v;
        }

        /// <summary>The ranges of one situation: never shorter than the original.</summary>
        static void Set (VesselRanges.Situation s, VesselRanges.Situation o, float range, TowerBubbleWatch.Ranges mode)
        {
            if (mode == TowerBubbleWatch.Ranges.Original) {
                s.load = o.load;
                s.unload = o.unload;
                s.pack = o.pack;
                s.unpack = o.unpack;
                return;
            }
            float inner = TowerBubbleWatch.Hysteresis * range;
            s.load = Math.Max (inner, o.load);
            s.unload = Math.Max (range, o.unload);
            if (mode == TowerBubbleWatch.Ranges.Full) {
                s.pack = Math.Max (range, o.pack);
                s.unpack = Math.Max (inner, o.unpack);
            } else {
                s.pack = o.pack;
                s.unpack = o.unpack;
            }
        }

        static void Copy (VesselRanges from, VesselRanges to)
        {
            Copy (from.prelaunch, to.prelaunch);
            Copy (from.landed, to.landed);
            Copy (from.splashed, to.splashed);
            Copy (from.flying, to.flying);
            Copy (from.subOrbital, to.subOrbital);
            Copy (from.orbit, to.orbit);
            Copy (from.escaping, to.escaping);
        }

        static void Copy (VesselRanges.Situation from, VesselRanges.Situation to)
        {
            to.load = from.load;
            to.unload = from.unload;
            to.pack = from.pack;
            to.unpack = from.unpack;
        }

        static bool Same (VesselRanges a, VesselRanges b)
        {
            return Same (a.prelaunch, b.prelaunch) && Same (a.landed, b.landed) && Same (a.splashed, b.splashed)
                   && Same (a.flying, b.flying) && Same (a.subOrbital, b.subOrbital) && Same (a.orbit, b.orbit)
                   && Same (a.escaping, b.escaping);
        }

        static bool Same (VesselRanges.Situation a, VesselRanges.Situation b)
        {
            return a.load == b.load && a.unload == b.unload && a.pack == b.pack && a.unpack == b.unpack;
        }

        static void Report (Entry e, TowerBubbleWatch.Event ev, int before, double now)
        {
            var w = e.Watch;
            var o = w.Last;
            switch (ev) {
            case TowerBubbleWatch.Event.Loaded:
                Debug.Log (Log + Who (e) + " loaded, packed: " + o.Parts + " parts (" + o.PartsStarted + " started), "
                           + o.Servos + " servos, distance " + F (o.Distance, "F0") + " m, "
                           + F (w.Elapsed (now), "F3") + " s after the pose, " + Ut (now));
                break;
            case TowerBubbleWatch.Event.Ready:
                Debug.Log (Log + Who (e) + " load finished: " + o.PartsStarted + "/" + o.Parts + " parts started, "
                           + o.ServosReady + "/" + o.Servos + " servos ready, pose still for "
                           + TowerBubbleWatch.SettleTicks + " physics steps (shift " + F (w.Shift, "G4")
                           + " m, tolerance " + F (w.Tolerance, "G4") + " m, largest " + F (w.SettleShiftMax, "G4")
                           + " m); unpack ranges written (pack " + F (w.Range, "F0") + " / unpack "
                           + F (TowerBubbleWatch.Hysteresis * w.Range, "F0") + " m), "
                           + F (now - w.LoadedAt, "F3") + " s after the load, " + Ut (now));
                break;
            case TowerBubbleWatch.Event.Unpacked:
                Debug.Log (Log + Who (e) + " unpacked by KSP" + (w.Unpacks > 1 ? " again (" + w.Unpacks + ")" : "")
                           + ", distance " + F (o.Distance, "F0") + " m, "
                           + (double.IsNaN (w.ReadyAt) ? "" : F (now - w.ReadyAt, "F3") + " s after ready, ") + Ut (now));
                break;
            case TowerBubbleWatch.Event.AlreadyUnpacked:
                Debug.Log (Log + Who (e) + " was already loaded and unpacked when posed (KSP's own ranges): held "
                           + "unpacked, " + Ut (now));
                break;
            case TowerBubbleWatch.Event.EarlyUnpack:
                Debug.LogWarning (Log + Who (e) + " UNPACKED BEFORE ITS LOAD WAS FINISHED (state was " + before + ": "
                                  + o.PartsStarted + "/" + o.Parts + " parts started, " + o.ServosReady + "/" + o.Servos
                                  + " servos ready, still " + w.StillTicks + " steps), distance " + F (o.Distance, "F0")
                                  + " m: KSP's own unpack range or another writer, " + Ut (now));
                break;
            case TowerBubbleWatch.Event.Unloaded:
                Debug.Log (Log + Who (e) + " unloaded (distance " + F (o.Distance, "F0") + " m): loading ranges again, "
                           + "pack and unpack the original, " + Ut (now));
                break;
            case TowerBubbleWatch.Event.Repacked:
                Debug.Log (Log + Who (e) + " packed while loaded (KSP packs every vessel at an active vessel switch): "
                           + "its parts stay built, KSP unpacks it again, " + Ut (now));
                break;
            case TowerBubbleWatch.Event.Lost:
                Debug.LogWarning (Log + Who (e) + " lost: the vessel is gone, " + Ut (now));
                break;
            }
        }

        // ------------------------------------------------------------------ status and helpers

        static List<double> Status (Entry e, double now)
        {
            var v = Resolve (e);
            return e.Watch.Status (now, LoadedOf (v), PackedOf (v), Distance (v));
        }

        static double LoadedOf (Vessel v)
        {
            return v == null ? double.NaN : (v.loaded ? 1.0 : 0.0);
        }

        static double PackedOf (Vessel v)
        {
            return v == null || !v.loaded ? double.NaN : (v.packed ? 1.0 : 0.0);
        }

        /// <summary>Distance to the active vessel as OrbitPhysicsManager measures it (transforms), NaN without one.</summary>
        static double Distance (Vessel v)
        {
            var a = FlightGlobals.ActiveVessel;
            if (v == null || a == null)
                return double.NaN;
            return Vector3d.Distance (v.transform.position, a.transform.position);
        }

        /// <summary>
        /// Forgets every bubble (end of the flight scene): the original ranges written back on the vessels
        /// still alive, nothing else.
        /// </summary>
        internal static void Clear (string why)
        {
            if (entries.Count == 0)
                return;
            int restored = 0;
            foreach (var e in entries.Values) {
                if (!e.Watch.Active)
                    continue;
                try {
                    var v = e.Vessel;
                    if (v != null && v.persistentId == e.Pid) {
                        Copy (e.Original, v.vesselRanges);
                        restored++;
                    }
                } catch (Exception) {
                    // the scene is being torn down: the next one rebuilds every vessel with the stock ranges
                }
            }
            Debug.Log (Log + "every bubble forgotten (" + entries.Count + ", original ranges written back on " + restored
                       + "): " + why);
            entries.Clear ();
            scratch.Clear ();
        }
    }
}
