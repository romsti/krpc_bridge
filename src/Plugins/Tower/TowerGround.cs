using System;
using System.Collections.Generic;
using System.Globalization;
using Expansions.Serenity;
using UnityEngine;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// The Unity side of conn.tower_site.tower_ground_v1: the ground joints of the catch towers a client asked
    /// for, keyed by persistentId, stepped from TowerAddon.FixedUpdate and made again from TowerAddon's
    /// onFloatingOriginShift handler. What is fixed and why: TowerGroundWatch (pure).
    ///
    /// THE JOINT. As KJRGroundJointModule makes it (decompiled): a FixedJoint added to the part's own rigid body
    /// (Part.rb: a hinge's fixed half, never Part.servoRb), connectedBody null (the world), anchor zero, axis up,
    /// breakForce and breakTorque infinite. Made on the first physics step where the tower is loaded, unpacked and
    /// landed: the pose it keeps is the pose it was unpacked in (TowerAnchor's rule, no free window).
    ///
    /// THE FLOATING ORIGIN. A joint to the world keeps its world anchor (connectedAnchor, in world space), and a
    /// floating origin shift moves every body: FloatingOrigin.setOffset (decompiled) moves the celestial bodies, then
    /// every vessel (Vessel.SetPosition: a landed vessel by offset + the Krakensbane frame offset, as the body and its
    /// terrain), then other objects, and only then fires onFloatingOriginShift. A joint left as it was would pull its
    /// part back by the whole offset (hundreds of metres) at the next physics step; KJR destroys its joints in that
    /// event and never makes them again. Here every joint is destroyed at once in the event (DestroyImmediate:
    /// Object.Destroy waits for the end of the frame, after the physics step) and made again at once, from the bodies'
    /// new positions (Physics.SyncTransforms first, so the physics engine has them too): every body and the terrain
    /// moved by the same offset, so the new anchor is the old one relative to the ground; no physics step runs without
    /// the joints, none with a stale one. No jolt, no ratchet.
    ///
    /// LIFE CYCLE. Nothing runs and nothing is fixed until a client poses (TowerGroundV1): a game whose clients never
    /// call it behaves as before. Posed, the joints are made at each unpack, made again at each floating origin
    /// shift and whenever one is found destroyed by someone else or the part count changed, destroyed when the tower
    /// packs, unloads or leaves the ground (made again when it can be held), at the release (any client, by
    /// persistentId) and at the end of the flight scene. Main thread only (kRPC calls, FixedUpdate, GameEvents).
    /// </summary>
    internal static class TowerGround
    {
        const string Log = "[KRPC.Bridge.Tower] ground joints ";

        sealed class HeldJoint
        {
            internal Part Part;
            internal FixedJoint Joint;
        }

        sealed class Entry
        {
            internal uint Pid;
            internal string Name;
            internal Vessel Vessel;
            internal TowerGroundWatch Watch;
            internal readonly List<HeldJoint> Joints = new List<HeldJoint> ();
            // The parts of the last selection, by index (the vessel's part order then).
            internal readonly List<Part> Parts = new List<Part> ();
            // The log of the shifts: the game time of the last line, the shifts since.
            internal double ShiftLoggedAt = double.NaN;
            internal int ShiftsSinceLog;
        }

        /// <summary>Game time between two log lines of the joints made again after floating origin shifts (s).</summary>
        const double ShiftLogPeriod = 10.0;

        static readonly Dictionary<uint, Entry> entries = new Dictionary<uint, Entry> ();
        // Scratch of Step and of the shift handler (no list allocated per physics step).
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
        /// Poses the ground joints of the tower <paramref name="v"/>: made now if it is loaded, unpacked and landed,
        /// at its first such physics step otherwise. Already posed: nothing changes. Returns the status.
        /// </summary>
        internal static List<double> Pose (Vessel v)
        {
            double now = Now ();
            Entry e;
            if (entries.TryGetValue (v.persistentId, out e) && e.Watch.Active) {
                e.Vessel = v;
                Debug.Log (Log + "already posed on " + Who (e) + " (" + (e.Watch.Held ? "held" : "waiting") + "), "
                           + Ut (now));
                StepOne (e, now);
                return Status (e, now);
            }
            if (e != null)
                DestroyJoints (e);
            e = new Entry { Pid = v.persistentId, Name = v.vesselName, Vessel = v, Watch = TowerGroundWatch.Pose (now) };
            entries [e.Pid] = e;
            Debug.Log (Log + "posed on " + Who (e) + ": loaded " + v.loaded + ", packed " + v.packed + ", "
                       + v.situation + ", " + F (OriginDistance (v), "F0") + " m from the floating origin, " + Ut (now));
            StepOne (e, now);
            return Status (e, now);
        }

        /// <summary>The status of the ground joints of <paramref name="v"/> (StateNone when none were posed).</summary>
        internal static List<double> Read (Vessel v)
        {
            Entry e;
            if (v != null && entries.TryGetValue (v.persistentId, out e))
                return Status (e, Now ());
            return TowerGroundWatch.NoneStatus (LoadedOf (v), PackedOf (v), OriginDistance (v), FrameRotating (),
                                                FrameSpeed ());
        }

        /// <summary>
        /// Releases the ground joints of <paramref name="v"/>: every joint destroyed, state Released; returns the
        /// status. Nothing posed: nothing done.
        /// </summary>
        internal static List<double> Release (Vessel v)
        {
            Entry e;
            if (v == null || !entries.TryGetValue (v.persistentId, out e))
                return Read (v);
            double now = Now ();
            if (e.Watch.Active) {
                e.Vessel = v;
                int n = DestroyJoints (e);
                e.Watch.Release (now);
                Debug.Log (Log + "released on " + Who (e) + ": " + n + " joint(s) destroyed, after "
                           + e.Watch.Makes + " make(s), " + e.Watch.RemadeAfterShift + " remade after a floating origin "
                           + "shift, " + e.Watch.Broken + " broken, " + Ut (now));
            }
            return Status (e, now);
        }

        /// <summary>The parts table (layout tower_ground_parts_v1) of the last selection; header only when none.</summary>
        internal static List<string> PartsTable (Vessel v)
        {
            Entry e;
            if (v == null || !entries.TryGetValue (v.persistentId, out e))
                return TowerGroundWatch.PartsTable (v != null ? v.persistentId : 0u, v != null ? v.vesselName : "",
                                                    TowerGroundWatch.StateNone, null);
            var rows = new List<TowerGroundWatch.PartRow> ();
            var s = e.Watch.Last;
            foreach (int i in TowerGroundWatch.RowOrder (s)) {
                var p = i < e.Parts.Count ? e.Parts [i] : null;
                if (p == null)
                    continue;
                rows.Add (new TowerGroundWatch.PartRow {
                    PartPersistentId = p.persistentId, PartFlightId = p.flightID,
                    Name = p.partInfo != null ? p.partInfo.name : p.name, PhysicsMass = p.physicsMass,
                    Kjr = s.Kjr [i], Hinge = s.Hinge [i], Mobile = s.Mobile [i], Held = Holds (e, p)
                });
            }
            return TowerGroundWatch.PartsTable (e.Pid, e.Name, e.Watch.State, rows);
        }

        // ------------------------------------------------------------------ the physics step

        /// <summary>One physics step of every posed tower (TowerAddon.FixedUpdate); nothing when none is posed.</summary>
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
                var act = e.Watch.Step (now, Observe (e, v));
                switch (act) {
                case TowerGroundWatch.Action.Lost:
                    DestroyJoints (e);
                    Debug.LogWarning (Log + "of " + Who (e) + " lost: the vessel is gone, " + Ut (now));
                    break;
                case TowerGroundWatch.Action.Release:
                    int n = DestroyJoints (e);
                    Debug.Log (Log + "of " + Who (e) + " released (" + e.Watch.Why + "): " + n + " joint(s) destroyed; "
                               + "made again when it is loaded, unpacked and landed, " + Ut (now));
                    break;
                case TowerGroundWatch.Action.Remake:
                    DestroyJoints (e);
                    Make (e, v, now, e.Watch.Why, false);
                    break;
                case TowerGroundWatch.Action.Make:
                    Make (e, v, now, "unpacked", false);
                    break;
                }
            } catch (Exception exc) {
                Debug.LogError (Log + "step failed on " + Who (e) + ": " + exc);
            }
        }

        /// <summary>
        /// onFloatingOriginShift (TowerAddon): every held joint destroyed at once and made again from the new positions
        /// (see the class). Fired by FloatingOrigin.setOffset after every vessel has moved.
        /// </summary>
        internal static void OnOriginShift (Vector3d offset, Vector3d nonFrame)
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
            // A landed vessel moved by the offset plus the Krakensbane frame offset (FloatingOrigin.setOffset).
            double moved = (offset + nonFrame).magnitude;
            bool synced = false;
            foreach (var e in scratch) {
                try {
                    if (!e.Watch.Shift (now, moved))
                        continue;
                    DestroyJoints (e);
                    var v = Resolve (e);
                    if (!CanHold (v)) {
                        e.Watch.MarkDropped ("floating origin shift while it cannot be held");
                        Debug.LogWarning (Log + "of " + Who (e) + " destroyed at a floating origin shift and NOT made "
                                          + "again (loaded " + (v != null && v.loaded) + ", packed " + (v != null && v.packed)
                                          + "): made again when it can be held, " + Ut (now));
                        continue;
                    }
                    if (!synced) {
                        Physics.SyncTransforms ();
                        synced = true;
                    }
                    if (!RemakeSame (e, v, now))
                        Make (e, v, now, "after a floating origin shift", true);
                    // One line at the first shift, then at most one per ShiftLogPeriod of game time: with Krakensbane
                    // active, FloatingOrigin shifts at every physics step.
                    e.ShiftsSinceLog++;
                    if (double.IsNaN (e.ShiftLoggedAt) || now - e.ShiftLoggedAt >= ShiftLogPeriod) {
                        Debug.Log (Log + "of " + Who (e) + " made again after " + e.ShiftsSinceLog + " floating origin "
                                   + "shift(s) (the last of " + F (moved, "F0") + " m): " + Alive (e) + " joint(s), "
                                   + e.Watch.RemadeAfterShift + " remade in all, " + Ut (now));
                        e.ShiftLoggedAt = now;
                        e.ShiftsSinceLog = 0;
                    }
                } catch (Exception exc) {
                    Debug.LogError (Log + "floating origin shift failed on " + Who (e) + ": " + exc);
                }
            }
            scratch.Clear ();
        }

        static bool CanHold (Vessel v)
        {
            return v != null && v.loaded && !v.packed && Landed (v) && v.rootPart != null && v.rootPart.rb != null;
        }

        static bool Landed (Vessel v)
        {
            return v.situation == Vessel.Situations.PRELAUNCH || v.situation == Vessel.Situations.LANDED;
        }

        static TowerGroundWatch.Observation Observe (Entry e, Vessel v)
        {
            var o = new TowerGroundWatch.Observation { Exists = v != null };
            if (v == null)
                return o;
            o.Loaded = v.loaded;
            o.Packed = v.packed;
            o.Landed = Landed (v);
            o.RootBody = v.rootPart != null && v.rootPart.rb != null;
            o.Parts = v.parts != null ? v.parts.Count : 0;
            o.Joints = Alive (e);
            return o;
        }

        // ------------------------------------------------------------------ the joints

        static void Make (Entry e, Vessel v, double now, string why, bool afterShift)
        {
            var parts = v.parts;
            e.Parts.Clear ();
            var index = new Dictionary<Part, int> (parts.Count);
            for (int i = 0; i < parts.Count; i++) {
                e.Parts.Add (parts [i]);
                if (parts [i] != null && !index.ContainsKey (parts [i]))
                    index [parts [i]] = i;
            }
            var infos = new List<TowerGroundWatch.PartInfo> (parts.Count);
            for (int i = 0; i < parts.Count; i++)
                infos.Add (Info (parts [i], index));
            var s = TowerGroundWatch.Select (infos);
            foreach (int i in s.Fixed) {
                var p = e.Parts [i];
                if (p == null || p.rb == null)
                    continue;
                e.Joints.Add (new HeldJoint { Part = p, Joint = Fix (p) });
            }
            if (afterShift) {
                // the shift handler logs (at most one line per ShiftLogPeriod)
                e.Watch.MarkRemadeAfterShift (now, s, e.Joints.Count, parts.Count);
                return;
            }
            e.Watch.MarkMade (now, s, e.Joints.Count, parts.Count);
            Debug.Log (Log + "of " + Who (e) + " made (" + why + "): " + e.Joints.Count + " joint(s), "
                       + F (s.MassFixed, "F2") + " t: " + Describe (e, s) + "; KJR's rule picks " + s.KjrPicked
                       + " (excluded " + s.KjrExcluded + "), hinges " + s.HingesFixed + "/" + s.Hinges + " fixed ("
                       + s.HingesSkipped + " skipped); " + F (OriginDistance (v), "F0") + " m from the floating origin"
                       + (OriginDistance (v) > TowerGroundWatch.KjrMaxDistanceFromOrigin ? " (beyond KJR's 500 m)" : "")
                       + ", " + Ut (now));
        }

        /// <summary>
        /// After a shift: the joints made again on the parts of the last selection, without selecting again, when the
        /// vessel still has exactly those parts (same count, same objects, each still in it with its rigid body).
        /// False: nothing done, the caller makes a new selection.
        /// </summary>
        static bool RemakeSame (Entry e, Vessel v, double now)
        {
            var s = e.Watch.Last;
            var parts = v.parts;
            if (s == null || parts == null || parts.Count != e.Parts.Count)
                return false;
            for (int i = 0; i < parts.Count; i++)
                if (!ReferenceEquals (parts [i], e.Parts [i]))
                    return false;
            foreach (int i in s.Fixed) {
                var p = e.Parts [i];
                if (p == null || p.rb == null || p.vessel != v)
                    return false;
            }
            foreach (int i in s.Fixed)
                e.Joints.Add (new HeldJoint { Part = e.Parts [i], Joint = Fix (e.Parts [i]) });
            e.Watch.MarkRemadeAfterShift (now, s, e.Joints.Count, parts.Count);
            return true;
        }

        static string Describe (Entry e, TowerGroundWatch.Selection s)
        {
            var said = new List<string> ();
            foreach (int i in s.Fixed) {
                var p = e.Parts [i];
                if (p == null)
                    continue;
                said.Add ((p.partInfo != null ? p.partInfo.name : p.name) + " " + F (p.physicsMass, "F2") + " t"
                          + (s.Kjr [i] && s.HingeBase [i] ? " (KJR, hinge)" : s.Kjr [i] ? " (KJR)" : " (hinge)"));
            }
            return string.Join (", ", said.ToArray ());
        }

        /// <summary>The joint KJRGroundJointModule makes (decompiled), on the part's own rigid body.</summary>
        static FixedJoint Fix (Part p)
        {
            var j = p.rb.gameObject.AddComponent<FixedJoint> ();
            j.connectedBody = null;
            j.anchor = Vector3.zero;
            j.axis = Vector3.up;
            j.breakForce = float.PositiveInfinity;
            j.breakTorque = float.PositiveInfinity;
            return j;
        }

        /// <summary>Destroys every joint of the entry at once (see the class); returns how many were alive.</summary>
        static int DestroyJoints (Entry e)
        {
            int n = 0;
            foreach (var h in e.Joints) {
                if (h.Joint != null) {
                    UnityEngine.Object.DestroyImmediate (h.Joint);
                    n++;
                }
            }
            e.Joints.Clear ();
            return n;
        }

        static int Alive (Entry e)
        {
            int n = 0;
            foreach (var h in e.Joints)
                if (h.Joint != null)
                    n++;
            return n;
        }

        static bool Holds (Entry e, Part p)
        {
            foreach (var h in e.Joints)
                if (h.Part == p && h.Joint != null)
                    return true;
            return false;
        }

        /// <summary>One part as the rule reads it (TowerGroundWatch.PartInfo).</summary>
        static TowerGroundWatch.PartInfo Info (Part p, Dictionary<Part, int> index)
        {
            var info = new TowerGroundWatch.PartInfo { Parent = -1, Symmetry = -1 };
            if (p == null)
                return info;
            int k;
            if (p.parent != null && index.TryGetValue (p.parent, out k))
                info.Parent = k;
            info.PhysicsMass = p.physicsMass;
            info.Significant = p.physicalSignificance == Part.PhysicalSignificance.FULL;
            info.Body = p.rb != null;
            info.Clamp = p.isLaunchClamp ();
            var servo = p.FindModuleImplementing<BaseServo> ();
            info.Servo = servo != null;
            info.Hinge = p.FindModuleImplementing<ModuleRoboticServoHinge> () != null;
            info.ParentSide = servo != null ? SideOf (servo, p, p.parent) : TowerGroundWatch.SideFixed;
            var parentServo = p.parent != null ? p.parent.FindModuleImplementing<BaseServo> () : null;
            info.SideOnParent = parentServo != null ? SideOf (parentServo, p.parent, p) : TowerGroundWatch.SideFixed;
            var c = p.symmetryCounterparts;
            if (c != null && c.Count > 0) {
                int g = index.TryGetValue (p, out k) ? k : int.MaxValue;
                foreach (var q in c)
                    if (q != null && index.TryGetValue (q, out k) && k < g)
                        g = k;
                info.Symmetry = g == int.MaxValue ? -1 : g;
            }
            return info;
        }

        /// <summary>
        /// The side of <paramref name="servoPart"/> (carrying <paramref name="servo"/>) where <paramref name="other"/> is
        /// attached: moving when the node is one of BaseServo.servoAttachNodes, not known for a surface attachment or no
        /// node; fixed otherwise (and for no part at all: the root servo's own body).
        /// </summary>
        static int SideOf (BaseServo servo, Part servoPart, Part other)
        {
            if (other == null)
                return TowerGroundWatch.SideFixed;
            var node = servoPart.FindAttachNodeByPart (other);
            if (node == null || node == servoPart.srfAttachNode)
                return TowerGroundWatch.SideUnknown;
            return TowerGroundWatch.IsServoNode (servo.servoAttachNodes, node.id) ? TowerGroundWatch.SideMoving
                : TowerGroundWatch.SideFixed;
        }

        // ------------------------------------------------------------------ status and helpers

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

        static List<double> Status (Entry e, double now)
        {
            var v = Resolve (e);
            return e.Watch.Status (now, LoadedOf (v), PackedOf (v), Alive (e), OriginDistance (v), FrameRotating (),
                                   FrameSpeed ());
        }

        static double LoadedOf (Vessel v)
        {
            return v == null ? double.NaN : (v.loaded ? 1.0 : 0.0);
        }

        static double PackedOf (Vessel v)
        {
            return v == null || !v.loaded ? double.NaN : (v.packed ? 1.0 : 0.0);
        }

        /// <summary>|the vessel's transform position| (m): KJR's measure of the distance to the floating origin; NaN unloaded.</summary>
        static double OriginDistance (Vessel v)
        {
            return v == null || !v.loaded ? double.NaN : ((Vector3d) v.transform.position).magnitude;
        }

        static double FrameRotating ()
        {
            return FlightGlobals.RefFrameIsRotating ? 1.0 : 0.0;
        }

        static double FrameSpeed ()
        {
            return Krakensbane.GetFrameVelocity ().magnitude;
        }

        /// <summary>
        /// Forgets every posed tower (end of the flight scene): the joints still alive destroyed, nothing else.
        /// </summary>
        internal static void Clear (string why)
        {
            if (entries.Count == 0)
                return;
            int n = 0;
            foreach (var e in entries.Values) {
                try {
                    n += DestroyJoints (e);
                } catch (Exception) {
                    // the scene is being torn down: its objects go with it
                }
            }
            Debug.Log (Log + "of every tower forgotten (" + entries.Count + ", " + n + " joint(s) destroyed): " + why);
            entries.Clear ();
            scratch.Clear ();
        }
    }
}
