using System;
using System.Collections.Generic;
using System.Reflection;
using Expansions.Serenity;
using UnityEngine;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// The Unity side of conn.tower.hinge_lock_at_rest_v1: the armed watchers, one per
    /// hinge part, stepped from TowerAddon.FixedUpdate, and the lock that bypasses IsMoving.
    ///
    /// THE LOCK. BaseServo.EngageServoLock (decompiled, KSP 1.12.5) is, in full:
    ///     if (servoIsLocked) return true;
    ///     if (!IsMoving ()) { Fields["servoIsLocked"].SetValue (true, this); ModifyLocked (null); return true; }
    ///     return false;
    /// Lock () below runs the two instructions after the test, nothing else: the field set through
    /// its BaseField (which fires OnValueModified, to which BaseServo.OnStart subscribed
    /// ModifyLocked, exactly as in the stock path), then ModifyLocked (null), protected, by
    /// reflection. ModifyLocked sees servoIsLocked true and prevServoIsLocked false (Update copies
    /// one into the other, PAW closed): onRoboticPartLockChanging, the servo transform recorded,
    /// RecurseCoordUpdate, RecurseAttachNodeUpdate, OnServoLockApplied (the hinge's lockAngle =
    /// its current angle), vessel.CycleAllAutoStrut (), ModifyServo, onRoboticPartLockChanged.
    ///
    /// THE ANGLE. The one kRPC's RoboticHinge.CurrentAngle reads (the client's trace too):
    /// modelInitialAngle + currentTransformAngle (), the second protected, by reflection. NOT the
    /// module's currentAngle field: ModuleRoboticServoHinge writes it only in UpdatePAWUI, i.e.
    /// while the part action window is open.
    ///
    /// LIFE CYCLE (nothing hooks a GameEvent, nothing is static and delegated): a watcher is
    /// stepped only while its vessel is loaded and unpacked; unloaded, packed, destroyed, or its
    /// part or module gone: it ends Lost. Every watcher is forgotten at the end of the flight
    /// scene (TowerAddon.OnDestroy). An ended watcher keeps its status until the part is armed
    /// again or the scene ends. Main thread only (kRPC calls and FixedUpdate).
    /// </summary>
    internal static class HingeRestLocks
    {
        sealed class Entry
        {
            internal Part Part;
            internal ModuleRoboticServoHinge Module;
            internal HingeRestWatch Watch;
        }

        static readonly Dictionary<uint, Entry> entries = new Dictionary<uint, Entry> ();
        // Scratch of Step (no list allocated per physics step).
        static readonly List<Entry> scratch = new List<Entry> ();

        const BindingFlags Inst = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;

        static bool resolved;
        static MethodInfo modifyLocked;      // BaseServo.ModifyLocked (object), protected
        static MethodInfo transformAngle;    // BaseServo.currentTransformAngle (), protected
        static FieldInfo averageRate;        // BaseServo.transformRateOfMotion, internal (diagnostic)

        /// <summary>The reflective members, looked up once; null when every one was found.</summary>
        static string Resolve ()
        {
            if (!resolved) {
                var t = typeof (BaseServo);
                modifyLocked = t.GetMethod ("ModifyLocked", Inst, null, new [] { typeof (object) }, null);
                transformAngle = t.GetMethod ("currentTransformAngle", Inst, null, Type.EmptyTypes, null);
                averageRate = t.GetField ("transformRateOfMotion", Inst);
                resolved = true;
            }
            if (modifyLocked == null)
                return "BaseServo.ModifyLocked(object) not found";
            if (transformAngle == null)
                return "BaseServo.currentTransformAngle() not found";
            return null;
        }

        static double Now ()
        {
            return Planetarium.GetUniversalTime ();
        }

        /// <summary>
        /// The angle of a loaded hinge as kRPC's CurrentAngle reads it, for conn.tower_site's
        /// TowerStatusV1: the reflective members looked up first; NaN when one is missing.
        /// </summary>
        internal static double CurrentAngle (ModuleRoboticServoHinge m)
        {
            if (m == null || Resolve () != null)
                return double.NaN;
            return Angle (m);
        }

        /// <summary>modelInitialAngle + currentTransformAngle (), as kRPC's CurrentAngle; NaN on failure.</summary>
        static double Angle (ModuleRoboticServoHinge m)
        {
            try {
                return m.modelInitialAngle + (float) transformAngle.Invoke (m, null);
            } catch (Exception) {
                return double.NaN;
            }
        }

        static double AverageRate (ModuleRoboticServoHinge m)
        {
            if (averageRate == null || m == null)
                return double.NaN;
            try {
                return Convert.ToDouble (averageRate.GetValue (m));
            } catch (Exception) {
                return double.NaN;
            }
        }

        /// <summary>The part's vessel is loaded and unpacked, the part and its module alive.</summary>
        static bool Live (Entry e)
        {
            if (e.Part == null || e.Module == null)
                return false;
            var v = e.Part.vessel;
            return v != null && v.loaded && !v.packed && v.state != Vessel.State.DEAD && e.Part.State != PartStates.DEAD;
        }

        static ModuleRoboticServoHinge Hinge (Part p)
        {
            var m = p.FindModuleImplementing<ModuleRoboticServoHinge> ();
            if (m == null)
                throw new InvalidOperationException ("[NOT_FOUND] no ModuleRoboticServoHinge on this part");
            return m;
        }

        /// <summary>
        /// Arms a watcher on the hinge of <paramref name="p"/> (replacing any earlier one of that
        /// part, which ends Cancelled) and returns its status. A hinge locked already: the watcher
        /// ends at once, AlreadyLocked, and nothing is touched.
        /// </summary>
        internal static List<double> Arm (Part p, double maxRate, double timeout)
        {
            var why = HingeRestWatch.Invalid (maxRate, timeout);
            if (why != null)
                throw new ArgumentException (why);
            var m = Hinge (p);
            var v = p.vessel;
            if (v == null || !v.loaded || v.packed)
                throw new InvalidOperationException ("[NOT_LOADED] the hinge's vessel must be loaded and unpacked");
            var missing = Resolve ();
            if (missing != null)
                throw new InvalidOperationException ("[NOT_FOUND] " + missing + " (another KSP version?)");
            double now = Now ();
            Entry old;
            if (entries.TryGetValue (p.persistentId, out old))
                old.Watch.End (HingeRestWatch.StateCancelled, now);
            double angle = Angle (m);
            var e = new Entry {
                Part = p, Module = m,
                Watch = HingeRestWatch.Arm (maxRate, timeout, now, angle, m.servoIsLocked)
            };
            entries [p.persistentId] = e;
            Debug.Log ("[KRPC.Bridge.Tower] hinge lock at rest armed on " + Name (p) + ": max " + maxRate.ToString ("R")
                       + " deg/s, timeout " + timeout.ToString ("R") + " s, angle " + angle.ToString ("F3")
                       + (m.servoIsLocked ? " (already locked: nothing to do)" : ""));
            return Status (e, now);
        }

        /// <summary>The status of the watcher of <paramref name="p"/> (StateNone when there is none).</summary>
        internal static List<double> Read (Part p)
        {
            Entry e;
            if (p != null && entries.TryGetValue (p.persistentId, out e))
                return Status (e, Now ());
            var m = p != null ? Hinge (p) : null;
            Resolve ();
            return HingeRestWatch.NoneStatus (m != null ? (m.servoIsLocked ? 1.0 : 0.0) : double.NaN,
                                              m != null && transformAngle != null ? Angle (m) : double.NaN);
        }

        /// <summary>Cancels the watcher of <paramref name="p"/> if armed; returns its status.</summary>
        internal static List<double> Cancel (Part p)
        {
            Entry e;
            if (p != null && entries.TryGetValue (p.persistentId, out e)) {
                if (e.Watch.State == HingeRestWatch.StateArmed)
                    Debug.Log ("[KRPC.Bridge.Tower] hinge lock at rest cancelled on " + Name (p));
                e.Watch.End (HingeRestWatch.StateCancelled, Now ());
            }
            return Read (p);
        }

        static List<double> Status (Entry e, double now)
        {
            bool live = Live (e);
            double locked = live ? (e.Module.servoIsLocked ? 1.0 : 0.0) : double.NaN;
            double angle = live && transformAngle != null ? Angle (e.Module) : e.Watch.LastAngle;
            return e.Watch.Status (now, live ? AverageRate (e.Module) : double.NaN, locked, angle);
        }

        /// <summary>One physics step of every armed watcher (TowerAddon.FixedUpdate).</summary>
        internal static void Step ()
        {
            if (entries.Count == 0)
                return;
            scratch.Clear ();
            foreach (var e in entries.Values)
                if (e.Watch.State == HingeRestWatch.StateArmed)
                    scratch.Add (e);
            if (scratch.Count == 0)
                return;
            double now = Now ();
            double stepTime = Time.fixedTime;
            foreach (var e in scratch) {
                if (!Live (e)) {
                    e.Watch.End (HingeRestWatch.StateLost, now);
                    Debug.Log ("[KRPC.Bridge.Tower] hinge lock at rest lost on " + Name (e.Part)
                               + ": vessel unloaded, packed or destroyed, or part gone");
                    continue;
                }
                var d = e.Watch.Step (now, stepTime, Angle (e.Module), e.Module.servoIsLocked);
                if (d == HingeRestWatch.Decision.Lock)
                    Lock (e, now);
                else if (d == HingeRestWatch.Decision.Ended && e.Watch.State == HingeRestWatch.StateExpired)
                    Debug.Log ("[KRPC.Bridge.Tower] hinge lock at rest expired on " + Name (e.Part) + " after "
                               + e.Watch.Steps + " steps, slowest " + e.Watch.MinRate.ToString ("F4") + " deg/s");
            }
            scratch.Clear ();
        }

        /// <summary>The two instructions of EngageServoLock after its IsMoving test (see above).</summary>
        static void Lock (Entry e, double now)
        {
            var m = e.Module;
            bool took = false;
            try {
                m.Fields ["servoIsLocked"].SetValue (true, m);
                modifyLocked.Invoke (m, new object [] { null });
                took = m.servoIsLocked;
            } catch (Exception exc) {
                Debug.LogError ("[KRPC.Bridge.Tower] hinge lock at rest failed on " + Name (e.Part) + ": " + exc);
            }
            e.Watch.Locked (took, now);
            Debug.Log ("[KRPC.Bridge.Tower] hinge lock at rest " + (took ? "locked " : "did not take on ") + Name (e.Part)
                       + " at " + e.Watch.LockAngle.ToString ("F3") + " deg, rate " + e.Watch.LockRate.ToString ("F4")
                       + " deg/s, KSP average " + AverageRate (m).ToString ("F4") + " deg/s, after "
                       + e.Watch.Elapsed (now).ToString ("F2") + " s");
        }

        /// <summary>Forgets every watcher (end of the flight scene).</summary>
        internal static void Clear (string why)
        {
            if (entries.Count > 0)
                Debug.Log ("[KRPC.Bridge.Tower] hinge locks at rest forgotten: " + why);
            entries.Clear ();
            scratch.Clear ();
        }

        static string Name (Part p)
        {
            if (p == null)
                return "?";
            return (p.partInfo != null ? p.partInfo.name : p.name) + "#" + p.flightID;
        }
    }
}
