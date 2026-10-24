using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using ModuleWheels;
using UnityEngine;

namespace KRPC.Bridge.Telemetry
{
    /// <summary>
    /// ContactV1 (refactor lot 3, line 3.2; audit C, R7): the contact of a vessel with the ground,
    /// by leg and by part, and the master tick of each first contact, read-only.
    ///
    /// WHERE IT RUNS. Inside the actuator pump, through the Telemetry observer, right after the
    /// V3 capture of the same physics step (<see cref="TelemetryObserver.Capture"/>): the ticks it
    /// latches are <see cref="GameClock.StepTick"/>, the master_tick of the V3 frame of that
    /// step, so a client joins the two on master_tick. Unity orders the pump and the wheels' own
    /// FixedUpdate freely: a wheel grounded by the physics of step N is seen at step N or N+1;
    /// Part.GroundContact set by a collision callback (after the physics of step N) is seen at
    /// step N+1. One step of jitter, the same for every leg.
    ///
    /// WHICH VESSELS. The active vessel at every step; any other loaded vessel for
    /// <see cref="ReadWatchTicks"/> master ticks after each contact_v1 that names it (a double
    /// flight reads its non-active booster: the first read starts the watch). At most
    /// <see cref="MaxRecords"/> vessels; a vessel unloaded, or neither active nor read for that
    /// long, is forgotten; nothing survives a scene (<see cref="ResetForScene"/>, from the
    /// pump's ControlWatcher.OnDestroy). A packed vessel is not stepped (its latches wait).
    ///
    /// COST. Per watched vessel and step: one pass over its parts (three booleans and a
    /// collision count each) and over its wheels (the controller's state, the deployment
    /// position), no allocation (the wheel list is rescanned, into preallocated lists, when the
    /// vessel object or its part count changes and every <see cref="RescanTicks"/> steps).
    ///
    /// KSP MEMBERS (KSP 1.12.5, installed Assembly-CSharp; assembly inspection, the
    /// name returned checked against the name asked, and decompiled where the behaviour matters):
    ///   * ModuleWheelBase (global namespace): public KSPWheelController Wheel (property),
    ///     public bool isGrounded and public WheelType wheelType (KSPFields); its FixedUpdate
    ///     copies Wheel.IsGrounded into isGrounded, then LandingDetectionUpdate sets
    ///     Part.GroundContact (layer 15 ground, or a part of a landed/splashed vessel: a barge) and
    ///     calls Vessel.checkLanded;
    ///   * KSPWheelController : VehiclePhysics.VehicleBase (MonoBehaviour): currentState =
    ///     wheelState[0], null before init. VehicleBase.WheelState: bool grounded (the wheel
    ///     collider's GetGroundHit), UnityEngine.WheelHit hit (normal, force, collider), float
    ///     downforce (hit.force + extra, cos of the contact angle, floored at minimumDownforce),
    ///     suspensionCompression (contactDepth / suspension travel: a fraction), contactDepth (m),
    ///     contactSpeed (d contactDepth / dt: the compression rate, m/s, NOT the touchdown speed),
    ///     normalizedLoad. Forces are in KSP's unit, kN (masses in tonnes): published in N;
    ///   * ModuleWheels.ModuleWheelDeployment: float position, float deployedPosition (deployed
    ///     when within 0.01, as its state machine decides);
    ///   * Part: uint persistentId, bool GroundContact, bool PermanentGroundContact (the two
    ///     Vessel.checkLanded reads), DictionaryValueList currentCollisions (Count);
    ///   * Vessel: bool Landed, Splashed, situation, double verticalSpeed, horizontalSrfSpeed,
    ///     Vector3d upAxis (UpdatePosVel, every step), float heightFromTerrain and Vector3
    ///     terrainNormal: refreshed every step by Vessel.UpdatePosVel -> GetHeightFromTerrain()
    ///     (VesselPrecalculate, loaded and unpacked): a raycast on layer 15 (Local Scenery,
    ///     mask 32768) from vesselTransform.position along -up, +600 m above the surface;
    ///     heightFromTerrain -1 without a hit, terrainNormal in vesselTransform's local frame.
    ///     Layer 15 only: over a barge (parts, layer 0) it is the sea floor or nothing.
    /// </summary>
    internal static class ContactWatch
    {
        /// <summary>Protocol of contact_v1 (a removed or moved field raises it).</summary>
        internal const int Protocol = 1;

        /// <summary>Schema of contact_v1 (+1 when a field is appended).</summary>
        internal const int Schema = 1;

        /// <summary>Doubles of the header section.</summary>
        internal const int HeaderStride = 45;

        /// <summary>Doubles of one wheel row.</summary>
        internal const int WheelStride = 25;

        /// <summary>Vessels watched at once.</summary>
        internal const int MaxRecords = 8;

        /// <summary>Master ticks a vessel stays watched after a read or after it stopped being active (60 s at 50 Hz).</summary>
        internal const long ReadWatchTicks = 3000;

        /// <summary>Master ticks between two unconditional wheel rescans.</summary>
        internal const long RescanTicks = 250;

        /// <summary>A leg is deployed when its position is within this of deployedPosition (KSP's own test).</summary>
        const float DeployedTolerance = 0.01f;

        /// <summary>Contact transitions published on the event bus V2 (kinds).</summary>
        internal const string KindFirst = "contact.first", KindLost = "contact.lost", KindLeg = "contact.leg",
            KindAllLegs = "contact.all_legs", KindLanded = "contact.landed",
            KindSituation = "contact.situation_landed", KindCollision = "contact.collision";

        sealed class WatchRecord
        {
            internal uint Pid;
            internal Vessel Vessel;
            internal readonly ContactLatch Latch = new ContactLatch ();
            internal readonly List<ModuleWheelBase> Wheels = new List<ModuleWheelBase> (ContactLatch.MaxWheels);
            internal readonly List<ModuleWheelDeployment> Deployments =
                new List<ModuleWheelDeployment> (ContactLatch.MaxWheels);
            internal readonly List<Part> WheelParts = new List<Part> (ContactLatch.MaxWheels);
            internal Vessel ScannedVessel;
            internal int ScannedPartCount = -1;
            internal long ScanTick = -1;
            internal long LastReadTick = -1;
        }

        static readonly List<WatchRecord> records = new List<WatchRecord> (MaxRecords);
        static int failures;

        // ------------------------------------------------------------------ the step

        /// <summary>
        /// One physics step (the pump, after the V3 capture): the active vessel, then every other
        /// watched vessel. The caller catches: an exception here must never break the pump.
        /// </summary>
        internal static void Step (Vessel active)
        {
            long tick = GameClock.StepTick;
            if (active != null && active.loaded)
                Ensure (active.persistentId, active, tick).LastReadTick = tick;
            for (int i = records.Count - 1; i >= 0; i--) {
                var r = records [i];
                if (!Refresh (r)) {
                    records.RemoveAt (i);
                    continue;
                }
                bool isActive = active != null && ReferenceEquals (r.Vessel, active);
                if (!isActive && r.LastReadTick >= 0 && tick - r.LastReadTick > ReadWatchTicks) {
                    records.RemoveAt (i);
                    continue;
                }
                if (r.Vessel.packed)
                    continue;
                try {
                    StepRecord (r, tick);
                } catch (Exception exc) {
                    if (failures++ < 5)
                        BridgeLog.Error ("Telemetry", "contact watch of " + r.Pid + ": " + exc.Message);
                }
            }
        }

        /// <summary>The record's vessel is still the loaded vessel of that persistentId (re-found if needed).</summary>
        static bool Refresh (WatchRecord r)
        {
            var v = r.Vessel;
            if (v != null && v.loaded && v.persistentId == r.Pid)
                return true;
            v = VesselRegistry.FindLoaded (r.Pid);
            r.Vessel = v;
            return v != null;
        }

        /// <summary>The record of a persistentId, created when absent (the oldest-read non-active one makes room).</summary>
        static WatchRecord Ensure (uint pid, Vessel vessel, long tick)
        {
            for (int i = 0; i < records.Count; i++)
                if (records [i].Pid == pid) {
                    if (vessel != null)
                        records [i].Vessel = vessel;
                    return records [i];
                }
            if (records.Count >= MaxRecords) {
                int oldest = 0;
                for (int i = 1; i < records.Count; i++)
                    if (records [i].LastReadTick < records [oldest].LastReadTick)
                        oldest = i;
                records.RemoveAt (oldest);
            }
            var r = new WatchRecord { Pid = pid, Vessel = vessel, LastReadTick = tick };
            records.Add (r);
            return r;
        }

        static WatchRecord Find (uint pid)
        {
            for (int i = 0; i < records.Count; i++)
                if (records [i].Pid == pid)
                    return records [i];
            return null;
        }

        static void StepRecord (WatchRecord r, long tick)
        {
            var v = r.Vessel;
            if (NeedsRescan (r, v, tick))
                Rescan (r, v, tick);
            var latch = r.Latch;
            int n = latch.WheelCount;
            for (int i = 0; i < n; i++) {
                bool grounded;
                double rate;
                WheelNow (r.Wheels [i], out grounded, out rate);
                latch.Grounded [i] = grounded;
                latch.CompressionRate [i] = rate;
                latch.Deployed [i] = IsDeployed (r.Deployments [i]);
            }
            int groundNoWheel = 0, groundWheel = 0, colliding = 0;
            uint firstGround = 0;
            var parts = v.parts;
            for (int p = 0; p < parts.Count; p++) {
                var part = parts [p];
                if ((object)part == null)
                    continue;
                if (part.GroundContact || part.PermanentGroundContact) {
                    if (IsWheelPart (r, part))
                        groundWheel++;
                    else
                        groundNoWheel++;
                    if (firstGround == 0)
                        firstGround = part.persistentId;
                }
                var collisions = part.currentCollisions;
                if (collisions != null && collisions.Count > 0)
                    colliding++;
            }
            double ut = UniversalTime ();
            latch.Step (tick, ut, groundNoWheel, groundWheel, firstGround, colliding, v.Landed,
                SituationLanded (v), v.verticalSpeed, v.horizontalSrfSpeed, Tilt (v));
            if (latch.Transitions != 0)
                Publish (r, tick);
        }

        /// <summary>The transitions of the step, onto the event bus V2 (value: Vessel.verticalSpeed, m/s up).</summary>
        static void Publish (WatchRecord r, long tick)
        {
            var l = r.Latch;
            int t = l.Transitions;
            uint pid = r.Pid;
            if ((t & ContactLatch.EventFirstContact) != 0)
                EventBus.RecordV2 (KindFirst, tick, pid, 0u, l.FirstContactPartPid, l.FirstContactVerticalSpeed,
                    "source=" + l.FirstContactSource + " episode=" + l.Episodes);
            if ((t & ContactLatch.EventWheel) != 0)
                for (int i = 0; i < l.WheelCount; i++)
                    if (l.NewlyGrounded [i])
                        EventBus.RecordV2 (KindLeg, tick, pid, 0u, l.WheelPartPid [i], r.Vessel.verticalSpeed,
                            "ordinal=" + l.WheelOrdinal [i] + (l.IsLeg [i] ? " leg" : " wheel"));
            if ((t & ContactLatch.EventAllLegs) != 0)
                EventBus.RecordV2 (KindAllLegs, tick, pid, 0u, 0u, r.Vessel.verticalSpeed, "");
            if ((t & ContactLatch.EventLanded) != 0)
                EventBus.RecordV2 (KindLanded, tick, pid, 0u, 0u, r.Vessel.verticalSpeed, "");
            if ((t & ContactLatch.EventSituationLanded) != 0)
                EventBus.RecordV2 (KindSituation, tick, pid, 0u, 0u, r.Vessel.verticalSpeed,
                    r.Vessel.situation.ToString ());
            if ((t & ContactLatch.EventCollision) != 0)
                EventBus.RecordV2 (KindCollision, tick, pid, 0u, 0u, r.Vessel.verticalSpeed, "");
            if ((t & ContactLatch.EventContactLost) != 0)
                EventBus.RecordV2 (KindLost, tick, pid, 0u, 0u, r.Vessel.verticalSpeed, "");
        }

        static bool NeedsRescan (WatchRecord r, Vessel v, long tick)
        {
            if (r.ScanTick < 0 || !ReferenceEquals (r.ScannedVessel, v) || v.parts.Count != r.ScannedPartCount ||
                    tick - r.ScanTick >= RescanTicks)
                return true;
            for (int i = 0; i < r.Wheels.Count; i++) {
                var w = r.Wheels [i];
                if (w == null || w.part == null || !ReferenceEquals (w.part.vessel, v))
                    return true;
            }
            return false;
        }

        /// <summary>The wheel modules of the vessel, in part and module order, into the record's lists.</summary>
        static void Rescan (WatchRecord r, Vessel v, long tick)
        {
            var latch = r.Latch;
            latch.SnapshotWheels ();
            r.Wheels.Clear ();
            r.Deployments.Clear ();
            r.WheelParts.Clear ();
            int n = 0;
            var parts = v.parts;
            for (int p = 0; p < parts.Count && n < ContactLatch.MaxWheels; p++) {
                var part = parts [p];
                if ((object)part == null || part.Modules == null)
                    continue;
                ModuleWheelDeployment deployment = null;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var d = part.Modules [m] as ModuleWheelDeployment;
                    if (d != null) {
                        deployment = d;
                        break;
                    }
                }
                int ordinal = 0;
                for (int m = 0; m < part.Modules.Count && n < ContactLatch.MaxWheels; m++) {
                    var wheel = part.Modules [m] as ModuleWheelBase;
                    if (wheel == null)
                        continue;
                    r.Wheels.Add (wheel);
                    r.Deployments.Add (deployment);
                    r.WheelParts.Add (part);
                    latch.WheelPartPid [n] = part.persistentId;
                    latch.WheelOrdinal [n] = ordinal++;
                    latch.IsLeg [n] = wheel.wheelType == WheelType.LEG;
                    n++;
                }
            }
            latch.WheelCount = n;
            latch.RemapFromSnapshot ();
            r.ScannedVessel = v;
            r.ScannedPartCount = parts.Count;
            r.ScanTick = tick;
        }

        static bool IsWheelPart (WatchRecord r, Part part)
        {
            for (int i = 0; i < r.WheelParts.Count; i++)
                if (ReferenceEquals (r.WheelParts [i], part))
                    return true;
            return false;
        }

        /// <summary>WheelState.grounded and contactSpeed; the module's isGrounded before the controller is initialised.</summary>
        static void WheelNow (ModuleWheelBase wheel, out bool grounded, out double rate)
        {
            grounded = false;
            rate = double.NaN;
            if (wheel == null)
                return;
            var controller = wheel.Wheel;
            var state = controller != null ? controller.currentState : null;
            if (state != null) {
                grounded = state.grounded;
                rate = state.contactSpeed;
            } else {
                grounded = wheel.isGrounded;
            }
        }

        static bool IsDeployed (ModuleWheelDeployment deployment)
        {
            if (deployment == null)
                return true;        // a fixed leg or wheel is always out
            return Math.Abs (deployment.position - deployment.deployedPosition) <= DeployedTolerance;
        }

        static bool SituationLanded (Vessel v)
        {
            return v.situation == Vessel.Situations.LANDED || v.situation == Vessel.Situations.SPLASHED;
        }

        /// <summary>The angle between the nose (ReferenceTransform.up) and the local vertical (Vessel.upAxis), rad; NaN unknown.</summary>
        static double Tilt (Vessel v)
        {
            var reference = v.ReferenceTransform;
            if (reference == null)
                return double.NaN;
            return Angle (reference.up, v.upAxis);
        }

        static double Angle (Vector3 a, Vector3d b)
        {
            double na = Math.Sqrt ((double)a.x * a.x + (double)a.y * a.y + (double)a.z * a.z);
            double nb = Math.Sqrt (b.x * b.x + b.y * b.y + b.z * b.z);
            if (!(na > 0.0) || !(nb > 0.0))
                return double.NaN;
            double c = ((double)a.x * b.x + (double)a.y * b.y + (double)a.z * b.z) / (na * nb);
            if (c > 1.0)
                c = 1.0;
            if (c < -1.0)
                c = -1.0;
            return Math.Acos (c);
        }

        static double UniversalTime ()
        {
            try {
                return Planetarium.fetch != null ? Planetarium.GetUniversalTime () : double.NaN;
            } catch (Exception) {
                return double.NaN;
            }
        }

        /// <summary>Forgets every watched vessel (end of the flight scene).</summary>
        internal static void ResetForScene ()
        {
            records.Clear ();
            failures = 0;
        }

        /// <summary>Vessels watched now (tests, status).</summary>
        internal static int WatchedCount {
            get { return records.Count; }
        }

        // ------------------------------------------------------------------ the read

        /// <summary>
        /// contact_v1: the header (<see cref="HeaderStride"/> doubles) and one row per wheel
        /// (<see cref="WheelStride"/>). Live fields are read now; latched fields come from the
        /// watcher's last step (last_step_tick). A loaded vessel that is not watched yet starts
        /// being watched (its latches fill from the next step). A vessel not loaded: found 0, the
        /// rest NaN or -1, no row. "" means the active vessel.
        /// </summary>
        internal static List<double> Read (string persistentId)
        {
            Vessel v;
            uint pid;
            if (string.IsNullOrEmpty (persistentId)) {
                v = FlightGlobals.ActiveVessel;
                pid = v != null ? v.persistentId : 0u;
                if (v != null && !v.loaded)
                    v = null;
            } else {
                pid = VesselRegistry.ParsePersistentId (persistentId);
                v = VesselRegistry.FindLoaded (pid);
            }
            long tick = GameClock.StepTick;
            WatchRecord r = null;
            if (v != null) {
                r = Ensure (pid, v, tick);
                r.LastReadTick = tick;
                if (r.ScanTick < 0 || NeedsRescan (r, v, tick))
                    Rescan (r, v, tick);
            }
            var output = new List<double> (HeaderStride + WheelStride * (r != null ? r.Latch.WheelCount : 0));
            Header (output, pid, v, r, tick);
            if (r != null)
                for (int i = 0; i < r.Latch.WheelCount; i++)
                    Row (output, r, i, v);
            return output;
        }

        static void Header (List<double> o, uint pid, Vessel v, WatchRecord r, long tick)
        {
            const double nan = double.NaN;
            var l = r != null ? r.Latch : null;
            bool stepped = l != null && l.Stepped;
            o.Add (Protocol);
            o.Add (Schema);
            o.Add (pid);
            o.Add (v != null ? 1.0 : 0.0);
            o.Add (tick);
            o.Add (UniversalTime ());
            o.Add (stepped ? 1.0 : 0.0);
            o.Add (stepped ? l.WatchStartTick : -1);
            o.Add (stepped ? l.LastStepTick : -1);
            o.Add (stepped ? (l.StartedInContact ? 1.0 : 0.0) : nan);
            o.Add (stepped ? (l.ContactNow ? 1.0 : 0.0) : nan);
            // Live, now.
            int ground = 0, permanent = 0, colliding = 0, wheelsGrounded = 0, legs = 0, legsDeployed = 0;
            if (v != null) {
                var parts = v.parts;
                for (int p = 0; p < parts.Count; p++) {
                    var part = parts [p];
                    if ((object)part == null)
                        continue;
                    if (part.GroundContact || part.PermanentGroundContact)
                        ground++;
                    if (part.PermanentGroundContact)
                        permanent++;
                    var collisions = part.currentCollisions;
                    if (collisions != null && collisions.Count > 0)
                        colliding++;
                }
                for (int i = 0; i < r.Wheels.Count; i++) {
                    bool g;
                    double rate;
                    WheelNow (r.Wheels [i], out g, out rate);
                    if (g)
                        wheelsGrounded++;
                    if (r.Latch.IsLeg [i]) {
                        legs++;
                        if (IsDeployed (r.Deployments [i]))
                            legsDeployed++;
                    }
                }
            }
            o.Add (v != null ? (v.Landed ? 1.0 : 0.0) : nan);
            o.Add (v != null ? (v.Splashed ? 1.0 : 0.0) : nan);
            o.Add (v != null ? (double)(int)v.situation : nan);
            o.Add (v != null ? ground : nan);
            o.Add (v != null ? permanent : nan);
            o.Add (v != null ? colliding : nan);
            o.Add (v != null ? wheelsGrounded : nan);
            o.Add (v != null ? legs : nan);
            o.Add (v != null ? legsDeployed : nan);
            // Latched by the watcher.
            o.Add (stepped ? l.Episodes : 0);
            o.Add (stepped ? l.FirstEpisodeTick : -1);
            o.Add (stepped ? l.AirborneSinceTick : -1);
            o.Add (stepped ? l.FirstContactTick : -1);
            o.Add (stepped ? l.FirstContactUt : nan);
            o.Add (stepped ? l.FirstContactSource : 0);
            o.Add (stepped ? l.FirstContactPartPid : 0);
            o.Add (stepped ? l.FirstContactVerticalSpeed : nan);
            o.Add (stepped ? l.FirstContactHorizontalSpeed : nan);
            o.Add (stepped ? l.FirstContactTilt : nan);
            o.Add (stepped ? l.AllLegsTick : -1);
            o.Add (stepped ? l.LandedTick : -1);
            o.Add (stepped ? l.SituationLandedTick : -1);
            o.Add (stepped ? l.LostTick : -1);
            o.Add (stepped ? l.CollisionTick : -1);
            // Terrain under the vessel (KSP's own per-step raycast) and kinematics, now.
            double tnx = nan, tny = nan, tnz = nan, terrainTilt = nan;
            if (v != null && v.heightFromTerrain >= 0f && v.vesselTransform != null && v.ReferenceTransform != null) {
                Vector3 world = v.vesselTransform.TransformDirection (v.terrainNormal);
                Vector3 body = v.ReferenceTransform.InverseTransformDirection (world);
                tnx = body.x;
                tny = body.y;
                tnz = body.z;
                terrainTilt = Angle (world, v.upAxis);
            }
            o.Add (v != null ? (double)v.heightFromTerrain : nan);
            o.Add (tnx);
            o.Add (tny);
            o.Add (tnz);
            o.Add (terrainTilt);
            o.Add (v != null ? Tilt (v) : nan);
            o.Add (v != null ? v.verticalSpeed : nan);
            o.Add (v != null ? v.horizontalSrfSpeed : nan);
            o.Add (r != null ? r.Latch.WheelCount : 0);
            o.Add (WheelStride);
        }

        static void Row (List<double> o, WatchRecord r, int i, Vessel v)
        {
            const double nan = double.NaN;
            var l = r.Latch;
            var wheel = r.Wheels [i];
            var part = r.WheelParts [i];
            var deployment = r.Deployments [i];
            o.Add (l.WheelPartPid [i]);
            o.Add (part != null ? part.flightID : 0u);
            o.Add (l.WheelOrdinal [i]);
            o.Add (wheel != null ? (double)(int)wheel.wheelType : nan);
            o.Add (deployment != null ? deployment.position : nan);
            o.Add (deployment != null ? deployment.deployedPosition : nan);
            o.Add (IsDeployed (deployment) ? 1.0 : 0.0);
            var controller = wheel != null ? wheel.Wheel : null;
            var state = controller != null ? controller.currentState : null;
            o.Add (state != null ? (state.grounded ? 1.0 : 0.0) : nan);
            o.Add (wheel != null ? (wheel.isGrounded ? 1.0 : 0.0) : nan);
            o.Add (part != null ? (part.GroundContact ? 1.0 : 0.0) : nan);
            o.Add (state != null ? state.downforce * Frames.KilonewtonToNewton : nan);
            o.Add (state != null ? state.hit.force * Frames.KilonewtonToNewton : nan);
            o.Add (state != null ? state.suspensionCompression : nan);
            o.Add (state != null ? state.contactDepth : nan);
            o.Add (state != null ? state.contactSpeed : nan);
            o.Add (state != null ? state.normalizedLoad : nan);
            double nx = nan, ny = nan, nz = nan, layer = -1;
            if (state != null && state.grounded && v.ReferenceTransform != null) {
                Vector3 body = v.ReferenceTransform.InverseTransformDirection (state.hit.normal);
                nx = body.x;
                ny = body.y;
                nz = body.z;
                var collider = state.hit.collider;
                if (collider != null && collider.gameObject != null)
                    layer = collider.gameObject.layer;
            }
            o.Add (nx);
            o.Add (ny);
            o.Add (nz);
            o.Add (layer);
            o.Add (l.FirstGroundedTick [i]);
            o.Add (l.FirstGroundedUt [i]);
            o.Add (l.FirstGroundedRate [i]);
            o.Add (l.WheelTransitions [i]);
            o.Add (l.LastChangeTick [i]);
        }
    }
}
