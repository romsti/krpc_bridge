using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using UnityEngine;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// The actuator authority of the bridge (refactor R3: the static state and logic of the
    /// former ActuatorsAddon; the KSP addons are ControlAddon and ControlWatcher): the legacy
    /// per-engine throttle and per-gimbal leases, the atomic protocol v2 and its exclusive
    /// owner, AttitudeV1's authority on the active vessel and on a non-active one, and the
    /// physics-step pump.
    ///
    /// A lease always restores the original fields, so losing the Python client cannot leave an
    /// engine decoupled from the main throttle for good. Every access runs on the Unity thread:
    /// kRPC's RPC bodies and the watcher's FixedUpdate both run inside the physics loop.
    ///
    /// What observes (KRPC.Bridge.Telemetry) is called from the pump through
    /// <see cref="Observer"/>, at the points where the former single DLL called it: this class
    /// never depends on the telemetry, a telemetry change never touches it.
    /// </summary>
    internal static class Actuation
    {
        internal const int ProtocolVersion = 2;
        // Schema of control_status_v2 (layout control_v2): 2 since refactor R3 appended the
        // controller's vessel persistentId.
        internal const int ControlStatusSchema = 2;
        internal const int ResultNone = 0;
        internal const int ResultQueued = 1;
        internal const int ResultApplied = 2;
        internal const int ResultFrameExpired = -1;
        internal const int ResultAuthorityLost = -2;
        internal const int ResultTopologyChanged = -3;
        internal const int ResultApplyFailed = -4;
        internal const int ResultSuperseded = -5;

        sealed class ThrottleLease
        {
            internal bool OriginalEnabled;
            internal float OriginalPercentage;
            // Core v2: the expiry (wall guard, the legacy float arithmetic, no step budget).
            internal readonly Lease Term = new Lease ();
        }

        sealed class GimbalLease
        {
            internal bool OriginalActive;
            internal Vector3 OriginalActuationLocal;
            internal List<Quaternion> OriginalRotations;
            internal float TargetXDegrees;
            internal float TargetYDegrees;
            internal float AppliedXDegrees;
            internal float AppliedYDegrees;
            // QW5: last PHYSICS step at which the ramp moved (game time, no longer
            // Time.realtimeSinceStartup).
            internal long LastResponseTick;
            internal readonly Lease Term = new Lease ();
            // M4: an AttitudeV1 lease, handed back WITHOUT a jolt (the stock Lerp starts again
            // from the deflection reached) instead of the rotation from before the lease.
            internal bool Bumpless;
            // the deflection (x, y) is the MODULE's, in pivot 0's frame; each pivot turns
            // by its projection (GimbalPivots.cs), like the stock ModuleGimbal. Null: the same
            // (x, y) on every pivot, in its own frame (V1, unchanged).
            internal GimbalPivotCommand Pivots;
        }

        internal sealed class EngineFrameCommand
        {
            internal ModuleEngines Engine;
            internal float Percentage;
        }

        internal sealed class GimbalFrameCommand
        {
            internal ModuleGimbal Gimbal;
            internal float XDegrees;
            internal float YDegrees;
        }

        sealed class PendingControlFrame
        {
            internal string Token;
            internal int Sequence;
            internal long ApplyTick;
            internal long ValidUntilTick;
            internal float LeaseSeconds;
            internal List<EngineFrameCommand> Engines;
            internal List<GimbalFrameCommand> Gimbals;
            internal long TopologyGeneration;
        }

        static readonly Dictionary<ModuleEngines, ThrottleLease> throttleLeases =
            new Dictionary<ModuleEngines, ThrottleLease> ();
        static readonly Dictionary<ModuleGimbal, GimbalLease> gimbalLeases =
            new Dictionary<ModuleGimbal, GimbalLease> ();

        // Core v2: the pump tick is a counter hosted by the game clock (clock_v1 maps it onto the
        // master tick); its value and its increments are the ones it always had.
        static readonly ClockCounter pumpTicks = GameClock.Counter ("actuators.pump_tick");
        static long topologyGeneration;

        /// <summary>Pump physics-step counter (read by AttitudeV1 and the order probe).</summary>
        internal static long CurrentPhysicsTick {
            get { return pumpTicks.Value; }
        }
        static Guid observedVesselId = Guid.Empty;
        static ulong observedTopologyHash;
        static IList<double> lastSnapshot = new List<double> ();

        // Core v2: the v2 controller's authority (owner, token, wall guard) is one Lease. Also
        // held by AttitudeV1 on the active vessel (TryHoldForAttitude).
        static readonly Lease controller = new Lease ();
        static Guid controllerVesselId = Guid.Empty;
        // Refactor R3 (state per vessel): the persistentId of the controller's vessel, published
        // by control_status_v2 (NaN without a controller).
        static uint controllerVesselPid;
        static long controllerTopologyGeneration;
        static PendingControlFrame pendingFrame;
        static int lastAcceptedSequence;
        static int lastAppliedSequence;
        static int lastResult;
        static long lastAppliedTick;

        /// <summary>
        /// What observes, registered by KRPC.Bridge.Telemetry at load (TelemetryAddon.Awake);
        /// null without it. Called at the points of the pump where the single Actuators DLL
        /// called DynamicsV3, DynamicsTracked and ResponseTracker, so the order of a physics step
        /// is the one it always was.
        /// </summary>
        internal static IActuationObserver Observer;

        /// <summary>
        /// One step of the physics loop, called by ControlWatcher at every FixedUpdate of the
        /// Flight scene. All the state is static, so it survives the creation and destruction
        /// of the watcher from scene to scene.
        /// </summary>
        internal static void Pump ()
        {
            pumpTicks.Increment ();
            // Core v2: one window of the vessel registry (the fingerprint of a vessel is walked
            // once in this pump, then served from the memo: no game code runs in between).
            int window = VesselRegistry.OpenWindow ();
            try {
                PumpInWindow ();
            } finally {
                VesselRegistry.CloseWindow (window);
            }
        }

        // Scratch of the pump (Unity main thread only): no list allocated per physics step.
        static readonly List<ModuleEngines> expiredEnginesScratch = new List<ModuleEngines> ();
        static readonly List<ModuleGimbal> expiredGimbalsScratch = new List<ModuleGimbal> ();

        static void PumpInWindow ()
        {
            long physicsTick = pumpTicks.Value;
            var vessel = FlightGlobals.ActiveVessel;
            ObserveTopology (vessel);
            float now = Time.realtimeSinceStartup;
            if (controller.Held && controller.ExpiredReal (now))
                ExpireController ();
            // AttitudeV1's leases on a NON-ACTIVE vessel (ramp, expiry, handed back when
            // that vessel becomes the active one -- BEFORE any frame of the active vessel -- or
            // is no longer loaded). Returns at once when there is none: active path unchanged.
            PumpInactive (vessel, now);
            ApplyPendingFrame (vessel);

            var expiredEngines = expiredEnginesScratch;
            expiredEngines.Clear ();
            foreach (var pair in throttleLeases) {
                if (pair.Key == null || pair.Value.Term.ExpiredReal (now))
                    expiredEngines.Add (pair.Key);
            }
            for (int i = 0; i < expiredEngines.Count; i++)
                RestoreThrottle (expiredEngines [i]);

            var expiredGimbals = expiredGimbalsScratch;
            expiredGimbals.Clear ();
            foreach (var pair in gimbalLeases) {
                if (!GimbalReady (pair.Key) || pair.Value.Term.ExpiredReal (now))
                    expiredGimbals.Add (pair.Key);
                else
                    ApplyGimbal (pair.Key, pair.Value);
            }
            for (int i = 0; i < expiredGimbals.Count; i++)
                RestoreGimbal (expiredGimbals [i]);

            // What observes: the active vessel's DynamicsV3 (and AeroActuatorV1) frame, then
            // the vessels followed by persistentId (double flights), in this order.
            var observer = Observer;
            if (observer != null)
                observer.Capture (vessel, physicsTick, topologyGeneration, lastAcceptedSequence,
                    lastAppliedSequence, lastAppliedTick, lastResult, controllerVesselId);

            // AttitudeV1 (shadow, born closed): witness of the Earlyish stage and fallback
            // onto the pump when it stops running; returns at once without an armed vessel.
            // After the captures: this tick's V3 does not depend on it.
            try {
                AttitudeV1.PumpWatch ();
            } catch (Exception exc) {
                BridgeLog.Error ("AttitudeV1.PumpWatch: " + exc.Message);
            }
        }

        internal static void RequireLegacyControlAvailable ()
        {
            if (controller.Held && controller.ExpiredReal (Time.realtimeSinceStartup))
                ExpireController ();
            if (controller.Held)
                throw new InvalidOperationException (BridgeErrors.Tag (BridgeErrors.Busy,
                    "controle v2 deja acquis par " + controller.Owner));
        }

        internal static string AcquireControl (string owner, float leaseSeconds)
        {
            if (string.IsNullOrWhiteSpace (owner))
                throw new ArgumentException ("owner vide", nameof (owner));
            if (owner.Length > 80)
                throw new ArgumentOutOfRangeException (nameof (owner));
            ValidateControllerLease (leaseSeconds);
            var vessel = RequireLoadedActiveVessel ();
            ObserveTopology (vessel);
            if (controller.Held && controller.ExpiredReal (Time.realtimeSinceStartup))
                ExpireController ();
            if (controller.Held)
                throw new InvalidOperationException (BridgeErrors.Tag (BridgeErrors.Busy,
                    "controle v2 deja acquis par " + controller.Owner));

            // A legacy lease has no owner identity. Acquiring the v2 authority is an
            // explicit handover, so restore it before installing the new owner.
            RestoreAll ();
            controller.Grant (owner.Trim (), Guid.NewGuid ().ToString ("N"),
                Time.realtimeSinceStartup, leaseSeconds);
            controllerVesselId = vessel.id;
            controllerVesselPid = vessel.persistentId;
            controllerTopologyGeneration = topologyGeneration;
            pendingFrame = null;
            lastAcceptedSequence = 0;
            lastAppliedSequence = 0;
            lastAppliedTick = 0;
            lastResult = ResultNone;
            return controller.Token;
        }

        internal static bool RenewControl (string token, float leaseSeconds)
        {
            ValidateControllerLease (leaseSeconds);
            RequireController (token);
            controller.RenewReal (Time.realtimeSinceStartup, leaseSeconds);
            return true;
        }

        internal static int ReleaseControl (string token)
        {
            RequireController (token);
            int restored = RestoreAll ();
            ClearController (ResultAuthorityLost, false);
            return restored;
        }

        internal static int QueueControlFrame (
            string token, int sequence, int requestedApplyTick, int validUntilTick,
            float leaseSeconds, List<EngineFrameCommand> engines,
            List<GimbalFrameCommand> gimbals)
        {
            RequireController (token);
            var vessel = RequireLoadedActiveVessel ();
            ObserveTopology (vessel);
            if (vessel.id != controllerVesselId ||
                    topologyGeneration != controllerTopologyGeneration) {
                ClearController (ResultTopologyChanged, true);
                throw new InvalidOperationException (BridgeErrors.Tag (BridgeErrors.Transient,
                    "topologie du vaisseau modifiee : reacquerir le controle"));
            }
            if (sequence <= 0 || sequence <= lastAcceptedSequence)
                throw new ArgumentOutOfRangeException (
                    nameof (sequence), "sequence doit etre strictement croissante");
            ValidateActuatorLease (leaseSeconds);

            long physicsTick = pumpTicks.Value;
            long earliest = physicsTick + 1;
            long applyTick = requestedApplyTick <= 0 ? earliest : requestedApplyTick;
            if (applyTick < earliest || applyTick > physicsTick + 250)
                throw new ArgumentOutOfRangeException (
                    nameof (requestedApplyTick), "tick hors fenetre [prochain, +250]");
            long deadline = validUntilTick <= 0 ? applyTick : validUntilTick;
            if (deadline < applyTick || deadline > applyTick + 250)
                throw new ArgumentOutOfRangeException (
                    nameof (validUntilTick), "deadline anterieure ou trop lointaine");

            bool superseded = pendingFrame != null;
            pendingFrame = new PendingControlFrame {
                Token = token,
                Sequence = sequence,
                ApplyTick = applyTick,
                ValidUntilTick = deadline,
                LeaseSeconds = leaseSeconds,
                Engines = engines,
                Gimbals = gimbals,
                TopologyGeneration = topologyGeneration
            };
            lastAcceptedSequence = sequence;
            lastResult = superseded ? ResultSuperseded : ResultQueued;
            return (int)applyTick;
        }

        internal static IList<double> ControlStatus ()
        {
            float remaining = !controller.Held
                ? 0f
                : controller.RemainingReal (Time.realtimeSinceStartup);
            return new List<double> {
                ProtocolVersion,
                pumpTicks.Value,
                topologyGeneration,
                !controller.Held ? 0.0 : 1.0,
                remaining,
                pendingFrame == null ? 0.0 : pendingFrame.Sequence,
                pendingFrame == null ? 0.0 : pendingFrame.ApplyTick,
                lastAcceptedSequence,
                lastAppliedSequence,
                lastResult,
                lastAppliedTick,
                // Schema 2 (refactor R3): the vessel the v2 authority drives, NaN without one.
                controller.Held ? (double)controllerVesselPid : double.NaN
            };
        }

        /// <summary>
        /// AcquireControl, for the vessel the client names (refactor R3, state per vessel): the
        /// v2 protocol drives the ACTIVE vessel only, so a persistentId that is not the loaded
        /// active vessel is refused (NOT_LOADED) instead of acquiring the authority over another
        /// vessel than the one the client means (a switch of active vessel, an FMRS jump).
        /// </summary>
        internal static string AcquireControlFor (string owner, float leaseSeconds, string persistentId)
        {
            uint pid = VesselLookup.ParsePersistentId (persistentId);
            var active = FlightGlobals.ActiveVessel;
            if (active == null || !active.loaded || active.persistentId != pid)
                throw new InvalidOperationException (BridgeErrors.Tag (BridgeErrors.NotLoaded,
                    "vessel " + persistentId + " is not the loaded active vessel: protocol v2 " +
                    "drives the active vessel only"));
            return AcquireControl (owner, leaseSeconds);
        }

        internal static IList<double> ReadControlSnapshot ()
        {
            var vessel = RequireLoadedActiveVessel ();
            ObserveTopology (vessel);
            // kRPC executes this short RPC body on Unity's physics thread. Capture on
            // demand so every row belongs to this callback without allocating a full
            // vessel snapshot on every tick when no client is reading it.
            CaptureSnapshot (vessel);
            return new List<double> (lastSnapshot);
        }

        internal static bool ThrottleIsLeased (ModuleEngines engine)
        {
            return throttleLeases.ContainsKey (engine) || InactiveThrottleLease (engine) != null;
        }

        internal static bool GimbalIsLeased (ModuleGimbal gimbal)
        {
            return gimbalLeases.ContainsKey (gimbal) || InactiveGimbalLease (gimbal) != null;
        }

        internal static void GimbalLeaseTarget (
            ModuleGimbal gimbal, out float xDegrees, out float yDegrees)
        {
            GimbalLease lease;
            if (gimbalLeases.TryGetValue (gimbal, out lease) ||
                    (lease = InactiveGimbalLease (gimbal)) != null) {
                xDegrees = lease.TargetXDegrees;
                yDegrees = lease.TargetYDegrees;
                return;
            }
            xDegrees = gimbal.actuationLocal.x;
            yDegrees = gimbal.actuationLocal.y;
        }

        internal static void ValidateEngineFrameCommand (
            ModuleEngines engine, double percentage)
        {
            if (engine == null)
                throw new InvalidOperationException (BridgeErrors.Tag (BridgeErrors.NotFound,
                    "moteur detruit"));
            if (double.IsNaN (percentage) || double.IsInfinity (percentage) ||
                    percentage < 0.0 || percentage > 100.0)
                throw new ArgumentOutOfRangeException (
                    nameof (percentage), "throttle independant attendu entre 0 et 100");
        }

        internal static void ValidateGimbalFrameCommand (
            ModuleGimbal gimbal, double xDegrees, double yDegrees)
        {
            RequireGimbalTransforms (gimbal);
            if (gimbal.gimbalLock)
                throw new InvalidOperationException ("gimbal verrouille");
            if (double.IsNaN (xDegrees) || double.IsInfinity (xDegrees) ||
                    double.IsNaN (yDegrees) || double.IsInfinity (yDegrees))
                throw new ArgumentOutOfRangeException ("gimbal", "consigne non finie");
        }

        static void ApplyPendingFrame (Vessel vessel)
        {
            long physicsTick = pumpTicks.Value;
            if (pendingFrame == null || physicsTick < pendingFrame.ApplyTick)
                return;
            var frame = pendingFrame;
            pendingFrame = null;
            if (!controller.Held || frame.Token != controller.Token) {
                lastResult = ResultAuthorityLost;
                return;
            }
            if (vessel == null || !vessel.loaded || vessel.id != controllerVesselId ||
                    frame.TopologyGeneration != topologyGeneration) {
                ClearController (ResultTopologyChanged, true);
                return;
            }
            if (physicsTick > frame.ValidUntilTick) {
                lastResult = ResultFrameExpired;
                return;
            }

            // Observation only (DynamicsExtV1 armed): baselines BEFORE the commands are
            // written, so the first realized response of each actuator can be dated.
            var observer = Observer;
            bool trackResponse = observer != null && observer.TracksResponses;
            if (trackResponse)
                observer.BeforeApply (
                    vessel.id, frame.Sequence, frame.Engines, frame.Gimbals);

            try {
                for (int i = 0; i < frame.Engines.Count; i++)
                    Command (frame.Engines [i].Engine,
                        frame.Engines [i].Percentage, frame.LeaseSeconds);
                for (int i = 0; i < frame.Gimbals.Count; i++)
                    CommandGimbal (frame.Gimbals [i].Gimbal,
                        frame.Gimbals [i].XDegrees, frame.Gimbals [i].YDegrees,
                        frame.LeaseSeconds);
                lastAppliedSequence = frame.Sequence;
                lastAppliedTick = physicsTick;
                lastResult = ResultApplied;
                if (trackResponse)
                    observer.AfterApply (physicsTick);
            } catch (Exception exc) {
                if (trackResponse)
                    observer.ClearResponses ();
                RestoreAll ();
                lastResult = ResultApplyFailed;
                BridgeLog.Error (
                    "Actuators v2 frame " + frame.Sequence + " refusee: " + exc);
            }
        }

        static void ObserveTopology (Vessel vessel)
        {
            if (vessel == null || !vessel.loaded) {
                if (observedVesselId != Guid.Empty || controller.Held)
                    ClearController (ResultTopologyChanged, true);
                observedVesselId = Guid.Empty;
                observedTopologyHash = 0;
                lastSnapshot = new List<double> ();
                ResetObservedVessel ();
                return;
            }
            ulong hash = ComputeTopologyHash (vessel);
            if (observedVesselId == vessel.id && observedTopologyHash == hash)
                return;
            bool replacingObservedVessel = observedVesselId != Guid.Empty;
            topologyGeneration++;
            if (replacingObservedVessel || controller.Held)
                ClearController (ResultTopologyChanged, true);
            observedVesselId = vessel.id;
            observedTopologyHash = hash;
            lastSnapshot = new List<double> ();
            ResetObservedVessel ();
        }

        static void ResetObservedVessel ()
        {
            var observer = Observer;
            if (observer != null)
                observer.ResetActiveVessel ();
        }

        /// <summary>
        /// The topology fingerprint of a vessel: Core v2 VesselRegistry.TopologyHash, the same
        /// function moved to the Core, walked once per vessel and per window of our own code
        /// (the pump, the AttitudeV1 tick) and served from the memo within it.
        /// </summary>
        internal static ulong ComputeTopologyHash (Vessel vessel)
        {
            return VesselRegistry.TopologyHash (vessel);
        }

        static void CaptureSnapshot (Vessel vessel)
        {
            if (vessel == null || !vessel.loaded || vessel.id != observedVesselId) {
                lastSnapshot = new List<double> ();
                return;
            }
            const int headerStride = 14;
            const int engineStride = 18;
            const int gimbalStride = 19;
            const int thrustTransformStride = 10;
            var engines = new List<double> ();
            var gimbals = new List<double> ();
            var transforms = new List<double> ();
            int engineCount = 0;
            int gimbalCount = 0;
            int transformCount = 0;
            var reference = vessel.ReferenceTransform;
            Vector3 com = vessel.CurrentCoM;

            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                if (part == null)
                    continue;
                int engineOrdinal = 0;
                int gimbalOrdinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var engine = part.Modules [m] as ModuleEngines;
                    if (engine != null) {
                        int ownTransformCount = engine.thrustTransforms == null
                            ? 0 : engine.thrustTransforms.Count;
                        engines.Add (part.flightID);
                        engines.Add (engineOrdinal);
                        engines.Add (engine.EngineIgnited ? 1.0 : 0.0);
                        engines.Add (engine.requestedThrottle);
                        engines.Add (engine.currentThrottle);
                        engines.Add (engine.finalThrust);
                        engines.Add (engine.maxThrust);
                        engines.Add (engine.thrustPercentage);
                        engines.Add (engine.independentThrottle ? 1.0 : 0.0);
                        engines.Add (engine.independentThrottlePercentage);
                        engines.Add (engine.useEngineResponseTime ? 1.0 : 0.0);
                        engines.Add (engine.engineAccelerationSpeed);
                        engines.Add (engine.engineDecelerationSpeed);
                        engines.Add (engine.requestedMassFlow);
                        engines.Add (engine.propellantReqMet);
                        engines.Add (engine.realIsp);
                        engines.Add (ownTransformCount);
                        engines.Add (ThrottleIsLeased (engine) ? 1.0 : 0.0);
                        engineCount++;

                        for (int t = 0; t < ownTransformCount; t++) {
                            var transform = engine.thrustTransforms [t];
                            float multiplier = 1f;
                            if (engine.thrustTransformMultipliers != null &&
                                    t < engine.thrustTransformMultipliers.Count)
                                multiplier = engine.thrustTransformMultipliers [t];
                            transforms.Add (part.flightID);
                            transforms.Add (engineOrdinal);
                            transforms.Add (t);
                            transforms.Add (multiplier);
                            if (transform == null || reference == null) {
                                for (int n = 0; n < 6; n++)
                                    transforms.Add (double.NaN);
                            } else {
                                Vector3 lever = reference.InverseTransformDirection (
                                    transform.position - com);
                                Vector3 direction = reference.InverseTransformDirection (
                                    transform.forward.normalized);
                                transforms.Add (lever.x);
                                transforms.Add (lever.y);
                                transforms.Add (lever.z);
                                transforms.Add (direction.x);
                                transforms.Add (direction.y);
                                transforms.Add (direction.z);
                            }
                            transformCount++;
                        }
                        engineOrdinal++;
                    }

                    var gimbal = part.Modules [m] as ModuleGimbal;
                    if (gimbal != null) {
                        float targetX;
                        float targetY;
                        GimbalLeaseTarget (gimbal, out targetX, out targetY);
                        int ownTransformCount = gimbal.gimbalTransforms == null
                            ? 0 : gimbal.gimbalTransforms.Count;
                        gimbals.Add (part.flightID);
                        gimbals.Add (gimbalOrdinal++);
                        gimbals.Add (gimbal.gimbalLock ? 1.0 : 0.0);
                        gimbals.Add (gimbal.gimbalActive ? 1.0 : 0.0);
                        gimbals.Add (gimbal.gimbalLimiter);
                        gimbals.Add (gimbal.gimbalRange);
                        gimbals.Add (gimbal.gimbalRangeXN);
                        gimbals.Add (gimbal.gimbalRangeXP);
                        gimbals.Add (gimbal.gimbalRangeYN);
                        gimbals.Add (gimbal.gimbalRangeYP);
                        gimbals.Add (gimbal.useGimbalResponseSpeed ? 1.0 : 0.0);
                        gimbals.Add (gimbal.gimbalResponseSpeed);
                        gimbals.Add (gimbal.actuationLocal.x);
                        gimbals.Add (gimbal.actuationLocal.y);
                        gimbals.Add (gimbal.actuationLocal.z);
                        gimbals.Add (ownTransformCount);
                        gimbals.Add (GimbalIsLeased (gimbal) ? 1.0 : 0.0);
                        gimbals.Add (targetX);
                        gimbals.Add (targetY);
                        gimbalCount++;
                    }
                }
            }

            var output = new List<double> (
                headerStride + engines.Count + gimbals.Count + transforms.Count) {
                ProtocolVersion,
                pumpTicks.Value,
                Planetarium.GetUniversalTime (),
                TimeWarp.fixedDeltaTime,
                vessel.persistentId,
                topologyGeneration,
                lastAppliedSequence,
                lastResult,
                engineCount,
                engineStride,
                gimbalCount,
                gimbalStride,
                transformCount,
                thrustTransformStride
            };
            output.AddRange (engines);
            output.AddRange (gimbals);
            output.AddRange (transforms);
            lastSnapshot = output;
        }

        static Vessel RequireLoadedActiveVessel ()
        {
            var vessel = FlightGlobals.ActiveVessel;
            if (vessel == null || !vessel.loaded)
                throw new InvalidOperationException (BridgeErrors.Tag (BridgeErrors.NotLoaded,
                    "aucun vaisseau actif et charge"));
            return vessel;
        }

        static void RequireController (string token)
        {
            if (controller.Held && controller.ExpiredReal (Time.realtimeSinceStartup))
                ExpireController ();
            if (string.IsNullOrEmpty (token) || token != controller.Token)
                throw new InvalidOperationException (BridgeErrors.Tag (controllerTokens.CodeOf (token),
                    "token de controle absent, expire ou invalide"));
        }

        static void ValidateControllerLease (float leaseSeconds)
        {
            if (float.IsNaN (leaseSeconds) || float.IsInfinity (leaseSeconds) ||
                    leaseSeconds < 0.1f || leaseSeconds > 5f)
                throw new ArgumentOutOfRangeException (
                    nameof (leaseSeconds), "bail controleur attendu entre 0.1 et 5 s");
        }

        static void ValidateActuatorLease (float leaseSeconds)
        {
            if (float.IsNaN (leaseSeconds) || float.IsInfinity (leaseSeconds) ||
                    leaseSeconds < 0.05f || leaseSeconds > 1f)
                throw new ArgumentOutOfRangeException (
                    nameof (leaseSeconds), "bail actionneur attendu entre 0.05 et 1 s");
        }

        // Refactor R1: why the last controller tokens stopped being valid.
        static readonly TokenMemory controllerTokens = new TokenMemory (8);

        /// <summary>The controller's wall lease ran out: its token is EXPIRED, not invalid.</summary>
        static void ExpireController ()
        {
            var token = controller.Token;
            ClearController (ResultAuthorityLost, true);
            controllerTokens.Remember (token, BridgeErrors.TokenExpired);
        }

        static void ClearController (int result, bool restore)
        {
            controllerTokens.Remember (controller.Token, BridgeErrors.TokenInvalid);
            if (restore)
                RestoreAll ();
            controller.Clear ();
            controllerVesselId = Guid.Empty;
            controllerVesselPid = 0;
            controllerTopologyGeneration = 0;
            pendingFrame = null;
            lastResult = result;
        }

        internal static void ResetForScene ()
        {
            ClearController (ResultTopologyChanged, true);
            observedVesselId = Guid.Empty;
            observedTopologyHash = 0;
            lastSnapshot = new List<double> ();
            var observer = Observer;
            if (observer != null)
                observer.ResetForScene ();
            AttitudeV1.ResetForScene ();
            // AttitudeV1 has just handed its authorities back; sweep whatever is left.
            ReleaseAllInactive ();
        }

        internal static void Command (ModuleEngines engine, float percentage, float leaseSeconds)
        {
            CommandInto (throttleLeases, engine, percentage, leaseSeconds);
        }

        /// <summary>
        /// Body of Command, on the given lease dictionary: the active vessel owners' one
        /// (throttleLeases), or that of an AttitudeV1 authority on a non-active vessel.
        /// Same arithmetic, same order.
        /// </summary>
        static void CommandInto (Dictionary<ModuleEngines, ThrottleLease> leases,
            ModuleEngines engine, float percentage, float leaseSeconds)
        {
            ThrottleLease lease;
            if (!leases.TryGetValue (engine, out lease)) {
                lease = new ThrottleLease {
                    OriginalEnabled = engine.independentThrottle,
                    OriginalPercentage = engine.independentThrottlePercentage
                };
                leases.Add (engine, lease);
            }

            engine.independentThrottlePercentage = Math.Min (100f, Math.Max (0f, percentage));
            engine.independentThrottle = true;
            lease.Term.RenewReal (Time.realtimeSinceStartup, Math.Min (1f, Math.Max (0.05f, leaseSeconds)));
        }

        internal static bool ReleaseThrottle (ModuleEngines engine)
        {
            if (!throttleLeases.ContainsKey (engine))
                return false;
            RestoreThrottle (engine);
            return true;
        }

        internal static void CommandGimbal (
            ModuleGimbal gimbal, float xDegrees, float yDegrees, float leaseSeconds)
        {
            CommandGimbalInto (gimbalLeases, gimbal, xDegrees, yDegrees, leaseSeconds);
        }

        /// <summary>
        /// Body of CommandGimbal, on the given lease dictionary (see CommandInto); returns the
        /// lease. Same arithmetic, same order.
        /// </summary>
        static GimbalLease CommandGimbalInto (Dictionary<ModuleGimbal, GimbalLease> leases,
            ModuleGimbal gimbal, float xDegrees, float yDegrees, float leaseSeconds,
            GimbalPivotCommand pivots = null)
        {
            RequireGimbalTransforms (gimbal);
            if (gimbal.gimbalLock)
                throw new InvalidOperationException (
                    "gimbal verrouille : le deverrouiller avant de demander un bail");
            GimbalLease lease;
            if (!leases.TryGetValue (gimbal, out lease)) {
                var rotations = new List<Quaternion> (gimbal.gimbalTransforms.Count);
                for (int i = 0; i < gimbal.gimbalTransforms.Count; i++)
                    rotations.Add (gimbal.gimbalTransforms [i].localRotation);
                lease = new GimbalLease {
                    OriginalActive = gimbal.gimbalActive,
                    OriginalActuationLocal = gimbal.actuationLocal,
                    AppliedXDegrees = gimbal.actuationLocal.x,
                    AppliedYDegrees = gimbal.actuationLocal.y,
                    LastResponseTick = pumpTicks.Value,
                    OriginalRotations = rotations
                };
                if (pivots != null) {
                    // the ramp starts from the MODULE deflection stock holds (pivot 0),
                    // not from actuationLocal (the LAST pivot's, in its own frame).
                    lease.AppliedXDegrees = (float)pivots.StartX;
                    lease.AppliedYDegrees = (float)pivots.StartY;
                }
                leases.Add (gimbal, lease);
            }
            lease.Pivots = pivots;

            float now = Time.realtimeSinceStartup;
            ApplyGimbal (gimbal, lease);

            float limiter = Math.Min (100f, Math.Max (0f, gimbal.gimbalLimiter)) * 0.01f;
            lease.TargetXDegrees = ClampDirectional (
                xDegrees, gimbal.gimbalRangeXN * limiter, gimbal.gimbalRangeXP * limiter);
            lease.TargetYDegrees = ClampDirectional (
                yDegrees, gimbal.gimbalRangeYN * limiter, gimbal.gimbalRangeYP * limiter);
            if (!gimbal.useGimbalResponseSpeed) {
                lease.AppliedXDegrees = lease.TargetXDegrees;
                lease.AppliedYDegrees = lease.TargetYDegrees;
            }
            lease.Term.RenewReal (now, ClampLeaseSeconds (leaseSeconds));
            gimbal.gimbalActive = false;
            ApplyGimbal (gimbal, lease);
            return lease;
        }

        internal static bool ReleaseGimbal (ModuleGimbal gimbal)
        {
            if (!gimbalLeases.ContainsKey (gimbal))
                return false;
            RestoreGimbal (gimbal);
            return true;
        }

        // ------------------------------------------- M4: AttitudeV1's authority
        internal const float AttitudeAuthoritySeconds = 1f;
        internal const float AttitudeGimbalLeaseSeconds = 0.25f;

        /// <summary>
        /// M4 (work item 11): AttitudeV1 takes the exclusive actuator authority (the v2 owner)
        /// from INSIDE the plugin, on the ACTIVE vessel, so that no client (S7 in Python, a
        /// legacy lease) moves the same gimbals. Refused (false) when another holds it or a
        /// legacy lease is active. Renewed at every step that writes (real time,
        /// AttitudeAuthoritySeconds).
        /// </summary>
        internal static bool TryHoldForAttitude (string owner, Vessel vessel, ref string token)
        {
            if (vessel == null || !vessel.loaded || !ReferenceEquals (vessel, FlightGlobals.ActiveVessel))
                return false;
            float now = Time.realtimeSinceStartup;
            if (controller.Held && controller.ExpiredReal (now))
                ExpireController ();
            if (controller.Held) {
                if (token == null || token != controller.Token)
                    return false;
                controller.RenewReal (now, AttitudeAuthoritySeconds);
                return true;
            }
            if (throttleLeases.Count > 0 || gimbalLeases.Count > 0)
                return false;
            ObserveTopology (vessel);
            controller.Grant (owner, Guid.NewGuid ().ToString ("N"), now, AttitudeAuthoritySeconds);
            controllerVesselId = vessel.id;
            controllerVesselPid = vessel.persistentId;
            controllerTopologyGeneration = topologyGeneration;
            pendingFrame = null;
            lastAcceptedSequence = 0;
            lastAppliedSequence = 0;
            lastAppliedTick = 0;
            lastResult = ResultNone;
            token = controller.Token;
            return true;
        }

        internal static bool HoldsForAttitude (string token)
        {
            return (token != null && token == controller.Token &&
                controller.ValidReal (Time.realtimeSinceStartup)) || HoldsInactive (token);
        }

        /// <summary>Hands the gimbals back (without a jolt) and the authority, if it really is its own.</summary>
        internal static void ReleaseForAttitude (string token)
        {
            if (IsInactiveToken (token)) {
                // the authority of a non-active vessel; the active vessel's is not
                // touched.
                ReleaseInactive (token);
                return;
            }
            if (token == null || token != controller.Token)
                return;
            RestoreAll ();
            ClearController (ResultAuthorityLost, false);
        }

        /// <summary>Another owner (v2 or a legacy lease) forbids arming M4.</summary>
        internal static bool OtherOwnerPresent ()
        {
            if (controller.Held && controller.ValidReal (Time.realtimeSinceStartup))
                return true;
            return throttleLeases.Count > 0 || gimbalLeases.Count > 0;
        }

        /// <summary>
        /// AttitudeV1's gimbal setting (degrees) under its authority: the same lease and the same
        /// QW5 ramp as the others, but handed back without a jolt. False when the authority is
        /// no longer its own.
        /// </summary>
        internal static bool CommandGimbalForAttitude (string token, ModuleGimbal gimbal,
            float xDegrees, float yDegrees)
        {
            if (IsInactiveToken (token))
                return CommandGimbalInactive (token, gimbal, xDegrees, yDegrees);
            if (!HoldsForAttitude (token))
                return false;
            // a gimbal still leased by a non-active authority is never leased twice.
            if (InactiveGimbalLease (gimbal) != null)
                return false;
            CommandGimbal (gimbal, xDegrees, yDegrees, AttitudeGimbalLeaseSeconds);
            GimbalLease lease;
            if (gimbalLeases.TryGetValue (gimbal, out lease))
                lease.Bumpless = true;
            return true;
        }

        /// <summary>
        /// the same setting for a MAPPED module (several non-parallel pivots): (x, y) is the
        /// module's deflection in pivot 0's frame, each pivot turns by its projection
        /// (GimbalPivots.cs). `pivots` null: exactly the V1 overload above.
        /// </summary>
        internal static bool CommandGimbalForAttitude (string token, ModuleGimbal gimbal,
            float xDegrees, float yDegrees, GimbalPivotCommand pivots)
        {
            if (pivots == null)
                return CommandGimbalForAttitude (token, gimbal, xDegrees, yDegrees);
            if (IsInactiveToken (token))
                return CommandGimbalInactive (token, gimbal, xDegrees, yDegrees, pivots);
            if (!HoldsForAttitude (token))
                return false;
            if (InactiveGimbalLease (gimbal) != null)
                return false;
            var lease = CommandGimbalInto (gimbalLeases, gimbal, xDegrees, yDegrees,
                AttitudeGimbalLeaseSeconds, pivots);
            lease.Bumpless = true;
            return true;
        }

        /// <summary>
        /// Lateral force (work item 19, continued): an engine's independent throttle, in percent
        /// (0-100, independentThrottlePercentage), under AttitudeV1's authority. The same lease as
        /// the others (original fields restored at the end, at expiry in real time, when the
        /// authority is handed back, at a scene change), for AttitudeThrottleLeaseSeconds renewed
        /// at every write. False when the authority is no longer its own.
        /// </summary>
        internal static bool CommandThrottleForAttitude (string token, ModuleEngines engine,
            float percentage)
        {
            if (IsInactiveToken (token))
                return CommandThrottleInactive (token, engine, percentage);
            if (engine == null || !HoldsForAttitude (token))
                return false;
            if (float.IsNaN (percentage) || float.IsInfinity (percentage))
                return false;
            if (InactiveThrottleLease (engine) != null)
                return false;
            Command (engine, percentage, AttitudeThrottleLeaseSeconds);
            return true;
        }

        /// <summary>
        /// Lateral force: hands back an engine's independent throttle leased under the authority
        /// `token` (the active vessel's: ReleaseThrottle, as before; a non-active vessel's,
        /// in its own leases).
        /// </summary>
        internal static bool ReleaseThrottleForAttitude (string token, ModuleEngines engine)
        {
            if (IsInactiveToken (token)) {
                var auth = inactiveAuthorities [token];
                ThrottleLease lease;
                if (engine == null || !auth.Throttles.TryGetValue (engine, out lease))
                    return false;
                auth.Throttles.Remove (engine);
                RestoreThrottleFields (engine, lease);
                return true;
            }
            return ReleaseThrottle (engine);
        }

        internal const float AttitudeThrottleLeaseSeconds = 0.25f;

        // ------------------------------------ AttitudeV1's authority, NON-ACTIVE vessel
        //
        // In a double flight there are two boosters and one FlightGlobals.ActiveVessel. The v2
        // owner's authority (controller) and the legacy leases (throttleLeases, gimbalLeases)
        // belong to the ACTIVE vessel: the S7 and legacy-lease RPCs require the active vessel,
        // and ObserveTopology hands everything back when it changes. So the non-active booster
        // has no other possible owner than its own AttitudeV1 loop.
        //
        // On EXPLICIT request only (AttitudeArmV2, option 1), AttitudeV1 takes here an authority
        // PER non-active VESSEL: a token, its gimbal and throttle leases in ITS dictionaries,
        // never in the active vessel's. So RestoreAll, ClearController, OtherOwnerPresent and
        // TryHoldForAttitude neither see nor touch those leases: the active vessel's path is
        // unchanged. The leases follow the same rules as the others (QW5 ramp per physics step,
        // gimbals handed back without a jolt, original throttle fields restored, expiry in real
        // time); the authority is handed back at its expiry (AttitudeAuthoritySeconds without a
        // write), when the vessel becomes ACTIVE (the usual path takes over), is no longer loaded
        // or is packed, and at a scene change.
        //
        // KSP (Assembly-CSharp decompiled, assembly inspection): ModuleGimbal.FixedUpdate reads
        // base.vessel.ctrlState (its own vessel) and returns when gimbalActive is false;
        // ModuleEngines.UpdateThrottle reads independentThrottlePercentage, else
        // base.vessel.ctrlState.mainThrottle; Vessel.FeedInputFeed calls OnFlyByWire then
        // propagateControlUpdate. No test of the active vessel. Measured in flight (a flight of two boosters): the non-active booster's fly-by-wire chain is called at every
        // physics step (6767 calls) and the roll was written there 1875 times.

        sealed class InactiveAuthority
        {
            // Core v2: owner, token and wall guard of this vessel's authority.
            internal readonly Lease Lease = new Lease ();
            internal Vessel Vessel;
            internal Guid VesselId;
            internal readonly Dictionary<ModuleGimbal, GimbalLease> Gimbals =
                new Dictionary<ModuleGimbal, GimbalLease> ();
            internal readonly Dictionary<ModuleEngines, ThrottleLease> Throttles =
                new Dictionary<ModuleEngines, ThrottleLease> ();
        }

        static readonly Dictionary<string, InactiveAuthority> inactiveAuthorities =
            new Dictionary<string, InactiveAuthority> ();
        static readonly List<string> inactiveScratch = new List<string> ();
        static readonly List<ModuleGimbal> inactiveGimbalScratch = new List<ModuleGimbal> ();
        static readonly List<ModuleEngines> inactiveEngineScratch = new List<ModuleEngines> ();

        /// <summary>Count of AttitudeV1 authorities held on non-active vessels.</summary>
        internal static int InactiveAuthorityCount {
            get { return inactiveAuthorities.Count; }
        }

        internal const int InactiveStatusVersion = 1;
        internal const int InactiveStatusStride = 6;

        /// <summary>
        /// read-only: [version (1), count of authorities, row stride (6)], then per
        /// authority: the vessel's persistentId, gimbals leased, throttles leased, lease left (s
        /// of real time), the vessel is active now (0/1; 1 = handed back at the next step), held
        /// (HoldsForAttitude: 0/1).
        /// </summary>
        internal static IList<double> InactiveStatus ()
        {
            float now = Time.realtimeSinceStartup;
            var output = new List<double> (3 + inactiveAuthorities.Count * InactiveStatusStride) {
                InactiveStatusVersion,
                inactiveAuthorities.Count,
                InactiveStatusStride
            };
            foreach (var pair in inactiveAuthorities) {
                var auth = pair.Value;
                var v = auth.Vessel;
                output.Add (v != null ? (double)v.persistentId : double.NaN);
                output.Add (auth.Gimbals.Count);
                output.Add (auth.Throttles.Count);
                output.Add (auth.Lease.RemainingReal (now));
                output.Add (v != null && ReferenceEquals (v, FlightGlobals.ActiveVessel) ? 1.0 : 0.0);
                output.Add (HoldsInactive (pair.Key) ? 1.0 : 0.0);
            }
            return output;
        }

        static bool IsInactiveToken (string token)
        {
            return token != null && inactiveAuthorities.Count > 0 &&
                inactiveAuthorities.ContainsKey (token);
        }

        static bool HoldsInactive (string token)
        {
            InactiveAuthority auth;
            if (token == null || inactiveAuthorities.Count == 0 ||
                    !inactiveAuthorities.TryGetValue (token, out auth))
                return false;
            return auth.Lease.ValidReal (Time.realtimeSinceStartup) && InactiveUsable (auth.Vessel);
        }

        /// <summary>Loaded, not packed, and NOT the active vessel.</summary>
        static bool InactiveUsable (Vessel vessel)
        {
            return vessel != null && vessel.loaded && !vessel.packed &&
                !ReferenceEquals (vessel, FlightGlobals.ActiveVessel);
        }

        /// <summary>
        /// takes (or renews) AttitudeV1's authority on a loaded, unpacked, NON-ACTIVE vessel.
        /// Refused (false) for the active vessel (TryHoldForAttitude serves it, unchanged), when
        /// another authority already holds that vessel, when a lease of the active vessel still
        /// covers one of its modules, or when `token` is the active vessel's authority (the caller
        /// hands it back first). Renewed at every step that writes.
        /// </summary>
        internal static bool TryHoldForAttitudeInactive (string owner, Vessel vessel,
            ref string token)
        {
            if (!InactiveUsable (vessel))
                return false;
            if (token != null && token == controller.Token)
                return false;
            float now = Time.realtimeSinceStartup;
            InactiveAuthority auth;
            if (token != null && inactiveAuthorities.TryGetValue (token, out auth)) {
                if (ReferenceEquals (auth.Vessel, vessel) && auth.VesselId == vessel.id &&
                        auth.Lease.ValidReal (now)) {
                    auth.Lease.RenewReal (now, AttitudeAuthoritySeconds);
                    return true;
                }
                ReleaseInactive (token);
            }
            token = null;
            foreach (var pair in inactiveAuthorities)
                if (pair.Value.VesselId == vessel.id)
                    return false;
            if (MainLeaseOn (vessel))
                return false;
            auth = new InactiveAuthority {
                Vessel = vessel,
                VesselId = vessel.id
            };
            auth.Lease.Grant (owner, Guid.NewGuid ().ToString ("N"), now, AttitudeAuthoritySeconds);
            inactiveAuthorities.Add (auth.Lease.Token, auth);
            token = auth.Lease.Token;
            return true;
        }

        /// <summary>Does a lease of the active vessel (v2, legacy) cover a module of `vessel`?</summary>
        static bool MainLeaseOn (Vessel vessel)
        {
            foreach (var pair in gimbalLeases)
                if (pair.Key != null && ReferenceEquals (pair.Key.vessel, vessel))
                    return true;
            foreach (var pair in throttleLeases)
                if (pair.Key != null && ReferenceEquals (pair.Key.vessel, vessel))
                    return true;
            return false;
        }

        static bool CommandGimbalInactive (string token, ModuleGimbal gimbal, float xDegrees,
            float yDegrees, GimbalPivotCommand pivots = null)
        {
            if (!HoldsInactive (token))
                return false;
            var auth = inactiveAuthorities [token];
            if (gimbal == null || !ReferenceEquals (gimbal.vessel, auth.Vessel) ||
                    gimbalLeases.ContainsKey (gimbal))
                return false;
            var lease = CommandGimbalInto (auth.Gimbals, gimbal, xDegrees, yDegrees,
                AttitudeGimbalLeaseSeconds, pivots);
            lease.Bumpless = true;
            return true;
        }

        static bool CommandThrottleInactive (string token, ModuleEngines engine, float percentage)
        {
            if (engine == null || !HoldsInactive (token))
                return false;
            if (float.IsNaN (percentage) || float.IsInfinity (percentage))
                return false;
            var auth = inactiveAuthorities [token];
            if (!ReferenceEquals (engine.vessel, auth.Vessel) || throttleLeases.ContainsKey (engine))
                return false;
            CommandInto (auth.Throttles, engine, percentage, AttitudeThrottleLeaseSeconds);
            return true;
        }

        static GimbalLease InactiveGimbalLease (ModuleGimbal gimbal)
        {
            if (inactiveAuthorities.Count == 0 || gimbal == null)
                return null;
            GimbalLease lease;
            foreach (var pair in inactiveAuthorities)
                if (pair.Value.Gimbals.TryGetValue (gimbal, out lease))
                    return lease;
            return null;
        }

        static ThrottleLease InactiveThrottleLease (ModuleEngines engine)
        {
            if (inactiveAuthorities.Count == 0 || engine == null)
                return null;
            ThrottleLease lease;
            foreach (var pair in inactiveAuthorities)
                if (pair.Value.Throttles.TryGetValue (engine, out lease))
                    return lease;
            return null;
        }

        /// <summary>Hands back the leases of a non-active authority (gimbals without a jolt) and forgets it.</summary>
        static void ReleaseInactive (string token)
        {
            InactiveAuthority auth;
            if (token == null || !inactiveAuthorities.TryGetValue (token, out auth))
                return;
            inactiveAuthorities.Remove (token);
            foreach (var pair in auth.Throttles)
                RestoreThrottleFields (pair.Key, pair.Value);
            auth.Throttles.Clear ();
            foreach (var pair in auth.Gimbals)
                RestoreGimbalFields (pair.Key, pair.Value);
            auth.Gimbals.Clear ();
        }

        static void ReleaseAllInactive ()
        {
            if (inactiveAuthorities.Count == 0)
                return;
            var tokens = new List<string> (inactiveAuthorities.Keys);
            for (int i = 0; i < tokens.Count; i++)
                ReleaseInactive (tokens [i]);
        }

        /// <summary>
        /// Per physics step (Pump): ramp and expiry of the non-active leases like the others'; the
        /// whole authority is handed back at its expiry, or when its vessel becomes the active
        /// vessel, is no longer loaded or is packed.
        /// </summary>
        static void PumpInactive (Vessel active, float now)
        {
            if (inactiveAuthorities.Count == 0)
                return;
            inactiveScratch.Clear ();
            foreach (var pair in inactiveAuthorities) {
                var auth = pair.Value;
                var v = auth.Vessel;
                if (auth.Lease.ExpiredReal (now) || v == null || !v.loaded || v.packed ||
                        ReferenceEquals (v, active)) {
                    inactiveScratch.Add (pair.Key);
                    continue;
                }
                inactiveEngineScratch.Clear ();
                foreach (var t in auth.Throttles)
                    if (t.Key == null || t.Value.Term.ExpiredReal (now))
                        inactiveEngineScratch.Add (t.Key);
                for (int i = 0; i < inactiveEngineScratch.Count; i++) {
                    var engine = inactiveEngineScratch [i];
                    var lease = auth.Throttles [engine];
                    auth.Throttles.Remove (engine);
                    RestoreThrottleFields (engine, lease);
                }
                inactiveGimbalScratch.Clear ();
                foreach (var g in auth.Gimbals) {
                    if (!GimbalReady (g.Key) || g.Value.Term.ExpiredReal (now))
                        inactiveGimbalScratch.Add (g.Key);
                    else
                        ApplyGimbal (g.Key, g.Value);
                }
                for (int i = 0; i < inactiveGimbalScratch.Count; i++) {
                    var gimbal = inactiveGimbalScratch [i];
                    var lease = auth.Gimbals [gimbal];
                    auth.Gimbals.Remove (gimbal);
                    RestoreGimbalFields (gimbal, lease);
                }
            }
            for (int i = 0; i < inactiveScratch.Count; i++)
                ReleaseInactive (inactiveScratch [i]);
        }

        internal static int GimbalLeaseCount {
            get { return gimbalLeases.Count; }
        }

        internal static int RestoreAll ()
        {
            var engines = new List<ModuleEngines> (throttleLeases.Keys);
            for (int i = 0; i < engines.Count; i++)
                RestoreThrottle (engines [i]);
            var gimbals = new List<ModuleGimbal> (gimbalLeases.Keys);
            for (int i = 0; i < gimbals.Count; i++)
                RestoreGimbal (gimbals [i]);
            return engines.Count + gimbals.Count;
        }

        static void RestoreThrottle (ModuleEngines engine)
        {
            ThrottleLease lease;
            if (!throttleLeases.TryGetValue (engine, out lease))
                return;
            throttleLeases.Remove (engine);
            RestoreThrottleFields (engine, lease);
        }

        /// <summary>The original fields of a leased engine (lease already removed from its dictionary).</summary>
        static void RestoreThrottleFields (ModuleEngines engine, ThrottleLease lease)
        {
            if (engine == null)
                return;
            engine.independentThrottlePercentage = lease.OriginalPercentage;
            engine.independentThrottle = lease.OriginalEnabled;
        }

        /// <summary>
        /// QW5 (GNC plan v3): the lease ramp moves by ONE stock Lerp step per physics step, like
        /// ModuleGimbal.FixedUpdate (Lerp(old, new, gimbalResponseSpeed *
        /// TimeWarp.fixedDeltaTime)), whatever the game/wall ratio. The former ramp integrated
        /// Time.realtimeSinceStartup: at 0.6x real time, the leased gimbal moved ~1.7x faster per
        /// step than stock. Several calls in the same step (RPC + Pump) move it once.
        /// </summary>
        static void ApplyGimbal (ModuleGimbal gimbal, GimbalLease lease)
        {
            gimbal.gimbalActive = false;
            long physicsTick = pumpTicks.Value;
            if (gimbal.useGimbalResponseSpeed) {
                long elapsedTicks = Math.Max (0L, physicsTick - lease.LastResponseTick);
                float fixedStep = Math.Max (0.001f, TimeWarp.fixedDeltaTime);
                float perTick = Math.Min (
                    1f, Math.Max (0f, gimbal.gimbalResponseSpeed * fixedStep));
                float factor = elapsedTicks <= 0L ? 0f : (float)(1.0 - Math.Pow (
                    1.0 - perTick, elapsedTicks));
                lease.AppliedXDegrees +=
                    (lease.TargetXDegrees - lease.AppliedXDegrees) * factor;
                lease.AppliedYDegrees +=
                    (lease.TargetYDegrees - lease.AppliedYDegrees) * factor;
                lease.LastResponseTick = physicsTick;
            } else if (!gimbal.useGimbalResponseSpeed) {
                lease.AppliedXDegrees = lease.TargetXDegrees;
                lease.AppliedYDegrees = lease.TargetYDegrees;
                lease.LastResponseTick = physicsTick;
            }
            gimbal.actuationLocal = new Vector3 (
                lease.AppliedXDegrees, lease.AppliedYDegrees, 0f);
            if (lease.Pivots != null) {
                ApplyGimbalPivots (gimbal, lease);
                return;
            }
            for (int i = 0; i < gimbal.gimbalTransforms.Count; i++) {
                var secondAxis = gimbal.flipYZ ? Vector3.forward : Vector3.up;
                gimbal.gimbalTransforms [i].localRotation =
                    gimbal.initRots [i]
                    * Quaternion.AngleAxis (lease.AppliedXDegrees, gimbal.xMult * Vector3.right)
                    * Quaternion.AngleAxis (lease.AppliedYDegrees, gimbal.yMult * secondAxis);
            }
        }

        /// <summary>
        /// a MAPPED lease. The ramp (above) carries the module's deflection; each pivot turns
        /// by its projection, bounded to the module's box (GimbalPivotCommand.Pivot), with the
        /// composition of ModuleGimbal.FixedUpdate.
        /// </summary>
        static void ApplyGimbalPivots (ModuleGimbal gimbal, GimbalLease lease)
        {
            var secondAxis = gimbal.flipYZ ? Vector3.forward : Vector3.up;
            for (int i = 0; i < gimbal.gimbalTransforms.Count; i++) {
                double xi, yi;
                lease.Pivots.Pivot (i, lease.AppliedXDegrees, lease.AppliedYDegrees, out xi, out yi);
                gimbal.gimbalTransforms [i].localRotation =
                    gimbal.initRots [i]
                    * Quaternion.AngleAxis ((float)xi, gimbal.xMult * Vector3.right)
                    * Quaternion.AngleAxis ((float)yi, gimbal.yMult * secondAxis);
            }
        }

        static void RestoreGimbal (ModuleGimbal gimbal)
        {
            GimbalLease lease;
            if (!gimbalLeases.TryGetValue (gimbal, out lease))
                return;
            gimbalLeases.Remove (gimbal);
            RestoreGimbalFields (gimbal, lease);
        }

        /// <summary>Hands a leased gimbal back (lease already removed from its dictionary).</summary>
        static void RestoreGimbalFields (ModuleGimbal gimbal, GimbalLease lease)
        {
            if (gimbal == null)
                return;
            if (lease.Bumpless) {
                // M4: ModuleGimbal.FixedUpdate starts again from initRots, computes the stock
                // target (ctrlState) then does Lerp(oldActuationLocal[i], target). By setting
                // oldActuationLocal (public) and actuationLocal to the deflection the lease
                // REACHED, and leaving the nozzles where they are, the gimbal resumes its stock
                // ramp from where it is: no jump.
                var applied = new Vector3 (lease.AppliedXDegrees, lease.AppliedYDegrees, 0f);
                gimbal.actuationLocal = applied;
                if (gimbal.oldActuationLocal != null && lease.Pivots != null) {
                    // each pivot resumes its stock ramp from ITS reached rotation.
                    for (int i = 0; i < gimbal.oldActuationLocal.Length; i++) {
                        double xi, yi;
                        lease.Pivots.Pivot (i, applied.x, applied.y, out xi, out yi);
                        gimbal.oldActuationLocal [i] = new Vector3 ((float)xi, (float)yi, 0f);
                    }
                } else if (gimbal.oldActuationLocal != null)
                    for (int i = 0; i < gimbal.oldActuationLocal.Length; i++)
                        gimbal.oldActuationLocal [i] = applied;
                gimbal.gimbalActive = lease.OriginalActive;
                return;
            }
            if (gimbal.gimbalTransforms != null) {
                int count = Math.Min (
                    gimbal.gimbalTransforms.Count, lease.OriginalRotations.Count);
                for (int i = 0; i < count; i++)
                    if (gimbal.gimbalTransforms [i] != null)
                        gimbal.gimbalTransforms [i].localRotation = lease.OriginalRotations [i];
            }
            gimbal.actuationLocal = lease.OriginalActuationLocal;
            gimbal.gimbalActive = lease.OriginalActive;
        }

        static void RequireGimbalTransforms (ModuleGimbal gimbal)
        {
            if (!GimbalReady (gimbal))
                throw new InvalidOperationException (BridgeErrors.Tag (BridgeErrors.Transient,
                    "transforms de gimbal absents ou incoherents"));
        }

        static bool GimbalReady (ModuleGimbal gimbal)
        {
            return gimbal != null && gimbal.gimbalTransforms != null &&
                   gimbal.initRots != null && gimbal.gimbalTransforms.Count > 0 &&
                   gimbal.gimbalTransforms.Count == gimbal.initRots.Count;
        }

        static float ClampDirectional (float value, float negativeRange, float positiveRange)
        {
            if (float.IsNaN (value) || float.IsInfinity (value))
                throw new ArgumentOutOfRangeException (nameof (value));
            return Math.Min (
                Math.Max (value, -Math.Abs (negativeRange)), Math.Abs (positiveRange));
        }

        static float ClampLeaseSeconds (float leaseSeconds)
        {
            if (float.IsNaN (leaseSeconds) || float.IsInfinity (leaseSeconds))
                throw new ArgumentOutOfRangeException (nameof (leaseSeconds));
            return Math.Min (1f, Math.Max (0.05f, leaseSeconds));
        }
    }
}
