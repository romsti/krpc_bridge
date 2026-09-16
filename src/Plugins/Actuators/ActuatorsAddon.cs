using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using UnityEngine;

namespace KRPC.Bridge.Actuators
{
    /// <summary>
    /// Enregistre le service et porte les baux de poussee et de gimbal independants.
    ///
    /// Un bail restaure toujours les champs d'origine. Une perte du client Python ne
    /// peut donc pas laisser un moteur durablement decouple de la manette principale.
    /// Tous les acces ont lieu sur le thread Unity : les corps RPC de kRPC et ce
    /// FixedUpdate s'executent tous deux dans la boucle physique.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Instantly, true)]
    public sealed class ActuatorsAddon : MonoBehaviour
    {
        internal const int ProtocolVersion = 2;
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
            internal float ExpiresAt;
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
            internal float LastResponseAt;
            internal float ExpiresAt;
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

        static long physicsTick;
        static long topologyGeneration;
        static Guid observedVesselId = Guid.Empty;
        static ulong observedTopologyHash;
        static IList<double> lastSnapshot = new List<double> ();

        static string controllerOwner;
        static string controllerToken;
        static Guid controllerVesselId = Guid.Empty;
        static long controllerTopologyGeneration;
        static float controllerExpiresAt;
        static PendingControlFrame pendingFrame;
        static int lastAcceptedSequence;
        static int lastAppliedSequence;
        static int lastResult;
        static long lastAppliedTick;

        void Awake ()
        {
            // Enregistrement SEUL (Instantly = avant que Core ne resolve). Le
            // travail par-tick (application des trames, rampe/expiration des
            // baux, watchdog TTL, ObserveTopology) vit dans ActuatorsWatcher,
            // un addon Flight : l'objet Instantly est DETRUIT a l'entree de la
            // scene de vol (BridgeCore le documente : sans DontDestroyOnLoad un
            // KSPAddon meurt a chaque changement de scene), donc son
            // FixedUpdate ne peut pas porter la boucle physique. Meme decoupage
            // que FMRS (FmrsWatcher) et OCISLY (AutoRestoreAddon).
            ModRegistry.Register ("Actuators", Resolve);
        }

        static PluginStatus Resolve ()
        {
            var assembly = ModRegistry.FindAssembly ("Assembly-CSharp");
            return new PluginStatus {
                Available = true,
                ModVersion = assembly == null ? "" : ModRegistry.VersionOf (assembly),
                Report = "API KSP stock resolue"
            };
        }

        /// <summary>
        /// Un pas de la boucle physique, appele par ActuatorsWatcher a chaque
        /// FixedUpdate de la scene Flight. Tout l'etat est statique, donc il
        /// survit a la creation/destruction du watcher au fil des scenes.
        /// </summary>
        internal static void Pump ()
        {
            physicsTick++;
            var vessel = FlightGlobals.ActiveVessel;
            ObserveTopology (vessel);
            float now = Time.realtimeSinceStartup;
            if (controllerToken != null && now >= controllerExpiresAt)
                ClearController (ResultAuthorityLost, true);
            ApplyPendingFrame (vessel);

            var expiredEngines = new List<ModuleEngines> ();
            foreach (var pair in throttleLeases) {
                if (pair.Key == null || now >= pair.Value.ExpiresAt)
                    expiredEngines.Add (pair.Key);
            }
            for (int i = 0; i < expiredEngines.Count; i++)
                RestoreThrottle (expiredEngines [i]);

            var expiredGimbals = new List<ModuleGimbal> ();
            foreach (var pair in gimbalLeases) {
                if (!GimbalReady (pair.Key) || now >= pair.Value.ExpiresAt)
                    expiredGimbals.Add (pair.Key);
                else
                    ApplyGimbal (pair.Key, pair.Value, now);
            }
            for (int i = 0; i < expiredGimbals.Count; i++)
                RestoreGimbal (expiredGimbals [i]);

            DynamicsV3.Capture (
                vessel, physicsTick, topologyGeneration,
                lastAcceptedSequence, lastAppliedSequence,
                lastAppliedTick, lastResult);
        }

        internal static void RequireLegacyControlAvailable ()
        {
            if (controllerToken != null && Time.realtimeSinceStartup >= controllerExpiresAt)
                ClearController (ResultAuthorityLost, true);
            if (controllerToken != null)
                throw new InvalidOperationException (
                    "controle v2 deja acquis par " + controllerOwner);
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
            if (controllerToken != null && Time.realtimeSinceStartup >= controllerExpiresAt)
                ClearController (ResultAuthorityLost, true);
            if (controllerToken != null)
                throw new InvalidOperationException (
                    "controle v2 deja acquis par " + controllerOwner);

            // A legacy lease has no owner identity. Acquiring the v2 authority is an
            // explicit handover, so restore it before installing the new owner.
            RestoreAll ();
            controllerOwner = owner.Trim ();
            controllerToken = Guid.NewGuid ().ToString ("N");
            controllerVesselId = vessel.id;
            controllerTopologyGeneration = topologyGeneration;
            controllerExpiresAt = Time.realtimeSinceStartup + leaseSeconds;
            pendingFrame = null;
            lastAcceptedSequence = 0;
            lastAppliedSequence = 0;
            lastAppliedTick = 0;
            lastResult = ResultNone;
            return controllerToken;
        }

        internal static bool RenewControl (string token, float leaseSeconds)
        {
            ValidateControllerLease (leaseSeconds);
            RequireController (token);
            controllerExpiresAt = Time.realtimeSinceStartup + leaseSeconds;
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
                throw new InvalidOperationException (
                    "topologie du vaisseau modifiee : reacquerir le controle");
            }
            if (sequence <= 0 || sequence <= lastAcceptedSequence)
                throw new ArgumentOutOfRangeException (
                    nameof (sequence), "sequence doit etre strictement croissante");
            ValidateActuatorLease (leaseSeconds);

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
            float remaining = controllerToken == null
                ? 0f
                : Math.Max (0f, controllerExpiresAt - Time.realtimeSinceStartup);
            return new List<double> {
                ProtocolVersion,
                physicsTick,
                topologyGeneration,
                controllerToken == null ? 0.0 : 1.0,
                remaining,
                pendingFrame == null ? 0.0 : pendingFrame.Sequence,
                pendingFrame == null ? 0.0 : pendingFrame.ApplyTick,
                lastAcceptedSequence,
                lastAppliedSequence,
                lastResult,
                lastAppliedTick
            };
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
            return throttleLeases.ContainsKey (engine);
        }

        internal static bool GimbalIsLeased (ModuleGimbal gimbal)
        {
            return gimbalLeases.ContainsKey (gimbal);
        }

        internal static void GimbalLeaseTarget (
            ModuleGimbal gimbal, out float xDegrees, out float yDegrees)
        {
            GimbalLease lease;
            if (gimbalLeases.TryGetValue (gimbal, out lease)) {
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
                throw new InvalidOperationException ("moteur detruit");
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
            if (pendingFrame == null || physicsTick < pendingFrame.ApplyTick)
                return;
            var frame = pendingFrame;
            pendingFrame = null;
            if (controllerToken == null || frame.Token != controllerToken) {
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
            } catch (Exception exc) {
                RestoreAll ();
                lastResult = ResultApplyFailed;
                BridgeLog.Error (
                    "Actuators v2 frame " + frame.Sequence + " refusee: " + exc);
            }
        }

        static void ObserveTopology (Vessel vessel)
        {
            if (vessel == null || !vessel.loaded) {
                if (observedVesselId != Guid.Empty || controllerToken != null)
                    ClearController (ResultTopologyChanged, true);
                observedVesselId = Guid.Empty;
                observedTopologyHash = 0;
                lastSnapshot = new List<double> ();
                DynamicsV3.Reset ();
                return;
            }
            ulong hash = ComputeTopologyHash (vessel);
            if (observedVesselId == vessel.id && observedTopologyHash == hash)
                return;
            bool replacingObservedVessel = observedVesselId != Guid.Empty;
            topologyGeneration++;
            if (replacingObservedVessel || controllerToken != null)
                ClearController (ResultTopologyChanged, true);
            observedVesselId = vessel.id;
            observedTopologyHash = hash;
            lastSnapshot = new List<double> ();
            DynamicsV3.Reset ();
        }

        static ulong ComputeTopologyHash (Vessel vessel)
        {
            unchecked {
                ulong hash = 1469598103934665603UL;
                MixHash (ref hash, vessel.persistentId);
                MixHash (ref hash, (uint)vessel.parts.Count);
                for (int p = 0; p < vessel.parts.Count; p++) {
                    var part = vessel.parts [p];
                    if (part == null) {
                        MixHash (ref hash, UInt32.MaxValue);
                        continue;
                    }
                    MixHash (ref hash, part.flightID);
                    MixHash (ref hash, (uint)part.Modules.Count);
                    for (int m = 0; m < part.Modules.Count; m++) {
                        var module = part.Modules [m];
                        if (module == null) {
                            MixHash (ref hash, UInt32.MaxValue);
                            continue;
                        }
                        var engine = module as ModuleEngines;
                        if (engine != null)
                            MixHash (ref hash, (uint)(engine.thrustTransforms == null
                                ? 0 : engine.thrustTransforms.Count));
                        var gimbal = module as ModuleGimbal;
                        if (gimbal != null)
                            MixHash (ref hash, (uint)(gimbal.gimbalTransforms == null
                                ? 0 : gimbal.gimbalTransforms.Count));
                    }
                }
                return hash;
            }
        }

        static void MixHash (ref ulong hash, uint value)
        {
            unchecked {
                hash ^= value;
                hash *= 1099511628211UL;
            }
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
                physicsTick,
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
                throw new InvalidOperationException ("aucun vaisseau actif et charge");
            return vessel;
        }

        static void RequireController (string token)
        {
            if (controllerToken != null && Time.realtimeSinceStartup >= controllerExpiresAt)
                ClearController (ResultAuthorityLost, true);
            if (string.IsNullOrEmpty (token) || token != controllerToken)
                throw new InvalidOperationException ("token de controle absent, expire ou invalide");
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

        static void ClearController (int result, bool restore)
        {
            if (restore)
                RestoreAll ();
            controllerOwner = null;
            controllerToken = null;
            controllerVesselId = Guid.Empty;
            controllerTopologyGeneration = 0;
            controllerExpiresAt = 0f;
            pendingFrame = null;
            lastResult = result;
        }

        internal static void ResetForScene ()
        {
            ClearController (ResultTopologyChanged, true);
            observedVesselId = Guid.Empty;
            observedTopologyHash = 0;
            lastSnapshot = new List<double> ();
            DynamicsV3.Reset ();
        }

        internal static void Command (ModuleEngines engine, float percentage, float leaseSeconds)
        {
            ThrottleLease lease;
            if (!throttleLeases.TryGetValue (engine, out lease)) {
                lease = new ThrottleLease {
                    OriginalEnabled = engine.independentThrottle,
                    OriginalPercentage = engine.independentThrottlePercentage
                };
                throttleLeases.Add (engine, lease);
            }

            engine.independentThrottlePercentage = Math.Min (100f, Math.Max (0f, percentage));
            engine.independentThrottle = true;
            lease.ExpiresAt = Time.realtimeSinceStartup
                              + Math.Min (1f, Math.Max (0.05f, leaseSeconds));
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
            RequireGimbalTransforms (gimbal);
            if (gimbal.gimbalLock)
                throw new InvalidOperationException (
                    "gimbal verrouille : le deverrouiller avant de demander un bail");
            GimbalLease lease;
            if (!gimbalLeases.TryGetValue (gimbal, out lease)) {
                var rotations = new List<Quaternion> (gimbal.gimbalTransforms.Count);
                for (int i = 0; i < gimbal.gimbalTransforms.Count; i++)
                    rotations.Add (gimbal.gimbalTransforms [i].localRotation);
                lease = new GimbalLease {
                    OriginalActive = gimbal.gimbalActive,
                    OriginalActuationLocal = gimbal.actuationLocal,
                    AppliedXDegrees = gimbal.actuationLocal.x,
                    AppliedYDegrees = gimbal.actuationLocal.y,
                    LastResponseAt = Time.realtimeSinceStartup,
                    OriginalRotations = rotations
                };
                gimbalLeases.Add (gimbal, lease);
            }

            float now = Time.realtimeSinceStartup;
            ApplyGimbal (gimbal, lease, now);

            float limiter = Math.Min (100f, Math.Max (0f, gimbal.gimbalLimiter)) * 0.01f;
            lease.TargetXDegrees = ClampDirectional (
                xDegrees, gimbal.gimbalRangeXN * limiter, gimbal.gimbalRangeXP * limiter);
            lease.TargetYDegrees = ClampDirectional (
                yDegrees, gimbal.gimbalRangeYN * limiter, gimbal.gimbalRangeYP * limiter);
            if (!gimbal.useGimbalResponseSpeed) {
                lease.AppliedXDegrees = lease.TargetXDegrees;
                lease.AppliedYDegrees = lease.TargetYDegrees;
            }
            lease.ExpiresAt = now + ClampLeaseSeconds (leaseSeconds);
            gimbal.gimbalActive = false;
            ApplyGimbal (gimbal, lease, now);
        }

        internal static bool ReleaseGimbal (ModuleGimbal gimbal)
        {
            if (!gimbalLeases.ContainsKey (gimbal))
                return false;
            RestoreGimbal (gimbal);
            return true;
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
            if (engine == null)
                return;
            engine.independentThrottlePercentage = lease.OriginalPercentage;
            engine.independentThrottle = lease.OriginalEnabled;
        }

        static void ApplyGimbal (ModuleGimbal gimbal, GimbalLease lease, float now)
        {
            gimbal.gimbalActive = false;
            if (gimbal.useGimbalResponseSpeed) {
                float elapsed = Math.Max (0f, now - lease.LastResponseAt);
                float fixedStep = Math.Max (0.001f, TimeWarp.fixedDeltaTime);
                float perTick = Math.Min (
                    1f, Math.Max (0f, gimbal.gimbalResponseSpeed * fixedStep));
                float factor = elapsed <= 0f ? 0f : (float)(1.0 - Math.Pow (
                    1.0 - perTick, elapsed / fixedStep));
                lease.AppliedXDegrees +=
                    (lease.TargetXDegrees - lease.AppliedXDegrees) * factor;
                lease.AppliedYDegrees +=
                    (lease.TargetYDegrees - lease.AppliedYDegrees) * factor;
                lease.LastResponseAt = now;
            } else if (!gimbal.useGimbalResponseSpeed) {
                lease.AppliedXDegrees = lease.TargetXDegrees;
                lease.AppliedYDegrees = lease.TargetYDegrees;
                lease.LastResponseAt = now;
            }
            gimbal.actuationLocal = new Vector3 (
                lease.AppliedXDegrees, lease.AppliedYDegrees, 0f);
            for (int i = 0; i < gimbal.gimbalTransforms.Count; i++) {
                var secondAxis = gimbal.flipYZ ? Vector3.forward : Vector3.up;
                gimbal.gimbalTransforms [i].localRotation =
                    gimbal.initRots [i]
                    * Quaternion.AngleAxis (lease.AppliedXDegrees, gimbal.xMult * Vector3.right)
                    * Quaternion.AngleAxis (lease.AppliedYDegrees, gimbal.yMult * secondAxis);
            }
        }

        static void RestoreGimbal (ModuleGimbal gimbal)
        {
            GimbalLease lease;
            if (!gimbalLeases.TryGetValue (gimbal, out lease))
                return;
            gimbalLeases.Remove (gimbal);
            if (gimbal == null)
                return;
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
                throw new InvalidOperationException (
                    "transforms de gimbal absents ou incoherents");
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

    /// <summary>
    /// Pilote physique par-vol des actionneurs v2.
    ///
    /// Cree a CHAQUE entree de scene Flight et detruit a la sortie ; son
    /// FixedUpdate porte toute la boucle (application des trames atomiques,
    /// rampe/expiration des baux, watchdog TTL, ObserveTopology). L'etat vit en
    /// STATIQUE dans ActuatorsAddon, donc il survit a la destruction de ce
    /// MonoBehaviour au reload de scene — meme patron que FmrsWatcher et
    /// AutoRestoreAddon. Sans ce compagnon la boucle etait portee par l'addon
    /// [Instantly], qui meurt a l'entree du vol : le service repondait aux
    /// lectures mais n'appliquait plus rien (physicsTick fige).
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Flight, false)]
    public sealed class ActuatorsWatcher : MonoBehaviour
    {
        void FixedUpdate ()
        {
            ActuatorsAddon.Pump ();
        }

        void OnDestroy ()
        {
            // Sortie de vol : relacher l'autorite et restaurer les champs stock
            // captures, comme l'ancien OnSceneLoadRequested le faisait.
            ActuatorsAddon.ResetForScene ();
        }
    }
}
