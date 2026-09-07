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

        static readonly Dictionary<ModuleEngines, ThrottleLease> throttleLeases =
            new Dictionary<ModuleEngines, ThrottleLease> ();
        static readonly Dictionary<ModuleGimbal, GimbalLease> gimbalLeases =
            new Dictionary<ModuleGimbal, GimbalLease> ();

        void Awake ()
        {
            ModRegistry.Register ("Actuators", Resolve);
            GameEvents.onGameSceneLoadRequested.Add (OnSceneLoadRequested);
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

        void FixedUpdate ()
        {
            if (throttleLeases.Count == 0 && gimbalLeases.Count == 0)
                return;

            float now = Time.realtimeSinceStartup;
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
        }

        void OnSceneLoadRequested (GameScenes scene)
        {
            RestoreAll ();
        }

        void OnDestroy ()
        {
            GameEvents.onGameSceneLoadRequested.Remove (OnSceneLoadRequested);
            RestoreAll ();
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
}
