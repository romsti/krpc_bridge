using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using UnityEngine;

namespace KRPC.Bridge.Actuators
{
    /// <summary>
    /// Enregistre le service et porte les baux de poussee independante.
    ///
    /// Un bail restaure toujours les champs d'origine. Une perte du client Python ne
    /// peut donc pas laisser un moteur durablement decouple de la manette principale.
    /// Tous les acces ont lieu sur le thread Unity : les corps RPC de kRPC et ce
    /// FixedUpdate s'executent tous deux dans la boucle physique.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Instantly, true)]
    public sealed class ActuatorsAddon : MonoBehaviour
    {
        sealed class Lease
        {
            internal bool OriginalEnabled;
            internal float OriginalPercentage;
            internal float ExpiresAt;
        }

        static readonly Dictionary<ModuleEngines, Lease> leases =
            new Dictionary<ModuleEngines, Lease> ();

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
            if (leases.Count == 0)
                return;

            float now = Time.realtimeSinceStartup;
            var expired = new List<ModuleEngines> ();
            foreach (var pair in leases) {
                if (pair.Key == null || now >= pair.Value.ExpiresAt)
                    expired.Add (pair.Key);
            }
            for (int i = 0; i < expired.Count; i++)
                Restore (expired [i]);
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
            Lease lease;
            if (!leases.TryGetValue (engine, out lease)) {
                lease = new Lease {
                    OriginalEnabled = engine.independentThrottle,
                    OriginalPercentage = engine.independentThrottlePercentage
                };
                leases.Add (engine, lease);
            }

            engine.independentThrottlePercentage = Math.Min (100f, Math.Max (0f, percentage));
            engine.independentThrottle = true;
            lease.ExpiresAt = Time.realtimeSinceStartup
                              + Math.Min (1f, Math.Max (0.05f, leaseSeconds));
        }

        internal static bool Release (ModuleEngines engine)
        {
            if (!leases.ContainsKey (engine))
                return false;
            Restore (engine);
            return true;
        }

        internal static int RestoreAll ()
        {
            var engines = new List<ModuleEngines> (leases.Keys);
            for (int i = 0; i < engines.Count; i++)
                Restore (engines [i]);
            return engines.Count;
        }

        static void Restore (ModuleEngines engine)
        {
            Lease lease;
            if (!leases.TryGetValue (engine, out lease))
                return;
            leases.Remove (engine);
            if (engine == null)
                return;
            engine.independentThrottlePercentage = lease.OriginalPercentage;
            engine.independentThrottle = lease.OriginalEnabled;
        }
    }
}
