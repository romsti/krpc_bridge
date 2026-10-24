using System;
using KRPC.Bridge.Core;
using UnityEngine;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// Registration of the Control plugin (refactor R3: the KSP side of the former ActuatorsAddon).
    ///
    /// Registration ONLY (Instantly = before the Core resolves). The per-tick work (frames
    /// applied, lease ramps and expiry, TTL watchdog, ObserveTopology) lives in
    /// <see cref="ControlWatcher"/>, a Flight addon: the Instantly object is DESTROYED when the
    /// flight scene loads (BridgeCore documents it: without DontDestroyOnLoad a KSPAddon dies at
    /// every scene change), so its FixedUpdate could not carry the physics loop. Same split as
    /// FMRS (FmrsWatcher) and OCISLY (AutoRestoreAddon).
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Instantly, true)]
    public sealed class ControlAddon : MonoBehaviour
    {
        void Awake ()
        {
            ModRegistry.Register ("Control", Resolve);
            // Core v2: the masks of the Control layouts, declared into the bit registry.
            ControlBits.Declare ();
            // Refactor R1: the schema descriptors of every frame and return (layout_v1).
            ControlLayouts.Declare ();
        }

        static PluginStatus Resolve ()
        {
            var assembly = ModRegistry.FindAssembly ("Assembly-CSharp");
            return new PluginStatus {
                Available = true,
                ModVersion = assembly == null ? "" : ModRegistry.VersionOf (assembly),
                Report = "stock KSP API resolved"
            };
        }
    }

    /// <summary>
    /// The physics driver of the actuators, per flight.
    ///
    /// Created at EVERY entry into the Flight scene and destroyed at its exit; its FixedUpdate
    /// carries the whole loop (atomic frames applied, lease ramps and expiry, TTL watchdog,
    /// ObserveTopology, then what observes, through Actuation.Observer). The state lives,
    /// STATIC, in Actuation, so it survives the destruction of this MonoBehaviour at a scene
    /// reload - the pattern of FmrsWatcher and AutoRestoreAddon. Without this companion the
    /// loop was carried by the Instantly addon, which dies when the flight starts: the service
    /// answered reads but applied nothing (the pump tick frozen).
    ///
    /// It is the ONLY Flight addon of Control and Telemetry with a FixedUpdate: the order of a
    /// physics step (leases, frames, captures, AttitudeV1's witness) is written in one method,
    /// Actuation.Pump, never left to Unity's order between two MonoBehaviours.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Flight, false)]
    public sealed class ControlWatcher : MonoBehaviour
    {
        void FixedUpdate ()
        {
            Actuation.Pump ();
        }

        void OnDestroy ()
        {
            // Leaving the flight: release the authority and restore the stock fields captured,
            // as the former OnSceneLoadRequested did.
            Actuation.ResetForScene ();
        }
    }
}
