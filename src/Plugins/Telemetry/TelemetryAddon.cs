using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using KRPC.Bridge.Control;
using UnityEngine;

namespace KRPC.Bridge.Telemetry
{
    /// <summary>
    /// Registration of the Telemetry plugin (refactor R3).
    ///
    /// Registration ONLY, at load (Instantly): the service, its bit sets and layouts, and the
    /// observer the actuator pump calls (<see cref="Actuation.Observer"/>). Telemetry has NO
    /// FixedUpdate of its own: its captures run inside Control's pump (ControlWatcher), at the
    /// point of the physics step where they always ran, so no order between two MonoBehaviours
    /// is left to Unity.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Instantly, true)]
    public sealed class TelemetryAddon : MonoBehaviour
    {
        void Awake ()
        {
            ModRegistry.Register ("Telemetry", Resolve);
            TelemetryBits.Declare ();
            TelemetryLayouts.Declare ();
            // An instance (never a static method group): what the pump calls back.
            Actuation.Observer = new TelemetryObserver ();
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
    /// The telemetry as the actuator pump sees it (<see cref="IActuationObserver"/>): exactly the
    /// calls the single Actuators DLL made, in the same order.
    /// </summary>
    internal sealed class TelemetryObserver : IActuationObserver
    {
        public void Capture (Vessel vessel, long physicsTick, long topologyGeneration,
            int lastAcceptedSequence, int lastAppliedSequence, long lastAppliedTick,
            int lastResult, Guid controllerVesselId)
        {
            DynamicsV3.Capture (
                vessel, physicsTick, topologyGeneration,
                lastAcceptedSequence, lastAppliedSequence,
                lastAppliedTick, lastResult);

            // Vessels followed by persistentId (double flights). Returns at once when none is
            // tracked, so the active-vessel path above is untouched.
            DynamicsTracked.Pump (
                physicsTick, controllerVesselId, lastAcceptedSequence,
                lastAppliedSequence, lastAppliedTick, lastResult);

            // Refactor lot 3 (ContactV1): the contact latches of the watched vessels, after the
            // captures (they read nothing the frames publish, and write nothing). Caught here: the
            // pump calls this without a try, and AttitudeV1's watch runs after it.
            try {
                ContactWatch.Step (vessel);
            } catch (Exception exc) {
                if (contactFailures++ < 5)
                    BridgeLog.Error ("Telemetry", "contact watch: " + exc.Message);
            }
        }

        int contactFailures;

        public void ResetActiveVessel ()
        {
            DynamicsV3.Reset ();
        }

        public void ResetForScene ()
        {
            DynamicsV3.Reset ();
            DynamicsTracked.ResetForScene ();
            ResponseTracker.Clear ();
            ContactWatch.ResetForScene ();
        }

        public bool TracksResponses {
            get { return DynamicsExt.Enabled; }
        }

        public void BeforeApply (Guid vesselId, int sequence,
            List<Actuation.EngineFrameCommand> engines, List<Actuation.GimbalFrameCommand> gimbals)
        {
            ResponseTracker.BeforeApply (vesselId, sequence, engines, gimbals);
        }

        public void AfterApply (long tick)
        {
            ResponseTracker.AfterApply (tick);
        }

        public void ClearResponses ()
        {
            ResponseTracker.Clear ();
        }
    }
}
