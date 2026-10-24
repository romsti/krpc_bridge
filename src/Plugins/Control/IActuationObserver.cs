using System;
using System.Collections.Generic;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// What observes the actuators, seen from what flies (refactor R3).
    ///
    /// KRPC.Bridge.Telemetry registers its implementation into <see cref="Actuation.Observer"/>
    /// at load. Control calls it at exactly the points where the single Actuators DLL called
    /// DynamicsV3, DynamicsTracked and ResponseTracker, so a physics step runs in the order it
    /// always did; Control itself references nothing of the telemetry. Main thread only, like
    /// every caller.
    /// </summary>
    internal interface IActuationObserver
    {
        /// <summary>
        /// The capture of one physics step, after the leases and the frame of that step: the
        /// active vessel's frame (null vessel: nothing loaded), then the vessels followed by
        /// persistentId. The v2 controller's sequence numbers and vessel go into the frames.
        /// </summary>
        void Capture (Vessel vessel, long physicsTick, long topologyGeneration,
            int lastAcceptedSequence, int lastAppliedSequence, long lastAppliedTick,
            int lastResult, Guid controllerVesselId);

        /// <summary>The observed active vessel changed (another vessel, or its topology): its stream restarts.</summary>
        void ResetActiveVessel ();

        /// <summary>Leaving the flight scene: every stream and every tracked vessel is gone.</summary>
        void ResetForScene ();

        /// <summary>True when the response of each commanded actuator is followed (DynamicsExtV1 armed).</summary>
        bool TracksResponses { get; }

        /// <summary>Baselines of the actuators a v2 frame is about to command.</summary>
        void BeforeApply (Guid vesselId, int sequence, List<Actuation.EngineFrameCommand> engines,
            List<Actuation.GimbalFrameCommand> gimbals);

        /// <summary>The frame was applied at that pump tick.</summary>
        void AfterApply (long tick);

        /// <summary>The frame failed: forget the baselines.</summary>
        void ClearResponses ();
    }
}
