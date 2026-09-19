using System;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Service.Attributes;
using SCFlight = KRPC.SpaceCenter.Services.Flight;
using SCVessel = KRPC.SpaceCenter.Services.Vessel;
using UnityEngine;

namespace KRPC.Bridge.Actuators
{
    /// <summary>
    /// PDG2 dynamic-state transport.
    ///
    /// The payload deliberately stays an IList&lt;double&gt;: that is a kRPC-native type,
    /// cheap to transport and immune to one malformed custom class signature disabling
    /// the whole server.  See docs/API.md for the exact flat schema.
    /// </summary>
    public static partial class ActuatorsService
    {
        /// <summary>Protocol version of DynamicsSnapshotV3.</summary>
        [KRPCProperty]
        public static int DynamicsProtocolVersion {
            get { return DynamicsV3.ProtocolVersion; }
        }

        /// <summary>
        /// Latest atomic dynamic/actuator frame captured by the Flight FixedUpdate
        /// watcher. Empty until the first physics tick in flight.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> DynamicsSnapshotV3 ()
        {
            return DynamicsV3.ReadLatest ();
        }

        /// <summary>
        /// FixedUpdate history newer than sinceTick. The return header is followed by
        /// length-prefixed DynamicsSnapshotV3 frames. maxFrames is clamped to 1..128.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> DynamicsFramesV3 (int sinceTick = 0, int maxFrames = 64)
        {
            return DynamicsV3.ReadHistory (sinceTick, maxFrames);
        }

        /// <summary>
        /// Compact status: protocol, schema, latest tick, history count/capacity,
        /// capability mask, latest applied sequence/tick and actuator result.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> DynamicsStatusV3 ()
        {
            return DynamicsV3.ReadStatus ();
        }
    }

    /// <summary>
    /// Per-vessel capture state: the frame ring, the latest frame and the previous-tick
    /// values used for finite differences.
    ///
    /// One instance follows the active vessel (the historical DynamicsSnapshotV3 path, whose
    /// frames are unchanged); DynamicsTracked keeps one more instance per vessel a client
    /// asked to follow by persistentId. Keeping the previous-tick state per instance is what
    /// stops two vessels from differentiating each other's velocities.
    /// </summary>
    internal sealed class DynamicsChannel
    {
        internal readonly LinkedList<double[]> history = new LinkedList<double[]> ();
        internal double[] latest = new double[0];
        internal long latestCapabilities;
        internal long latestTick;
        internal int latestAppliedSequence;
        internal long latestAppliedTick;
        internal int latestResult;

        // kRPC SpaceCenter wrapper used only to read the official 0.6.x Flight
        // live-aerodynamics API in the same FixedUpdate as the native state.
        // Reused across ticks to avoid a pair of wrapper allocations at 50 Hz.
        internal Vessel serviceInternalVessel;
        internal SCVessel serviceVessel;
        internal SCFlight serviceFlightBody;

        internal bool havePrevious;
        internal long previousTick;
        internal readonly double[] previousSurfaceVelocity = new double[3];
        internal readonly double[] previousOrbitalVelocity = new double[3];
        internal readonly double[] previousAngularVelocity = new double[3];

        // Supplemental same-tick frames (DynamicsExtV1), captured only while enabled.
        internal readonly DynamicsExtState ext = new DynamicsExtState ();

        internal void Reset ()
        {
            history.Clear ();
            latest = new double[0];
            latestCapabilities = 0;
            latestTick = 0;
            latestAppliedSequence = 0;
            latestAppliedTick = 0;
            latestResult = 0;
            serviceInternalVessel = null;
            serviceVessel = null;
            serviceFlightBody = null;
            havePrevious = false;
            previousTick = 0;
            ext.Reset ();
        }

        internal IList<double> ReadLatest ()
        {
            return new List<double> (latest);
        }

        internal IList<double> ReadStatus ()
        {
            return new List<double> {
                DynamicsV3.ProtocolVersion,
                DynamicsV3.SchemaVersion,
                latestTick,
                history.Count,
                DynamicsV3.HistoryCapacity,
                latestCapabilities,
                latestAppliedSequence,
                latestAppliedTick,
                latestResult
            };
        }

        internal IList<double> ReadHistory (int sinceTick, int maxFrames)
        {
            return DynamicsV3.ReadRing (history, DynamicsV3.ProtocolVersion,
                DynamicsV3.SchemaVersion, sinceTick, maxFrames);
        }
    }

    /// <summary>
    /// Captures one coherent pre-integration FixedUpdate state and keeps a short ring.
    ///
    /// ActuatorsAddon.Pump applies a queued command first, updates leased gimbals, then
    /// calls Capture. Therefore frame tick k means: state sampled in FixedUpdate k after
    /// command sequence metadata for k has been applied, but before the next observed
    /// physics response. Comparing k, k+1, ... gives the real command-to-response delay.
    /// </summary>
    internal static class DynamicsV3
    {
        internal const int ProtocolVersion = 3;
        internal const int SchemaVersion = 2;
        internal const int HeaderStride = 22;
        // Schema 3.2 appends canonical SI realized/aero/residual quantities to the
        // untouched 67-value 3.1 state prefix. Old logs therefore remain decodable.
        internal const int StateStrideV31 = 67;
        internal const int StateStride = 116;
        internal const int EngineStride = 20;
        internal const int GimbalStride = 19;
        internal const int TransformStride = 10;
        internal const int HistoryCapacity = 1000; // ~20 s at 50 Hz; survives long GNC stalls during instrumentation

        // State offsets read back by DynamicsExtV1 (same tick, same array).
        internal const int StateAngularVelocity = 19;   // BODY frame, see docs/API.md
        internal const int StateAngularAcceleration = 22;
        internal const int StateCoM = 26;
        internal const int StateMoi = 29;               // tonne.m^2 (KSP native)
        internal const int StateEngineTorqueBody = 73;
        internal const int StateAeroTorqueBody = 85;

        // Capability bits.  Missing/unknown quantities remain NaN, never guessed.
        const long CapPosition = 1L << 0;
        const long CapOrbitalVelocity = 1L << 1;
        const long CapSurfaceVelocity = 1L << 2;
        const long CapSurfaceAcceleration = 1L << 3;
        const long CapOrbitalAcceleration = 1L << 4;
        const long CapRotation = 1L << 5;
        const long CapAngularVelocity = 1L << 6;
        const long CapAngularAcceleration = 1L << 7;
        const long CapMass = 1L << 8;
        const long CapCoM = 1L << 9;
        const long CapMoi = 1L << 10;
        const long CapGravity = 1L << 11;
        const long CapAtmosphere = 1L << 12;
        const long CapAeroForce = 1L << 13;
        const long CapAeroTorque = 1L << 14;
        const long CapDragVector = 1L << 15;
        const long CapLiftVector = 1L << 16;
        const long CapBodySurfaceVelocity = 1L << 17;
        const long CapAoA = 1L << 18;
        const long CapSideslip = 1L << 19;
        const long CapGlobalThrottle = 1L << 20;
        const long CapActuators = 1L << 21;
        const long CapRealizedEngineWrench = 1L << 22;
        const long CapKrpcLiveAeroForce = 1L << 23;
        const long CapKrpcLiveAeroTorque = 1L << 24;
        const long CapKrpcLiveAeroComponents = 1L << 25;
        const long CapKrpcAeroAngles = 1L << 26;
        const long CapKrpcAeroThermo = 1L << 27;
        const long CapExternalForceResidual = 1L << 28;
        const long CapUnexplainedNonAeroForce = 1L << 29;
        const long CapRealizedEngineForceOnVessel = 1L << 30;

        static readonly Dictionary<string, MemberInfo> memberCache =
            new Dictionary<string, MemberInfo> (StringComparer.Ordinal);
        static readonly Dictionary<string, MethodInfo> methodCache =
            new Dictionary<string, MethodInfo> (StringComparer.Ordinal);

        // The active-vessel channel: the historical DynamicsSnapshotV3 stream.
        static readonly DynamicsChannel active = new DynamicsChannel ();

        internal static DynamicsChannel Active {
            get { return active; }
        }

        internal static void Reset ()
        {
            active.Reset ();
            AeroActuatorV1.Reset ();
        }

        internal static IList<double> ReadLatest ()
        {
            return active.ReadLatest ();
        }

        internal static IList<double> ReadStatus ()
        {
            return active.ReadStatus ();
        }

        internal static IList<double> ReadHistory (int sinceTick, int maxFrames)
        {
            return active.ReadHistory (sinceTick, maxFrames);
        }

        /// <summary>
        /// Shared ring reader: the six-value history header, then length-prefixed frames
        /// newer than sinceTick. The tick is always at offset 2 of a frame.
        /// </summary>
        internal static IList<double> ReadRing (
            LinkedList<double[]> history, int protocol, int schema,
            int sinceTick, int maxFrames)
        {
            int limit = Math.Min (128, Math.Max (1, maxFrames));
            long oldestTick = history.First == null ? 0 : FrameTick (history.First.Value);
            long newestTick = history.Last == null ? 0 : FrameTick (history.Last.Value);
            bool droppedBefore = sinceTick > 0 && oldestTick > 0 && sinceTick < oldestTick - 1;

            var selected = new List<double[]> (limit);
            for (var node = history.First; node != null && selected.Count < limit;
                    node = node.Next) {
                if (FrameTick (node.Value) > sinceTick)
                    selected.Add (node.Value);
            }

            // history header: protocol, schema, frame_count, oldest_tick, newest_tick,
            // dropped_before. Every frame is length-prefixed afterwards.
            var output = new List<double> (6 + selected.Count * 128) {
                protocol,
                schema,
                selected.Count,
                oldestTick,
                newestTick,
                droppedBefore ? 1.0 : 0.0
            };
            for (int i = 0; i < selected.Count; i++) {
                output.Add (selected [i].Length);
                output.AddRange (selected [i]);
            }
            return output;
        }

        static long FrameTick (double[] frame)
        {
            if (frame == null || frame.Length < 3)
                return 0;
            return (long)frame [2];
        }

        internal static void Capture (
            Vessel vessel, long physicsTick, long topologyGeneration,
            int lastAcceptedSequence, int lastAppliedSequence,
            long lastAppliedTick, int lastResult)
        {
            if (vessel == null || !vessel.loaded) {
                Reset ();
                return;
            }
            CaptureInto (active, vessel, physicsTick, topologyGeneration,
                lastAcceptedSequence, lastAppliedSequence, lastAppliedTick, lastResult,
                true);
        }

        /// <summary>
        /// One capture into one channel. The active channel also drives AeroActuatorV1;
        /// a tracked channel does not (that transport stays active-vessel only).
        /// </summary>
        internal static void CaptureInto (
            DynamicsChannel ch, Vessel vessel, long physicsTick, long topologyGeneration,
            int lastAcceptedSequence, int lastAppliedSequence,
            long lastAppliedTick, int lastResult, bool captureAeroActuators)
        {
            if (vessel == null || !vessel.loaded) {
                ch.Reset ();
                return;
            }

            long capabilities = 0;
            var state = NewNaNArray (StateStride);

            // ----------------------------- translational/rotational state
            double[] position;
            if (TryWorldPosition (vessel, out position)) {
                Set3 (state, 0, position);
                capabilities |= CapPosition;
            }

            double[] orbitalVelocity;
            if (TryReadVectorMember (vessel,
                    new[] { "obt_velocity", "orbitalVelocity", "rb_velocity" },
                    out orbitalVelocity)) {
                Set3 (state, 3, orbitalVelocity);
                capabilities |= CapOrbitalVelocity;
            }

            double[] surfaceVelocity;
            if (TryReadVectorMember (vessel,
                    new[] { "srf_velocity", "surfaceVelocity" }, out surfaceVelocity)) {
                Set3 (state, 6, surfaceVelocity);
                capabilities |= CapSurfaceVelocity;
            }

            double dt = Math.Max (1e-6, TimeWarp.fixedDeltaTime);
            if (ch.havePrevious && physicsTick == ch.previousTick + 1) {
                if ((capabilities & CapSurfaceVelocity) != 0) {
                    Set3 (state, 9, DifferenceOverDt (
                        surfaceVelocity, ch.previousSurfaceVelocity, dt));
                    capabilities |= CapSurfaceAcceleration;
                }
                if ((capabilities & CapOrbitalVelocity) != 0) {
                    Set3 (state, 12, DifferenceOverDt (
                        orbitalVelocity, ch.previousOrbitalVelocity, dt));
                    capabilities |= CapOrbitalAcceleration;
                }
            }

            double[] rotation;
            if (TryRotation (vessel, out rotation)) {
                Set4 (state, 15, rotation);
                capabilities |= CapRotation;
            }

            // Vessel.angularVelocity is NOT world: VesselPrecalculate.CalculatePhysicsStats
            // stores the mass-weighted mean of Inverse(ReferenceTransform.rotation) *
            // rb.angularVelocity, i.e. the vessel BODY frame (right, forward, bottom). The
            // schema-1 name "angular_velocity_world" is kept for wire compatibility only;
            // the derivative below is therefore the body-frame angular acceleration too.
            double[] angularVelocity;
            if (TryReadVectorMember (vessel,
                    new[] { "angularVelocity", "angular_velocity" },
                    out angularVelocity)) {
                Set3 (state, 19, angularVelocity);
                capabilities |= CapAngularVelocity;
                if (ch.havePrevious && physicsTick == ch.previousTick + 1) {
                    Set3 (state, 22, DifferenceOverDt (
                        angularVelocity, ch.previousAngularVelocity, dt));
                    capabilities |= CapAngularAcceleration;
                }
            }

            // KSP's native rigid-body mass unit is the tonne; force is kN. Keeping the
            // native pair avoids a silent factor-of-1000 in an optimizer. The schema and
            // Python decoder call this field mass_tonnes explicitly.
            state [25] = vessel.totalMass;
            capabilities |= CapMass;

            Vector3 com = vessel.CurrentCoM;
            state [26] = com.x;
            state [27] = com.y;
            state [28] = com.z;
            capabilities |= CapCoM;

            // Vessel.MOI is the DIAGONAL of the inertia tensor about the CoM, in the vessel
            // body frame, in tonne.m^2 (it is built from rb.mass, in tonnes). kRPC's
            // Vessel.moment_of_inertia is exactly this value * 1000 (kg.m^2).
            double[] moi;
            if (TryReadVectorMember (vessel, new[] { "MOI", "moi" }, out moi)) {
                Set3 (state, 29, moi);
                capabilities |= CapMoi;
            }

            double[] gravity;
            if (position != null && TryGravity (vessel, position, out gravity)) {
                Set3 (state, 32, gravity);
                capabilities |= CapGravity;
            }

            // ------------------------------------------ atmosphere / navigation
            bool atmosphere = false;
            atmosphere |= SetOptionalDouble (state, 35, vessel, "atmDensity");
            atmosphere |= SetOptionalDouble (state, 36, vessel, "staticPressurekPa");
            atmosphere |= SetOptionalDouble (state, 37, vessel, "dynamicPressurekPa");
            atmosphere |= SetOptionalDouble (state, 38, vessel, "mach");
            atmosphere |= SetOptionalDouble (state, 39, vessel, "altitude");
            atmosphere |= SetOptionalDouble (state, 40, vessel, "radarAltitude");
            atmosphere |= SetOptionalDouble (state, 41, vessel, "latitude");
            atmosphere |= SetOptionalDouble (state, 42, vessel, "longitude");
            if (atmosphere)
                capabilities |= CapAtmosphere;

            object ctrlState = ReadMember (vessel, new[] { "ctrlState" });
            if (ctrlState != null && SetOptionalDouble (
                    state, 43, ctrlState, "mainThrottle"))
                capabilities |= CapGlobalThrottle;

            double[] aeroForce;
            if (TryReadVectorMember (vessel,
                    new[] { "aerodynamicForce", "aeroForce" }, out aeroForce)) {
                Set3 (state, 44, aeroForce);
                capabilities |= CapAeroForce;
            }
            double[] aeroTorque;
            if (TryReadVectorMember (vessel,
                    new[] { "aerodynamicTorque", "aeroTorque" }, out aeroTorque)) {
                Set3 (state, 47, aeroTorque);
                capabilities |= CapAeroTorque;
            }
            double[] dragVector;
            if (TryReadVectorMember (vessel,
                    new[] { "dragVector" }, out dragVector)) {
                Set3 (state, 50, dragVector);
                capabilities |= CapDragVector;
            }
            double[] liftVector;
            if (TryReadVectorMember (vessel,
                    new[] { "liftVector", "liftForce" }, out liftVector)) {
                Set3 (state, 53, liftVector);
                capabilities |= CapLiftVector;
            }

            if (surfaceVelocity != null)
                state [56] = Norm3 (surfaceVelocity);
            if (orbitalVelocity != null)
                state [57] = Norm3 (orbitalVelocity);
            SetOptionalDouble (state, 58, vessel, "geeForce");
            state [59] = (int)vessel.situation;
            state [60] = vessel.missionTime;
            state [61] = vessel.packed ? 1.0 : 0.0;

            if (surfaceVelocity != null && vessel.ReferenceTransform != null) {
                var v = new Vector3 (
                    (float)surfaceVelocity [0],
                    (float)surfaceVelocity [1],
                    (float)surfaceVelocity [2]);
                Vector3 local = vessel.ReferenceTransform.InverseTransformDirection (v);
                state [62] = local.x;
                state [63] = local.y;
                state [64] = local.z;
                capabilities |= CapBodySurfaceVelocity;
            }
            if (SetOptionalDoubleAny (state, 65, vessel,
                    new[] { "angleOfAttack", "AoA", "aoa" }))
                capabilities |= CapAoA;
            if (SetOptionalDoubleAny (state, 66, vessel,
                    new[] { "sideslipAngle", "sideslip", "beta" }))
                capabilities |= CapSideslip;

            // -------------------------------------- canonical kRPC 0.6 live aero
            // The 3.1 raw reflection fields above are kept byte-for-byte for wire
            // compatibility. 3.2 uses SpaceCenter.Flight: those values have explicit
            // SI units and the vessel body reference frame (right, forward, bottom).
            double[] aeroForceBody = null;
            double[] aeroTorqueBody = null;
            double[] aeroLiftBody = null;
            double[] aeroDragBody = null;
            double[] aeroSideBody = null;
            SCFlight flightBody = GetBodyFlight (ch, vessel);
            if (flightBody != null) {
                bool haveAeroForce = TryTuple3 (() => flightBody.AerodynamicForce, out aeroForceBody);
                bool haveAeroTorque = TryTuple3 (() => flightBody.AerodynamicTorque, out aeroTorqueBody);
                if (haveAeroForce)
                    capabilities |= CapKrpcLiveAeroForce;
                if (haveAeroTorque)
                    capabilities |= CapKrpcLiveAeroTorque;

                bool haveLift = TryTuple3 (() => flightBody.Lift, out aeroLiftBody);
                bool haveDrag = TryTuple3 (() => flightBody.Drag, out aeroDragBody);
                // kRPC 0.6.0 exposes total force, lift and drag. Side-force is
                // reconstructed as the exact remainder so the bridge does not take
                // a compile-time dependency on a newer client surface.
                if (haveAeroForce && haveLift && haveDrag) {
                    aeroSideBody = new[] {
                        aeroForceBody [0] - aeroLiftBody [0] - aeroDragBody [0],
                        aeroForceBody [1] - aeroLiftBody [1] - aeroDragBody [1],
                        aeroForceBody [2] - aeroLiftBody [2] - aeroDragBody [2]
                    };
                    capabilities |= CapKrpcLiveAeroComponents;
                }

                double scalar;
                bool haveAngles = false;
                if (TryScalar (() => flightBody.AngleOfAttack, out scalar)) {
                    state [105] = scalar;
                    haveAngles = true;
                }
                if (TryScalar (() => flightBody.SideslipAngle, out scalar)) {
                    state [106] = scalar;
                    haveAngles = true;
                }
                if (haveAngles)
                    capabilities |= CapKrpcAeroAngles;

                bool haveThermo = false;
                if (TryScalar (() => flightBody.DynamicPressure, out scalar)) {
                    state [100] = scalar;
                    haveThermo = true;
                }
                if (TryScalar (() => flightBody.StaticPressure, out scalar)) {
                    state [101] = scalar;
                    haveThermo = true;
                }
                if (TryScalar (() => flightBody.AtmosphereDensity, out scalar)) {
                    state [102] = scalar;
                    haveThermo = true;
                }
                if (TryScalar (() => flightBody.SpeedOfSound, out scalar)) {
                    state [103] = scalar;
                    haveThermo = true;
                }
                if (TryScalar (() => flightBody.TrueAirSpeed, out scalar)) {
                    state [104] = scalar;
                    haveThermo = true;
                }
                if (haveThermo)
                    capabilities |= CapKrpcAeroThermo;
            }

            // --------------------------------------------------- actionuator state
            var engines = new List<double> ();
            var gimbals = new List<double> ();
            var transforms = new List<double> ();
            int engineCount = 0;
            int gimbalCount = 0;
            int transformCount = 0;
            var reference = vessel.ReferenceTransform;
            Vector3 realizedEngineForceBody = Vector3.zero;
            Vector3 realizedEngineTorqueBody = Vector3.zero;
            Vector3 realizedEngineForceWorld = Vector3.zero;
            Vector3 realizedEngineTorqueWorld = Vector3.zero;
            bool haveRealizedEngineWrench = reference != null;

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
                        engines.Add (ActuatorsAddon.ThrottleIsLeased (engine) ? 1.0 : 0.0);
                        engines.Add (ReadDoubleOrNaN (engine, new[] { "flameout" }));
                        engines.Add (ReadDoubleOrNaN (engine, new[] { "minThrust" }));
                        engineCount++;

                        // ModuleEngines.finalThrust is the engine total (kN). Split it
                        // across its thrust transforms according to the stock multiplier
                        // weights, preserving a negative multiplier as a direction flip.
                        double multiplierWeightSum = 0.0;
                        for (int tw = 0; tw < ownTransformCount; tw++) {
                            double mw = 1.0;
                            if (engine.thrustTransformMultipliers != null &&
                                    tw < engine.thrustTransformMultipliers.Count)
                                mw = engine.thrustTransformMultipliers [tw];
                            multiplierWeightSum += Math.Abs (mw);
                        }
                        if (Math.Abs (engine.finalThrust) > 1e-9 &&
                                (ownTransformCount == 0 || multiplierWeightSum <= 1e-12))
                            haveRealizedEngineWrench = false;

                        for (int t = 0; t < ownTransformCount; t++) {
                            var transform = engine.thrustTransforms [t];
                            float multiplier = 1f;
                            if (engine.thrustTransformMultipliers != null &&
                                    t < engine.thrustTransformMultipliers.Count)
                                multiplier = engine.thrustTransformMultipliers [t];

                            if (transform != null && reference != null &&
                                    multiplierWeightSum > 1e-12) {
                                double signedWeight = multiplier / multiplierWeightSum;
                                float nozzleForceN = (float)(
                                    engine.finalThrust * 1000.0 * signedWeight);
                                // KSP thrustTransforms point along the exhaust/nozzle direction.
                                // The force applied to the vessel is the reaction force, i.e.
                                // opposite transform.forward. Keep the raw transform geometry
                                // unchanged elsewhere; only the realized wrench uses vessel force.
                                Vector3 forceWorld = -nozzleForceN * transform.forward.normalized;
                                Vector3 leverWorld = transform.position - com;
                                Vector3 forceBody = reference.InverseTransformDirection (forceWorld);
                                Vector3 leverBody = reference.InverseTransformDirection (leverWorld);
                                realizedEngineForceWorld += forceWorld;
                                realizedEngineTorqueWorld += Cross3 (leverWorld, forceWorld);
                                realizedEngineForceBody += forceBody;
                                realizedEngineTorqueBody += Cross3 (leverBody, forceBody);
                            } else if (ownTransformCount > 0) {
                                haveRealizedEngineWrench = false;
                            }

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
                        ActuatorsAddon.GimbalLeaseTarget (
                            gimbal, out targetX, out targetY);
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
                        gimbals.Add (ActuatorsAddon.GimbalIsLeased (gimbal) ? 1.0 : 0.0);
                        gimbals.Add (targetX);
                        gimbals.Add (targetY);
                        gimbalCount++;
                    }
                }
            }
            capabilities |= CapActuators;

            // ---------------------------------------------------- schema 3.2 extension
            if (haveRealizedEngineWrench) {
                Set3 (state, 67, realizedEngineForceBody);
                Set3 (state, 70, realizedEngineForceWorld);
                Set3 (state, 73, realizedEngineTorqueBody);
                Set3 (state, 76, realizedEngineTorqueWorld);
                capabilities |= CapRealizedEngineWrench;
                capabilities |= CapRealizedEngineForceOnVessel;
            }

            double[] aeroForceWorld = BodyToWorld (reference, aeroForceBody);
            double[] aeroTorqueWorld = BodyToWorld (reference, aeroTorqueBody);
            if (aeroForceBody != null) {
                Set3 (state, 79, aeroForceBody);
                if (aeroForceWorld != null)
                    Set3 (state, 82, aeroForceWorld);
            }
            if (aeroTorqueBody != null) {
                Set3 (state, 85, aeroTorqueBody);
                if (aeroTorqueWorld != null)
                    Set3 (state, 88, aeroTorqueWorld);
            }
            if (aeroLiftBody != null)
                Set3 (state, 91, aeroLiftBody);
            if (aeroDragBody != null)
                Set3 (state, 94, aeroDragBody);
            if (aeroSideBody != null)
                Set3 (state, 97, aeroSideBody);

            // Translational residual in an inertial/world frame. Mass is native KSP
            // tonnes, hence *1000 for kg. This is deliberately called EXTERNAL, not
            // aerodynamic: RCS/contact/joint forces and model/frame errors also live here.
            if ((capabilities & CapOrbitalAcceleration) != 0 &&
                    (capabilities & CapGravity) != 0 &&
                    (capabilities & CapMass) != 0 &&
                    (capabilities & CapRealizedEngineWrench) != 0) {
                double massKg = state [25] * 1000.0;
                var externalWorld = new[] {
                    massKg * (state [12] - state [32]) - state [70],
                    massKg * (state [13] - state [33]) - state [71],
                    massKg * (state [14] - state [34]) - state [72]
                };
                Set3 (state, 107, externalWorld);
                double[] externalBody = WorldToBody (reference, externalWorld);
                if (externalBody != null)
                    Set3 (state, 110, externalBody);
                capabilities |= CapExternalForceResidual;

                if (aeroForceWorld != null) {
                    var unexplained = new[] {
                        externalWorld [0] - aeroForceWorld [0],
                        externalWorld [1] - aeroForceWorld [1],
                        externalWorld [2] - aeroForceWorld [2]
                    };
                    Set3 (state, 113, unexplained);
                    capabilities |= CapUnexplainedNonAeroForce;
                }
            }

            var frame = new List<double> (
                HeaderStride + StateStride + engines.Count + gimbals.Count + transforms.Count) {
                ProtocolVersion,
                SchemaVersion,
                physicsTick,
                Planetarium.GetUniversalTime (),
                TimeWarp.fixedDeltaTime,
                vessel.persistentId,
                topologyGeneration,
                lastAcceptedSequence,
                lastAppliedSequence,
                lastAppliedTick,
                lastResult,
                capabilities,
                StateStride,
                engineCount,
                EngineStride,
                gimbalCount,
                GimbalStride,
                transformCount,
                TransformStride,
                HistoryCapacity,
                ch.history.Count,
                0.0 // capture phase: pre-integration FixedUpdate
            };
            frame.AddRange (state);
            frame.AddRange (engines);
            frame.AddRange (gimbals);
            frame.AddRange (transforms);

            ch.latest = frame.ToArray ();
            ch.latestCapabilities = capabilities;
            ch.latestTick = physicsTick;
            ch.latestAppliedSequence = lastAppliedSequence;
            ch.latestAppliedTick = lastAppliedTick;
            ch.latestResult = lastResult;
            ch.history.AddLast (ch.latest);
            while (ch.history.Count > HistoryCapacity)
                ch.history.RemoveFirst ();

            // Supplemental surface observability, captured in the SAME FixedUpdate
            // but transported separately so DynamicsSnapshotV3/PDG2 schema stays stable.
            if (captureAeroActuators) {
                try {
                    AeroActuatorV1.Capture (vessel, physicsTick);
                } catch { }
            }

            // DynamicsExtV1: same FixedUpdate and tick, separate transport, born closed.
            // It only READS the finished state array above, so the v3 frame is identical
            // whether it runs or not.
            if (DynamicsExt.Enabled) {
                try {
                    DynamicsExt.Capture (ch, vessel, physicsTick, topologyGeneration,
                        state, captureAeroActuators);
                } catch {
                    // Pure observability: never perturb FixedUpdate or control.
                    ch.ext.Reset ();
                }
            } else if (ch.ext.HasData) {
                ch.ext.Reset ();
            }

            ch.havePrevious = true;
            ch.previousTick = physicsTick;
            if (surfaceVelocity != null)
                Copy3 (surfaceVelocity, ch.previousSurfaceVelocity);
            if (orbitalVelocity != null)
                Copy3 (orbitalVelocity, ch.previousOrbitalVelocity);
            if (angularVelocity != null)
                Copy3 (angularVelocity, ch.previousAngularVelocity);
        }

        internal static double[] NewNaNArray (int count)
        {
            var values = new double[count];
            for (int i = 0; i < count; i++)
                values [i] = double.NaN;
            return values;
        }

        static void Set3 (double[] target, int offset, double[] value)
        {
            target [offset] = value [0];
            target [offset + 1] = value [1];
            target [offset + 2] = value [2];
        }

        static void Set3 (double[] target, int offset, Vector3 value)
        {
            target [offset] = value.x;
            target [offset + 1] = value.y;
            target [offset + 2] = value.z;
        }

        static SCFlight GetBodyFlight (DynamicsChannel ch, Vessel vessel)
        {
            try {
                if (!ReferenceEquals (ch.serviceInternalVessel, vessel) ||
                        ch.serviceVessel == null || ch.serviceFlightBody == null) {
                    ch.serviceInternalVessel = vessel;
                    ch.serviceVessel = new SCVessel (vessel);
                    ch.serviceFlightBody = ch.serviceVessel.Flight (ch.serviceVessel.ReferenceFrame);
                }
                return ch.serviceFlightBody;
            } catch {
                ch.serviceInternalVessel = null;
                ch.serviceVessel = null;
                ch.serviceFlightBody = null;
                return null;
            }
        }

        static bool TryTuple3 (Func<Tuple<double, double, double>> read, out double[] value)
        {
            value = null;
            try {
                var tuple = read ();
                if (tuple == null)
                    return false;
                value = new[] { tuple.Item1, tuple.Item2, tuple.Item3 };
                return IsFinite3 (value);
            } catch {
                return false;
            }
        }

        static bool TryScalar<T> (Func<T> read, out double value)
        {
            value = double.NaN;
            try {
                value = Convert.ToDouble (
                    read (), System.Globalization.CultureInfo.InvariantCulture);
                return !double.IsNaN (value) && !double.IsInfinity (value);
            } catch {
                return false;
            }
        }

        static bool IsFinite3 (double[] value)
        {
            return value != null && value.Length >= 3 &&
                !double.IsNaN (value [0]) && !double.IsInfinity (value [0]) &&
                !double.IsNaN (value [1]) && !double.IsInfinity (value [1]) &&
                !double.IsNaN (value [2]) && !double.IsInfinity (value [2]);
        }

        internal static Vector3 Cross3 (Vector3 a, Vector3 b)
        {
            return new Vector3 (
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x);
        }

        static double Dot3 (Vector3 a, Vector3 b)
        {
            return (double)a.x * b.x + (double)a.y * b.y +
                (double)a.z * b.z;
        }

        static double[] BodyToWorld (Transform reference, double[] body)
        {
            if (reference == null || !IsFinite3 (body))
                return null;

            // The verifier intentionally exposes only InverseTransformDirection.
            // For a pure rotation, body->world is the transpose of world->body.
            // Project the body vector on the body-space images of the three
            // world basis vectors. This is equivalent to TransformDirection,
            // while remaining compatible with both the verification stubs and
            // Unity/KSP.
            Vector3 b = new Vector3 (
                (float)body [0], (float)body [1], (float)body [2]);
            Vector3 worldXInBody = reference.InverseTransformDirection (Vector3.right);
            Vector3 worldYInBody = reference.InverseTransformDirection (Vector3.up);
            Vector3 worldZInBody = reference.InverseTransformDirection (Vector3.forward);
            return new[] {
                Dot3 (b, worldXInBody),
                Dot3 (b, worldYInBody),
                Dot3 (b, worldZInBody)
            };
        }

        static double[] WorldToBody (Transform reference, double[] world)
        {
            if (reference == null || !IsFinite3 (world))
                return null;
            Vector3 body = reference.InverseTransformDirection (new Vector3 (
                (float)world [0], (float)world [1], (float)world [2]));
            return new[] { (double)body.x, (double)body.y, (double)body.z };
        }

        /// <summary>WorldToBody for DynamicsExtV1 (same transform, same float rounding).</summary>
        internal static double[] WorldToBodyVector (Transform reference, double[] world)
        {
            return WorldToBody (reference, world);
        }

        static void Set4 (double[] target, int offset, double[] value)
        {
            target [offset] = value [0];
            target [offset + 1] = value [1];
            target [offset + 2] = value [2];
            target [offset + 3] = value [3];
        }

        static void Copy3 (double[] source, double[] target)
        {
            target [0] = source [0];
            target [1] = source [1];
            target [2] = source [2];
        }

        static double[] DifferenceOverDt (double[] current, double[] previous, double dt)
        {
            return new[] {
                (current [0] - previous [0]) / dt,
                (current [1] - previous [1]) / dt,
                (current [2] - previous [2]) / dt
            };
        }

        static double Norm3 (double[] value)
        {
            return Math.Sqrt (
                value [0] * value [0] +
                value [1] * value [1] +
                value [2] * value [2]);
        }

        static bool TryWorldPosition (Vessel vessel, out double[] value)
        {
            value = null;
            object result = InvokeZeroArg (vessel, new[] { "GetWorldPos3D" });
            if (TryVector3 (result, out value))
                return true;
            if (vessel.ReferenceTransform == null)
                return false;
            Vector3 p = vessel.ReferenceTransform.position;
            value = new[] { (double)p.x, (double)p.y, (double)p.z };
            return true;
        }

        static bool TryRotation (Vessel vessel, out double[] value)
        {
            value = null;
            object reference = vessel.ReferenceTransform;
            if (reference != null) {
                object rotation = ReadMember (reference, new[] { "rotation" });
                if (TryQuaternion (rotation, out value))
                    return true;
            }
            object transform = ReadMember (vessel, new[] { "transform" });
            object fallback = ReadMember (transform, new[] { "rotation" });
            return TryQuaternion (fallback, out value);
        }

        static bool TryGravity (Vessel vessel, double[] vesselPosition, out double[] gravity)
        {
            gravity = null;
            object body = vessel.mainBody;
            if (body == null)
                return false;
            double mu;
            if (!TryDouble (ReadMember (body, new[] { "gravParameter" }), out mu) || mu <= 0.0)
                return false;
            double[] bodyPosition;
            if (!TryReadVectorMember (body, new[] { "position" }, out bodyPosition))
                return false;
            double dx = bodyPosition [0] - vesselPosition [0];
            double dy = bodyPosition [1] - vesselPosition [1];
            double dz = bodyPosition [2] - vesselPosition [2];
            double r2 = dx * dx + dy * dy + dz * dz;
            if (r2 <= 1.0)
                return false;
            double invR = 1.0 / Math.Sqrt (r2);
            double scale = mu * invR * invR * invR;
            gravity = new[] { dx * scale, dy * scale, dz * scale };
            return true;
        }

        static bool TryReadVectorMember (object target, string[] names, out double[] value)
        {
            return TryVector3 (ReadMember (target, names), out value);
        }

        static bool SetOptionalDouble (double[] state, int index, object target, string name)
        {
            double value;
            if (!TryDouble (ReadMember (target, new[] { name }), out value))
                return false;
            state [index] = value;
            return true;
        }

        static bool SetOptionalDoubleAny (
            double[] state, int index, object target, string[] names)
        {
            double value;
            if (!TryDouble (ReadMember (target, names), out value))
                return false;
            state [index] = value;
            return true;
        }

        static double ReadDoubleOrNaN (object target, string[] names)
        {
            double value;
            return TryDouble (ReadMember (target, names), out value)
                ? value
                : double.NaN;
        }

        static object ReadMember (object target, string[] names)
        {
            if (target == null)
                return null;
            Type type = target.GetType ();
            for (int i = 0; i < names.Length; i++) {
                string cacheKey = type.AssemblyQualifiedName + "|" + names [i];
                MemberInfo member;
                if (!memberCache.TryGetValue (cacheKey, out member)) {
                    member = (MemberInfo)type.GetProperty (
                        names [i], BindingFlags.Instance | BindingFlags.Public |
                        BindingFlags.NonPublic)
                        ?? type.GetField (
                            names [i], BindingFlags.Instance | BindingFlags.Public |
                            BindingFlags.NonPublic);
                    memberCache [cacheKey] = member;
                }
                if (member == null)
                    continue;
                try {
                    var property = member as PropertyInfo;
                    if (property != null)
                        return property.GetValue (target, null);
                    var field = member as FieldInfo;
                    if (field != null)
                        return field.GetValue (target);
                } catch {
                    // A single unavailable live member must not kill the physics loop.
                }
            }
            return null;
        }

        static object InvokeZeroArg (object target, string[] names)
        {
            if (target == null)
                return null;
            Type type = target.GetType ();
            for (int i = 0; i < names.Length; i++) {
                string cacheKey = type.AssemblyQualifiedName + "|" + names [i] + "()";
                MethodInfo method;
                if (!methodCache.TryGetValue (cacheKey, out method)) {
                    method = type.GetMethod (
                        names [i], BindingFlags.Instance | BindingFlags.Public |
                        BindingFlags.NonPublic, null, Type.EmptyTypes, null);
                    methodCache [cacheKey] = method;
                }
                if (method == null)
                    continue;
                try {
                    return method.Invoke (target, null);
                } catch {
                    // Best-effort capability discovery only.
                }
            }
            return null;
        }

        static bool TryDouble (object value, out double number)
        {
            number = double.NaN;
            if (value == null)
                return false;
            try {
                number = Convert.ToDouble (
                    value, System.Globalization.CultureInfo.InvariantCulture);
                return !double.IsNaN (number) && !double.IsInfinity (number);
            } catch {
                return false;
            }
        }

        static bool TryVector3 (object value, out double[] vector)
        {
            vector = null;
            if (value == null)
                return false;
            var unity = value is Vector3 ? (Vector3)value : default (Vector3);
            if (value is Vector3) {
                vector = new[] { (double)unity.x, (double)unity.y, (double)unity.z };
                return true;
            }
            if (value is Vector3d) {
                var d = (Vector3d)value;
                vector = new[] { d.x, d.y, d.z };
                return true;
            }
            double x, y, z;
            if (TryDouble (ReadMember (value, new[] { "x" }), out x) &&
                    TryDouble (ReadMember (value, new[] { "y" }), out y) &&
                    TryDouble (ReadMember (value, new[] { "z" }), out z)) {
                vector = new[] { x, y, z };
                return true;
            }
            return false;
        }

        static bool TryQuaternion (object value, out double[] quaternion)
        {
            quaternion = null;
            if (value == null)
                return false;
            double x, y, z, w;
            if (TryDouble (ReadMember (value, new[] { "x" }), out x) &&
                    TryDouble (ReadMember (value, new[] { "y" }), out y) &&
                    TryDouble (ReadMember (value, new[] { "z" }), out z) &&
                    TryDouble (ReadMember (value, new[] { "w" }), out w)) {
                quaternion = new[] { x, y, z, w };
                return true;
            }
            return false;
        }
    }
}
