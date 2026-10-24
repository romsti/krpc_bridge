using System;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Bridge.Core;
using KRPC.Service.Attributes;
using SCFlight = KRPC.SpaceCenter.Services.Flight;
using SCVessel = KRPC.SpaceCenter.Services.Vessel;
using UnityEngine;
using KRPC.Bridge.Control;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Telemetry
{
    /// <summary>
    /// Dynamic-state transport.
    ///
    /// The payload deliberately stays an IList&lt;double&gt;: that is a kRPC-native type,
    /// cheap to transport and immune to one malformed custom class signature disabling
    /// the whole server.  See docs/API.md for the exact flat schema.
    /// </summary>
    public static partial class TelemetryService
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

        /// <summary>
        /// QW6 identity check, read-only: captures the active vessel's v3 frame and its
        /// aero-actuator rows twice in this call, through the legacy reflection readers and
        /// through the cache, and compares the doubles bit for bit. Returns protocol (1),
        /// v3 length, v3 mismatches, first v3 mismatch (-1), aero length, aero mismatches,
        /// first aero mismatch (-1), legacy and cached capture cost (microseconds, mean of
        /// five). Nothing is written to the rings.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> DynamicsReflectionCheckV1 ()
        {
            return DynamicsV3.ReflectionCheck ();
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
        // Core v2: the preallocated frame ring. Its newest frame is the latest snapshot (the
        // two were always the same array).
        internal readonly Recorder history = new Recorder (DynamicsV3.HistoryCapacity);
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

        // Scratch of one capture (Unity main thread only): no state array and no row list
        // allocated per physics step. Read by DynamicsExtV1 within the same capture only.
        internal readonly double[] stateScratch = new double[DynamicsV3.StateStride];
        internal readonly List<double> engineRows = new List<double> ();
        internal readonly List<double> gimbalRows = new List<double> ();
        internal readonly List<double> transformRows = new List<double> ();

        /// <summary>True when the ring holds a frame (the legacy latest.Length &gt; 0).</summary>
        internal bool HasLatest {
            get { return history.Count > 0; }
        }

        internal void Reset ()
        {
            history.Clear ();
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
            return history.CopyNewest ();
        }

        internal IList<double> ReadStatus ()
        {
            return new List<double> {
                DynamicsV3.ProtocolVersion,
                DynamicsV3.StatusSchemaVersion,
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
            return history.ReadSince (DynamicsV3.ProtocolVersion,
                DynamicsV3.SchemaVersion, sinceTick, maxFrames);
        }
    }

    /// <summary>
    /// Captures one coherent pre-integration FixedUpdate state and keeps a short ring.
    ///
    /// Actuation.Pump applies a queued command first, updates leased gimbals, then
    /// calls Capture. Therefore frame tick k means: state sampled in FixedUpdate k after
    /// command sequence metadata for k has been applied, but before the next observed
    /// physics response. Comparing k, k+1, ... gives the real command-to-response delay.
    /// </summary>
    internal static class DynamicsV3
    {
        internal const int ProtocolVersion = 3;
        // Schema 3 (refactor R3): the frame ends with the clock section (ClockStride values).
        internal const int SchemaVersion = 3;
        // dynamics_status_v3 did not change: it keeps the schema it had (2).
        internal const int StatusSchemaVersion = 2;
        internal const int HeaderStride = 22;
        // The trailing clock section: the GameClock master tick of the capture's step.
        internal const int ClockStride = 1;
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
        internal const long CapPosition = 1L << 0;
        internal const long CapOrbitalVelocity = 1L << 1;
        internal const long CapSurfaceVelocity = 1L << 2;
        internal const long CapSurfaceAcceleration = 1L << 3;
        internal const long CapOrbitalAcceleration = 1L << 4;
        internal const long CapRotation = 1L << 5;
        internal const long CapAngularVelocity = 1L << 6;
        internal const long CapAngularAcceleration = 1L << 7;
        internal const long CapMass = 1L << 8;
        internal const long CapCoM = 1L << 9;
        internal const long CapMoi = 1L << 10;
        internal const long CapGravity = 1L << 11;
        internal const long CapAtmosphere = 1L << 12;
        internal const long CapAeroForce = 1L << 13;
        internal const long CapAeroTorque = 1L << 14;
        internal const long CapDragVector = 1L << 15;
        internal const long CapLiftVector = 1L << 16;
        internal const long CapBodySurfaceVelocity = 1L << 17;
        internal const long CapAoA = 1L << 18;
        internal const long CapSideslip = 1L << 19;
        internal const long CapGlobalThrottle = 1L << 20;
        internal const long CapActuators = 1L << 21;
        internal const long CapRealizedEngineWrench = 1L << 22;
        internal const long CapKrpcLiveAeroForce = 1L << 23;
        internal const long CapKrpcLiveAeroTorque = 1L << 24;
        internal const long CapKrpcLiveAeroComponents = 1L << 25;
        internal const long CapKrpcAeroAngles = 1L << 26;
        internal const long CapKrpcAeroThermo = 1L << 27;
        internal const long CapExternalForceResidual = 1L << 28;
        internal const long CapUnexplainedNonAeroForce = 1L << 29;
        internal const long CapRealizedEngineForceOnVessel = 1L << 30;

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

        // The history reader of every transport is the Core v2 Recorder's ReadSince: the
        // six-value header, then length-prefixed frames newer than sinceTick (tick at offset 2),
        // exactly the LinkedList reader it replaced (build/tests/core, LegacyRing.cs).

        /// <summary>
        /// The member names the capture reads by reflection, allocated once (they were 42 arrays
        /// built at every physics step). Same names, same order, same lookups.
        /// </summary>
        static class Names
        {
            internal static readonly string[] ObtVelocity = { "obt_velocity", "orbitalVelocity", "rb_velocity" };
            internal static readonly string[] SrfVelocity = { "srf_velocity", "surfaceVelocity" };
            internal static readonly string[] AngularVelocity = { "angularVelocity", "angular_velocity" };
            internal static readonly string[] Moi = { "MOI", "moi" };
            internal static readonly string[] AtmDensity = { "atmDensity" };
            internal static readonly string[] StaticPressure = { "staticPressurekPa" };
            internal static readonly string[] DynamicPressure = { "dynamicPressurekPa" };
            internal static readonly string[] Mach = { "mach" };
            internal static readonly string[] Altitude = { "altitude" };
            internal static readonly string[] RadarAltitude = { "radarAltitude" };
            internal static readonly string[] Latitude = { "latitude" };
            internal static readonly string[] Longitude = { "longitude" };
            internal static readonly string[] CtrlState = { "ctrlState" };
            internal static readonly string[] MainThrottle = { "mainThrottle" };
            internal static readonly string[] AeroForce = { "aerodynamicForce", "aeroForce" };
            internal static readonly string[] AeroTorque = { "aerodynamicTorque", "aeroTorque" };
            internal static readonly string[] DragVector = { "dragVector" };
            internal static readonly string[] LiftVector = { "liftVector", "liftForce" };
            internal static readonly string[] GeeForce = { "geeForce" };
            internal static readonly string[] AoA = { "angleOfAttack", "AoA", "aoa" };
            internal static readonly string[] Sideslip = { "sideslipAngle", "sideslip", "beta" };
            internal static readonly string[] Flameout = { "flameout" };
            internal static readonly string[] MinThrust = { "minThrust" };
            internal static readonly string[] WorldPos = { "GetWorldPos3D" };
            internal static readonly string[] Rotation = { "rotation" };
            internal static readonly string[] TransformMember = { "transform" };
            internal static readonly string[] GravParameter = { "gravParameter" };
            internal static readonly string[] Position = { "position" };
            internal static readonly string[] X = { "x" };
            internal static readonly string[] Y = { "y" };
            internal static readonly string[] Z = { "z" };
            internal static readonly string[] W = { "w" };
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
            // Core v2: the channel's scratch, refilled with NaN (what NewNaNArray gave).
            var state = ch.stateScratch;
            FillNaN (state);

            // ----------------------------- translational/rotational state
            double[] position;
            if (TryWorldPosition (vessel, out position)) {
                Set3 (state, 0, position);
                capabilities |= CapPosition;
            }

            double[] orbitalVelocity;
            if (TryReadVectorMember (vessel, Names.ObtVelocity, out orbitalVelocity)) {
                Set3 (state, 3, orbitalVelocity);
                capabilities |= CapOrbitalVelocity;
            }

            double[] surfaceVelocity;
            if (TryReadVectorMember (vessel, Names.SrfVelocity, out surfaceVelocity)) {
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
            if (TryReadVectorMember (vessel, Names.AngularVelocity, out angularVelocity)) {
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
            if (TryReadVectorMember (vessel, Names.Moi, out moi)) {
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
            atmosphere |= SetOptionalDouble (state, 35, vessel, Names.AtmDensity);
            atmosphere |= SetOptionalDouble (state, 36, vessel, Names.StaticPressure);
            atmosphere |= SetOptionalDouble (state, 37, vessel, Names.DynamicPressure);
            atmosphere |= SetOptionalDouble (state, 38, vessel, Names.Mach);
            atmosphere |= SetOptionalDouble (state, 39, vessel, Names.Altitude);
            atmosphere |= SetOptionalDouble (state, 40, vessel, Names.RadarAltitude);
            atmosphere |= SetOptionalDouble (state, 41, vessel, Names.Latitude);
            atmosphere |= SetOptionalDouble (state, 42, vessel, Names.Longitude);
            if (atmosphere)
                capabilities |= CapAtmosphere;

            object ctrlState = ReadMember (vessel, Names.CtrlState);
            if (ctrlState != null && SetOptionalDouble (
                    state, 43, ctrlState, Names.MainThrottle))
                capabilities |= CapGlobalThrottle;

            double[] aeroForce;
            if (TryReadVectorMember (vessel, Names.AeroForce, out aeroForce)) {
                Set3 (state, 44, aeroForce);
                capabilities |= CapAeroForce;
            }
            double[] aeroTorque;
            if (TryReadVectorMember (vessel, Names.AeroTorque, out aeroTorque)) {
                Set3 (state, 47, aeroTorque);
                capabilities |= CapAeroTorque;
            }
            double[] dragVector;
            if (TryReadVectorMember (vessel, Names.DragVector, out dragVector)) {
                Set3 (state, 50, dragVector);
                capabilities |= CapDragVector;
            }
            double[] liftVector;
            if (TryReadVectorMember (vessel, Names.LiftVector, out liftVector)) {
                Set3 (state, 53, liftVector);
                capabilities |= CapLiftVector;
            }

            if (surfaceVelocity != null)
                state [56] = Norm3 (surfaceVelocity);
            if (orbitalVelocity != null)
                state [57] = Norm3 (orbitalVelocity);
            SetOptionalDouble (state, 58, vessel, Names.GeeForce);
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
            if (SetOptionalDoubleAny (state, 65, vessel, Names.AoA))
                capabilities |= CapAoA;
            if (SetOptionalDoubleAny (state, 66, vessel, Names.Sideslip))
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
            // Core v2: the channel's row lists, cleared (no list allocated per step).
            var engines = ch.engineRows;
            var gimbals = ch.gimbalRows;
            var transforms = ch.transformRows;
            engines.Clear ();
            gimbals.Clear ();
            transforms.Clear ();
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
                        engines.Add (Actuation.ThrottleIsLeased (engine) ? 1.0 : 0.0);
                        engines.Add (ReadDoubleOrNaN (engine, Names.Flameout));
                        engines.Add (ReadDoubleOrNaN (engine, Names.MinThrust));
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
                                realizedEngineTorqueWorld += AttitudeGeometry.Cross3 (leverWorld, forceWorld);
                                realizedEngineForceBody += forceBody;
                                realizedEngineTorqueBody += AttitudeGeometry.Cross3 (leverBody, forceBody);
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
                        Actuation.GimbalLeaseTarget (
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
                        gimbals.Add (Actuation.GimbalIsLeased (gimbal) ? 1.0 : 0.0);
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

            // Core v2: the frame is written straight into the ring's next buffer, the same
            // values in the same order as the List + ToArray it replaces (no allocation once
            // the ring has wrapped). history.Count is the count BEFORE this frame, as before.
            int length = HeaderStride + StateStride + engines.Count + gimbals.Count + transforms.Count +
                ClockStride;
            var frame = ch.history.Reserve (length);
            int h = 0;
            frame [h++] = ProtocolVersion;
            frame [h++] = SchemaVersion;
            frame [h++] = physicsTick;
            frame [h++] = Planetarium.GetUniversalTime ();
            frame [h++] = TimeWarp.fixedDeltaTime;
            frame [h++] = vessel.persistentId;
            frame [h++] = topologyGeneration;
            frame [h++] = lastAcceptedSequence;
            frame [h++] = lastAppliedSequence;
            frame [h++] = lastAppliedTick;
            frame [h++] = lastResult;
            frame [h++] = capabilities;
            frame [h++] = StateStride;
            frame [h++] = engineCount;
            frame [h++] = EngineStride;
            frame [h++] = gimbalCount;
            frame [h++] = GimbalStride;
            frame [h++] = transformCount;
            frame [h++] = TransformStride;
            frame [h++] = HistoryCapacity;
            frame [h++] = ch.history.Count;
            frame [h++] = 0.0; // capture phase: pre-integration FixedUpdate
            Array.Copy (state, 0, frame, h, StateStride);
            h += StateStride;
            engines.CopyTo (frame, h);
            h += engines.Count;
            gimbals.CopyTo (frame, h);
            h += gimbals.Count;
            transforms.CopyTo (frame, h);
            h += transforms.Count;
            // Schema 3 (refactor R3): the clock section, after every row.
            frame [h] = GameClock.StepTick;

            ch.latestCapabilities = capabilities;
            ch.latestTick = physicsTick;
            ch.latestAppliedSequence = lastAppliedSequence;
            ch.latestAppliedTick = lastAppliedTick;
            ch.latestResult = lastResult;
            ch.history.Commit (length);

            // Supplemental surface observability, captured in the SAME FixedUpdate
            // but transported separately so the DynamicsSnapshotV3 schema stays stable.
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

        /// <summary>Body of DynamicsReflectionCheckV1. Scratch channels only; no ring is touched.</summary>
        internal static IList<double> ReflectionCheck ()
        {
            var vessel = FlightGlobals.ActiveVessel;
            if (vessel == null || !vessel.loaded)
                throw new InvalidOperationException (BridgeErrors.Tag (BridgeErrors.NotLoaded,
                    "aucun vaisseau actif et charge"));
            long tick = Actuation.CurrentPhysicsTick;
            bool previous = ReflectionCache.UseLegacy;
            double[] legacyFrame, cachedFrame;
            List<double> legacyAero = new List<double> (), cachedAero = new List<double> ();
            double legacyMicros = 0.0, cachedMicros = 0.0;
            try {
                // Identity: one capture per path, fresh channels, same tick and inputs.
                ReflectionCache.UseLegacy = true;
                var a = new DynamicsChannel ();
                CaptureInto (a, vessel, tick, 0, 0, 0, 0, 0, false);
                AeroActuatorV1.BuildRows (vessel, legacyAero);
                ReflectionCache.UseLegacy = false;
                var b = new DynamicsChannel ();
                CaptureInto (b, vessel, tick, 0, 0, 0, 0, 0, false);
                AeroActuatorV1.BuildRows (vessel, cachedAero);
                legacyFrame = a.history.NewestArray ();
                cachedFrame = b.history.NewestArray ();
                // Cost: five alternated captures per path, caches already warm.
                var scratch = new DynamicsChannel ();
                for (int k = 0; k < 5; k++) {
                    ReflectionCache.UseLegacy = true;
                    long t0 = System.Diagnostics.Stopwatch.GetTimestamp ();
                    scratch.Reset ();
                    CaptureInto (scratch, vessel, tick, 0, 0, 0, 0, 0, false);
                    long t1 = System.Diagnostics.Stopwatch.GetTimestamp ();
                    ReflectionCache.UseLegacy = false;
                    scratch.Reset ();
                    CaptureInto (scratch, vessel, tick, 0, 0, 0, 0, 0, false);
                    long t2 = System.Diagnostics.Stopwatch.GetTimestamp ();
                    legacyMicros += (t1 - t0) * 1e6 / System.Diagnostics.Stopwatch.Frequency / 5.0;
                    cachedMicros += (t2 - t1) * 1e6 / System.Diagnostics.Stopwatch.Frequency / 5.0;
                }
            } finally {
                ReflectionCache.UseLegacy = previous;
            }
            int firstV3;
            int mismatchV3 = CompareBits (legacyFrame, cachedFrame, out firstV3);
            int firstAero;
            int mismatchAero = CompareBits (legacyAero.ToArray (), cachedAero.ToArray (), out firstAero);
            return new List<double> {
                1.0,
                cachedFrame.Length,
                mismatchV3,
                firstV3,
                cachedAero.Count,
                mismatchAero,
                firstAero,
                legacyMicros,
                cachedMicros
            };
        }

        /// <summary>Doubles compared by their 64 bits (NaN equals the same NaN).</summary>
        internal static int CompareBits (double[] a, double[] b, out int first)
        {
            first = -1;
            int mismatches = Math.Abs (a.Length - b.Length);
            if (mismatches > 0)
                first = Math.Min (a.Length, b.Length);
            for (int i = 0; i < Math.Min (a.Length, b.Length); i++) {
                if (BitConverter.DoubleToInt64Bits (a [i]) != BitConverter.DoubleToInt64Bits (b [i])) {
                    mismatches++;
                    if (first < 0 || i < first)
                        first = i;
                }
            }
            return mismatches;
        }

        internal static double[] NewNaNArray (int count)
        {
            var values = new double[count];
            for (int i = 0; i < count; i++)
                values [i] = double.NaN;
            return values;
        }

        internal static void FillNaN (double[] values)
        {
            for (int i = 0; i < values.Length; i++)
                values [i] = double.NaN;
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
            object result = InvokeZeroArg (vessel, Names.WorldPos);
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
                object rotation = ReadMember (reference, Names.Rotation);
                if (TryQuaternion (rotation, out value))
                    return true;
            }
            object transform = ReadMember (vessel, Names.TransformMember);
            object fallback = ReadMember (transform, Names.Rotation);
            return TryQuaternion (fallback, out value);
        }

        static bool TryGravity (Vessel vessel, double[] vesselPosition, out double[] gravity)
        {
            gravity = null;
            object body = vessel.mainBody;
            if (body == null)
                return false;
            double mu;
            if (!TryDouble (ReadMember (body, Names.GravParameter), out mu) || mu <= 0.0)
                return false;
            double[] bodyPosition;
            if (!TryReadVectorMember (body, Names.Position, out bodyPosition))
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

        static bool SetOptionalDouble (double[] state, int index, object target, string[] name)
        {
            double value;
            if (!TryDouble (ReadMember (target, name), out value))
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

        /// <summary>
        /// QW6: cached reflection (ReflectionCache), same members and same values as the
        /// legacy reader below, which stays for dynamics_reflection_check_v1.
        /// </summary>
        static object ReadMember (object target, string[] names)
        {
            if (ReflectionCache.UseLegacy)
                return ReadMemberLegacy (target, names);
            return ReflectionCache.ReadPropertyThenField (target, names);
        }

        static object InvokeZeroArg (object target, string[] names)
        {
            if (ReflectionCache.UseLegacy)
                return InvokeZeroArgLegacy (target, names);
            return ReflectionCache.InvokeZeroArg (target, names);
        }

        /// <summary>The reader flown until QW6, verbatim. Reference of the identity check.</summary>
        static object ReadMemberLegacy (object target, string[] names)
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

        /// <summary>The method reader flown until QW6, verbatim.</summary>
        static object InvokeZeroArgLegacy (object target, string[] names)
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
            if (TryDouble (ReadMember (value, Names.X), out x) &&
                    TryDouble (ReadMember (value, Names.Y), out y) &&
                    TryDouble (ReadMember (value, Names.Z), out z)) {
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
            // QW6: a Unity Quaternion is read directly instead of four reflective reads of
            // its x/y/z/w fields. Same widening float -> double, same finiteness rule as
            // TryDouble, so the same doubles (or the same refusal) come out.
            if (!ReflectionCache.UseLegacy && value is Quaternion) {
                var q = (Quaternion)value;
                double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
                if (!IsFinite3 (new[] { qx, qy, qz }) || double.IsNaN (qw) || double.IsInfinity (qw))
                    return false;
                quaternion = new[] { qx, qy, qz, qw };
                return true;
            }
            double x, y, z, w;
            if (TryDouble (ReadMember (value, Names.X), out x) &&
                    TryDouble (ReadMember (value, Names.Y), out y) &&
                    TryDouble (ReadMember (value, Names.Z), out z) &&
                    TryDouble (ReadMember (value, Names.W), out w)) {
                quaternion = new[] { x, y, z, w };
                return true;
            }
            return false;
        }
    }
}
