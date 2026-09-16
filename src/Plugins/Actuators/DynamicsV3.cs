using System;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Service.Attributes;
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
        internal const int SchemaVersion = 1;
        internal const int HeaderStride = 22;
        internal const int StateStride = 67;
        internal const int EngineStride = 20;
        internal const int GimbalStride = 19;
        internal const int TransformStride = 10;
        internal const int HistoryCapacity = 300; // ~6 s at 50 Hz

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

        static readonly LinkedList<double[]> history = new LinkedList<double[]> ();
        static readonly Dictionary<string, MemberInfo> memberCache =
            new Dictionary<string, MemberInfo> (StringComparer.Ordinal);
        static readonly Dictionary<string, MethodInfo> methodCache =
            new Dictionary<string, MethodInfo> (StringComparer.Ordinal);

        static double[] latest = new double[0];
        static long latestCapabilities;
        static long latestTick;
        static int latestAppliedSequence;
        static long latestAppliedTick;
        static int latestResult;

        static bool havePrevious;
        static long previousTick;
        static double[] previousSurfaceVelocity = new double[3];
        static double[] previousOrbitalVelocity = new double[3];
        static double[] previousAngularVelocity = new double[3];

        internal static void Reset ()
        {
            history.Clear ();
            latest = new double[0];
            latestCapabilities = 0;
            latestTick = 0;
            latestAppliedSequence = 0;
            latestAppliedTick = 0;
            latestResult = 0;
            havePrevious = false;
            previousTick = 0;
        }

        internal static IList<double> ReadLatest ()
        {
            return new List<double> (latest);
        }

        internal static IList<double> ReadStatus ()
        {
            return new List<double> {
                ProtocolVersion,
                SchemaVersion,
                latestTick,
                history.Count,
                HistoryCapacity,
                latestCapabilities,
                latestAppliedSequence,
                latestAppliedTick,
                latestResult
            };
        }

        internal static IList<double> ReadHistory (int sinceTick, int maxFrames)
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
                ProtocolVersion,
                SchemaVersion,
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
            if (havePrevious && physicsTick == previousTick + 1) {
                if ((capabilities & CapSurfaceVelocity) != 0) {
                    Set3 (state, 9, DifferenceOverDt (
                        surfaceVelocity, previousSurfaceVelocity, dt));
                    capabilities |= CapSurfaceAcceleration;
                }
                if ((capabilities & CapOrbitalVelocity) != 0) {
                    Set3 (state, 12, DifferenceOverDt (
                        orbitalVelocity, previousOrbitalVelocity, dt));
                    capabilities |= CapOrbitalAcceleration;
                }
            }

            double[] rotation;
            if (TryRotation (vessel, out rotation)) {
                Set4 (state, 15, rotation);
                capabilities |= CapRotation;
            }

            double[] angularVelocity;
            if (TryReadVectorMember (vessel,
                    new[] { "angularVelocity", "angular_velocity" },
                    out angularVelocity)) {
                Set3 (state, 19, angularVelocity);
                capabilities |= CapAngularVelocity;
                if (havePrevious && physicsTick == previousTick + 1) {
                    Set3 (state, 22, DifferenceOverDt (
                        angularVelocity, previousAngularVelocity, dt));
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

            // --------------------------------------------------- actionuator state
            var engines = new List<double> ();
            var gimbals = new List<double> ();
            var transforms = new List<double> ();
            int engineCount = 0;
            int gimbalCount = 0;
            int transformCount = 0;
            var reference = vessel.ReferenceTransform;

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
                history.Count,
                0.0 // capture phase: pre-integration FixedUpdate
            };
            frame.AddRange (state);
            frame.AddRange (engines);
            frame.AddRange (gimbals);
            frame.AddRange (transforms);

            latest = frame.ToArray ();
            latestCapabilities = capabilities;
            latestTick = physicsTick;
            latestAppliedSequence = lastAppliedSequence;
            latestAppliedTick = lastAppliedTick;
            latestResult = lastResult;
            history.AddLast (latest);
            while (history.Count > HistoryCapacity)
                history.RemoveFirst ();

            havePrevious = true;
            previousTick = physicsTick;
            if (surfaceVelocity != null)
                Copy3 (surfaceVelocity, previousSurfaceVelocity);
            if (orbitalVelocity != null)
                Copy3 (orbitalVelocity, previousOrbitalVelocity);
            if (angularVelocity != null)
                Copy3 (angularVelocity, previousAngularVelocity);
        }

        static double[] NewNaNArray (int count)
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
