using System;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Service.Attributes;
using UnityEngine;

namespace KRPC.Bridge.Actuators
{
    /// <summary>
    /// DynamicsExtV1: supplemental quantities captured in the SAME Pump and physics tick as
    /// DynamicsSnapshotV3, but transported separately so the v3 schema (and PDG2, which
    /// decodes it in the flight loop) stays byte-identical. Join the two by physics_tick
    /// and vessel persistent id.
    ///
    /// Born closed: nothing is captured until a client arms it with DynamicsExtEnableV1,
    /// and the capture stops by itself when that lease runs out. See docs/API.md.
    /// </summary>
    public static partial class ActuatorsService
    {
        /// <summary>
        /// Arms (enabled = true) or disarms the DynamicsExtV1 capture. The lease is
        /// 0.5-60 s of real time and must be renewed; returns DynamicsExtStatusV1.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> DynamicsExtEnableV1 (bool enabled, float leaseSeconds = 5f)
        {
            return DynamicsExt.Enable (enabled, leaseSeconds);
        }

        /// <summary>Latest DynamicsExtV1 frame of the active vessel. Empty while disarmed.</summary>
        [KRPCProcedure]
        public static IList<double> DynamicsExtSnapshotV1 ()
        {
            return DynamicsV3.Active.ext.ReadLatest ();
        }

        /// <summary>
        /// DynamicsExtV1 history of the active vessel newer than sinceTick, in the same
        /// six-value header + length-prefixed layout as DynamicsFramesV3.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> DynamicsExtFramesV1 (int sinceTick = 0, int maxFrames = 64)
        {
            return DynamicsV3.Active.ext.ReadHistory (sinceTick, maxFrames);
        }

        /// <summary>
        /// protocol, schema, armed, lease_remaining_s, latest_tick, history_count,
        /// history_capacity, capability_mask (active vessel).
        /// </summary>
        [KRPCProcedure]
        public static IList<double> DynamicsExtStatusV1 ()
        {
            return DynamicsExt.ReadStatus ();
        }
    }

    /// <summary>Per-channel DynamicsExtV1 ring and the one-tick memory of the INDI shadow.</summary>
    internal sealed class DynamicsExtState
    {
        internal readonly LinkedList<double[]> history = new LinkedList<double[]> ();
        internal double[] latest = new double[0];
        internal long latestTick;
        internal long latestCapabilities;

        internal bool havePrevious;
        internal long previousTick;
        internal long previousTopology;
        internal bool havePrevOmegaDot;
        internal readonly double[] prevOmegaDot = new double[3];
        internal bool havePrevPredB;
        internal readonly double[] prevPredB = new double[3];
        internal bool havePrevPredM;
        internal readonly double[] prevPredM = new double[3];
        internal bool havePrevControlTorque;
        internal readonly double[] prevControlTorque = new double[3];
        internal bool havePrevAuxTorque;
        internal readonly double[] prevAuxTorque = new double[3];
        internal bool havePrevAeroTorque;
        internal readonly double[] prevAeroTorque = new double[3];
        internal bool haveFilter;
        internal readonly double[] filteredResidual = new double[3];
        internal int validStreak;
        internal Dictionary<ulong, double> prevActuators = new Dictionary<ulong, double> ();
        internal Dictionary<ulong, double> curActuators = new Dictionary<ulong, double> ();

        internal bool HasData {
            get { return latest.Length > 0 || history.Count > 0 || havePrevious; }
        }

        internal void Reset ()
        {
            history.Clear ();
            latest = new double[0];
            latestTick = 0;
            latestCapabilities = 0;
            ResetMemory ();
        }

        /// <summary>Drops the one-tick memory but keeps the ring (tick gap, topology change).</summary>
        internal void ResetMemory ()
        {
            havePrevious = false;
            previousTick = 0;
            previousTopology = 0;
            havePrevOmegaDot = false;
            havePrevPredB = false;
            havePrevPredM = false;
            havePrevControlTorque = false;
            havePrevAuxTorque = false;
            havePrevAeroTorque = false;
            haveFilter = false;
            validStreak = 0;
            prevActuators.Clear ();
            curActuators.Clear ();
        }

        internal IList<double> ReadLatest ()
        {
            return new List<double> (latest);
        }

        internal IList<double> ReadHistory (int sinceTick, int maxFrames)
        {
            return DynamicsV3.ReadRing (history, DynamicsExt.ProtocolVersion,
                DynamicsExt.SchemaVersion, sinceTick, maxFrames);
        }
    }

    internal static class DynamicsExt
    {
        internal const int ProtocolVersion = 1;
        internal const int SchemaVersion = 1;
        internal const int HeaderStride = 19;
        internal const int StateStride = 52;
        internal const int EngineStride = 9;
        internal const int SurfaceStride = 13;
        internal const int RcsStride = 16;
        internal const int BStride = 9;
        internal const int ResponseStride = 12;
        internal const int HistoryCapacity = 1000; // same ~20 s as the v3 ring
        internal const double IndiFilterTauSeconds = 0.1;
        internal const int IndiLagTicks = 1;
        const float MinLeaseSeconds = 0.5f;
        const float MaxLeaseSeconds = 60f;
        const double DegToRad = Math.PI / 180.0;

        // Capability bits. Missing quantities remain NaN, never guessed.
        const long CapRotationalStateBody = 1L << 0;
        const long CapMoiKgM2 = 1L << 1;
        const long CapRcsRealized = 1L << 2;
        const long CapReactionWheelRealized = 1L << 3;
        const long CapControlTorque = 1L << 4;
        const long CapEngineAvailability = 1L << 5;
        const long CapControlSurfaces = 1L << 6;
        const long CapBMatrix = 1L << 7;
        const long CapIndiDeltaOmegaDot = 1L << 8;
        const long CapIndiPredictionB = 1L << 9;
        const long CapIndiPredictionM = 1L << 10;
        const long CapIndiPredictionAero = 1L << 11;
        const long CapIndiResidualB = 1L << 12;
        const long CapIndiResidualM = 1L << 13;
        const long CapResponseTracking = 1L << 14;

        // B row kinds.
        const int BKindGimbal = 1;
        const int BKindEngineThrust = 2;

        static bool enabled;
        static float expiresAt;

        // ModuleControlSurface keeps the realized angle and its target in PROTECTED fields
        // (decompiled KSP 1.12.5: deflection is rate-limited toward action, then written to
        // ctrlSurface.localRotation in the same update). Resolved once, read by reflection.
        static bool surfaceFieldsResolved;
        static FieldInfo surfaceDeflectionField;
        static FieldInfo surfaceActionField;

        /// <summary>Armed and lease still valid. Closes itself when the lease runs out.</summary>
        internal static bool Enabled {
            get {
                if (!enabled)
                    return false;
                if (Time.realtimeSinceStartup < expiresAt)
                    return true;
                enabled = false;
                ResponseTracker.Clear ();
                return false;
            }
        }

        internal static IList<double> Enable (bool on, float leaseSeconds)
        {
            if (float.IsNaN (leaseSeconds) || float.IsInfinity (leaseSeconds) ||
                    leaseSeconds < MinLeaseSeconds || leaseSeconds > MaxLeaseSeconds)
                throw new ArgumentOutOfRangeException (
                    nameof (leaseSeconds), "bail DynamicsExt attendu entre 0.5 et 60 s");
            if (on) {
                enabled = true;
                expiresAt = Time.realtimeSinceStartup + leaseSeconds;
            } else {
                enabled = false;
                expiresAt = 0f;
                ResponseTracker.Clear ();
            }
            return ReadStatus ();
        }

        internal static IList<double> ReadStatus ()
        {
            var ext = DynamicsV3.Active.ext;
            bool armed = Enabled;
            float remaining = armed ? Math.Max (0f, expiresAt - Time.realtimeSinceStartup) : 0f;
            return new List<double> {
                ProtocolVersion,
                SchemaVersion,
                armed ? 1.0 : 0.0,
                remaining,
                ext.latestTick,
                ext.history.Count,
                HistoryCapacity,
                ext.latestCapabilities
            };
        }

        /// <summary>
        /// One DynamicsExtV1 frame for one channel. v3 is the finished v3 state array of
        /// the same tick (body-frame omega/omega_dot, MOI, CoM, realized engine and live
        /// aero torques); nothing here writes game state.
        /// </summary>
        internal static void Capture (
            DynamicsChannel ch, Vessel vessel, long physicsTick, long topologyGeneration,
            double[] v3, bool isActiveChannel)
        {
            var x = ch.ext;
            double dt = Math.Max (1e-6, TimeWarp.fixedDeltaTime);
            bool consecutive = x.havePrevious && physicsTick == x.previousTick + 1 &&
                topologyGeneration == x.previousTopology;
            if (!consecutive)
                x.ResetMemory ();

            long caps = 0;
            var st = DynamicsV3.NewNaNArray (StateStride);
            var reference = vessel.ReferenceTransform;
            // Same CoM as the v3 levers of this tick (vessel.CurrentCoM, world).
            Vector3 com = new Vector3 (
                (float)v3 [DynamicsV3.StateCoM],
                (float)v3 [DynamicsV3.StateCoM + 1],
                (float)v3 [DynamicsV3.StateCoM + 2]);
            bool haveCom = Finite3 (v3, DynamicsV3.StateCoM);

            // ------------------------------------------------ rotational state (body)
            if (Finite3 (v3, DynamicsV3.StateAngularVelocity)) {
                Copy3 (v3, DynamicsV3.StateAngularVelocity, st, 0);
                caps |= CapRotationalStateBody;
            }
            bool haveOmegaDot = Finite3 (v3, DynamicsV3.StateAngularAcceleration);
            if (haveOmegaDot)
                Copy3 (v3, DynamicsV3.StateAngularAcceleration, st, 3);

            double[] inertia = null;
            if (Finite3 (v3, DynamicsV3.StateMoi) &&
                    v3 [DynamicsV3.StateMoi] > 0.0 &&
                    v3 [DynamicsV3.StateMoi + 1] > 0.0 &&
                    v3 [DynamicsV3.StateMoi + 2] > 0.0) {
                inertia = new[] {
                    v3 [DynamicsV3.StateMoi] * 1000.0,
                    v3 [DynamicsV3.StateMoi + 1] * 1000.0,
                    v3 [DynamicsV3.StateMoi + 2] * 1000.0
                };
                Set3 (st, 6, inertia);
                caps |= CapMoiKgM2;
            }

            // ------------------------------------------------------ actuator walk
            var engineRows = new List<double> ();
            var surfaceRows = new List<double> ();
            var rcsRows = new List<double> ();
            var bRows = new List<double> ();
            int engineRowCount = 0, surfaceRowCount = 0, rcsRowCount = 0, bRowCount = 0;

            var rcsForceWorld = new double[3];
            var rcsTorqueWorld = new double[3];
            bool rcsValid = reference != null && haveCom;
            var wheelTorqueBody = new double[3];
            bool wheelValid = true;
            double sumAvailable = 0.0, sumMax = 0.0, sumFlow = 0.0, sumRequested = 0.0;
            bool engineAvailabilityValid = true;
            var bDeltaTorqueBody = new double[3];
            bool bValid = reference != null && haveCom;
            bool bComplete = true;
            var partEngines = new List<ModuleEngines> ();
            x.curActuators.Clear ();
            ResolveSurfaceFields ();

            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                if (part == null)
                    continue;

                partEngines.Clear ();
                int engineOrdinal = 0;
                int gimbalOrdinal = 0;
                int rcsOrdinal = 0;
                int aeroSurfaceOrdinal = 0;
                int controlSurfaceOrdinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var engine = part.Modules [m] as ModuleEngines;
                    if (engine != null) {
                        partEngines.Add (engine);
                        double available = double.NaN, maxNow = double.NaN, ispNow = double.NaN;
                        try {
                            float pressureAtm = (float)part.staticPressureAtm;
                            // Stock "max thrust at this atmosphere" (the dV app's own
                            // function; no side effect). atmTemp is unused by the stock
                            // body, so the default is passed on purpose.
                            available = engine.MaxThrustOutputAtm (
                                true, true, pressureAtm, 310.0, part.atmDensity);
                            maxNow = engine.MaxThrustOutputAtm (
                                true, false, pressureAtm, 310.0, part.atmDensity);
                            ispNow = engine.atmosphereCurve.Evaluate (pressureAtm);
                        } catch {
                            engineAvailabilityValid = false;
                        }
                        double perThrottle = (engine.currentThrottle >= 0.01f &&
                                engine.finalThrust > 0f)
                            ? engine.finalThrust / (double)engine.currentThrottle
                            : double.NaN;
                        // CalculateThrust: massFlow = requested mass per tick (FuelUsage
                        // included), scaled by the fraction actually delivered.
                        double realizedFlow = (engine.EngineIgnited && engine.finalThrust > 0f)
                            ? engine.MassFlow () * ((double)engine.propellantReqMet * 0.01)
                              / dt * 1000.0
                            : 0.0;
                        double requestedFlow = (double)engine.requestedMassFlow * 1000.0;
                        engineRows.Add (part.flightID);
                        engineRows.Add (engineOrdinal);
                        engineRows.Add (available);
                        engineRows.Add (maxNow);
                        engineRows.Add (perThrottle);
                        engineRows.Add (realizedFlow);
                        engineRows.Add (requestedFlow);
                        engineRows.Add (ispNow);
                        engineRows.Add (part.staticPressureAtm);
                        engineRowCount++;
                        if (IsFinite (available))
                            sumAvailable += available;
                        if (IsFinite (maxNow))
                            sumMax += maxNow;
                        sumFlow += realizedFlow;
                        sumRequested += requestedFlow;

                        // B column of this engine's thrust: dM/dT in N.m per kN, body.
                        if (bValid) {
                            Vector3 column = EngineThrustColumn (engine, com);
                            Vector3 columnBody = reference.InverseTransformDirection (column);
                            double u = engine.finalThrust;
                            double du;
                            bool haveDu = DeltaActuator (x, Key (part.flightID,
                                engineOrdinal, BKindEngineThrust, 0), u, out du);
                            AddBRow (bRows, part.flightID, engineOrdinal,
                                BKindEngineThrust, 0, columnBody, u, du);
                            bRowCount++;
                            if (haveDu)
                                AddScaled (bDeltaTorqueBody, columnBody, du);
                            else
                                bComplete = false;
                        }
                        engineOrdinal++;
                    }

                    var gimbal = part.Modules [m] as ModuleGimbal;
                    if (gimbal != null) {
                        if (bValid)
                            GimbalColumns (x, part, gimbal, gimbalOrdinal, partEngines,
                                com, reference, bRows, bDeltaTorqueBody,
                                ref bRowCount, ref bComplete);
                        gimbalOrdinal++;
                    }

                    var rcs = part.Modules [m] as ModuleRCS;
                    if (rcs != null) {
                        RcsRow (part, rcs, rcsOrdinal++, com, reference, haveCom,
                            rcsRows, rcsForceWorld, rcsTorqueWorld, ref rcsValid);
                        rcsRowCount++;
                    }

                    var wheel = part.Modules [m] as ModuleReactionWheel;
                    if (wheel != null) {
                        try {
                            // ModuleReactionWheel.ActiveUpdate applies
                            // part.AddTorque(ReferenceTransform.rotation * -inputVector),
                            // inputVector in kN.m on the vessel (pitch, roll, yaw) = body axes.
                            if (wheel.moduleIsEnabled && wheel.operational &&
                                    wheel.wheelState == ModuleReactionWheel.WheelState.Active) {
                                wheelTorqueBody [0] -= wheel.inputVector.x * 1000.0;
                                wheelTorqueBody [1] -= wheel.inputVector.y * 1000.0;
                                wheelTorqueBody [2] -= wheel.inputVector.z * 1000.0;
                            }
                        } catch {
                            wheelValid = false;
                        }
                    }

                    var surface = part.Modules [m] as ModuleControlSurface;
                    if (surface != null) {
                        bool aero = surface is ModuleAeroSurface;
                        int ordinal = aero ? aeroSurfaceOrdinal++ : controlSurfaceOrdinal++;
                        SurfaceRow (part, surface, aero, ordinal, surfaceRows);
                        surfaceRowCount++;
                    }
                }
            }
            if (bValid)
                caps |= CapBMatrix;
            if (engineAvailabilityValid) {
                st [24] = sumAvailable;
                st [25] = sumMax;
                st [26] = sumFlow;
                st [27] = sumRequested;
                caps |= CapEngineAvailability;
            }
            if (surfaceRowCount > 0 && surfaceDeflectionField != null)
                caps |= CapControlSurfaces;

            double[] rcsTorqueBody = null;
            if (rcsValid) {
                double[] rcsForceBody = DynamicsV3.WorldToBodyVector (reference, rcsForceWorld);
                rcsTorqueBody = DynamicsV3.WorldToBodyVector (reference, rcsTorqueWorld);
                if (rcsForceBody != null && rcsTorqueBody != null) {
                    Set3 (st, 9, rcsForceBody);
                    Set3 (st, 12, rcsForceWorld);
                    Set3 (st, 15, rcsTorqueBody);
                    caps |= CapRcsRealized;
                } else {
                    rcsTorqueBody = null;
                }
            }
            if (wheelValid) {
                Set3 (st, 18, wheelTorqueBody);
                caps |= CapReactionWheelRealized;
            }

            // Realized control torque = engines (v3, r x F per nozzle) + RCS + wheels.
            double[] engineTorque = Finite3 (v3, DynamicsV3.StateEngineTorqueBody)
                ? Slice3 (v3, DynamicsV3.StateEngineTorqueBody) : null;
            double[] auxTorque = (rcsTorqueBody != null && wheelValid)
                ? new[] {
                    rcsTorqueBody [0] + wheelTorqueBody [0],
                    rcsTorqueBody [1] + wheelTorqueBody [1],
                    rcsTorqueBody [2] + wheelTorqueBody [2]
                } : null;
            double[] controlTorque = null;
            if (engineTorque != null && auxTorque != null) {
                controlTorque = new[] {
                    engineTorque [0] + auxTorque [0],
                    engineTorque [1] + auxTorque [1],
                    engineTorque [2] + auxTorque [2]
                };
                Set3 (st, 21, controlTorque);
                caps |= CapControlTorque;
            }
            double[] aeroTorque = Finite3 (v3, DynamicsV3.StateAeroTorqueBody)
                ? Slice3 (v3, DynamicsV3.StateAeroTorqueBody) : null;

            // ------------------------------------- INDI shadow (plan GNC 10.8, point 7)
            // Delta omega_dot measured at tick k, predictions for the increment captured at
            // tick k, residual of tick k against the prediction of tick k-1 (lag 1: the
            // actuator state read in FixedUpdate k acts during the physics step after it).
            double[] deltaOmegaDot = null;
            if (haveOmegaDot && x.havePrevOmegaDot) {
                deltaOmegaDot = new[] {
                    v3 [DynamicsV3.StateAngularAcceleration] - x.prevOmegaDot [0],
                    v3 [DynamicsV3.StateAngularAcceleration + 1] - x.prevOmegaDot [1],
                    v3 [DynamicsV3.StateAngularAcceleration + 2] - x.prevOmegaDot [2]
                };
                Set3 (st, 28, deltaOmegaDot);
                caps |= CapIndiDeltaOmegaDot;
            }

            double[] predB = null;
            if (inertia != null && bValid && bComplete && auxTorque != null &&
                    x.havePrevAuxTorque) {
                predB = new double[3];
                for (int k = 0; k < 3; k++)
                    predB [k] = (bDeltaTorqueBody [k] + auxTorque [k] - x.prevAuxTorque [k])
                        / inertia [k];
                Set3 (st, 31, predB);
                caps |= CapIndiPredictionB;
            }
            double[] predM = null;
            if (inertia != null && controlTorque != null && x.havePrevControlTorque) {
                predM = new double[3];
                for (int k = 0; k < 3; k++)
                    predM [k] = (controlTorque [k] - x.prevControlTorque [k]) / inertia [k];
                Set3 (st, 34, predM);
                caps |= CapIndiPredictionM;
            }
            if (inertia != null && aeroTorque != null && x.havePrevAeroTorque) {
                var predAero = new double[3];
                for (int k = 0; k < 3; k++)
                    predAero [k] = (aeroTorque [k] - x.prevAeroTorque [k]) / inertia [k];
                Set3 (st, 37, predAero);
                caps |= CapIndiPredictionAero;
            }

            bool residualB = false;
            if (deltaOmegaDot != null && x.havePrevPredB) {
                var r = new double[3];
                for (int k = 0; k < 3; k++)
                    r [k] = deltaOmegaDot [k] - x.prevPredB [k];
                Set3 (st, 40, r);
                caps |= CapIndiResidualB;
                residualB = true;
                // Same first-order H(z) on both terms == H applied to their difference.
                double alpha = dt / (IndiFilterTauSeconds + dt);
                if (!x.haveFilter) {
                    Copy3 (r, 0, x.filteredResidual, 0);
                    x.haveFilter = true;
                } else {
                    for (int k = 0; k < 3; k++)
                        x.filteredResidual [k] += alpha * (r [k] - x.filteredResidual [k]);
                }
                Set3 (st, 46, x.filteredResidual);
            }
            if (deltaOmegaDot != null && x.havePrevPredM) {
                var r = new double[3];
                for (int k = 0; k < 3; k++)
                    r [k] = deltaOmegaDot [k] - x.prevPredM [k];
                Set3 (st, 43, r);
                caps |= CapIndiResidualM;
            }
            x.validStreak = residualB ? x.validStreak + 1 : 0;
            st [49] = IndiFilterTauSeconds;
            st [50] = IndiLagTicks;
            st [51] = x.validStreak;

            // ---------------------------------------------- response to the last v2 frame
            var responseRows = new List<double> ();
            int responseRowCount = 0;
            if (isActiveChannel && ResponseTracker.Follows (vessel.id)) {
                ResponseTracker.Update (physicsTick);
                responseRowCount = ResponseTracker.AppendRows (responseRows);
                if (ResponseTracker.Tracking)
                    caps |= CapResponseTracking;
            }

            // ---------------------------------------------------------------- frame
            var frame = new List<double> (
                HeaderStride + StateStride + engineRows.Count + surfaceRows.Count +
                rcsRows.Count + bRows.Count + responseRows.Count) {
                ProtocolVersion,
                SchemaVersion,
                physicsTick,
                Planetarium.GetUniversalTime (),
                TimeWarp.fixedDeltaTime,
                vessel.persistentId,
                topologyGeneration,
                caps,
                StateStride,
                engineRowCount,
                EngineStride,
                surfaceRowCount,
                SurfaceStride,
                rcsRowCount,
                RcsStride,
                bRowCount,
                BStride,
                responseRowCount,
                ResponseStride
            };
            frame.AddRange (st);
            frame.AddRange (engineRows);
            frame.AddRange (surfaceRows);
            frame.AddRange (rcsRows);
            frame.AddRange (bRows);
            frame.AddRange (responseRows);

            x.latest = frame.ToArray ();
            x.latestTick = physicsTick;
            x.latestCapabilities = caps;
            x.history.AddLast (x.latest);
            while (x.history.Count > HistoryCapacity)
                x.history.RemoveFirst ();

            // ------------------------------------------------------- one-tick memory
            x.havePrevious = true;
            x.previousTick = physicsTick;
            x.previousTopology = topologyGeneration;
            x.havePrevOmegaDot = haveOmegaDot;
            if (haveOmegaDot)
                Copy3 (v3, DynamicsV3.StateAngularAcceleration, x.prevOmegaDot, 0);
            x.havePrevPredB = predB != null;
            if (predB != null)
                Copy3 (predB, 0, x.prevPredB, 0);
            x.havePrevPredM = predM != null;
            if (predM != null)
                Copy3 (predM, 0, x.prevPredM, 0);
            x.havePrevControlTorque = controlTorque != null;
            if (controlTorque != null)
                Copy3 (controlTorque, 0, x.prevControlTorque, 0);
            x.havePrevAuxTorque = auxTorque != null;
            if (auxTorque != null)
                Copy3 (auxTorque, 0, x.prevAuxTorque, 0);
            x.havePrevAeroTorque = aeroTorque != null;
            if (aeroTorque != null)
                Copy3 (aeroTorque, 0, x.prevAeroTorque, 0);
            var swap = x.prevActuators;
            x.prevActuators = x.curActuators;
            x.curActuators = swap;
            x.curActuators.Clear ();
        }

        // ------------------------------------------------------------------ B matrix

        /// <summary>
        /// dM/dT of one engine, N.m per kN of finalThrust, world frame, about com. Same
        /// multiplier split and reaction-force sign as the v3 realized wrench.
        /// </summary>
        static Vector3 EngineThrustColumn (ModuleEngines engine, Vector3 com)
        {
            var column = Vector3.zero;
            int count = engine.thrustTransforms == null ? 0 : engine.thrustTransforms.Count;
            double weightSum = MultiplierWeightSum (engine, count);
            if (weightSum <= 1e-12)
                return column;
            for (int t = 0; t < count; t++) {
                var transform = engine.thrustTransforms [t];
                if (transform == null)
                    continue;
                float weight = (float)(Multiplier (engine, t) / weightSum);
                Vector3 direction = transform.forward.normalized;
                Vector3 lever = transform.position - com;
                // Force on the vessel is -forward per unit thrust; 1 kN = 1000 N.
                column += (1000f * weight) * DynamicsV3.Cross3 (lever, -1f * direction);
            }
            return column;
        }

        /// <summary>
        /// dM/d(delta_x) and dM/d(delta_y) of one gimbal module, N.m per degree, body.
        ///
        /// ModuleGimbal (stock FixedUpdate, and the bridge lease) sets
        ///   t.localRotation = initRots[i] * AngleAxis(dx, xMult*right)
        ///                     * AngleAxis(dy, yMult*(flipYZ ? forward : up))
        /// so a change of dx is a WORLD rotation about ax = Q0*(xMult*right), with
        /// Q0 = parent.rotation*initRots[i], and a change of dy a world rotation about
        /// ay = Q0*AngleAxis(dx, xMult*right)*(yMult*second). Every thrust transform that
        /// is t or a child of t turns with it: d(dir) = a x dir, d(pos) = a x (pos - t.pos).
        /// </summary>
        static void GimbalColumns (
            DynamicsExtState x, Part part, ModuleGimbal gimbal, int gimbalOrdinal,
            List<ModuleEngines> partEngines, Vector3 com, Transform reference,
            List<double> bRows, double[] bDeltaTorqueBody,
            ref int bRowCount, ref bool bComplete)
        {
            if (gimbal.gimbalTransforms == null || gimbal.initRots == null)
                return;
            // Realized deflection. A leased gimbal writes actuationLocal every tick; a
            // locked stock gimbal is reset to initRots, i.e. zero.
            bool leased = ActuatorsAddon.GimbalIsLeased (gimbal);
            float dx = (!leased && gimbal.gimbalLock) ? 0f : gimbal.actuationLocal.x;
            float dy = (!leased && gimbal.gimbalLock) ? 0f : gimbal.actuationLocal.y;

            Vector3 columnX = Vector3.zero;
            Vector3 columnY = Vector3.zero;
            int count = Math.Min (gimbal.gimbalTransforms.Count, gimbal.initRots.Count);
            for (int i = 0; i < count; i++) {
                var pivot = gimbal.gimbalTransforms [i];
                if (pivot == null)
                    continue;
                Quaternion parentRotation = pivot.parent != null
                    ? pivot.parent.rotation : Quaternion.identity;
                Quaternion q0 = parentRotation * gimbal.initRots [i];
                Vector3 xAxisLocal = gimbal.xMult * Vector3.right;
                Vector3 second = gimbal.flipYZ ? Vector3.forward : Vector3.up;
                Vector3 axisX = (q0 * xAxisLocal).normalized;
                Vector3 axisY = (q0 * Quaternion.AngleAxis (dx, xAxisLocal)
                                 * (gimbal.yMult * second)).normalized;

                for (int e = 0; e < partEngines.Count; e++) {
                    var engine = partEngines [e];
                    int transformCount = engine.thrustTransforms == null
                        ? 0 : engine.thrustTransforms.Count;
                    double weightSum = MultiplierWeightSum (engine, transformCount);
                    if (weightSum <= 1e-12)
                        continue;
                    for (int t = 0; t < transformCount; t++) {
                        var nozzle = engine.thrustTransforms [t];
                        if (nozzle == null || !nozzle.IsChildOf (pivot))
                            continue;
                        float thrustN = (float)(engine.finalThrust * 1000.0 *
                            (Multiplier (engine, t) / weightSum));
                        Vector3 direction = nozzle.forward.normalized;
                        Vector3 force = -thrustN * direction;
                        Vector3 lever = nozzle.position - com;
                        Vector3 offset = nozzle.position - pivot.position;
                        columnX += GimbalTorqueDerivative (axisX, direction, force, lever,
                            offset, thrustN);
                        columnY += GimbalTorqueDerivative (axisY, direction, force, lever,
                            offset, thrustN);
                    }
                }
            }

            Vector3 columnXBody = reference.InverseTransformDirection (
                (float)DegToRad * columnX);
            Vector3 columnYBody = reference.InverseTransformDirection (
                (float)DegToRad * columnY);
            double du;
            bool haveDx = DeltaActuator (x, Key (part.flightID, gimbalOrdinal, BKindGimbal, 0),
                dx, out du);
            AddBRow (bRows, part.flightID, gimbalOrdinal, BKindGimbal, 0, columnXBody, dx, du);
            if (haveDx)
                AddScaled (bDeltaTorqueBody, columnXBody, du);
            bool haveDy = DeltaActuator (x, Key (part.flightID, gimbalOrdinal, BKindGimbal, 1),
                dy, out du);
            AddBRow (bRows, part.flightID, gimbalOrdinal, BKindGimbal, 1, columnYBody, dy, du);
            if (haveDy)
                AddScaled (bDeltaTorqueBody, columnYBody, du);
            bRowCount += 2;
            if (!haveDx || !haveDy)
                bComplete = false;
        }

        /// <summary>d(r x F)/d(theta) for a rotation of the nozzle about axis, per radian.</summary>
        static Vector3 GimbalTorqueDerivative (
            Vector3 axis, Vector3 direction, Vector3 force, Vector3 lever, Vector3 offset,
            float thrustN)
        {
            Vector3 dForce = -thrustN * DynamicsV3.Cross3 (axis, direction);
            Vector3 dLever = DynamicsV3.Cross3 (axis, offset);
            return DynamicsV3.Cross3 (dLever, force) + DynamicsV3.Cross3 (lever, dForce);
        }

        static double MultiplierWeightSum (ModuleEngines engine, int count)
        {
            double sum = 0.0;
            for (int t = 0; t < count; t++)
                sum += Math.Abs (Multiplier (engine, t));
            return sum;
        }

        static double Multiplier (ModuleEngines engine, int t)
        {
            if (engine.thrustTransformMultipliers != null &&
                    t < engine.thrustTransformMultipliers.Count)
                return engine.thrustTransformMultipliers [t];
            return 1.0;
        }

        static ulong Key (uint flightId, int ordinal, int kind, int axis)
        {
            return ((ulong)flightId << 32) | ((ulong)(uint)ordinal << 8) |
                   ((ulong)(uint)kind << 4) | (ulong)(uint)axis;
        }

        /// <summary>Stores u for the next tick and returns u - u_previous when known.</summary>
        static bool DeltaActuator (DynamicsExtState x, ulong key, double u, out double du)
        {
            x.curActuators [key] = u;
            double previous;
            if (x.prevActuators.TryGetValue (key, out previous) && IsFinite (u) &&
                    IsFinite (previous)) {
                du = u - previous;
                return true;
            }
            du = double.NaN;
            return false;
        }

        static void AddBRow (
            List<double> rows, uint flightId, int ordinal, int kind, int axis,
            Vector3 columnBody, double u, double du)
        {
            rows.Add (flightId);
            rows.Add (ordinal);
            rows.Add (kind);
            rows.Add (axis);
            rows.Add (columnBody.x);
            rows.Add (columnBody.y);
            rows.Add (columnBody.z);
            rows.Add (u);
            rows.Add (du);
        }

        static void AddScaled (double[] target, Vector3 column, double scale)
        {
            target [0] += column.x * scale;
            target [1] += column.y * scale;
            target [2] += column.z * scale;
        }

        // --------------------------------------------------------------------- RCS

        /// <summary>
        /// ModuleRCS.FixedUpdate (KSP 1.12.5, also inherited by ModuleRCSFX) resets
        /// thrustForces[] every tick, then applies AddForceAtPosition(-v * thrustForces[i],
        /// thruster position) with v = useZaxis ? forward : up, in kN - except for a module
        /// marked isJustForShow, which fills thrustForces but applies nothing.
        /// </summary>
        static void RcsRow (
            Part part, ModuleRCS rcs, int ordinal, Vector3 com, Transform reference,
            bool haveCom, List<double> rows, double[] forceWorldTotal,
            double[] torqueWorldTotal, ref bool valid)
        {
            var force = Vector3.zero;
            var torque = Vector3.zero;
            double thrustSum = 0.0;
            int nozzles = 0;
            try {
                int count = rcs.thrusterTransforms == null ? 0 : rcs.thrusterTransforms.Count;
                int forces = rcs.thrustForces == null ? 0 : rcs.thrustForces.Length;
                nozzles = count;
                for (int i = 0; i < Math.Min (count, forces); i++) {
                    float thrust = rcs.thrustForces [i];
                    var thruster = rcs.thrusterTransforms [i];
                    if (thruster == null || !(thrust > 0f))
                        continue;
                    thrustSum += thrust;
                    if (rcs.isJustForShow)
                        continue;
                    Vector3 axis = rcs.useZaxis ? thruster.forward : thruster.up;
                    Vector3 f = (-1000f * thrust) * axis;
                    force += f;
                    torque += DynamicsV3.Cross3 (thruster.position - com, f);
                }
            } catch {
                valid = false;
            }
            Vector3 forceBody = reference != null
                ? reference.InverseTransformDirection (force) : Vector3.zero;
            Vector3 torqueBody = reference != null
                ? reference.InverseTransformDirection (torque) : Vector3.zero;
            forceWorldTotal [0] += force.x;
            forceWorldTotal [1] += force.y;
            forceWorldTotal [2] += force.z;
            torqueWorldTotal [0] += torque.x;
            torqueWorldTotal [1] += torque.y;
            torqueWorldTotal [2] += torque.z;

            rows.Add (part.flightID);
            rows.Add (ordinal);
            rows.Add (rcs.moduleIsEnabled ? 1.0 : 0.0);
            rows.Add (rcs.rcsEnabled ? 1.0 : 0.0);
            rows.Add (rcs.rcs_active ? 1.0 : 0.0);
            rows.Add (rcs.isJustForShow ? 1.0 : 0.0);
            rows.Add (nozzles);
            rows.Add (thrustSum);
            rows.Add (rcs.thrusterPower);
            rows.Add (rcs.realISP);
            if (reference == null || !haveCom) {
                for (int n = 0; n < 6; n++)
                    rows.Add (double.NaN);
            } else {
                rows.Add (forceBody.x);
                rows.Add (forceBody.y);
                rows.Add (forceBody.z);
                rows.Add (torqueBody.x);
                rows.Add (torqueBody.y);
                rows.Add (torqueBody.z);
            }
        }

        // ---------------------------------------------------------- control surfaces

        static void ResolveSurfaceFields ()
        {
            if (surfaceFieldsResolved)
                return;
            surfaceFieldsResolved = true;
            const BindingFlags flags = BindingFlags.Instance | BindingFlags.Public |
                BindingFlags.NonPublic;
            try {
                surfaceDeflectionField = typeof (ModuleControlSurface).GetField ("deflection", flags);
                surfaceActionField = typeof (ModuleControlSurface).GetField ("action", flags);
            } catch {
                surfaceDeflectionField = null;
                surfaceActionField = null;
            }
        }

        static void SurfaceRow (
            Part part, ModuleControlSurface surface, bool aero, int ordinal, List<double> rows)
        {
            rows.Add (part.flightID);
            rows.Add (aero ? 1.0 : 2.0);
            rows.Add (ordinal);
            rows.Add (ReadFloatField (surfaceDeflectionField, surface));
            rows.Add (ReadFloatField (surfaceActionField, surface));
            rows.Add (surface.ctrlSurfaceRange);
            rows.Add (surface.authorityLimiter);
            rows.Add (surface.deploy ? 1.0 : 0.0);
            rows.Add (surface.deployAngle);
            double currentDeployAngle = double.NaN;
            try {
                currentDeployAngle = surface.currentDeployAngle;
            } catch { }
            rows.Add (currentDeployAngle);
            rows.Add (surface.actuatorSpeed);
            rows.Add (surface.useExponentialSpeed ? 1.0 : 0.0);
            rows.Add ((surface.ignorePitch ? 1.0 : 0.0) + (surface.ignoreYaw ? 2.0 : 0.0) +
                      (surface.ignoreRoll ? 4.0 : 0.0));
        }

        static double ReadFloatField (FieldInfo field, object target)
        {
            if (field == null)
                return double.NaN;
            try {
                object value = field.GetValue (target);
                if (value is float)
                    return (float)value;
                if (value is double)
                    return (double)value;
            } catch { }
            return double.NaN;
        }

        // ------------------------------------------------------------------ helpers

        static bool IsFinite (double value)
        {
            return !double.IsNaN (value) && !double.IsInfinity (value);
        }

        static bool Finite3 (double[] values, int offset)
        {
            return values != null && values.Length >= offset + 3 &&
                IsFinite (values [offset]) && IsFinite (values [offset + 1]) &&
                IsFinite (values [offset + 2]);
        }

        static double[] Slice3 (double[] values, int offset)
        {
            return new[] { values [offset], values [offset + 1], values [offset + 2] };
        }

        static void Copy3 (double[] source, int sourceOffset, double[] target, int targetOffset)
        {
            target [targetOffset] = source [sourceOffset];
            target [targetOffset + 1] = source [sourceOffset + 1];
            target [targetOffset + 2] = source [sourceOffset + 2];
        }

        static void Set3 (double[] target, int offset, double[] value)
        {
            target [offset] = value [0];
            target [offset + 1] = value [1];
            target [offset + 2] = value [2];
        }
    }

    /// <summary>
    /// First realized response of each actuator to the last applied v2 control frame.
    ///
    /// Baselines are sampled BEFORE the frame's commands are written (same Pump), then
    /// every captured tick compares the realized value with its baseline: the first tick
    /// whose value moved by more than the threshold is that actuator's response tick.
    /// Engine: currentThrottle (0-1) against the command percentage/100. Gimbal: the
    /// realized actuationLocal x/y (deg) against the commanded x/y. A tick equal to the
    /// apply tick means the change was already visible in the same FixedUpdate.
    /// Only armed with DynamicsExtV1; it never writes game state.
    /// </summary>
    internal static class ResponseTracker
    {
        const double EngineThreshold = 1e-4;
        const double GimbalThresholdDegrees = 1e-4;

        sealed class Entry
        {
            internal int Kind;              // 1 engine, 2 gimbal
            internal ModuleEngines Engine;
            internal ModuleGimbal Gimbal;
            internal uint FlightId;
            internal int Ordinal;
            internal double BaselineX, CommandX, RealizedX;
            internal double BaselineY = double.NaN, CommandY = double.NaN, RealizedY = double.NaN;
            internal double FirstResponseTick = double.NaN;
        }

        static readonly List<Entry> entries = new List<Entry> ();
        static Guid vesselId = Guid.Empty;
        static int sequence;
        static long applyTick;
        static bool pending;

        internal static bool Tracking {
            get { return entries.Count > 0 && applyTick > 0; }
        }

        /// <summary>Rows belong to the vessel the frame was applied to, and to no other.</summary>
        internal static bool Follows (Guid id)
        {
            return entries.Count > 0 && vesselId != Guid.Empty && vesselId == id;
        }

        internal static void Clear ()
        {
            entries.Clear ();
            vesselId = Guid.Empty;
            sequence = 0;
            applyTick = 0;
            pending = false;
        }

        /// <summary>Baselines of every actuator of a frame about to be applied.</summary>
        internal static void BeforeApply (
            Guid frameVesselId, int frameSequence,
            List<ActuatorsAddon.EngineFrameCommand> engines,
            List<ActuatorsAddon.GimbalFrameCommand> gimbals)
        {
            Clear ();
            vesselId = frameVesselId;
            sequence = frameSequence;
            pending = true;
            for (int i = 0; engines != null && i < engines.Count; i++) {
                var engine = engines [i].Engine;
                if (engine == null)
                    continue;
                entries.Add (new Entry {
                    Kind = 1,
                    Engine = engine,
                    FlightId = engine.part == null ? 0u : engine.part.flightID,
                    Ordinal = ModuleOrdinal (engine),
                    BaselineX = engine.currentThrottle,
                    CommandX = engines [i].Percentage * 0.01,
                    RealizedX = engine.currentThrottle
                });
            }
            for (int i = 0; gimbals != null && i < gimbals.Count; i++) {
                var gimbal = gimbals [i].Gimbal;
                if (gimbal == null)
                    continue;
                entries.Add (new Entry {
                    Kind = 2,
                    Gimbal = gimbal,
                    FlightId = gimbal.part == null ? 0u : gimbal.part.flightID,
                    Ordinal = ModuleOrdinal (gimbal),
                    BaselineX = gimbal.actuationLocal.x,
                    CommandX = gimbals [i].XDegrees,
                    RealizedX = gimbal.actuationLocal.x,
                    BaselineY = gimbal.actuationLocal.y,
                    CommandY = gimbals [i].YDegrees,
                    RealizedY = gimbal.actuationLocal.y
                });
            }
        }

        internal static void AfterApply (long tick)
        {
            if (!pending)
                return;
            applyTick = tick;
            pending = false;
        }

        internal static void Update (long tick)
        {
            if (applyTick <= 0 || tick < applyTick)
                return;
            for (int i = 0; i < entries.Count; i++) {
                var e = entries [i];
                try {
                    if (e.Kind == 1) {
                        if (e.Engine == null)
                            continue;
                        e.RealizedX = e.Engine.currentThrottle;
                        if (double.IsNaN (e.FirstResponseTick) &&
                                Math.Abs (e.RealizedX - e.BaselineX) > EngineThreshold)
                            e.FirstResponseTick = tick;
                    } else {
                        if (e.Gimbal == null)
                            continue;
                        e.RealizedX = e.Gimbal.actuationLocal.x;
                        e.RealizedY = e.Gimbal.actuationLocal.y;
                        if (double.IsNaN (e.FirstResponseTick) &&
                                (Math.Abs (e.RealizedX - e.BaselineX) > GimbalThresholdDegrees ||
                                 Math.Abs (e.RealizedY - e.BaselineY) > GimbalThresholdDegrees))
                            e.FirstResponseTick = tick;
                    }
                } catch {
                    // A destroyed module simply stops updating its row.
                }
            }
        }

        internal static int AppendRows (List<double> rows)
        {
            if (applyTick <= 0)
                return 0;
            for (int i = 0; i < entries.Count; i++) {
                var e = entries [i];
                rows.Add (e.Kind);
                rows.Add (e.FlightId);
                rows.Add (e.Ordinal);
                rows.Add (sequence);
                rows.Add (applyTick);
                rows.Add (e.FirstResponseTick);
                rows.Add (e.BaselineX);
                rows.Add (e.CommandX);
                rows.Add (e.RealizedX);
                rows.Add (e.BaselineY);
                rows.Add (e.CommandY);
                rows.Add (e.RealizedY);
            }
            return entries.Count;
        }

        /// <summary>Ordinal of a module among the modules of the same type on its part.</summary>
        static int ModuleOrdinal (PartModule module)
        {
            var part = module.part;
            if (part == null)
                return -1;
            bool engine = module is ModuleEngines;
            int ordinal = 0;
            for (int m = 0; m < part.Modules.Count; m++) {
                var candidate = part.Modules [m];
                if (ReferenceEquals (candidate, module))
                    return ordinal;
                if (engine ? candidate is ModuleEngines : candidate is ModuleGimbal)
                    ordinal++;
            }
            return -1;
        }
    }
}
