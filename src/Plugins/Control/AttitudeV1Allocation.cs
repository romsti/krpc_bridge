using System;
using System.Collections.Generic;
using System.Diagnostics;
using KRPC.Bridge.Core;
using KRPC.Bridge.Model;
using UnityEngine;
using SCReferenceFrame = KRPC.SpaceCenter.Services.ReferenceFrame;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// AttitudeV1, (option 4096 of AttitudeOptionsV1, BORN CLOSED): the engine-by-engine
    /// allocation IN SHADOW. The maths is EngineAllocation.cs (Model, pure); this file reads the vessel and
    /// publishes. It WRITES NO ACTUATOR and changes nothing the loop does: without the option nothing here
    /// runs; with it, it reads the law's input (before the lateral-force layer shifts the gimbal bounds)
    /// and the law's output, solves on its own copies, and publishes a named block
    /// (attitude_allocation_v1) and a 1000-frame ring (attitude_allocation_frames_v1).
    ///
    /// Per step, in TickEntry:
    ///   * AllocationPrepare, after the law's columns are built and before the lateral-force layer: the
    ///     engines the differential may drive (CollectMotifEngines' rule: slope, nozzle centroid, torque
    ///     per newton and thrust direction at the nozzles' realized orientation, the realized command
    ///     u0 = (currentThrottle - throttleMin) / ((1 - throttleMin) tp), the stock spool's Lerp factors),
    ///     the usable gimbal modules (the law's torque columns, realized deflection and box, their force
    ///     columns as the translation layer reads them, their Lerp factor), the ctrlState pitch / roll /
    ///     yaw (the law's columns, the served command), the RCS translation X / Y (AddRcsTranslationColumns),
    ///     the mass, thrust and main throttle, the realized lateral force, and the live lateral demand;
    ///   * AllocationSolve, after the law: the torque target M_d - M (the law's allocation target), the
    ///     axial force D asks (every engine at the served main throttle, realized at the horizon), the
    ///     lateral force of the live translation request (the gap the nose does not give, as
    ///     TranslationGap, read only), the cascade, the commands; then ONE direction of the free authority
    ///     or ONE entry of the model table, every second step, alternately (round robin: the four free directions
    ///     every sixteen steps, the table of LevelGrid + 1 throttle levels every 16 (LevelGrid + 1) steps, 1.6 s);
    ///     the block, the frame.
    /// Capability bit 23 on a step it solved.
    ///
    /// STEP 2 (option 8192, implies 4096, BORN CLOSED): the allocation WRITES, in place of the law's allocation AND of
    /// the lateral-force layer, on every step where the pitch/yaw channel writes and the allocation's own solve of the
    /// step is sound (AllocationWriteRule in EngineAllocation.cs). Its solve is the shadow's (horizon H, the realized
    /// change of every actuator at the largest time constant of those present) with what the write needs: the ctrlState
    /// axes the law may not write held where they are (frozen by option 1, the roll when the loop does not write it), the
    /// RCS dead zone in the command (ModuleRCS zeroes a component under EPSILON: the RCS-only channels start from what
    /// the RCS realizes, a command inside the zone is snapped to 0 or +-EPSILON and held, the others take the rest:
    /// SolveDeadZone), and the free lateral authority along the live demand (every second step, steady state, the
    /// attitude held) published for the PF in place of an earlier change's. The write:
    ///   * at Earlyish (AllocationDecide, AllocationCommandGimbals): every gimbal module of the allocation leased at its
    ///     command, through the loop's own authority and pivots, as the law leases them; the ctrlState pitch / roll /
    ///     yaw to be written by the fly-by-wire link (the law's own path: frozen axes, roll channel, surfaces); the
    ///     layer told it did not write (no common, no feed-forward, no throttle or X/Y of its own: AllocationTakesLayer);
    ///   * at the fly-by-wire (AllocationFlyByWire, after the main throttle of option 512, before the layer's writes):
    ///     the throttle of every engine it drives, independent under the loop's lease, at main + kappa (c - main), the
    ///     main throttle served THIS step and kappa keeping every engine in [0, 1] (the layer's WriteScale: a cut main
    ///     throttle cuts them all); the RCS translation ctrlState.X / Y. The layer's leases it does not take are given
    ///     back. On a step it does not write, every lease and X / Y of its own is given back first, then the law and the
    ///     layer write as without the option.
    /// A lease refused, a solve stale, not converged or saturated, a gimbal the law drives missing: the law and the layer
    /// write, from the realized state; the allocation takes the hand back after ResumeSteps sound solves (the horizon).
    /// Capability bit 24 on a step it wrote.
    /// </summary>
    internal static partial class AttitudeV1
    {
        internal const long CapEngineAllocation = 1L << 23;
        internal const long CapEngineAllocationWrite = 1L << 24;
        internal const int AllocationProtocolVersion = 1;
        // Schema 2 (step 2): the block, so the frame header, ends with the write fields.
        internal const int AllocationSchemaVersion = 2;
        internal const int AllocationEngineRowStride = 6;
        internal const int AllocationGimbalRowStride = 8;
        internal const int AllocationCtrlStride = 10;
        internal const int AllocationClockStride = 1;
        internal const int AllocationBlockProtocol = 1;
        internal const int AllocationBlockSchema = 2;
        internal const string AllocationLayoutName = "attitude_allocation";
        /// <summary>The model table: the current main throttle and k / LevelGrid, k = 1..LevelGrid.</summary>
        internal const int LevelGrid = 4;
        const int TableLevels = LevelGrid + 1;
        const int Directions = 4;

        /// <summary>The block's values (the frame header carries them after protocol, schema, tick, ut, id).</summary>
        internal static readonly string[] AllocationFields = {
            "state", "reasons", "stages", "iterations", "converged", "variables", "engine_count", "gimbal_count",
            "horizon_steps", "main_throttle", "thrust_n", "mass_kg",
            "lateral_demand", "want_x_ms2", "want_z_ms2", "lateral_now_x_ms2", "lateral_now_z_ms2",
            "target_fx_n", "target_fy_n", "target_fz_n", "target_mx_nm", "target_my_nm", "target_mz_nm",
            "achieved_fx_n", "achieved_fy_n", "achieved_fz_n", "achieved_mx_nm", "achieved_my_nm", "achieved_mz_nm",
            "authority_fx_n", "authority_fy_n", "authority_fz_n", "authority_mx_nm", "authority_my_nm",
            "authority_mz_nm",
            "lateral_gimbals_x_n", "lateral_gimbals_z_n", "lateral_throttles_x_n", "lateral_throttles_z_n",
            "lateral_rcs_x_n", "lateral_rcs_z_n",
            "pitch_yaw_gimbals_nm", "pitch_yaw_throttles_nm", "pitch_yaw_rcs_nm",
            "throttle_min", "throttle_max", "gimbal_fraction_max", "rcs_max", "rcs_dead_zone",
            "free_px_ms2", "free_nx_ms2", "free_pz_ms2", "free_nz_ms2", "free_min_ms2", "free_age_steps",
            "model_now_ms2", "model_level_1_ms2", "model_level_2_ms2", "model_level_3_ms2", "model_level_4_ms2",
            "model_now_throttle", "model_age_steps",
            "cost_us"
        };

        /// <summary>
        /// Schema 2 (step 2, option 8192): the write's values, APPENDED after attitude_tick and ut in the block (after the
        /// fields above in the frame header). write_state 0 and NaN values without the option.
        /// </summary>
        internal static readonly string[] AllocationWriteFields = {
            "write_state", "fallback_reasons", "write_streak", "resume_steps", "write_count", "fallback_count",
            "dead_zone_snapped", "free_demand_ms2", "free_demand_age_steps", "fbw_kappa", "lease_failures",
            "decay_steps", "missed_writes"
        };

        internal sealed class AllocationState
        {
            internal readonly EngineAllocation Alloc = new EngineAllocation ();
            internal readonly EngineAllocation Auth = new EngineAllocation ();
            internal readonly Recorder History = new Recorder (HistoryCapacity);
            internal readonly double[] Block = new double[AllocationFields.Length + AllocationWriteFields.Length];
            internal long BlockTick = -1;
            internal double BlockUt = double.NaN;
            internal bool Prepared;
            internal long PreparedTick = -1;
            internal double CostUs;
            internal long Exceptions;

            // Engines (this step).
            internal readonly List<ModuleEngines> EngineList = new List<ModuleEngines> ();
            internal int Engines;
            internal double[] Slope = new double[0], K = new double[0], Tmin = new double[0], Tp = new double[0];
            internal double[] FinalN = new double[0], U0 = new double[0], FUp = new double[0], FDown = new double[0];
            internal double[] Dir = new double[0], Tpn = new double[0];
            internal uint[] EngineIds = new uint[0];

            // Gimbal modules (usable ones, this step), 2 columns each.
            internal int Gimbals;
            internal int[] GimbalOf = new int[0];          // index in Entry.Gimbals
            internal double[] GT = new double[0];          // torque columns, 6 per module (x then y)
            internal double[] GF = new double[0];          // force columns, 6 per module
            internal double[] GU0 = new double[0], GLo = new double[0], GHi = new double[0];   // 2 per module
            internal double[] GFactor = new double[0], GThrustN = new double[0];
            internal uint[] GimbalIds = new uint[0];
            internal int[] GimbalOrdinals = new int[0];

            // ctrlState: rotation (the law's columns) and translation X / Y.
            internal readonly double[] RotT = new double[9];
            internal readonly double[] Served = new double[5];
            internal readonly double[] LinF = new double[6], LinT = new double[6];
            internal bool Linear;
            internal readonly double[] FPos = new double[6], FNeg = new double[6], TPos = new double[6], TNeg = new double[6];
            internal double Epsilon = double.NaN;

            internal double MassKg, ThrustN, MainThrottle, Horizon = 1.0, DiffPriority = 1.0;
            internal double LateralNowX, LateralNowZ;
            internal bool Demand, DemandTilt;
            internal double WantX = double.NaN, WantZ = double.NaN;
            internal long Reasons;

            // Round robin of the authorities.
            internal int FreeCursor, TableCursor;
            internal readonly double[] Free = new double[Directions];
            internal readonly long[] FreeTick = new long[Directions];
            internal readonly double[] Table = new double[TableLevels * Directions];
            internal readonly long[] TableTick = new long[TableLevels * Directions];
            internal double TableNowThrottle = double.NaN;

            // Step 2 (option 8192): the write.
            internal bool RotationRcsOnly;         // the ctrlState rotation columns are the RCS's alone (dead zone)
            internal bool RollOurs;                // the loop writes the roll (its column free, its row in)
            internal bool[] GimbalIncluded = new bool[0];   // by Entry.Gimbals index: in this step's allocation
            internal long SolvedTick = -1;
            internal int SolvedState;
            internal long SolvedReasons;
            internal bool SolvedConverged;
            internal long ExceptionTick = -1;
            internal bool Write;                   // decided at this step's Earlyish, consumed by the fly-by-wire
            internal long WriteTick = -1;
            internal long LastWriteTick = -1;
            internal int WriteState;
            internal long Fallbacks;
            internal int Streak;
            internal int ResumeSteps = 1;
            internal long WriteCount, FallbackCount, LeaseFailures, Missed;
            internal bool LeaseLost;
            internal int Snapped;
            internal double[] Delta = new double[0];        // per engine: command - main throttle
            internal readonly double[] CmdRot = new double[3];
            internal double CmdX = double.NaN, CmdY = double.NaN;
            internal readonly HashSet<ModuleEngines> Leased = new HashSet<ModuleEngines> ();
            internal readonly List<ModuleEngines> HeldEngines = new List<ModuleEngines> ();   // leased at the last write
            internal double[] HeldDelta = new double[0];                                       // their offsets, decaying
            internal long DecaySteps;
            internal bool LinWritten;
            internal float LinWrittenX, LinWrittenY;
            internal double FbwKappa = double.NaN;
            internal double FreeDemand = double.NaN;
            internal long FreeDemandTick = -1;

            /// <summary>The write's own state back to born (an arming, the authority handed back). Leases: the caller's.</summary>
            internal void ResetWrite ()
            {
                Write = false;
                WriteTick = -1;
                LastWriteTick = -1;
                Streak = 0;
                LeaseLost = false;
                Leased.Clear ();
                HeldEngines.Clear ();
                CmdX = CmdY = double.NaN;
            }

            internal void Reset ()
            {
                ResetWrite ();
                LinWritten = false;
                SolvedTick = -1;
                ExceptionTick = -1;
                WriteState = AllocationWriteRule.StateOff;
                Fallbacks = 0;
                WriteCount = FallbackCount = LeaseFailures = Missed = DecaySteps = 0;
                Snapped = 0;
                FbwKappa = double.NaN;
                FreeDemand = double.NaN;
                FreeDemandTick = -1;
                Prepared = false;
                PreparedTick = -1;
                BlockTick = -1;
                BlockUt = double.NaN;
                for (int k = 0; k < Block.Length; k++)
                    Block [k] = double.NaN;
                Block [0] = EngineAllocation.StateNotComputed;
                History.Clear ();
                Alloc.ForgetWarm ();
                Auth.ForgetWarm ();
                FreeCursor = TableCursor = 0;
                for (int d = 0; d < Directions; d++) {
                    Free [d] = double.NaN;
                    FreeTick [d] = -1;
                }
                for (int k = 0; k < Table.Length; k++) {
                    Table [k] = double.NaN;
                    TableTick [k] = -1;
                }
                TableNowThrottle = double.NaN;
                CostUs = double.NaN;
            }

            internal void Size (int engines, int gimbals)
            {
                if (Slope.Length < engines) {
                    Slope = new double[engines];
                    K = new double[engines];
                    Tmin = new double[engines];
                    Tp = new double[engines];
                    FinalN = new double[engines];
                    U0 = new double[engines];
                    FUp = new double[engines];
                    FDown = new double[engines];
                    Dir = new double[3 * engines];
                    Tpn = new double[3 * engines];
                    EngineIds = new uint[engines];
                    Delta = new double[engines];
                }
                if (GimbalIncluded.Length < gimbals)
                    GimbalIncluded = new bool[gimbals];
                if (GimbalOf.Length < gimbals) {
                    GimbalOf = new int[gimbals];
                    GT = new double[6 * gimbals];
                    GF = new double[6 * gimbals];
                    GU0 = new double[2 * gimbals];
                    GLo = new double[2 * gimbals];
                    GHi = new double[2 * gimbals];
                    GFactor = new double[gimbals];
                    GThrustN = new double[gimbals];
                    GimbalIds = new uint[gimbals];
                    GimbalOrdinals = new int[gimbals];
                }
            }
        }

        // ---------------------------------------------------------------- RPC bodies

        /// <summary>
        /// The named block of the last step of the token's vessel: [protocol, schema, n, n values]
        /// (AllocationFields, attitude_tick, ut, then AllocationWriteFields). State 0 and NaN values without option 4096 or before a
        /// first step.
        /// </summary>
        internal static IList<double> AllocationV1 (string token)
        {
            var e = RequireToken (token);
            var al = e.Al;
            int n = AllocationFields.Length + 2 + AllocationWriteFields.Length;
            var reply = new List<double> (3 + n) { AllocationBlockProtocol, AllocationBlockSchema, n };
            bool on = AttitudeWrite.AllocationOn (e.Options) && al.BlockTick >= 0;
            for (int k = 0; k < AllocationFields.Length; k++)
                reply.Add (on ? al.Block [k] : (k == 0 ? EngineAllocation.StateNotComputed : double.NaN));
            reply.Add (on ? al.BlockTick : double.NaN);
            reply.Add (on ? al.BlockUt : double.NaN);
            for (int k = 0; k < AllocationWriteFields.Length; k++)
                reply.Add (on ? al.Block [AllocationFields.Length + k] : (k == 0 ? AllocationWriteRule.StateOff : double.NaN));
            return reply;
        }

        internal static IList<double> ReadAllocationFrames (string persistentId, int sinceTick, int maxFrames)
        {
            uint pid = VesselLookup.ParsePersistentId (persistentId);
            Entry entry;
            var history = entries.TryGetValue (pid, out entry) ? entry.Al.History : emptyRing;
            return history.ReadSince (AllocationProtocolVersion, AllocationSchemaVersion, sinceTick, maxFrames);
        }

        // ------------------------------------------------------------------ the step

        static bool allocationWarm;

        /// <summary>
        /// The first time option 4096 is set (in the RPC, not in a physics step): one tiny solve of every path, so that the
        /// JIT compiles the cascade there and not in the first shadow step (14-48 ms in the replay harness).
        /// </summary>
        static void AllocationWarmUp ()
        {
            if (allocationWarm)
                return;
            allocationWarm = true;
            try {
                var x = new EngineAllocation ();
                x.Resize (2);
                for (int j = 0; j < 2; j++) {
                    for (int r = 0; r < EngineAllocation.Rows; r++)
                        x.Col [6 * j + r] = r == j + 3 || r == j ? 1.0 : 0.0;
                    x.U0 [j] = 0.0;
                    x.Lo [j] = -1.0;
                    x.Hi [j] = 1.0;
                    x.Pref [j] = 0.0;
                    x.Priority [j] = 1.0;
                    x.PhiUp [j] = x.PhiDown [j] = 1.0;
                    x.Kind [j] = EngineAllocation.KindGimbal;
                }
                for (int r = 0; r < EngineAllocation.Rows; r++) {
                    x.Target [r] = 0.5;
                    x.RowOn [r] = true;
                }
                x.Solve (true);
                x.Target [3] = 10.0;
                x.Solve (true);
                x.AuthorityAlong (1.0, 0.0);
            } catch (Exception exc) {
                BridgeLog.Error ("AttitudeV1 allocation warm-up: " + exc.Message);
            }
        }

        /// <summary>A failure of the shadow: counted, said five times, the step's snapshot dropped; never the loop's.</summary>
        static void AllocationFailed (Entry e, Exception exc)
        {
            var al = e.Al;
            al.Prepared = false;
            al.ExceptionTick = ownTick;
            al.Exceptions++;
            if (al.Exceptions <= 5)
                BridgeLog.Error ("AttitudeV1 allocation (ombre D264): " + exc);
        }

        // Scratch (Unity main thread only).
        static readonly double[] allocFactors = new double[256];

        /// <summary>
        /// Option 4096: the snapshot of this step's actuators, read where the law's columns are complete and the
        /// gimbal bounds not yet shifted by the lateral-force layer. Reads only.
        /// </summary>
        static void AllocationPrepare (Entry e, Vessel vessel, Transform reference, Vector3 com, AttitudeInput inp,
            int n, int nGimbalColumns, double dt, double[] served, bool precision, bool rotationRcsOnly)
        {
            var al = e.Al;
            long t0 = Stopwatch.GetTimestamp ();
            al.Prepared = false;
            al.Reasons = 0;
            int maxEngines = e.Engines.Count, maxGimbals = e.Gimbals.Count;
            al.Size (maxEngines, maxGimbals);
            al.RotationRcsOnly = rotationRcsOnly;
            for (int g = 0; g < al.GimbalIncluded.Length; g++)
                al.GimbalIncluded [g] = false;

            // ---------------------------------------------------------- engines
            al.EngineList.Clear ();
            int ne = 0;
            double thrustN = 0.0, latX = 0.0, latZ = 0.0;
            for (int k = 0; k < e.Engines.Count; k++) {
                var engine = e.Engines [k];
                if (engine == null)
                    continue;
                double fin = engine.finalThrust * 1000.0;
                Vector3 direction;
                bool haveDirection = AttitudeGeometry.EngineDirection (engine, reference, out direction);
                if (fin > 0.0) {
                    thrustN += fin;
                    if (haveDirection) {
                        latX += fin * direction.x;
                        latZ += fin * direction.z;
                    }
                }
                if (!engine.EngineIgnited || engine.throttleLocked || !engine.isOperational || engine.getFlameoutState)
                    continue;
                if (engine.independentThrottle && !Actuation.ThrottleIsLeased (engine))
                    continue;
                double tp = engine.thrustPercentage * 0.01;
                if (!(tp > 0.0) || !haveDirection)
                    continue;
                Vector3 centroid, torquePerNewton;
                if (!AttitudeGeometry.EngineLever (engine, com, reference, out centroid, out torquePerNewton))
                    continue;
                double tmin = engine.throttleMin;
                if (!(tmin >= 0.0) || !(tmin < 1.0))
                    tmin = 0.0;
                double cur = engine.currentThrottle;
                double perThrottle = cur > 1e-3 && engine.finalThrust > 0f
                    ? engine.finalThrust * 1000.0 / cur : engine.maxThrust * 1000.0;
                double slope = perThrottle * (1.0 - tmin) * tp;
                if (!(slope > 0.0) || double.IsInfinity (slope))
                    continue;
                al.EngineList.Add (engine);
                al.Slope [ne] = slope;
                al.K [ne] = perThrottle;
                al.Tmin [ne] = tmin;
                al.Tp [ne] = tp;
                al.FinalN [ne] = fin;
                al.U0 [ne] = (cur - tmin) / ((1.0 - tmin) * tp);
                bool instant = !engine.useEngineResponseTime || engine.throttleUseAlternate;
                al.FUp [ne] = instant ? 1.0 : Math.Max (0.0, engine.engineAccelerationSpeed * dt);
                al.FDown [ne] = instant ? 1.0 : Math.Max (0.0, engine.engineDecelerationSpeed * dt);
                al.Dir [3 * ne] = direction.x;
                al.Dir [3 * ne + 1] = direction.y;
                al.Dir [3 * ne + 2] = direction.z;
                al.Tpn [3 * ne] = torquePerNewton.x;
                al.Tpn [3 * ne + 1] = torquePerNewton.y;
                al.Tpn [3 * ne + 2] = torquePerNewton.z;
                al.EngineIds [ne] = engine.part != null ? engine.part.flightID : 0u;
                ne++;
            }
            al.Engines = ne;
            al.ThrustN = thrustN;

            // ---------------------------------------------------------- gimbals
            int ng = 0;
            for (int g = 0; g < e.Gimbals.Count && 2 * g + 1 < nGimbalColumns; g++) {
                var gr = e.Gimbals [g];
                var gimbal = gr.Gimbal;
                int jx = 2 * g, jy = 2 * g + 1;
                if (gimbal == null || !(inp.UMax [jx] > inp.UMin [jx] || inp.UMax [jy] > inp.UMin [jy]))
                    continue;
                Vector3 ax, ay, fx, fy;
                bool geometry = gr.PivotsMapped
                    ? AttitudeGeometry.GimbalAxesAndForcePivots (gimbal, gr.PartEngines, reference, gr.Pivots,
                        (float)inp.U0 [jx], (float)inp.U0 [jy], out ax, out ay, out fx, out fy)
                    : AttitudeGeometry.GimbalAxesAndForce (gimbal, gr.PartEngines, reference, (float)inp.U0 [jx],
                        out ax, out ay, out fx, out fy);
                if (!geometry)
                    continue;
                al.GimbalOf [ng] = g;
                for (int r = 0; r < 3; r++) {
                    al.GT [6 * ng + r] = inp.B [r * n + jx];
                    al.GT [6 * ng + 3 + r] = inp.B [r * n + jy];
                }
                al.GF [6 * ng] = fx.x;
                al.GF [6 * ng + 1] = fx.y;
                al.GF [6 * ng + 2] = fx.z;
                al.GF [6 * ng + 3] = fy.x;
                al.GF [6 * ng + 4] = fy.y;
                al.GF [6 * ng + 5] = fy.z;
                al.GU0 [2 * ng] = inp.U0 [jx];
                al.GU0 [2 * ng + 1] = inp.U0 [jy];
                al.GLo [2 * ng] = inp.UMin [jx];
                al.GLo [2 * ng + 1] = inp.UMin [jy];
                al.GHi [2 * ng] = inp.UMax [jx];
                al.GHi [2 * ng + 1] = inp.UMax [jy];
                al.GFactor [ng] = AttitudeTranslation.RampFactor (gimbal.useGimbalResponseSpeed,
                    gimbal.gimbalResponseSpeed, dt);
                double gt = 0.0;
                if (gr.PartEngines != null)
                    for (int k = 0; k < gr.PartEngines.Count; k++)
                        if (gr.PartEngines [k] != null)
                            gt += gr.PartEngines [k].finalThrust * 1000.0;
                al.GThrustN [ng] = gt;
                al.GimbalIds [ng] = gr.Part != null ? gr.Part.flightID : 0u;
                al.GimbalOrdinals [ng] = gr.Ordinal;
                al.GimbalIncluded [g] = true;
                ng++;
            }
            al.Gimbals = ng;

            // ---------------------------------------------------------- ctrlState
            for (int a = 0; a < 3; a++) {
                int j = nGimbalColumns + a;
                for (int r = 0; r < 3; r++)
                    al.RotT [3 * a + r] = j < n ? inp.B [r * n + j] : 0.0;
                al.Served [a] = served [a];
            }
            var ctrl = vessel.ctrlState;
            al.Served [3] = ctrl == null ? 0.0 : ctrl.X;
            al.Served [4] = ctrl == null ? 0.0 : ctrl.Y;
            al.MainThrottle = ctrl == null ? 0.0 : ctrl.mainThrottle;
            for (int k = 0; k < 6; k++)
                al.FPos [k] = al.FNeg [k] = al.TPos [k] = al.TNeg [k] = 0.0;
            bool linear = true;
            double epsilon = double.NaN;
            for (int i = 0; i < e.Rcs.Count; i++) {
                var rcs = e.Rcs [i];
                if (rcs == null)
                    continue;
                if (!AttitudeGeometry.AddRcsTranslationColumns (rcs, com, reference, precision, al.FPos, al.FNeg,
                        al.TPos, al.TNeg))
                    linear = false;
                if (rcs.moduleIsEnabled && rcs.rcsEnabled && (double.IsNaN (epsilon) || rcs.EPSILON > epsilon))
                    epsilon = rcs.EPSILON;
            }
            al.Epsilon = epsilon;
            bool any = false;
            for (int k = 0; k < 3; k++) {
                // Mean of the + and - columns, as the law's ctrlState columns: X at [0..2], Y at [3..5].
                al.LinF [k] = 0.5 * (al.FPos [k] + al.FNeg [k]);
                al.LinF [3 + k] = 0.5 * (al.FPos [3 + k] + al.FNeg [3 + k]);
                al.LinT [k] = 0.5 * (al.TPos [k] + al.TNeg [k]);
                al.LinT [3 + k] = 0.5 * (al.TPos [3 + k] + al.TNeg [3 + k]);
                any |= al.LinF [k] != 0.0 || al.LinF [3 + k] != 0.0 || al.LinT [k] != 0.0 || al.LinT [3 + k] != 0.0;
            }
            al.Linear = any;
            if (any && !linear)
                al.Reasons |= EngineAllocation.ReasonRcsModelApprox;
            // The RCS translation's realized lateral force (served X / Y; the rotation's force is left out).
            latX += al.LinF [0] * al.Served [3] + al.LinF [3] * al.Served [4];
            latZ += al.LinF [2] * al.Served [3] + al.LinF [5] * al.Served [4];

            al.MassKg = vessel.totalMass * 1000.0;
            al.LateralNowX = latX;
            al.LateralNowZ = latZ;
            al.DiffPriority = e.T.Params.DiffPriority;

            // ---------------------------------------------------------- horizon
            int nf = 0;
            for (int i = 0; i < ne && nf + 2 <= allocFactors.Length; i++) {
                allocFactors [nf++] = al.FUp [i];
                allocFactors [nf++] = al.FDown [i];
            }
            for (int g = 0; g < ng && nf < allocFactors.Length; g++)
                allocFactors [nf++] = al.GFactor [g];
            al.Horizon = EngineAllocation.Horizon (allocFactors, nf);

            // ---------------------------------------------------------- lateral demand
            AllocationDemand (e, vessel, reference, com);

            al.Prepared = true;
            al.PreparedTick = ownTick;
            al.CostUs = (Stopwatch.GetTimestamp () - t0) * 1e6 / Stopwatch.Frequency;
        }

        /// <summary>
        /// The lateral acceleration the nose does not give, from the live translation request (or the ring's), as
        /// TranslationGap computes it, on this file's own fields: the translation layer is not touched. Only a request
        /// that asks a lateral force (AttitudeTranslation.RequestTranslates: bit 2, as the layer): a request of the
        /// differential alone (mode 53, (0, 0, 0)) is no demand.
        /// </summary>
        static void AllocationDemand (Entry e, Vessel vessel, Transform reference, Vector3 com)
        {
            var al = e.Al;
            var t = e.T;
            al.Demand = false;
            al.DemandTilt = false;
            al.WantX = al.WantZ = double.NaN;
            double ut = Planetarium.GetUniversalTime ();
            if (!AttitudeTranslation.RequestTranslates (t.HaveRequest, t.Mode, ut, t.UtValidUntil) || t.Frame == null)
                return;
            double mass = al.MassKg, thrustAccel = mass > 0.0 ? al.ThrustN / mass : 0.0;
            if (!(mass > 0.0) || !(thrustAccel > 0.0) || vessel.mainBody == null)
                return;
            try {
                Vector3d body = vessel.mainBody.position;
                Vector3 upWorld = new Vector3 ((float)(com.x - body.x), (float)(com.y - body.y), (float)(com.z - body.z));
                Vector3 upBody = reference.InverseTransformDirection (AttitudeGeometry.Unit (upWorld));
                var aFrame = t.Frame;
                double rAx = t.Ax, rAy = t.Ay, rAz = t.Az;
                RingLateral (e, ref aFrame, ref rAx, ref rAy, ref rAz);
                double norm = Math.Sqrt (rAx * rAx + rAy * rAy + rAz * rAz);
                Vector3 demandBody = new Vector3 (0f, 0f, 0f);
                if (norm > 0.0) {
                    Vector3d dWorld = aFrame.DirectionToWorldSpace (new Vector3d (rAx / norm, rAy / norm, rAz / norm));
                    Vector3 unitWorld = AttitudeGeometry.Unit (new Vector3 ((float)dWorld.x, (float)dWorld.y,
                        (float)dWorld.z));
                    demandBody = reference.InverseTransformDirection (AttitudeGeometry.Scale (unitWorld, (float)norm));
                }
                allocUp [0] = upBody.x;
                allocUp [1] = upBody.y;
                allocUp [2] = upBody.z;
                allocDemand [0] = demandBody.x;
                allocDemand [1] = demandBody.y;
                allocDemand [2] = demandBody.z;
            } catch {
                return;
            }
            double ax, az, det;
            if (!AttitudeTranslation.LateralFromGap (allocDemand, allocUp, thrustAccel, t.Params.MinProjectionDet,
                    allocGap, out ax, out az, out det)) {
                al.DemandTilt = true;
                return;
            }
            al.Demand = true;
            al.WantX = ax;
            al.WantZ = az;
        }

        static readonly double[] allocUp = new double[3], allocDemand = new double[3], allocGap = new double[3];

        /// <summary>
        /// Fills an allocation's variables. phiSteps: the horizon (NaN: steady state, phi = 1). level: NaN for
        /// the realized state; else the model's trim at that main throttle (throttles at it, gimbals at neutral,
        /// ctrlState at 0, the gimbal columns scaled by the thrust of that level). Returns the variable count.
        /// </summary>
        static int AllocationFill (Entry e, EngineAllocation x, double phiSteps, double level, bool write = false)
        {
            var al = e.Al;
            int ne = al.Engines, ng = al.Gimbals;
            int n = ne + 2 * ng + 3 + (al.Linear ? 2 : 0);
            x.Resize (n);
            bool steady = double.IsNaN (phiSteps);
            bool model = !double.IsNaN (level);
            int j = 0;
            for (int i = 0; i < ne; i++, j++) {
                double s = al.Slope [i];
                for (int r = 0; r < 3; r++) {
                    x.Col [6 * j + r] = s * al.Dir [3 * i + r];
                    x.Col [6 * j + 3 + r] = s * al.Tpn [3 * i + r];
                }
                x.U0 [j] = model ? level : al.U0 [i];
                x.Lo [j] = 0.0;
                x.Hi [j] = 1.0;
                x.Pref [j] = model ? level : al.MainThrottle;
                x.Priority [j] = al.DiffPriority;
                x.PhiUp [j] = steady ? 1.0 : EngineAllocation.Phi (al.FUp [i], phiSteps);
                x.PhiDown [j] = steady ? 1.0 : EngineAllocation.Phi (al.FDown [i], phiSteps);
                x.Kind [j] = EngineAllocation.KindThrottle;
                x.DeadZoneEligible [j] = false;
            }
            for (int g = 0; g < ng; g++) {
                double scaleThrust = 1.0;
                if (model)
                    // The gimbal's columns at the thrust of this level: the part's engines at that throttle.
                    scaleThrust = AllocationThrustScale (e, g, level);
                double phi = steady ? 1.0 : EngineAllocation.Phi (al.GFactor [g], phiSteps);
                for (int axis = 0; axis < 2; axis++, j++) {
                    for (int r = 0; r < 3; r++) {
                        x.Col [6 * j + r] = scaleThrust * al.GF [6 * g + 3 * axis + r];
                        x.Col [6 * j + 3 + r] = scaleThrust * al.GT [6 * g + 3 * axis + r];
                    }
                    x.U0 [j] = model ? 0.0 : al.GU0 [2 * g + axis];
                    x.Lo [j] = al.GLo [2 * g + axis];
                    x.Hi [j] = al.GHi [2 * g + axis];
                    x.Pref [j] = 0.0;
                    x.Priority [j] = 1.0;
                    x.PhiUp [j] = phi;
                    x.PhiDown [j] = phi;
                    x.Kind [j] = EngineAllocation.KindGimbal;
                    x.DeadZoneEligible [j] = false;
                }
            }
            // Step 2 (write): the RCS dead zone (an RCS-only channel starts from what the RCS realizes of its served
            // command), and the ctrlState axes the loop may not write held where they are.
            double eps = write && !model ? al.Epsilon : double.NaN;
            bool deadZone = eps > 0.0;
            for (int a = 0; a < 3; a++, j++) {
                for (int r = 0; r < 3; r++) {
                    x.Col [6 * j + r] = 0.0;
                    x.Col [6 * j + 3 + r] = al.RotT [3 * a + r];
                }
                bool rcsOnly = deadZone && al.RotationRcsOnly;
                x.U0 [j] = model ? 0.0 : (rcsOnly ? EngineAllocation.DeadZoned (al.Served [a], eps) : al.Served [a]);
                x.Lo [j] = -1.0;
                x.Hi [j] = 1.0;
                if (write && !model && (((e.FrozenAxes >> a) & 1) != 0 || (a == 1 && !al.RollOurs))) {
                    x.Lo [j] = x.U0 [j];
                    x.Hi [j] = x.U0 [j];
                }
                x.Pref [j] = 0.0;
                x.Priority [j] = 3.0;
                x.PhiUp [j] = 1.0;
                x.PhiDown [j] = 1.0;
                x.Kind [j] = EngineAllocation.KindRotation;
                x.DeadZoneEligible [j] = rcsOnly;
            }
            if (al.Linear) {
                for (int c = 0; c < 2; c++, j++) {
                    for (int r = 0; r < 3; r++) {
                        x.Col [6 * j + r] = al.LinF [3 * c + r];
                        x.Col [6 * j + 3 + r] = al.LinT [3 * c + r];
                    }
                    x.U0 [j] = model ? 0.0 : (deadZone ? EngineAllocation.DeadZoned (al.Served [3 + c], eps)
                        : al.Served [3 + c]);
                    x.Lo [j] = -1.0;
                    x.Hi [j] = 1.0;
                    x.Pref [j] = 0.0;
                    x.Priority [j] = 3.0;
                    x.PhiUp [j] = 1.0;
                    x.PhiDown [j] = 1.0;
                    x.Kind [j] = EngineAllocation.KindLinear;
                    x.DeadZoneEligible [j] = deadZone;
                }
            }
            return n;
        }

        /// <summary>
        /// The thrust of gimbal module g at a main throttle level over its thrust now: its part's allocated
        /// engines at K (tmin + (1 - tmin) tp level), any other at its thrust now. 1 when it has no thrust now
        /// (its columns are null: nothing to scale).
        /// </summary>
        static double AllocationThrustScale (Entry e, int g, double level)
        {
            var al = e.Al;
            double now = al.GThrustN [g];
            if (!(now > 0.0))
                return 1.0;
            var gr = al.GimbalOf [g] < e.Gimbals.Count ? e.Gimbals [al.GimbalOf [g]] : null;
            if (gr == null || gr.PartEngines == null)
                return 1.0;
            double then = 0.0;
            for (int k = 0; k < gr.PartEngines.Count; k++) {
                var engine = gr.PartEngines [k];
                if (engine == null)
                    continue;
                int i = al.EngineList.IndexOf (engine);
                if (i < 0)
                    then += engine.finalThrust * 1000.0;
                else
                    then += al.K [i] * (al.Tmin [i] + (1.0 - al.Tmin [i]) * al.Tp [i] * level);
            }
            double s = then / now;
            return s >= 0.0 && !double.IsInfinity (s) ? s : 1.0;
        }

        /// <summary>
        /// Option 4096, after the law: the cascade on this step's snapshot, one direction of the free authority,
        /// one entry of the model table. Reads the law's output only; the block and the frame are written at the end
        /// of the step (AllocationPublish), after the write decision. Option 8192: the solve of the write (the axes the
        /// loop may not write held, the roll row only when the loop writes the roll, the RCS dead zone in the command:
        /// SolveDeadZone) and, every second step with a live demand, the free authority along it. Returns
        /// CapEngineAllocation when the cascade ran.
        /// </summary>
        static long AllocationSolve (Entry e, AttitudeOutput o, double ut)
        {
            var al = e.Al;
            if (!al.Prepared || al.PreparedTick != ownTick)
                return 0L;
            long t0 = Stopwatch.GetTimestamp ();
            long caps = 0L;
            bool write = (e.Options & AttitudeWrite.OptionEngineAllocationWrite) != 0;
            al.WriteState = AllocationWriteRule.StateOff;
            al.Fallbacks = 0L;
            al.RollOurs = e.Mode == ModeArmed && (e.AxesMask & AttitudeWrite.ArmRoll) != 0 &&
                (e.WriteMask & AttitudeWrite.ArmRoll) != 0;
            {
                var x = al.Alloc;
                AllocationFill (e, x, al.Horizon, double.NaN, write);
                int state;
                long reasons = al.Reasons;
                al.Snapped = 0;
                if (!o.Computed) {
                    state = EngineAllocation.StateNoLaw;
                    reasons |= EngineAllocation.ReasonNoLawTarget;
                    ClearAllocationOutputs (x);
                } else {
                    for (int r = 0; r < EngineAllocation.Rows; r++) {
                        x.Target [r] = 0.0;
                        x.RowOn [r] = false;
                    }
                    // (1) the axial force D asks: every engine at the served main throttle, realized at the horizon.
                    double axial = 0.0;
                    for (int i = 0; i < al.Engines; i++)
                        axial += x.Col [6 * i + EngineAllocation.RowFy] *
                            EngineAllocation.Realized (al.MainThrottle, x.U0 [i], x.PhiUp [i], x.PhiDown [i]);
                    x.Target [EngineAllocation.RowFy] = axial;
                    x.RowOn [EngineAllocation.RowFy] = true;
                    // (2, 4) the law's torque: M_d - M (its allocation target). Writing, the roll only when the loop
                    // writes it (else the AutoPilot's roll is not fought).
                    x.Target [EngineAllocation.RowMx] = o.TorqueAllocTarget [0];
                    x.Target [EngineAllocation.RowMy] = o.TorqueAllocTarget [1];
                    x.Target [EngineAllocation.RowMz] = o.TorqueAllocTarget [2];
                    x.RowOn [EngineAllocation.RowMx] = x.RowOn [EngineAllocation.RowMz] = true;
                    x.RowOn [EngineAllocation.RowMy] = !write || al.RollOurs;
                    // (3) the lateral force of the live demand: m want - the realized lateral force.
                    if (al.Demand) {
                        x.Target [EngineAllocation.RowFx] = al.MassKg * al.WantX - al.LateralNowX;
                        x.Target [EngineAllocation.RowFz] = al.MassKg * al.WantZ - al.LateralNowZ;
                        x.RowOn [EngineAllocation.RowFx] = x.RowOn [EngineAllocation.RowFz] = true;
                    } else {
                        reasons |= EngineAllocation.ReasonNoLateralDemand;
                    }
                    if (al.DemandTilt)
                        reasons |= EngineAllocation.ReasonTiltTooLarge;
                    bool ok = write ? x.SolveDeadZone (al.Epsilon, true) : x.Solve (true);
                    if (write)
                        al.Snapped = x.Snapped;
                    reasons |= x.Reasons;
                    state = ok ? EngineAllocation.StateSolved : EngineAllocation.StateNoActuator;
                    if (ok)
                        caps |= CapEngineAllocation;
                }
                al.SolvedTick = ownTick;
                al.SolvedState = state;
                al.SolvedReasons = reasons;
                al.SolvedConverged = state == EngineAllocation.StateSolved && x.Converged;
                // The authorities, ONE solve every second step (they cost two to three times the allocation): a free
                // direction, then two steps later a table entry. Writing with a live demand: the free authority along
                // it on the steps between.
                long phase = ownTick & 3L;
                if (phase == 0L)
                    AllocationFreeStep (e, o);
                else if (phase == 2L)
                    AllocationTableStep (e);
                else if (write && al.Demand)
                    AllocationFreeDemandStep (e, o);
                al.CostUs = (Double.IsNaN (al.CostUs) ? 0.0 : al.CostUs) +
                    (Stopwatch.GetTimestamp () - t0) * 1e6 / Stopwatch.Frequency;
            }
            return caps;
        }

        /// <summary>
        /// The block and the frame of this step's solve, written at the end of the step (after the write decision);
        /// nothing when the allocation did not solve this step.
        /// </summary>
        static void AllocationPublish (Entry e, double ut)
        {
            var al = e.Al;
            if (al.SolvedTick != ownTick)
                return;
            WriteAllocationBlock (e, al.Alloc, al.SolvedState, al.SolvedReasons, ut);
            WriteAllocationFrame (e, al.Alloc);
        }

        /// <summary>
        /// Option 8192, the free authority along the live demand for the PF (in place of an earlier change's): steady state, from the
        /// realized state, the axial force D asks and the law's pitch/yaw torque held, the largest lateral force
        /// increment along the demand's direction, plus the realized lateral force along it, over the mass (m/s2): the
        /// TOTAL lateral acceleration the engines, gimbals and RCS can give along the demand with the attitude served.
        /// </summary>
        static void AllocationFreeDemandStep (Entry e, AttitudeOutput o)
        {
            var al = e.Al;
            double want = Math.Sqrt (al.WantX * al.WantX + al.WantZ * al.WantZ);
            if (!o.Computed || !(want > 0.0) || double.IsInfinity (want) || !(al.MassKg > 0.0))
                return;
            double ux = al.WantX / want, uz = al.WantZ / want;
            var x = al.Auth;
            AllocationFill (e, x, double.NaN, double.NaN);
            for (int r = 0; r < EngineAllocation.Rows; r++) {
                x.Target [r] = 0.0;
                x.RowOn [r] = false;
            }
            double axial = 0.0;
            for (int i = 0; i < al.Engines; i++)
                axial += x.Col [6 * i + EngineAllocation.RowFy] * (al.MainThrottle - x.U0 [i]);
            x.Target [EngineAllocation.RowFy] = axial;
            x.Target [EngineAllocation.RowMx] = o.TorqueAllocTarget [0];
            x.Target [EngineAllocation.RowMz] = o.TorqueAllocTarget [2];
            x.RowOn [EngineAllocation.RowFy] = x.RowOn [EngineAllocation.RowMx] = x.RowOn [EngineAllocation.RowMz] = true;
            double along = x.AuthorityAlong (ux, uz);
            if (double.IsNaN (along))
                return;
            double now = al.LateralNowX * ux + al.LateralNowZ * uz;
            double total = (along + now) / al.MassKg;
            al.FreeDemand = total > 0.0 ? total : 0.0;
            al.FreeDemandTick = ownTick;
        }

        /// <summary>
        /// Option 8192, inside the pitch/yaw channel's write: does the allocation write this step? Its solve of THIS step
        /// sound, every gimbal module the law drives in it, no lease refused at its last write, no exception
        /// (AllocationWriteRule); after a fallback, ResumeSteps sound solves in a row. Sets the write's commands when it
        /// does: the throttle offsets (command - main throttle) for the fly-by-wire, the ctrlState rotation and the RCS
        /// translation. Never throws.
        /// </summary>
        static bool AllocationDecide (Entry e, AttitudeInput inp)
        {
            var al = e.Al;
            try {
                bool fresh = al.SolvedTick == ownTick && al.PreparedTick == ownTick && al.Prepared;
                bool missing = false;
                for (int g = 0; g < e.Gimbals.Count; g++) {
                    var gimbal = e.Gimbals [g].Gimbal;
                    int jx = 2 * g, jy = 2 * g + 1;
                    if (gimbal == null || !(inp.UMax [jx] > inp.UMin [jx] || inp.UMax [jy] > inp.UMin [jy]))
                        continue;
                    if (g >= al.GimbalIncluded.Length || !al.GimbalIncluded [g])
                        missing = true;
                }
                long fallbacks = AllocationWriteRule.SolveFallbacks (al.SolvedState, al.SolvedConverged,
                    al.SolvedReasons, fresh, al.LeaseLost, missing, al.ExceptionTick == ownTick);
                al.LeaseLost = false;
                al.ResumeSteps = AllocationWriteRule.ResumeSteps (al.Horizon);
                long published;
                bool write = AllocationWriteRule.Decide (fallbacks, ref al.Streak, al.ResumeSteps,
                    al.LastWriteTick == ownTick - 1, out published);
                al.Fallbacks = published;
                if (!write) {
                    al.WriteState = AllocationWriteRule.StateFallback;
                    al.FallbackCount++;
                    return false;
                }
                var x = al.Alloc;
                int ne = al.Engines, ng = al.Gimbals;
                for (int i = 0; i < ne; i++)
                    al.Delta [i] = x.Command [i] - al.MainThrottle;
                int j = ne + 2 * ng;
                for (int a = 0; a < 3; a++)
                    al.CmdRot [a] = x.Command [j + a];
                al.CmdX = al.Linear ? x.Command [j + 3] : double.NaN;
                al.CmdY = al.Linear ? x.Command [j + 4] : double.NaN;
                al.Write = true;
                al.WriteTick = ownTick;
                al.LastWriteTick = ownTick;
                al.WriteState = AllocationWriteRule.StateWriting;
                al.WriteCount++;
                return true;
            } catch (Exception exc) {
                AllocationFailed (e, exc);
                al.Write = false;
                al.WriteState = AllocationWriteRule.StateFallback;
                al.Fallbacks = AllocationWriteRule.FallbackException;
                al.Streak = 0;
                al.FallbackCount++;
                return false;
            }
        }

        /// <summary>
        /// Option 8192, the step's gimbals: every module of the allocation leased at its command under the loop's own
        /// authority (and pivots), as the law's write. A refusal is remembered (the next step falls back). Returns the
        /// modules leased.
        /// </summary>
        static int AllocationCommandGimbals (Entry e)
        {
            var al = e.Al;
            var x = al.Alloc;
            int leased = 0, ne = al.Engines;
            for (int k = 0; k < al.Gimbals; k++) {
                int g = al.GimbalOf [k];
                if (g < 0 || g >= e.Gimbals.Count)
                    continue;
                var gr = e.Gimbals [g];
                if (gr.Gimbal == null)
                    continue;
                float tx = (float)x.Command [ne + 2 * k], ty = (float)x.Command [ne + 2 * k + 1];
                try {
                    if (Actuation.CommandGimbalForAttitude (e.AuthorityToken, gr.Gimbal, tx, ty,
                            gr.PivotsMapped ? gr.Pivots : null)) {
                        leased++;
                        gr.HeldX = tx;
                        gr.HeldY = ty;
                        gr.HeldTick = ownTick;
                    } else {
                        al.LeaseLost = true;
                        al.LeaseFailures++;
                    }
                } catch (Exception exc) {
                    al.LeaseLost = true;
                    if (++e.FbwExceptions <= 5)
                        BridgeLog.Error ("AttitudeV1 allocation, bail cardan: " + exc.Message);
                }
            }
            return leased;
        }

        /// <summary>
        /// Option 8192: the allocation wrote this step, the lateral-force layer did not. Its common and feed-forward go
        /// back to zero (the realized deflection is the allocation's: at a fallback the law takes it as its own attitude
        /// part, from where it is), no throttle or X / Y of its own this step, and its reason says why.
        /// </summary>
        static void AllocationTakesLayer (Entry e, bool planned)
        {
            var t = e.T;
            for (int j = 0; j < t.Common.Length; j++) {
                t.Common [j] = 0.0;
                t.CommonRealized [j] = 0.0;
                t.CommonRealizedNext [j] = 0.0;
            }
            t.CommonComputed = false;
            t.DffP = t.DffY = t.DffPrevP = t.DffPrevY = 0.0;
            t.DCmdP = t.DCmdY = double.NaN;
            t.WriteThrottle = false;
            t.WriteRcs = false;
            if (planned)
                t.Reasons |= AttitudeTranslation.ReasonServedByAllocation;
        }

        /// <summary>
        /// Option 8192, our fly-by-wire link, after the main throttle of option 512 and before the layer's writes.
        ///   * A step the allocation decided to write (and the pitch/yaw channel wrote in this same callback): the
        ///     throttle of every engine it drives at main + kappa delta (main = the main throttle served this step, kappa
        ///     the layer's WriteScale: every engine within [0, 1]) under the loop's lease, the layer's throttle leases it
        ///     does not take given back, and ctrlState.X / Y.
        ///   * A step it does not write while the law writes pitch/yaw (a fallback): what the law and the layer will not
        ///     write themselves this step is handed back WITHOUT A JUMP. The throttle offsets and X / Y the allocation
        ///     left decay toward the main throttle and 0 by exp(-1 / H) a step (H the horizon: the time constant of the
        ///     slowest actuator, the rate at which the law's gimbals pick the torque up), and are given back once below
        ///     the allocation's own tolerance (EngineAllocation.TargetTolerance of their range); the ones the layer
        ///     writes this step (its differential, its X / Y: it starts from the realized state) are given back at once.
        ///   * Otherwise (the channel does not write: the AutoPilot's): every lease and X / Y of its own given back.
        /// Nothing may escape (the caller catches).
        /// </summary>
        static void AllocationFlyByWire (Entry e, FlightCtrlState state, bool pitchYawWrote)
        {
            var al = e.Al;
            var t = e.T;
            bool live = e.Mode == ModeArmed && !e.Dropped && pitchYawWrote;
            bool write = al.Write && al.WriteTick == ownTick && live;
            al.Write = false;
            double main = state.mainThrottle;
            if (!write) {
                bool layerThrottles = t.WriteThrottle && t.WriteThrottleTick == ownTick;
                bool layerRcs = t.WriteRcs && t.WriteRcsTick == ownTick;
                double decay = Math.Exp (-1.0 / (al.Horizon >= 1.0 ? al.Horizon : 1.0));
                if (!live || layerThrottles || !AllocationDecayThrottles (e, main, decay))
                    AllocationReleaseThrottles (e);
                if (!live || layerRcs || !AllocationDecayLinear (al, state, decay))
                    AllocationReleaseLinear (al, state);
                return;
            }
            int ne = al.Engines;
            double kappa = AttitudeTranslation.WriteScale (ne, al.Delta, main, 0.0, 1.0);
            al.FbwKappa = kappa;
            al.HeldEngines.Clear ();
            for (int i = 0; i < ne && i < al.EngineList.Count; i++) {
                double u = main + kappa * al.Delta [i];
                if (u > 1.0)
                    u = 1.0;
                if (!(u > 0.0))
                    u = 0.0;
                var engine = al.EngineList [i];
                if (Actuation.CommandThrottleForAttitude (e.AuthorityToken, engine, (float)(100.0 * u))) {
                    al.Leased.Add (engine);
                    t.Leased.Remove (engine);
                    if (al.HeldDelta.Length < ne)
                        al.HeldDelta = new double[e.Engines.Count > ne ? e.Engines.Count : ne];
                    al.HeldDelta [al.HeldEngines.Count] = u - main;
                    al.HeldEngines.Add (engine);
                } else {
                    al.LeaseLost = true;
                    al.LeaseFailures++;
                }
            }
            // An engine leased at an earlier write and left out now goes back to the main throttle.
            if (al.Leased.Count > al.HeldEngines.Count)
                AllocationReleaseNotHeld (e);
            // The layer's leases the allocation did not take (an engine it left out) go back to the main throttle.
            ReleaseDiffThrottles (e);
            // The layer's X / Y of its last write first (ReleaseRcs), then ours.
            ReleaseRcs (t, state);
            double cx = Clamp1 (al.CmdX), cy = Clamp1 (al.CmdY);
            if (al.Linear && !double.IsNaN (cx) && !double.IsNaN (cy)) {
                float fx = (float)cx, fy = (float)cy;
                state.X = fx;
                state.Y = fy;
                al.LinWritten = true;
                al.LinWrittenX = fx;
                al.LinWrittenY = fy;
            } else {
                AllocationReleaseLinear (al, state);
            }
        }

        /// <summary>
        /// A fallback step: the throttle offsets the allocation left (around the main throttle) decayed by `decay` and
        /// written again under the loop's lease. False -- the caller gives them back -- when none is held, a lease is
        /// refused, or every offset is below the tolerance (TargetTolerance of the throttle's range).
        /// </summary>
        static bool AllocationDecayThrottles (Entry e, double main, double decay)
        {
            var al = e.Al;
            int n = al.HeldEngines.Count;
            if (n == 0 || al.Leased.Count == 0 || !Actuation.HoldsForAttitude (e.AuthorityToken))
                return false;
            double largest = 0.0;
            for (int i = 0; i < n; i++) {
                al.HeldDelta [i] *= decay;
                largest = Math.Max (largest, Math.Abs (al.HeldDelta [i]));
            }
            if (!(largest > EngineAllocation.TargetTolerance))
                return false;
            double kappa = AttitudeTranslation.WriteScale (n, al.HeldDelta, main, 0.0, 1.0);
            al.FbwKappa = kappa;
            for (int i = 0; i < n; i++) {
                double u = main + kappa * al.HeldDelta [i];
                if (u > 1.0)
                    u = 1.0;
                if (!(u > 0.0))
                    u = 0.0;
                var engine = al.HeldEngines [i];
                if (engine == null || !al.Leased.Contains (engine) ||
                        !Actuation.CommandThrottleForAttitude (e.AuthorityToken, engine, (float)(100.0 * u)))
                    return false;
            }
            al.DecaySteps++;
            return true;
        }

        /// <summary>A fallback step: ctrlState.X / Y the allocation left, decayed and written again (as the throttles).</summary>
        static bool AllocationDecayLinear (AllocationState al, FlightCtrlState state, double decay)
        {
            if (!al.LinWritten || state == null)
                return false;
            // Someone else wrote X / Y since (the pilot, kRPC): theirs, not ours.
            if (state.X != al.LinWrittenX && state.X != 0f || state.Y != al.LinWrittenY && state.Y != 0f)
                return false;
            double x = al.LinWrittenX * decay, y = al.LinWrittenY * decay;
            if (!(Math.Max (Math.Abs (x), Math.Abs (y)) > EngineAllocation.TargetTolerance))
                return false;
            float fx = (float)x, fy = (float)y;
            state.X = fx;
            state.Y = fy;
            al.LinWrittenX = fx;
            al.LinWrittenY = fy;
            return true;
        }

        /// <summary>The allocation's leases not held by this write (an engine it no longer drives) given back.</summary>
        static void AllocationReleaseNotHeld (Entry e)
        {
            var al = e.Al;
            releaseScratch.Clear ();
            foreach (var engine in al.Leased)
                if (!al.HeldEngines.Contains (engine))
                    releaseScratch.Add (engine);
            foreach (var engine in releaseScratch) {
                al.Leased.Remove (engine);
                try {
                    if (engine != null && Actuation.HoldsForAttitude (e.AuthorityToken))
                        Actuation.ReleaseThrottleForAttitude (e.AuthorityToken, engine);
                } catch (Exception exc) {
                    BridgeLog.Error ("AttitudeV1 allocation: rendu d'une manette: " + exc.Message);
                }
            }
        }

        static readonly List<ModuleEngines> releaseScratch = new List<ModuleEngines> ();

        /// <summary>
        /// Every throttle the allocation leased given back (original fields: the engine follows the main throttle again,
        /// by its own spool). Without the authority they were already restored with it. Never throws.
        /// </summary>
        static void AllocationReleaseThrottles (Entry e)
        {
            var al = e.Al;
            al.HeldEngines.Clear ();
            if (al.Leased.Count == 0)
                return;
            if (!Actuation.HoldsForAttitude (e.AuthorityToken)) {
                al.Leased.Clear ();
                return;
            }
            foreach (var engine in al.Leased) {
                try {
                    if (engine != null)
                        Actuation.ReleaseThrottleForAttitude (e.AuthorityToken, engine);
                } catch (Exception exc) {
                    BridgeLog.Error ("AttitudeV1 allocation: rendu des manettes: " + exc.Message);
                }
            }
            al.Leased.Clear ();
        }

        /// <summary>ctrlState.X / Y still holding EXACTLY our last write go back to 0, once (as the layer's ReleaseRcs).</summary>
        static void AllocationReleaseLinear (AllocationState al, FlightCtrlState state)
        {
            if (!al.LinWritten)
                return;
            al.LinWritten = false;
            if (state == null)
                return;
            if (state.X == al.LinWrittenX)
                state.X = 0f;
            if (state.Y == al.LinWrittenY)
                state.Y = 0f;
        }

        static void ClearAllocationOutputs (EngineAllocation x)
        {
            x.Iterations = 0;
            x.Stages = 0;
            x.Converged = false;
            for (int r = 0; r < EngineAllocation.Rows; r++) {
                x.Target [r] = double.NaN;
                x.Achieved [r] = double.NaN;
                x.RowOn [r] = false;
            }
            for (int j = 0; j < x.Count; j++) {
                x.X [j] = 0.0;
                x.Command [j] = x.U0 [j];
            }
        }

        static readonly double[] dirX = { 1.0, -1.0, 0.0, 0.0 }, dirZ = { 0.0, 0.0, 1.0, -1.0 };

        /// <summary>One direction of the free authority (steady state, from the realized state, the law's torque held).</summary>
        static void AllocationFreeStep (Entry e, AttitudeOutput o)
        {
            var al = e.Al;
            int d = al.FreeCursor;
            al.FreeCursor = (d + 1) % Directions;
            if (!o.Computed) {
                al.Free [d] = double.NaN;
                al.FreeTick [d] = ownTick;
                return;
            }
            var x = al.Auth;
            AllocationFill (e, x, double.NaN, double.NaN);
            for (int r = 0; r < EngineAllocation.Rows; r++) {
                x.Target [r] = 0.0;
                x.RowOn [r] = false;
            }
            double axial = 0.0;
            for (int i = 0; i < al.Engines; i++)
                axial += x.Col [6 * i + EngineAllocation.RowFy] * (al.MainThrottle - x.U0 [i]);
            x.Target [EngineAllocation.RowFy] = axial;
            x.Target [EngineAllocation.RowMx] = o.TorqueAllocTarget [0];
            x.Target [EngineAllocation.RowMz] = o.TorqueAllocTarget [2];
            x.RowOn [EngineAllocation.RowFy] = x.RowOn [EngineAllocation.RowMx] = x.RowOn [EngineAllocation.RowMz] = true;
            double along = x.AuthorityAlong (dirX [d], dirZ [d]);
            al.Free [d] = al.MassKg > 0.0 ? along / al.MassKg : double.NaN;
            al.FreeTick [d] = ownTick;
        }

        /// <summary>One entry of the model table (steady state, from the trim at a main throttle level).</summary>
        static void AllocationTableStep (Entry e)
        {
            var al = e.Al;
            int k = al.TableCursor;
            al.TableCursor = (k + 1) % (TableLevels * Directions);
            int level = k / Directions, d = k % Directions;
            double throttle = level == 0 ? al.MainThrottle : (double)level / LevelGrid;
            if (level == 0)
                al.TableNowThrottle = throttle;
            var x = al.Auth;
            AllocationFill (e, x, double.NaN, throttle);
            for (int r = 0; r < EngineAllocation.Rows; r++) {
                x.Target [r] = 0.0;
                x.RowOn [r] = false;
            }
            x.RowOn [EngineAllocation.RowFy] = x.RowOn [EngineAllocation.RowMx] = x.RowOn [EngineAllocation.RowMz] = true;
            double along = x.AuthorityAlong (dirX [d], dirZ [d]);
            al.Table [k] = al.MassKg > 0.0 ? along / al.MassKg : double.NaN;
            al.TableTick [k] = ownTick;
        }

        /// <summary>The smallest of four directions (NaN if one is NaN), and the age of the oldest (steps).</summary>
        static double MinOf (double[] v, long[] ticks, int offset, int count, out double age)
        {
            double m = double.PositiveInfinity;
            long oldest = long.MaxValue;
            for (int k = 0; k < count; k++) {
                double a = v [offset + k];
                if (double.IsNaN (a) || ticks [offset + k] < 0) {
                    age = double.NaN;
                    return double.NaN;
                }
                if (a < m)
                    m = a;
                if (ticks [offset + k] < oldest)
                    oldest = ticks [offset + k];
            }
            age = ownTick - oldest;
            return m;
        }

        static void WriteAllocationBlock (Entry e, EngineAllocation x, int state, long reasons, double ut)
        {
            var al = e.Al;
            var v = al.Block;
            int ne = al.Engines, ng = al.Gimbals;
            bool solved = state == EngineAllocation.StateSolved;
            double lgx = 0, lgz = 0, ltx = 0, ltz = 0, lrx = 0, lrz = 0;
            double pgx = 0, pgz = 0, ptx = 0, ptz = 0, prx = 0, prz = 0;
            double tmin = double.NaN, tmax = double.NaN, gfrac = double.NaN, rcsMax = double.NaN;
            int deadZone = 0;
            if (solved) {
                tmin = double.PositiveInfinity;
                tmax = double.NegativeInfinity;
                gfrac = 0.0;
                rcsMax = 0.0;
                for (int j = 0; j < x.Count; j++) {
                    double xx = x.X [j];
                    double fX = x.Col [6 * j] * xx, fZ = x.Col [6 * j + 2] * xx;
                    double mX = x.Col [6 * j + 3] * xx, mZ = x.Col [6 * j + 5] * xx;
                    int kind = x.Kind [j];
                    if (kind == EngineAllocation.KindThrottle) {
                        ltx += fX;
                        ltz += fZ;
                        ptx += mX;
                        ptz += mZ;
                        tmin = Math.Min (tmin, x.Command [j]);
                        tmax = Math.Max (tmax, x.Command [j]);
                    } else if (kind == EngineAllocation.KindGimbal) {
                        lgx += fX;
                        lgz += fZ;
                        pgx += mX;
                        pgz += mZ;
                        double c = x.Command [j];
                        double range = c >= 0.0 ? x.Hi [j] : -x.Lo [j];
                        if (range > 0.0 && Math.Abs (c) / range > gfrac)
                            gfrac = Math.Abs (c) / range;
                    } else {
                        lrx += fX;
                        lrz += fZ;
                        prx += mX;
                        prz += mZ;
                        double c = Math.Abs (x.Command [j]);
                        if (c > rcsMax)
                            rcsMax = c;
                        // ModuleRCS.Update zeroes a component whose square is below EPSILON^2.
                        if (!double.IsNaN (al.Epsilon) && c > 0.0 && c * c < al.Epsilon * al.Epsilon &&
                                Math.Abs (x.X [j]) > 0.0)
                            deadZone++;
                    }
                }
                if (ne == 0) {
                    tmin = tmax = double.NaN;
                }
                if (deadZone > 0)
                    reasons |= EngineAllocation.ReasonRcsDeadZone;
            }
            double m = al.MassKg;
            int i = 0;
            v [i++] = state;
            v [i++] = reasons;
            v [i++] = x.Stages;
            v [i++] = x.Iterations;
            v [i++] = solved && x.Converged ? 1.0 : 0.0;
            v [i++] = x.Count;
            v [i++] = ne;
            v [i++] = ng;
            v [i++] = al.Horizon;
            v [i++] = al.MainThrottle;
            v [i++] = al.ThrustN;
            v [i++] = m;
            v [i++] = al.Demand ? 1.0 : 0.0;
            v [i++] = al.WantX;
            v [i++] = al.WantZ;
            v [i++] = m > 0.0 ? al.LateralNowX / m : double.NaN;
            v [i++] = m > 0.0 ? al.LateralNowZ / m : double.NaN;
            for (int r = 0; r < EngineAllocation.Rows; r++)
                v [i++] = solved && x.RowOn [r] ? x.Target [r] : double.NaN;
            for (int r = 0; r < EngineAllocation.Rows; r++)
                v [i++] = solved ? x.Achieved [r] : double.NaN;
            for (int r = 0; r < EngineAllocation.Rows; r++)
                v [i++] = solved ? x.RowAuthority [r] : double.NaN;
            v [i++] = solved ? lgx : double.NaN;
            v [i++] = solved ? lgz : double.NaN;
            v [i++] = solved ? ltx : double.NaN;
            v [i++] = solved ? ltz : double.NaN;
            v [i++] = solved ? lrx : double.NaN;
            v [i++] = solved ? lrz : double.NaN;
            v [i++] = solved ? Math.Sqrt (pgx * pgx + pgz * pgz) : double.NaN;
            v [i++] = solved ? Math.Sqrt (ptx * ptx + ptz * ptz) : double.NaN;
            v [i++] = solved ? Math.Sqrt (prx * prx + prz * prz) : double.NaN;
            v [i++] = tmin;
            v [i++] = tmax;
            v [i++] = gfrac;
            v [i++] = rcsMax;
            v [i++] = solved ? deadZone : double.NaN;
            double freeAge, tableAge;
            for (int d = 0; d < Directions; d++)
                v [i++] = al.Free [d];
            v [i++] = MinOf (al.Free, al.FreeTick, 0, Directions, out freeAge);
            v [i++] = freeAge;
            double oldest = 0.0;
            bool complete = true;
            for (int level = 0; level < TableLevels; level++) {
                double age;
                v [i++] = MinOf (al.Table, al.TableTick, level * Directions, Directions, out age);
                if (double.IsNaN (age))
                    complete = false;
                else if (age > oldest)
                    oldest = age;
            }
            tableAge = complete ? oldest : double.NaN;
            v [i++] = al.TableNowThrottle;
            v [i++] = tableAge;
            v [i++] = al.CostUs;
            // Schema 2: the write (option 8192; the state 0 and NaN without it).
            bool writeOption = (e.Options & AttitudeWrite.OptionEngineAllocationWrite) != 0;
            v [i++] = writeOption ? al.WriteState : AllocationWriteRule.StateOff;
            v [i++] = writeOption ? al.Fallbacks : double.NaN;
            v [i++] = writeOption ? al.Streak : double.NaN;
            v [i++] = writeOption ? al.ResumeSteps : double.NaN;
            v [i++] = writeOption ? al.WriteCount : double.NaN;
            v [i++] = writeOption ? al.FallbackCount : double.NaN;
            v [i++] = writeOption && solved ? al.Snapped : double.NaN;
            v [i++] = writeOption ? al.FreeDemand : double.NaN;
            v [i++] = writeOption && al.FreeDemandTick >= 0 ? ownTick - al.FreeDemandTick : double.NaN;
            v [i++] = writeOption ? al.FbwKappa : double.NaN;
            v [i++] = writeOption ? al.LeaseFailures : double.NaN;
            v [i++] = writeOption ? al.DecaySteps : double.NaN;
            v [i++] = writeOption ? al.Missed : double.NaN;
            al.BlockTick = ownTick;
            al.BlockUt = ut;
        }

        static void WriteAllocationFrame (Entry e, EngineAllocation x)
        {
            var al = e.Al;
            int ne = al.Engines, ng = al.Gimbals;
            int header = 5 + AllocationFields.Length + AllocationWriteFields.Length;
            int length = header + ne * AllocationEngineRowStride + ng * AllocationGimbalRowStride +
                AllocationCtrlStride + AllocationClockStride;
            var f = al.History.Reserve (length);
            int h = 0;
            f [h++] = AllocationProtocolVersion;
            f [h++] = AllocationSchemaVersion;
            f [h++] = ownTick;
            f [h++] = al.BlockUt;
            f [h++] = e.PersistentId;
            for (int k = 0; k < AllocationFields.Length + AllocationWriteFields.Length; k++)
                f [h++] = al.Block [k];
            int j = 0;
            for (int i = 0; i < ne; i++, j++) {
                f [h++] = al.EngineIds [i];
                f [h++] = al.U0 [i];
                f [h++] = j < x.Count ? x.Command [j] : double.NaN;
                f [h++] = j < x.Count ? x.X [j] : double.NaN;
                f [h++] = j < x.Count ? x.PhiUp [j] : double.NaN;
                f [h++] = al.Slope [i];
            }
            for (int g = 0; g < ng; g++, j += 2) {
                f [h++] = al.GimbalIds [g];
                f [h++] = al.GimbalOrdinals [g];
                f [h++] = al.GU0 [2 * g];
                f [h++] = al.GU0 [2 * g + 1];
                f [h++] = j < x.Count ? x.Command [j] : double.NaN;
                f [h++] = j + 1 < x.Count ? x.Command [j + 1] : double.NaN;
                f [h++] = j < x.Count ? x.PhiUp [j] : double.NaN;
                double fx = al.GF [6 * g], fy = al.GF [6 * g + 1], fz = al.GF [6 * g + 2];
                f [h++] = Math.Sqrt (fx * fx + fy * fy + fz * fz);
            }
            int nCtrl = al.Linear ? 5 : 3;
            for (int c = 0; c < 5; c++)
                f [h++] = al.Served [c];
            for (int c = 0; c < 5; c++)
                f [h++] = c < nCtrl && j + c < x.Count ? x.Command [j + c] : double.NaN;
            f [h] = GameClock.StepTick;
            al.History.Commit (length);
        }
    }
}
