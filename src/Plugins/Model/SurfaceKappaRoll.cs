using System;

namespace KRPC.Bridge.Model
{
    // -----------------------------------------------------------------------------------
    // BORN CLOSED (nothing at all until the client opens a window with
    // attitude_surfaces_roll_v1, reset at every arming): a deterministic, short and bounded ROLL
    // excitation in the coast, so that the physical slope kappa of the surfaces (SurfaceKappa.cs)
    // is PROVEN on body y (roll) on every flight instead of when the trajectory happens to excite
    // the roll. PURE (System only, compiled into build/tests like SurfaceKappa.cs).
    //
    // Why. A modeled surface is engaged (armed) only when every axis its column contributes to has
    // kappa proven, and every grid fin of the super hope has a roll column at 12 % of its
    // norm. a recorded flight (shadow, mode 13): x and z proven in the flip, roll
    // NEVER (kappa_y 0.908 +- 0.086, 4.9 effective batches, all from the flip): the coast brings no
    // roll evidence at all -- the AutoPilot damps the roll rate (target_roll NaN), its roll
    // command stays at 1e-5..1e-4, the fins' roll increments at a few N.m. Flight _191445 proved it
    // (0.988 +- 0.030) only because its flip happened to excite the roll twice as much. Armed, an
    // unproven roll keeps all four fins neutral: the armed mode would be inert.
    //
    // What. While the window is open, the model is in SHADOW with the kappa flag, the loop owns its
    // fly-by-wire link (roll armed, M3), runs the per-family rules (option 1: the ctrlState columns
    // hold the wheels and RCS only) and does not write the roll itself this step, roll is neither
    // proven nor any axis out of tolerance, the estimator counted roll at this tick, and the
    // surfaces carry most of the roll torque a roll command excites (the leak ratio below), the
    // loop ADDS u_e to the AutoPilot's ctrlState.roll in its fly-by-wire link: a square wave +-A_u
    // switching at the start and the middle of every estimator batch (zero mean per batch, every
    // batch the same information), the AutoPilot keeping its rate damping on top of it.
    //
    // Everything is computed, nothing tuned:
    //   * the MOMENT, the leak ratio r = |B_y| / |F_y|: B_y the roll torque per unit of
    //     ctrlState.roll of the wheels and the RCS (the law's ctrlState roll column, N.m), F_y the
    //     surfaces' (sum over the modeled surfaces of column_y x d(target)/d(roll), physical N.m). A
    //     roll command excites both together; the measured side subtracts the non-surface torque at
    //     its realized value and divides nothing but uses the inertia, so a proportional error of
    //     either (eps on the realized torque, delta on the inertia) is COLLINEAR with the surfaces'
    //     regressor and reads as a bias, not as noise: kappa_hat = kappa (1 + delta) + (delta + eps) r.
    //     r <= LeakMax = 1 / GainMargin^2 = 1/2 -- the surfaces carry at least two thirds of the
    //     excited torque -- keeps the leak (|delta| + |eps|) r within the inertia's own multiplicative
    //     error, which the law shares anyway. Measured (super hope, flights 09h12 and 19h14): wheels
    //     30 kN.m, RCS 4.5 kN.m per unit of roll, fins 7 kN.m below 1 kPa to 1.2 MN.m at 100 kPa:
    //     r 4.5 below 1 kPa, 0.5 at 3.5 kPa (~33 s before the ignition), 0.19 at 10 kPa. Replayed
    //     offline on both flights with the excitation injected: inertia +-3 % moves kappa by
    //     +-0.044, the wheels' torque +-3 % by -+0.012 (r <= 0.5); without this gate (from the fins'
    //     deployment, r ~4.5) by up to +0.23 / -0.18 -- the principal risk of a roll excitation;
    //   * the AMPLITUDE: the information still missing for the interval to fit in the proof band with
    //     the quantile in force -- m the margin of kappa to the band (of 1 when kappa is outside it),
    //     rho = t SE / m, D_miss = D_now max(rho^2 - 1, GainMargin^2 - 1), D_now the roll regressor's
    //     information partialled out of the nuisance: the square-root law (prudent: heterogeneous
    //     evidence shrinks faster), and at least as much new information as all the evidence before
    //     (a delivered plan leaves the trajectory's -- the flip's -- at most half the weight; the proof,
    //     judged at every batch, may stop it sooner: 09h12 replayed, kappa 0.937 at the proof, the
    //     flip's 0.906 still two thirds of it); spread over N_h batches:
    //     A_T = sqrt(D_miss / (c_batch N_h)) (torque, the law's unit), c_batch the information one
    //     batch of a unit square wave brings through H(z) (computed by running the law's own filter);
    //     A_u = A_T / |F_y|. Without a usable estimate (fewer than 2 batches, nothing finite): the bound;
    //   * N_h, the smallest number of batches K with t(1 - FA/2, K - 1) <= GainMargin t(1 - FA/2,
    //     BatchCapacity - 1): the excitation's evidence spread over enough batches that its own
    //     Student quantile is within the gain margin of the full store's (never a decision on
    //     evidence a few batches carry) -- 11 batches, 6.2 s with the plan R2 gains. The amplitude is
    //     held for N_h batches (a plan), then recomputed from the estimator;
    //   * the BOUND, the actuators' travel in ONE physics step: 2 |d(target)/d(roll)| A_u <=
    //     actuatorSpeed dt for every surface, so that every switch is answered within the step (the
    //     replica's same-step response, its regressor exact) and the excitation never makes a surface
    //     rate-limited for the AutoPilot's pitch and yaw -- 0.0126 for the super hope's grid fins
    //     (31.7 deg per unit of roll, 40 deg/s); and the command headroom |u_ap| + A_u <= 1. Replayed:
    //     roll excursion below 0.02 deg, roll rate below 0.15 deg/s, fin travel below 1 deg;
    //   * the STOP, at once (u_e = 0 from that step, the AutoPilot keeps damping the rate): roll
    //     proven, any axis out of tolerance, the window closed or expired, or any condition above.
    //     An engine torque in the step stops the estimator's counting, hence the excitation: never
    //     under thrust.
    // Mono-flight (reset with the estimator, at every arming), deterministic, nothing learned and
    // nothing scaled by kappa: the excitation only gives the estimator the evidence it judges.
    // No allocation per step.
    // -----------------------------------------------------------------------------------

    /// <summary>constants and the pure rules of the roll excitation.</summary>
    internal static class SurfaceKappaRoll
    {
        internal const int Protocol = 1;
        internal const int Schema = 1;
        internal const int ReplyLength = 26;
        internal const int RollAxis = 1;             // body y

        internal const int StateOff = 0;             // no window (born closed) or expired
        internal const int StateWaiting = 1;         // window open, a condition holds it back (reasons)
        internal const int StateExciting = 2;
        internal const int StateProven = 3;          // roll proven: nothing more to do in the arming
        internal const int StateOut = 4;             // an axis out of tolerance: nothing more to do

        // Why the excitation does not run this step (bits; 0 while it runs).
        internal const int ReasonNoWindow = 1 << 0;      // no window, or its game time is over
        internal const int ReasonNotShadow = 1 << 1;     // the model is not in shadow with the kappa flag
        internal const int ReasonLoop = 1 << 2;          // the loop does not own its fly-by-wire link (not armed, dropped)
        internal const int ReasonFamilies = 1 << 3;      // option 1 off: the ctrlState columns also hold the surfaces
        internal const int ReasonLawRoll = 1 << 4;       // the loop writes the roll itself this step
        internal const int ReasonProven = 1 << 5;        // roll proven
        internal const int ReasonOut = 1 << 6;           // an axis out of tolerance
        internal const int ReasonNotCounted = 1 << 7;    // the estimator did not count roll (engine, filter restart, authority)
        internal const int ReasonNoSurface = 1 << 8;     // no modeled surface answers the roll (F_y 0)
        internal const int ReasonExponential = 1 << 9;   // a surface's actuator is exponential (no one-step travel)
        internal const int ReasonLeak = 1 << 10;         // the non-surface share of the excited torque is too large

        /// <summary>The largest leak ratio |B_y| / |F_y|: 1 / GainMargin^2 = 1/2.</summary>
        internal static readonly double LeakMax = 1.0 / (SurfaceKappa.GainMargin * SurfaceKappa.GainMargin);

        static int horizon = -1;

        /// <summary>
        /// N_h: the smallest number of batches K with t(1 - FA/2, K - 1) at most GainMargin times the
        /// quantile of a full store, t(1 - FA/2, BatchCapacity - 1). 11 (FA 1e-3, sqrt 2, 512).
        /// </summary>
        internal static int HorizonBatches ()
        {
            if (horizon > 0)
                return horizon;
            double p = 1.0 - 0.5 * SurfaceKappa.FalseAlarm;
            double limit = SurfaceKappa.GainMargin * SurfaceKappa.StudentQuantile (p, SurfaceKappa.BatchCapacity - 1.0);
            int k = 2;
            while (k < SurfaceKappa.BatchCapacity && !(SurfaceKappa.StudentQuantile (p, k - 1.0) <= limit))
                k++;
            horizon = k;
            return k;
        }

        /// <summary>
        /// The information (sum of the filtered regressor squared) one batch of `batchSteps` steps of
        /// a unit square wave (+1 / -1, switching every half batch) brings through the law's H(z), in
        /// steady state. `f`, `x`, `y`: scratch (no allocation). 0 for unusable inputs.
        /// </summary>
        internal static double BatchInformation (int batchSteps, double dt, double omegaC, double zeta,
            Biquad3 f, double[] x, double[] y)
        {
            if (batchSteps < 2 || !(dt > 0.0) || !(omegaC > 0.0) || !(zeta > 0.0))
                return 0.0;
            int half = batchSteps / 2;
            int period = 2 * half;
            const int settle = 20, measure = 20;
            f.Configure (dt, omegaC, zeta);
            f.Reset ();
            x [0] = x [1] = x [2] = 0.0;
            f.Step (x, y);
            double previous = 0.0, sum = 0.0;
            for (int k = 0; k < period * (settle + measure); k++) {
                double s = (k / half) % 2 == 0 ? 1.0 : -1.0;
                x [0] = s - previous;
                previous = s;
                f.Step (x, y);
                if (k >= period * settle)
                    sum += y [0] * y [0];
            }
            double perPeriod = sum / measure;
            return perPeriod * batchSteps / period;
        }

        /// <summary>
        /// The torque amplitude A_T (the law's unit) of the plan that starts, from the roll axis of
        /// the estimator: sqrt(D_miss / (cBatch horizon)). NaN without a usable estimate (the bound
        /// then applies).
        /// </summary>
        internal static double PlanTorque (SurfaceKappaAxis y, double cBatch, int horizonBatches)
        {
            if (y == null || y.Count < 2 || !(cBatch > 0.0) || horizonBatches <= 0)
                return double.NaN;
            double kappa = y.Kappa, se = y.Se, t = y.T;
            if (double.IsNaN (kappa) || double.IsInfinity (kappa) || double.IsNaN (se) || double.IsInfinity (se) ||
                    double.IsNaN (t) || double.IsInfinity (t))
                return double.NaN;
            double dd = y.Total [0], dn = y.Total [1], nn = y.Total [2];
            double information = y.RegressorsUsed == 2 && nn > 0.0 ? dd - dn * dn / nn : dd;
            if (!(information > 0.0) || double.IsInfinity (information))
                return double.NaN;
            double reference = kappa > SurfaceKappa.ProofMin && kappa < SurfaceKappa.ProofMax ? kappa : 1.0;
            double margin = Math.Min (reference - SurfaceKappa.ProofMin, SurfaceKappa.ProofMax - reference);
            double rho = t * se / margin;
            double floor = SurfaceKappa.GainMargin * SurfaceKappa.GainMargin - 1.0;
            double missing = information * Math.Max (rho * rho - 1.0, floor);
            double amplitude = Math.Sqrt (missing / (cBatch * horizonBatches));
            return double.IsNaN (amplitude) || double.IsInfinity (amplitude) ? double.NaN : amplitude;
        }

        /// <summary>
        /// The largest command amplitude: the one-step travel of the slowest surface
        /// (speed dt / (2 gain), deg/s and deg per unit), and the headroom left by the
        /// AutoPilot's roll (1 - |served|). 0 when the surfaces do not answer.
        /// </summary>
        internal static double CommandBound (double gainMaxPerUnit, double speedMinPerSecond, double dt, double served)
        {
            double travel = speedMinPerSecond * dt;
            double bound = gainMaxPerUnit > 0.0 && travel > 0.0 && !double.IsInfinity (gainMaxPerUnit) ?
                travel / (2.0 * gainMaxPerUnit) : 0.0;
            double s = double.IsNaN (served) || double.IsInfinity (served) ? 0.0 : Math.Abs (served);
            double headroom = Math.Max (0.0, 1.0 - Math.Min (1.0, s));
            return Math.Min (bound, headroom);
        }

        /// <summary>The leak ratio |B_y| / |F_y| (both physical N.m per unit); infinity without surfaces.</summary>
        internal static double Leak (double nonSurfacePerUnit, double surfacePerUnitPhysical)
        {
            double f = Math.Abs (surfacePerUnitPhysical);
            if (!(f > 0.0) || double.IsInfinity (f))
                return double.PositiveInfinity;
            double b = Math.Abs (nonSurfacePerUnit);
            return double.IsNaN (b) ? double.PositiveInfinity : b / f;
        }
    }

    /// <summary>the roll excitation of one armed vessel (window, plan, this step's command, counters).</summary>
    internal sealed class SurfaceKappaRollState
    {
        // The client's window: the game time until which it allows the excitation (NaN: closed).
        internal double Until = double.NaN;
        internal long WindowCalls;
        // This step.
        internal int State = SurfaceKappaRoll.StateOff;
        internal int Reasons = SurfaceKappaRoll.ReasonNoWindow;
        internal double Value;                       // the command added this step (0: none)
        internal long ValueTick = -1;                // the attitude tick Value belongs to
        internal double LastUt = double.NaN;
        internal long LastTick = -1;
        // The plan.
        internal bool Active;
        internal double Sign = 1.0;
        internal int PlanBatches;
        internal double PlanTorque = double.NaN;     // the law's unit; NaN: the bound
        internal double Amplitude = double.NaN;      // command
        internal double AmplitudeTorque = double.NaN;    // the law's unit
        internal double BatchInformationValue = double.NaN;
        internal int BatchInformationSteps = -1;
        internal int Horizon;
        internal double BatchSeconds = double.NaN;   // the estimator's batch, game seconds
        internal double PlanSeconds = double.NaN;    // N_h batches: the length of a plan (the client's margin)
        // The last decision's inputs (published).
        internal double SurfacePerUnit = double.NaN;     // F_y, the law's unit per unit of roll
        internal double NonSurfacePerUnit = double.NaN;  // B_y, N.m per unit of roll
        internal double LeakRatio = double.NaN;
        // Counters.
        internal long Steps, Episodes, Plans, Writes;
        internal double LastWritten = double.NaN;    // what the fly-by-wire link added last
        readonly Biquad3 filter = new Biquad3 ();
        readonly double[] fx = new double[3], fy = new double[3];

        internal void Reset ()
        {
            Until = double.NaN;
            WindowCalls = 0;
            State = SurfaceKappaRoll.StateOff;
            Reasons = SurfaceKappaRoll.ReasonNoWindow;
            Value = 0.0;
            ValueTick = -1;
            LastUt = double.NaN;
            LastTick = -1;
            Active = false;
            Sign = 1.0;
            PlanBatches = 0;
            PlanTorque = Amplitude = AmplitudeTorque = double.NaN;
            BatchInformationValue = double.NaN;
            BatchInformationSteps = -1;
            Horizon = 0;
            BatchSeconds = PlanSeconds = double.NaN;
            SurfacePerUnit = NonSurfacePerUnit = LeakRatio = double.NaN;
            Steps = Episodes = Plans = Writes = 0;
            LastWritten = double.NaN;
        }

        /// <summary>
        /// One attitude tick, after the estimator's step and the loop's write decisions: the command
        /// to add to the AutoPilot's roll at this step's fly-by-wire (0: none), with State and
        /// Reasons. surfacePerUnit: F_y in the law's unit per unit of roll (signed); physical: N.m per
        /// law unit; nonSurfacePerUnit: B_y, N.m per unit (signed); gainMax: the largest
        /// |d(target)/d(roll)| (deg per unit); speedMin: the slowest actuator (deg/s); served: the
        /// AutoPilot's last roll. No allocation.
        /// </summary>
        internal double Step (long tick, double ut, bool shadowKappa, bool loop, bool families, bool lawRoll,
            bool exponential, SurfaceKappaEstimator estimator, bool countedRoll, double surfacePerUnit, double physical,
            double nonSurfacePerUnit, double gainMax, double speedMin, double dt, double omegaC, double zeta,
            double served)
        {
            LastTick = tick;
            LastUt = ut;
            Value = 0.0;
            ValueTick = tick;
            SurfacePerUnit = surfacePerUnit;
            NonSurfacePerUnit = nonSurfacePerUnit;
            LeakRatio = SurfaceKappaRoll.Leak (nonSurfacePerUnit, surfacePerUnit * physical);
            int reasons = 0;
            if (!(Until >= ut))
                reasons |= SurfaceKappaRoll.ReasonNoWindow;
            if (!shadowKappa || estimator == null)
                reasons |= SurfaceKappaRoll.ReasonNotShadow;
            if (!loop)
                reasons |= SurfaceKappaRoll.ReasonLoop;
            if (!families)
                reasons |= SurfaceKappaRoll.ReasonFamilies;
            if (lawRoll)
                reasons |= SurfaceKappaRoll.ReasonLawRoll;
            var roll = estimator == null ? null : estimator.Axes [SurfaceKappaRoll.RollAxis];
            if (estimator != null && estimator.BatchSteps > 0 && dt > 0.0) {
                // Published for the client's margin before the ignition: a batch and a plan, in game time.
                Horizon = SurfaceKappaRoll.HorizonBatches ();
                BatchSeconds = estimator.BatchSteps * dt;
                PlanSeconds = Horizon * BatchSeconds;
            }
            bool proven = roll != null && roll.Proven && !roll.Tripped;
            bool tripped = estimator != null && estimator.TrippedAxes () != 0;
            if (proven)
                reasons |= SurfaceKappaRoll.ReasonProven;
            if (tripped)
                reasons |= SurfaceKappaRoll.ReasonOut;
            if (!countedRoll)
                reasons |= SurfaceKappaRoll.ReasonNotCounted;
            double perUnit = Math.Abs (surfacePerUnit);
            if (!(perUnit > 0.0) || double.IsInfinity (perUnit) || !(gainMax > 0.0) || !(speedMin > 0.0))
                reasons |= SurfaceKappaRoll.ReasonNoSurface;
            if (exponential)
                reasons |= SurfaceKappaRoll.ReasonExponential;
            if (!(LeakRatio <= SurfaceKappaRoll.LeakMax))
                reasons |= SurfaceKappaRoll.ReasonLeak;
            Reasons = reasons;
            if (reasons != 0) {
                Active = false;
                PlanBatches = 0;
                State = (reasons & SurfaceKappaRoll.ReasonNoWindow) != 0 ? SurfaceKappaRoll.StateOff :
                    tripped ? SurfaceKappaRoll.StateOut : proven ? SurfaceKappaRoll.StateProven :
                    SurfaceKappaRoll.StateWaiting;
                return 0.0;
            }
            int batch = estimator.BatchSteps;
            int fill = estimator.BatchFill;
            int half = Math.Max (1, batch / 2);
            if (BatchInformationSteps != batch) {
                BatchInformationValue = SurfaceKappaRoll.BatchInformation (batch, dt, omegaC, zeta, filter, fx, fy);
                BatchInformationSteps = batch;
            }
            Horizon = SurfaceKappaRoll.HorizonBatches ();
            bool start = !Active;
            if (start) {
                Episodes++;
                PlanBatches = 0;
                PlanTorque = SurfaceKappaRoll.PlanTorque (roll, BatchInformationValue, Horizon);
                Plans++;
                Sign = -1.0;
            } else if (fill == 0 && ++PlanBatches >= Horizon) {
                PlanBatches = 0;
                PlanTorque = SurfaceKappaRoll.PlanTorque (roll, BatchInformationValue, Horizon);
                Plans++;
            }
            double bound = SurfaceKappaRoll.CommandBound (gainMax, speedMin, dt, served);
            double amplitude = double.IsNaN (PlanTorque) ? bound : Math.Min (bound, PlanTorque / perUnit);
            if (start || fill % half == 0)
                Sign = -Sign;
            Active = true;
            Amplitude = amplitude;
            AmplitudeTorque = amplitude * perUnit;
            Value = Sign * amplitude;
            Steps++;
            State = SurfaceKappaRoll.StateExciting;
            return Value;
        }
    }
}
