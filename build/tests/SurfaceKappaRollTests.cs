// the roll excitation of the physical slope kappa (Model/SurfaceKappaRoll.cs), pure tests.
//
// Checked:
//   * the computed constants: the horizon N_h from the Student quantiles (11), the information of a
//     unit batch through the law's H(z), the one-step-travel bound of the super hope's grid fins
//     (0.0126), the command headroom, the leak ratio at the measured q (0.19 at 10 kPa, 4.9 below
//     1 kPa), the plan's torque from the information still missing (and its floor, and the bound
//     without a usable estimate);
//   * born closed: without a window nothing is ever commanded, whatever the rest says; an expired
//     window stops it; every condition, alone, holds it back with its own reason;
//   * the waveform: +-A switching at the start and the middle of every batch, zero mean per batch,
//     the amplitude the bound or the plan, the plan held N_h batches then recomputed;
//   * the stop: at once on roll proven or an axis out of tolerance (its state), Reset closes the window;
//   * no allocation per step;
//   * a closed loop on body y (the AutoPilot's rate damping, the fins' and the wheels' torques, a
//     correlated disturbance): with the excitation roll is proven within N_h batches, kappa ~1; the
//     inertia off by 3 % moves kappa by about 3 %, the wheels' realized torque off by 3 % by less
//     than the leak bound; real defects (the replica x1.5, x0.6 on roll) go out of tolerance and are
//     never proven; without the excitation the same quiet coast proves nothing.
// Writes nothing into the golden vectors.

using System;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    internal static class SurfaceKappaRollTests
    {
        static int checks;
        static int failures;

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.WriteLine ("FAIL (surface kappa roll): " + what);
            }
        }

        static bool Near (double a, double b, double tolerance)
        {
            return Math.Abs (a - b) <= tolerance;
        }

        internal static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestConstants ();
            TestPlanTorque ();
            TestBornClosedAndReasons ();
            TestWaveformAndPlan ();
            TestStopAndReset ();
            TestNoAllocation ();
            TestClosedLoop ();
            checkCount = checks;
            return failures;
        }

        const double Dt = 0.02, OmegaC = 18.0, Zeta = 0.7;
        const double GainDeg = 31.68;        // 0.553 rad per unit of roll (the super hope's grid fins)
        const double SpeedDeg = 40.0;        // actuatorSpeed

        static void TestConstants ()
        {
            int h = SurfaceKappaRoll.HorizonBatches ();
            double p = 1.0 - 0.5 * SurfaceKappa.FalseAlarm;
            double limit = SurfaceKappa.GainMargin * SurfaceKappa.StudentQuantile (p, SurfaceKappa.BatchCapacity - 1.0);
            Check (h == 11, "horizon N_h is 11 batches (" + h + ")");
            Check (SurfaceKappa.StudentQuantile (p, h - 1.0) <= limit && SurfaceKappa.StudentQuantile (p, h - 2.0) > limit,
                "N_h is the smallest K with t(K-1) within the gain margin of the full store's quantile");
            var f = new Biquad3 ();
            double[] x = new double[3], y = new double[3];
            double c = SurfaceKappaRoll.BatchInformation (28, Dt, OmegaC, Zeta, f, x, y);
            Check (Near (c, 1.098, 0.002), "information of a unit batch of 28 steps 1.098 (" + c + ")");
            double c56 = SurfaceKappaRoll.BatchInformation (56, Dt, OmegaC, Zeta, f, x, y);
            Check (c56 > 0.0 && c56 < 2.0 * c, "a longer batch switches less often per step (" + c56 + ")");
            Check (SurfaceKappaRoll.BatchInformation (1, Dt, OmegaC, Zeta, f, x, y) == 0.0 &&
                SurfaceKappaRoll.BatchInformation (28, 0.0, OmegaC, Zeta, f, x, y) == 0.0, "unusable inputs give 0");
            double bound = SurfaceKappaRoll.CommandBound (GainDeg, SpeedDeg, Dt, 0.0);
            Check (Near (bound, 0.8 / (2.0 * GainDeg), 1e-12) && Near (bound, 0.0126, 0.0001),
                "one-step travel of the grid fins: 0.0126 (" + bound + ")");
            Check (Near (SurfaceKappaRoll.CommandBound (GainDeg, SpeedDeg, Dt, 0.995), 0.005, 1e-9),
                "the command headroom bounds it");
            Check (SurfaceKappaRoll.CommandBound (GainDeg, SpeedDeg, Dt, double.NaN) == bound,
                "an unknown AutoPilot roll leaves the travel bound");
            Check (SurfaceKappaRoll.CommandBound (0.0, SpeedDeg, Dt, 0.0) == 0.0 &&
                SurfaceKappaRoll.CommandBound (GainDeg, 0.0, Dt, 0.0) == 0.0, "no gain or no speed: 0");
            Check (Near (SurfaceKappaRoll.LeakMax, 0.5, 1e-12), "leak max 1 / GainMargin^2 = 0.5");
            // Flights 09h12 / 19h14: wheels 30 kN.m + RCS 4.5 kN.m per unit; fins 1.82e5 at 10 kPa, 7e3 below 1 kPa.
            Check (Near (SurfaceKappaRoll.Leak (-3.45e4, -1.82e5), 0.19, 0.001), "leak 0.19 at 10 kPa");
            Check (SurfaceKappaRoll.Leak (-3.45e4, -7.0e3) > 4.0, "leak above 4 below 1 kPa");
            Check (double.IsPositiveInfinity (SurfaceKappaRoll.Leak (-3.45e4, 0.0)), "no surface: infinite leak");
        }

        static SurfaceKappaAxis Axis (double kappa, double se, double t, int count, double dd, double dn, double nn,
            int regressors)
        {
            var a = new SurfaceKappaAxis ();
            a.Kappa = kappa;
            a.Se = se;
            a.T = t;
            a.Count = count;
            a.RegressorsUsed = regressors;
            a.Total [0] = dd;
            a.Total [1] = dn;
            a.Total [2] = nn;
            return a;
        }

        static void TestPlanTorque ()
        {
            const double c = 1.1;
            int h = 11;
            Check (double.IsNaN (SurfaceKappaRoll.PlanTorque (null, c, h)), "no axis: the bound");
            Check (double.IsNaN (SurfaceKappaRoll.PlanTorque (Axis (0.9, 0.08, 3.4, 1, 2e7, 0, 1, 2), c, h)),
                "one batch: the bound");
            Check (double.IsNaN (SurfaceKappaRoll.PlanTorque (Axis (double.NaN, 0.08, 3.4, 10, 2e7, 0, 1, 2), c, h)),
                "no kappa: the bound");
            Check (double.IsNaN (SurfaceKappaRoll.PlanTorque (Axis (0.9, 0.08, 3.4, 10, 0.0, 0, 1, 2), c, h)),
                "no information: the bound");
            // The 09h12 flight at the end of its flip: kappa 0.906, SE 0.087, t 3.37, D 2e7 (N.m)^2.
            double k = 0.906, se = 0.087, t = 3.37, d = 2.0e7;
            double m = k - SurfaceKappa.ProofMin;
            double rho = t * se / m;
            double miss = d * Math.Max (rho * rho - 1.0, 1.0);
            double expected = Math.Sqrt (miss / (c * h));
            double got = SurfaceKappaRoll.PlanTorque (Axis (k, se, t, 21, d, 0.0, 1.0, 2), c, h);
            Check (Near (got, expected, 1e-9 * expected) && got > 1000.0 && got < 2500.0,
                "09h12: about 1.7 kN.m over 11 batches (" + got + ")");
            // Partialled information: dd - dn^2 / nn with the nuisance in the fit, dd alone without it.
            double withNuisance = SurfaceKappaRoll.PlanTorque (Axis (k, se, t, 21, d, 2.0e6, 1.0e6, 2), c, h);
            double partial = d - 2.0e6 * 2.0e6 / 1.0e6;
            Check (Near (withNuisance, Math.Sqrt (partial * Math.Max (rho * rho - 1.0, 1.0) / (c * h)), 1e-6),
                "the information is partialled out of the nuisance");
            Check (Near (SurfaceKappaRoll.PlanTorque (Axis (k, se, t, 21, d, 2.0e6, 1.0e6, 1), c, h), got, 1e-9 * got),
                "one regressor: dd alone");
            // Already inside the band: the floor -- at least as much new information as before.
            double tight = SurfaceKappaRoll.PlanTorque (Axis (1.0, 0.01, 3.37, 50, d, 0.0, 1.0, 2), c, h);
            Check (Near (tight, Math.Sqrt (d / (c * h)), 1e-6), "inside the band: the floor (D_now / (c N_h))");
            // Outside the band: the margin of 1.
            double outside = SurfaceKappaRoll.PlanTorque (Axis (2.0, 0.3, 3.37, 50, d, 0.0, 1.0, 2), c, h);
            double m1 = 1.0 - SurfaceKappa.ProofMin;
            double r1 = 3.37 * 0.3 / m1;
            Check (Near (outside, Math.Sqrt (d * (r1 * r1 - 1.0) / (c * h)), 1e-6), "outside the band: the margin of 1");
        }

        // A fake estimator state for the state machine: roll proven / out as asked.
        static SurfaceKappaEstimator Estimator (bool proven, bool tripped, bool trippedOther = false)
        {
            var est = new SurfaceKappaEstimator ();
            est.BatchSteps = 28;
            est.BaseBatchSteps = 28;
            est.Axes [1].Proven = proven;
            est.Axes [1].Tripped = tripped;
            est.Axes [0].Tripped = trippedOther;
            return est;
        }

        static double StepOnce (SurfaceKappaRollState x, SurfaceKappaEstimator est, double ut, bool shadow = true,
            bool loop = true, bool families = true, bool lawRoll = false, bool exponential = false, bool counted = true,
            double perUnit = -1.82e5, double nonSurface = -3.45e4, double gain = GainDeg, double speed = SpeedDeg,
            double served = 0.0, long tick = 1)
        {
            return x.Step (tick, ut, shadow, loop, families, lawRoll, exponential, est, counted, perUnit, 1.0, nonSurface,
                gain, speed, Dt, OmegaC, Zeta, served);
        }

        static void TestBornClosedAndReasons ()
        {
            var est = Estimator (false, false);
            var x = new SurfaceKappaRollState ();
            // Born closed: no window, nothing, whatever the rest says.
            double v = StepOnce (x, est, 100.0);
            Check (v == 0.0 && x.State == SurfaceKappaRoll.StateOff && (x.Reasons & SurfaceKappaRoll.ReasonNoWindow) != 0 &&
                x.Steps == 0 && x.Episodes == 0, "born closed: nothing without a window");
            Check (Near (x.BatchSeconds, 0.56, 1e-12) && Near (x.PlanSeconds, 11 * 0.56, 1e-9),
                "a batch and a plan in game seconds, published for the client's margin");
            x.Until = 99.0;
            Check (StepOnce (x, est, 100.0) == 0.0 && x.State == SurfaceKappaRoll.StateOff, "an expired window: nothing");
            x.Until = 101.0;
            Check (StepOnce (x, est, 100.0) != 0.0 && x.State == SurfaceKappaRoll.StateExciting && x.Reasons == 0,
                "open window, every condition met: excites");
            int[] reasons = {
                SurfaceKappaRoll.ReasonNotShadow, SurfaceKappaRoll.ReasonLoop, SurfaceKappaRoll.ReasonFamilies,
                SurfaceKappaRoll.ReasonLawRoll, SurfaceKappaRoll.ReasonExponential, SurfaceKappaRoll.ReasonNotCounted,
                SurfaceKappaRoll.ReasonNoSurface, SurfaceKappaRoll.ReasonLeak, SurfaceKappaRoll.ReasonNoSurface,
                SurfaceKappaRoll.ReasonNoSurface
            };
            for (int i = 0; i < reasons.Length; i++) {
                var y = new SurfaceKappaRollState { Until = 101.0 };
                double u = StepOnce (y, est, 100.0, shadow: i != 0, loop: i != 1, families: i != 2, lawRoll: i == 3,
                    exponential: i == 4, counted: i != 5, perUnit: i == 6 ? 0.0 : (i == 7 ? -7.0e3 : -1.82e5),
                    gain: i == 8 ? 0.0 : GainDeg, speed: i == 9 ? 0.0 : SpeedDeg);
                Check (u == 0.0 && (y.Reasons & reasons [i]) != 0 && y.State == SurfaceKappaRoll.StateWaiting,
                    "condition " + i + " alone holds it back with reason " + reasons [i] + " (got " + y.Reasons + ")");
            }
            var z = new SurfaceKappaRollState { Until = 101.0 };
            Check (StepOnce (z, Estimator (true, false), 100.0) == 0.0 && z.State == SurfaceKappaRoll.StateProven &&
                (z.Reasons & SurfaceKappaRoll.ReasonProven) != 0, "roll proven: nothing, state proven");
            Check (StepOnce (z, Estimator (false, false, true), 100.0) == 0.0 && z.State == SurfaceKappaRoll.StateOut &&
                (z.Reasons & SurfaceKappaRoll.ReasonOut) != 0, "another axis out of tolerance: nothing, state out");
            Check (StepOnce (z, Estimator (false, true), 100.0) == 0.0 && z.State == SurfaceKappaRoll.StateOut,
                "roll out of tolerance: nothing, state out");
        }

        static void TestWaveformAndPlan ()
        {
            var est = Estimator (false, false);
            var x = new SurfaceKappaRollState { Until = 1e9 };
            double bound = SurfaceKappaRoll.CommandBound (GainDeg, SpeedDeg, Dt, 0.0);
            // No usable estimate: the bound. A batch: 28 steps, switches at fill 0 and 14.
            double sum = 0.0;
            double[] seen = new double[56];
            for (int k = 0; k < 56; k++) {
                SetFill (est, k % 28);
                seen [k] = StepOnce (x, est, 1.0 + k * Dt, tick: k + 1);
                if (k >= 28)
                    sum += seen [k];
            }
            Check (Near (Math.Abs (seen [0]), bound, 1e-12), "no usable estimate: the bound");
            bool square = true;
            for (int k = 1; k < 56; k++) {
                bool switchStep = (k % 14) == 0;
                bool switched = seen [k] != seen [k - 1];
                if (switched != switchStep)
                    square = false;
            }
            Check (square, "a square wave switching at the start and the middle of every batch");
            Check (Near (sum, 0.0, 1e-12), "zero mean over a batch");
            Check (x.Episodes == 1 && x.Plans == 1, "one episode, one plan so far");
            // A usable estimate: the plan's torque over |F_y|, under the bound.
            var est2 = Estimator (false, false);
            var a = est2.Axes [1];
            a.Kappa = 1.0;
            a.Se = 0.0001;
            a.T = 3.37;
            a.Count = 30;
            a.RegressorsUsed = 1;
            a.Total [0] = 100.0;       // tiny information: the floor, a tiny plan
            var y = new SurfaceKappaRollState { Until = 1e9 };
            SetFill (est2, 5);
            double u = StepOnce (y, est2, 1.0);
            double plan = Math.Sqrt (100.0 / (y.BatchInformationValue * 11));
            Check (Near (Math.Abs (u), plan / 1.82e5, 1e-12) && Math.Abs (u) < bound,
                "a usable estimate: the plan's torque over |F_y| (" + u + ")");
            Check (Near (y.AmplitudeTorque, plan, 1e-9), "published amplitude torque is the plan's");
            // The plan is held N_h batches, then recomputed.
            a.Total [0] = 400.0;
            long plans = y.Plans;
            for (int b = 1; b <= 11; b++) {
                SetFill (est2, 0);
                StepOnce (y, est2, 1.0 + b);
                Check (y.Plans == plans + (b == 11 ? 1 : 0), "plan held, recomputed after N_h batches (batch " + b + ")");
            }
            Check (Near (y.PlanTorque, Math.Sqrt (400.0 / (y.BatchInformationValue * 11)), 1e-9), "the new plan reads the estimator");
        }

        static void SetFill (SurfaceKappaEstimator est, int fill)
        {
            // BatchFill is the estimator's private count: drive it through counted steps of a quiet input.
            var f = typeof (SurfaceKappaEstimator).GetField ("batchFill",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            f.SetValue (est, fill);
        }

        static void TestStopAndReset ()
        {
            var est = Estimator (false, false);
            var x = new SurfaceKappaRollState { Until = 1e9 };
            Check (StepOnce (x, est, 1.0) != 0.0, "excites");
            est.Axes [1].Proven = true;
            Check (StepOnce (x, est, 1.02) == 0.0 && !x.Active && x.State == SurfaceKappaRoll.StateProven,
                "stops at once when roll is proven");
            est.Axes [1].Proven = false;
            Check (StepOnce (x, est, 1.04) != 0.0 && x.Episodes == 2, "a proof lost again: a new episode");
            est.Axes [2].Tripped = true;
            Check (StepOnce (x, est, 1.06) == 0.0 && x.State == SurfaceKappaRoll.StateOut, "stops at once on an axis out");
            est.Axes [2].Tripped = false;
            x.Until = 1.0;
            Check (StepOnce (x, est, 1.08) == 0.0 && x.State == SurfaceKappaRoll.StateOff, "stops when the window ends");
            x.Until = 1e9;
            x.Reset ();
            Check (double.IsNaN (x.Until) && StepOnce (x, est, 1.10) == 0.0 && x.Steps == 0 && x.Episodes == 0,
                "Reset closes the window and forgets everything");
        }

        static void TestNoAllocation ()
        {
            // The fill is held (set before each measured window): fill 0 switches and counts the plan at
            // every step, fill 14 switches, fill 5 holds -- every path of Step.
            foreach (int fill in new[] { 0, 14, 5 }) {
                var est = Estimator (false, false);
                SetFill (est, fill);
                var x = new SurfaceKappaRollState { Until = 1e9 };
                StepOnce (x, est, 1.0);
                long before = GC.GetAllocatedBytesForCurrentThread ();
                for (int k = 0; k < 2000; k++)
                    StepOnce (x, est, 1.0 + k * Dt, tick: k);
                long after = GC.GetAllocatedBytesForCurrentThread ();
                Check (after - before == 0, "no allocation per step, fill " + fill + " (" + (after - before) + " bytes)");
            }
        }

        // ------------------------------------------------------------------ closed loop on body y

        sealed class RollFlight
        {
            internal double Defect = 1.0;          // the physics: the fins act Defect times the replica
            internal double InertiaError;          // the estimator's inertia: (1 + error) x the true one
            internal double WheelError;            // the realized wheel torque given: (1 + error) x the true one
            internal bool Excite = true;
            internal int Seed = 11;
            internal readonly SurfaceKappaEstimator Estimator = new SurfaceKappaEstimator ();
            internal readonly SurfaceKappaRollState Roll = new SurfaceKappaRollState ();
            internal long FirstProven = -1, FirstOut = -1, ExcitedSteps;
            internal double MaxRollRate, MaxRollAngle;

            // Body y only (x and z quiet, not counted): the super hope at ~10 kPa.
            const double Iy = 2.48e5;              // kg m^2
            const double FinsPerUnit = -1.82e5;    // N.m per unit of roll (replica, the law's unit = N.m)
            const double WheelsPerUnit = -3.0e4;   // N.m per unit of roll
            const double Bandwidth = 9.2;          // the AutoPilot's roll rate loop (ap_diag), rad/s

            internal void Run (int steps)
            {
                var rng = new Random (Seed);
                double[] omega = new double[3], omegaDot = new double[3], inertia = new double[3];
                double[] nonFin = new double[3], deflection = new double[3], nuisance = new double[3];
                double w = 0.0, angle = 0.0, integral = 0.0, uPrevious = 0.0, wPrevious = 0.0, disturbance = 0.0;
                double wheelPrevious = 0.0;
                Roll.Until = 1e9;
                double kp = Bandwidth * Iy / (FinsPerUnit * Defect + WheelsPerUnit);
                for (int t = 0; t < steps; t++) {
                    // The tick: the estimator sees the previous step (its omega_dot, torques, increment).
                    omega [1] = w;
                    omegaDot [1] = (w - wPrevious) / Dt;
                    inertia [0] = inertia [2] = 9.5e6;
                    inertia [1] = Iy * (1.0 + InertiaError);
                    nonFin [1] = wheelPrevious * (1.0 + WheelError);
                    bool counted = Estimator.Step (t, t * Dt, t > 1, Dt, OmegaC, Zeta, omega, omegaDot, inertia, nonFin,
                        false, t > 1, deflection, nuisance, 2);
                    if (Estimator.Axes [1].Proven && FirstProven < 0)
                        FirstProven = t;
                    if (Estimator.Axes [1].Tripped && FirstOut < 0)
                        FirstOut = t;
                    // The AutoPilot damps the roll rate (PI, one step late).
                    integral += w * Dt;
                    double uAp = -kp * (w + 3.37 * integral);
                    double ue = Excite ? Roll.Step (t, t * Dt, true, true, true, false, false, Estimator,
                        counted, FinsPerUnit, 1.0, WheelsPerUnit, GainDeg, SpeedDeg, Dt, OmegaC, Zeta, uAp) : 0.0;
                    if (ue != 0.0)
                        ExcitedSteps++;
                    double u = Math.Max (-1.0, Math.Min (1.0, uAp + ue));
                    // The physics step: fins (Defect x the replica), wheels, a slow disturbance (aero, flex).
                    disturbance += 0.05 * (40.0 * (rng.NextDouble () - 0.5) - disturbance);
                    double fins = FinsPerUnit * Defect * u;
                    double wheels = WheelsPerUnit * u;
                    wPrevious = w;
                    w += (fins + wheels + disturbance) / Iy * Dt;
                    angle += w * Dt;
                    MaxRollRate = Math.Max (MaxRollRate, Math.Abs (w));
                    MaxRollAngle = Math.Max (MaxRollAngle, Math.Abs (angle));
                    // The capture of the step: the replica's deflection-driven increment (law unit).
                    deflection [1] = FinsPerUnit * (u - uPrevious);
                    uPrevious = u;
                    wheelPrevious = wheels;
                }
            }
        }

        static void Describe (string name, RollFlight f)
        {
            var y = f.Estimator.Axes [1];
            Console.WriteLine (string.Format (System.Globalization.CultureInfo.InvariantCulture,
                "  {0,-22} proven at {1,5} out at {2,5} kappa {3:0.0000} +- {4:0.0000} batches {5,3} " +
                "excited {6,5} steps, |roll| max {7:0.00000} deg, |p| max {8:0.0000} deg/s", name, f.FirstProven,
                f.FirstOut, y.Kappa, y.Se, y.Count, f.ExcitedSteps, f.MaxRollAngle * 180.0 / Math.PI,
                f.MaxRollRate * 180.0 / Math.PI));
        }

        static void TestClosedLoop ()
        {
            int h = SurfaceKappaRoll.HorizonBatches ();
            var right = new RollFlight ();
            right.Run (3000);
            Describe ("correct", right);
            var y = right.Estimator.Axes [1];
            Check (right.FirstProven > 0 && right.FirstProven <= 28 * (h + 2), "correct replica: roll proven within N_h batches (" +
                right.FirstProven + " steps)");
            Check (Near (y.Kappa, 1.0, 0.01) && !y.Tripped, "correct replica: kappa ~1 (" + y.Kappa + ")");
            Check (right.Roll.State == SurfaceKappaRoll.StateProven && right.ExcitedSteps > 0 &&
                right.ExcitedSteps <= right.FirstProven + 1, "the excitation stopped at the proof");
            Check (right.MaxRollAngle < 0.001 && right.MaxRollRate < 0.01,
                "roll excursion bounded (" + right.MaxRollAngle + " rad, " + right.MaxRollRate + " rad/s)");
            var quiet = new RollFlight { Excite = false };
            quiet.Run (3000);
            Describe ("no excitation", quiet);
            Check (quiet.FirstProven < 0, "without the excitation the quiet coast proves nothing");
            var inertia = new RollFlight { InertiaError = 0.03 };
            inertia.Run (3000);
            Describe ("inertia +3%", inertia);
            double r = 3.0e4 / 1.82e5;
            double bias = inertia.Estimator.Axes [1].Kappa - 1.0;
            Check (bias > 0.0 && bias < 0.03 * (1.0 + r) + 0.01, "inertia +3 %: kappa off by about 3 % (" + bias + ")");
            var wheel = new RollFlight { WheelError = 0.03 };
            wheel.Run (3000);
            Describe ("wheels +3%", wheel);
            double wbias = Math.Abs (wheel.Estimator.Axes [1].Kappa - 1.0);
            Check (wbias < 0.03 * SurfaceKappaRoll.LeakMax, "wheels +3 %: kappa off by less than the leak bound (" + wbias + ")");
            foreach (double defect in new[] { 1.5, 1.0 / 1.5, 2.5, 0.4 }) {
                var bad = new RollFlight { Defect = defect };
                bad.Run (3000);
                Describe ("defect x" + defect.ToString ("0.00", System.Globalization.CultureInfo.InvariantCulture), bad);
                Check (bad.FirstOut > 0 && bad.FirstProven < 0, "defect x" + defect.ToString ("0.00") +
                    " on roll: out of tolerance, never proven (kappa " + bad.Estimator.Axes [1].Kappa + ")");
            }
        }
    }
}
