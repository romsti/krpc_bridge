// the physical slope kappa of SurfaceModelV1 (Model/SurfaceKappa.cs), pure tests.
//
// A synthetic flight drives SurfaceKappaEstimator exactly as the attitude tick does: the INDI's raw
// omega_dot (a rigid body integrated from every torque), the law's engine + RCS + wheel torque, and
// per step the capture's two increments: the surfaces' deflection-driven torque in the LAW's unit
// and the rest of the aerodynamic increment (the hull, and the surfaces' own flow part) in N.m.
// The hull's increment follows the vessel's rotation and the surfaces' deflections (the confounder
// a recorded flight shows: the fins' torque alone gives a slope of 2-4); the engines'
// realized torque carries a 10 % error correlated with the commands. Checked:
//   * the Student quantile and the batch length against known values;
//   * a correct replica: kappa 1 proven within the tolerance, never out; with the nuisance left out
//     of the regression the same data give a biased slope (the reason it is a regressor);
//   * an earlier change's unit (the law given kN.m): kappa 1000, out of tolerance, never proven;
//   * real defects (the physics 2.5 and 0.4 times the replica): out, never proven;
//   * the engines' torque keeps the steps out for a filter memory after it stops;
//   * exact data give exactly 1 across filter restarts (gaps), and a merge of the batch store;
//   * no allocation per step; Reset forgets everything (mono-flight);
//   * the jackknife's coverage over repeated noisy flights;
//   * the body obeys Euler's equation (the gyroscopic torque omega x I omega, integrated at
//     the step's mid rate); a FLIP (yaw at 1.2 rad/s, an RCS-driven roll rate, the AutoPilot's pitch
//     answers moving the fins) gives kappa 1 with the rate given to the estimator, a slope far from 1
//     with it withheld (the estimator before an earlier change); the jackknife's degrees of freedom against their
//     brute-force values; out of tolerance needs evidence that more than one batch carries (the
//     interval of 6015f9c would have tripped), while a real defect spread over the batches still trips.
//   * the leak gate (born closed). The RCS commanded with the fins (a proportional RCS,
//     collinear with their deflections) and its realized torque measured 10 % low: where the fins are
//     weak (column x 0.1, the RCS carrying about 2.5 times their increment) the slope reads kappa +
//     eps beta = 1.2 and is PROVEN (inside [0.707, 1.414]) without the gate; with it, the leaky steps
//     are refused and kappa comes back within eps LeakMax of 1 (the steps left are the fins' slew while
//     the command holds); where the fins are strong (x 2, r 1/8) the gate counts nearly every step;
//     the published leak slope beta is about r without the gate, at most LeakMax with it; a real
//     defect still trips; the gate off is the estimator of before, step for step (same kappa, same
//     SE, same batches, the default argument and the explicit false alike).
//   * the inertia Vessel.MOI leaves out (born closed, mode flag 32 with 8). AddInertiaOffset against
//     the parallel-axis sums by hand (one grid fin of the super hope, the four of them, a wing out of the
//     plane, nothing for a mass on its own origin, refused inputs); the mode rule (32 only with 8); and a
//     body whose true inertia is the served one plus the offset (3.3 % on roll, as the super hope's in the
//     coast): the measured side on the served inertia reads kappa = I_served / I_true on every regressor
//     (proven: inside the band), on the physical inertia kappa 1.
// Writes nothing into the golden vectors.

using System;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    internal static class SurfaceKappaTests
    {
        static int checks;
        static int failures;
        static readonly bool trace = Environment.GetEnvironmentVariable ("BRIDGE_D166_TRACE") == "1";

        static void Trace (string name, Flight f)
        {
            if (trace)
                Console.WriteLine ("" + name + ": counted " + f.Estimator.StepsCounted + " of " +
                    f.Estimator.StepsSeen + ", first all-proven " + f.FirstProven + Describe (f.Estimator));
        }

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.WriteLine ("FAIL (surface kappa): " + what);
            }
        }

        internal static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestStudentQuantile ();
            TestBatchSteps ();
            TestExactAcrossRestartsAndMerge ();
            TestCorrectReplica ();
            TestUnitMutant ();
            TestPhysicalDefects ();
            TestEngineQuiet ();
            TestAuthorityAxes ();
            TestNoAllocationAndReset ();
            TestCoverage ();
            TestJackknifeDof ();
            TestGyroscopicFlip ();
            TestOutNeedsDispersedEvidence ();
            TestLeakGate ();
            TestPhysicalInertia ();
            checkCount = checks;
            return failures;
        }

        static bool Near (double a, double b, double tolerance)
        {
            return Math.Abs (a - b) <= tolerance;
        }

        // ------------------------------------------------------------------ maths

        static void TestStudentQuantile ()
        {
            // Reference values (two-sided 1e-3 and 5 %).
            Check (Near (SurfaceKappa.StudentQuantile (0.9995, 1.0), 636.6192, 1e-3), "t(0.9995, 1)");
            Check (Near (SurfaceKappa.StudentQuantile (0.9995, 10.0), 4.586894, 1e-5), "t(0.9995, 10)");
            Check (Near (SurfaceKappa.StudentQuantile (0.9995, 30.0), 3.645959, 1e-5), "t(0.9995, 30)");
            Check (Near (SurfaceKappa.StudentQuantile (0.975, 5.0), 2.570582, 1e-5), "t(0.975, 5)");
            Check (Near (SurfaceKappa.StudentQuantile (0.9995, 1e6), 3.290527, 1e-4), "t(0.9995, inf) = z");
            Check (double.IsNaN (SurfaceKappa.StudentQuantile (0.4, 5.0)) &&
                double.IsNaN (SurfaceKappa.StudentQuantile (0.9, 0.0)), "quantile outside its domain is NaN");
            Check (Near (SurfaceKappa.StudentCdf (0.0, 7.0), 0.5, 1e-15), "cdf(0) = 1/2");
            Check (Near (SurfaceKappa.LogGamma (5.0), Math.Log (24.0), 1e-12), "ln Gamma(5)");
            Check (Near (SurfaceKappa.LogGamma (0.5), 0.5 * Math.Log (Math.PI), 1e-12), "ln Gamma(1/2)");
        }

        static void TestBatchSteps ()
        {
            // The law's H(z) forgets an input to 1e-3 in ln(1000) / (zeta omega_c) = 0.548 s.
            Check (SurfaceKappa.BatchSteps (0.02, 18.0, 0.7) == 28, "batch of 28 steps with the plan R2 gains");
            Check (SurfaceKappa.BatchSteps (0.02, 0.0, 0.7) == 0, "no batch without a filter");
        }

        // ------------------------------------------------------------------ the synthetic flight

        /// <summary>
        /// A rigid body on three axes. physics: what the surfaces' deflections really do, per unit
        /// of the replica's torque; lawScale: what the law's unit makes of the replica's kN.m (1000
        /// right, 1 for an earlier change). Returns the estimator after `steps` ticks.
        /// </summary>
        sealed class Flight
        {
            internal double Physics = 1.0;
            internal double LawScale = 1000.0;
            internal double HullReadout = 1.0;       // true hull torque per unit of the readout
            internal double Noise = 2e3;             // N.m of unexplained torque (correlated in time)
            internal double EngineError = 0.1;       // relative error of the engines' realized torque
            internal int EngineFrom = -1, EngineTo = -1;
            internal int GapEvery = 0;               // a tick gap every that many ticks (0: none)
            internal bool OmitNuisance;              // feed zero for the nuisance (what the regression did)
            internal int Axes = 7;                   // the axes the surfaces can act on
            internal int Seed = 7;
            internal bool Flip;                      // the flip script (yaw 1.2 rad/s, roll RCS, pitch answers)
            internal bool WithholdRate;              // zero rate to the estimator (no gyroscopic term: 6015f9c)
            // the fins' column scale (q), a proportional RCS with the command (N.m per unit,
            // 0: the bang-bang RCS of before), its realized torque measured (1 - RcsError) times the
            // true one, and the leak gate given to the estimator.
            internal double QScale = 1.0;
            internal double RcsPerCommand;
            internal double RcsError;
            internal bool LeakGate;
            internal bool LeakGateExplicit;          // pass leakGate = false explicitly (the default otherwise)
            // what the served inertia (Vessel.MOI) leaves out of the body's true one (kg.m^2, body), and
            // whether the estimator is given the true inertia (the mode flag 32) or the served one.
            internal readonly double[] InertiaLeftOut = new double[3];
            internal bool PhysicalInertia;
            internal readonly SurfaceKappaEstimator Estimator = new SurfaceKappaEstimator ();
            internal long FirstProven = -1;
            internal bool EverProven;
            internal long CountedDuringEngines;
            internal long EngineQuietMinCounted = long.MaxValue;

            readonly double[] inertia = { 2.0e8, 1.0e7, 2.0e8 };
            readonly double[] inertiaTrue = new double[3];
            readonly double[] omega = new double[3], omegaDot = new double[3], nonFin = new double[3];
            readonly double[] deflection = new double[3], nuisance = new double[3];
            readonly double[] finTorque = new double[3], hullTorque = new double[3];
            readonly double[] engineTrue = new double[3], noise = new double[3];
            readonly double[] previousOmega = new double[3], mid = new double[3], gyro = new double[3];
            readonly double[] rateGiven = new double[3];
            double command, commandTarget, deflectionDeg, pitchAngle;
            internal double MaxYawRate, MaxRollRate, GyroPitchRms;

            internal void Run (int steps)
            {
                var rng = new Random (Seed);
                const double dt = 0.02;
                double gyroSquares = 0.0;
                for (int a = 0; a < 3; a++)
                    inertiaTrue [a] = inertia [a] + InertiaLeftOut [a];
                double[] inertiaGiven = PhysicalInertia ? inertiaTrue : inertia;
                for (int t = 0; t < steps; t++) {
                    // The AutoPilot's command: a new target now and then, the fins' actuator at 40 deg/s.
                    // the flip: the pitch command answers the pitch rate and angle (what the
                    // gyroscopic torque disturbs), as the AutoPilot does while the RCS turns the vessel.
                    if (Flip) {
                        command = Math.Max (-1.0, Math.Min (1.0, -40.0 * omega [0] - 8.0 * pitchAngle +
                            0.05 * (rng.NextDouble () - 0.5)));
                    } else {
                        if (t % 37 == 0)
                            commandTarget = (rng.NextDouble () * 2.0 - 1.0);
                        command += 0.2 * (commandTarget - command);
                    }
                    double target = 20.0 * command;
                    double step = Math.Max (-0.8, Math.Min (0.8, target - deflectionDeg));
                    deflectionDeg += step;
                    // The replica's deflection-driven torque increment (kN.m, body): a column per axis.
                    double colX = 60.0 * QScale, colY = 6.0 * QScale, colZ = 45.0 * QScale;     // kN.m per degree
                    double[] dKn = { colX * step, colY * step, colZ * step };
                    // The hull's increment follows the rotation (angle of attack, damping) and the
                    // deflections (the AutoPilot's): the confounder.
                    double[] hullReadout = new double[3];
                    for (int a = 0; a < 3; a++) {
                        double h = -4.0e6 * omega [a] * dt - 2.0e5 * omegaDot [a] * dt + 0.6e3 * dKn [a] +
                            (rng.NextDouble () - 0.5) * 2.0e3;
                        hullReadout [a] = h;
                        hullTorque [a] += HullReadout * h;
                        finTorque [a] += Physics * 1000.0 * dKn [a];
                        noise [a] = 0.9 * noise [a] + (rng.NextDouble () - 0.5) * Noise;
                    }
                    bool engines = t >= EngineFrom && t < EngineTo;
                    double[] engineMeasured = new double[3];
                    double[] rcsLeak = new double[3];
                    for (int a = 0; a < 3; a++) {
                        engineTrue [a] = engines ? 3.0e6 * command * (a == 1 ? 0.1 : 1.0) : 0.0;
                        engineMeasured [a] = engineTrue [a] * (1.0 - EngineError);
                        double rcs = 2.0e4 * Math.Sign (command) * (a == 1 ? 0.2 : 1.0);
                        if (Flip)
                            rcs = FlipRcs (a, t, command);
                        if (RcsPerCommand > 0.0) {
                            // the true RCS torque with the command, measured RcsError low.
                            double rcsTrue = RcsPerCommand * command * (a == 1 ? 0.1 : 1.0);
                            rcs = rcsTrue * (1.0 - RcsError);
                            rcsLeak [a] = rcsTrue - rcs;
                        }
                        nonFin [a] = engineMeasured [a] + rcs;
                        deflection [a] = LawScale * dKn [a];
                        nuisance [a] = OmitNuisance ? 0.0 : hullReadout [a];
                        previousOmega [a] = omega [a];
                    }
                    // Euler's equation, I omega_dot = M - omega x I omega, with the gyroscopic
                    // torque at the step's mid rate (implicit midpoint: a few fixed-point passes).
                    for (int pass = 0; pass < 4; pass++) {
                        for (int a = 0; a < 3; a++)
                            mid [a] = 0.5 * (previousOmega [a] + omega [a]);
                        SurfaceKappaEstimator.Gyroscopic (mid, inertiaTrue, gyro);
                        for (int a = 0; a < 3; a++) {
                            double total = finTorque [a] + hullTorque [a] + engineTrue [a] + nonFin [a] - engineMeasured [a] +
                                noise [a] - gyro [a] + rcsLeak [a];
                            omega [a] = previousOmega [a] + dt * total / inertiaTrue [a];
                        }
                    }
                    for (int a = 0; a < 3; a++) {
                        omegaDot [a] = (omega [a] - previousOmega [a]) / dt;
                        rateGiven [a] = WithholdRate ? 0.0 : omega [a];
                    }
                    pitchAngle += dt * omega [0];
                    MaxYawRate = Math.Max (MaxYawRate, Math.Abs (omega [2]));
                    MaxRollRate = Math.Max (MaxRollRate, Math.Abs (omega [1]));
                    gyroSquares += gyro [0] * gyro [0];
                    bool gap = GapEvery > 0 && t % GapEvery == GapEvery - 1;
                    long before = Estimator.StepsCounted;
                    if (LeakGate || LeakGateExplicit)
                        Estimator.Step (gap ? t + 1000000 : t, t * dt, !gap, dt, 18.0, 0.7, rateGiven, omegaDot, inertiaGiven,
                            nonFin, engines, true, deflection, nuisance, Axes, LeakGate, 1000.0 / LawScale);
                    else
                        Estimator.Step (gap ? t + 1000000 : t, t * dt, !gap, dt, 18.0, 0.7, rateGiven, omegaDot, inertiaGiven,
                            nonFin, engines, true, deflection, nuisance, Axes);
                    if (engines && Estimator.StepsCounted > before)
                        CountedDuringEngines++;
                    if (Estimator.ProvenAxes () == 7 && FirstProven < 0)
                        FirstProven = t;
                    if (Estimator.ProvenAxes () != 0)
                        EverProven = true;
                }
                GyroPitchRms = Math.Sqrt (gyroSquares / Math.Max (1, steps));
            }

            /// <summary>
            /// the flip's RCS (modeled, N.m): the yaw turned at full torque one way then the other
            /// (the vessel reaches about 1.2 rad/s), a roll torque that alternates every second (a roll
            /// rate of a few hundredths of a rad/s), and the pitch thrusters with the pitch command.
            /// </summary>
            static double FlipRcs (int axis, int t, double command)
            {
                if (axis == 2)
                    return (t % 300) < 150 ? 8.0e7 : -8.0e7;
                if (axis == 1)
                    return (t / 50) % 2 == 0 ? 5.0e5 : -5.0e5;
                return 2.0e7 * command;
            }
        }

        static string Describe (SurfaceKappaEstimator e)
        {
            string s = "";
            for (int a = 0; a < 3; a++) {
                var x = e.Axes [a];
                s += " [" + a + ": kappa " + x.Kappa.ToString ("G5") + " +- " + x.Se.ToString ("G3") + " t " +
                    x.T.ToString ("G3") + " n " + x.Nuisance.ToString ("G3") + " batches " + x.Count + " eff " +
                    x.EffectiveBatches.ToString ("G3") + " state " + x.State + "]";
            }
            return s;
        }

        static void TestExactAcrossRestartsAndMerge ()
        {
            // No noise, no hull: y is exactly the deflection's torque; gaps restart the filters.
            var f = new Flight { Noise = 0.0, GapEvery = 101 };
            f.Run (20000);
            Trace ("exact", f);
            var e = f.Estimator;
            Check (e.Restarts > 100, "gaps restarted the filters (" + e.Restarts + ")");
            Check (e.BatchSteps == 2 * e.BaseBatchSteps && e.Axes [0].Count < SurfaceKappa.BatchCapacity,
                "the batch store was merged once (batch " + e.BatchSteps + ", count " + e.Axes [0].Count + ")");
            for (int a = 0; a < 3; a++)
                Check (Near (e.Axes [a].Kappa, 1.0, 1e-6) && Near (e.Axes [a].Nuisance, 1.0, 1e-6),
                    "exact data, kappa 1 and nuisance 1 across restarts, axis " + a + Describe (e));
        }

        static void TestCorrectReplica ()
        {
            var f = new Flight ();
            f.Run (6000);
            Trace ("correct", f);
            var e = f.Estimator;
            for (int a = 0; a < 3; a++) {
                var x = e.Axes [a];
                Check (x.Proven && !x.Tripped, "correct replica: axis " + a + " proven" + Describe (e));
                Check (Math.Abs (x.Kappa - 1.0) <= x.T * x.Se && x.Consistent,
                    "correct replica: kappa consistent with 1 on axis " + a + Describe (e));
            }
            Check (f.FirstProven > 0 && f.FirstProven < 3000, "proven within a minute (" + f.FirstProven + ")");
            // The same flight without the nuisance regressor: the slope is biased by the hull's
            // increment, which follows the deflections (a recorded flight: 1.9-4.1 instead of 1).
            var g = new Flight { OmitNuisance = true };
            g.Run (6000);
            Trace ("correct without nuisance", g);
            Check (g.Estimator.Axes [0].Kappa > 1.3, "without the nuisance the slope is confounded (" +
                g.Estimator.Axes [0].Kappa.ToString ("G4") + ")");
        }

        static void TestUnitMutant ()
        {
            // the law given the replica's kN.m as N.m.
            var f = new Flight { LawScale = 1.0 };
            f.Run (6000);
            Trace ("unit", f);
            var e = f.Estimator;
            Check (!f.EverProven, "unit: never proven");
            Check (e.TrippedAxes () == 7, "unit: every axis out of tolerance" + Describe (e));
            Check (Near (e.Axes [0].Kappa / 1000.0, 1.0, 0.05) && Near (e.Axes [2].Kappa / 1000.0, 1.0, 0.05),
                "unit: kappa 1000" + Describe (e));
        }

        static void TestPhysicalDefects ()
        {
            foreach (double defect in new[] { 2.5, 0.4 }) {
                var f = new Flight { Physics = defect };
                f.Run (6000);
                Trace ("defect x" + defect, f);
                var e = f.Estimator;
                Check (!f.EverProven, "defect x" + defect + ": never proven" + Describe (e));
                Check ((e.TrippedAxes () & 5) == 5, "defect x" + defect + ": pitch and yaw out of tolerance" + Describe (e));
                Check (Near (e.Axes [0].Kappa / defect, 1.0, 0.1), "defect x" + defect + ": kappa near it" + Describe (e));
            }
        }

        static void TestEngineQuiet ()
        {
            // A 10 % error on the engines' torque, steps 1000-2000: none of those steps, nor the
            // filter memory after them, is counted; kappa stays 1.
            var f = new Flight { EngineFrom = 1000, EngineTo = 2000 };
            f.Run (6000);
            Trace ("engines 1000-2000", f);
            var e = f.Estimator;
            Check (f.CountedDuringEngines == 0, "no step counted while the engines push (" + f.CountedDuringEngines + ")");
            Check (e.Axes [0].Proven && e.Axes [2].Proven && e.TrippedAxes () == 0,
                "engines' error kept out: still proven" + Describe (e));
            var g = new Flight { EngineFrom = 1000, EngineTo = 2000 };
            g.Run (2000 + SurfaceKappa.BatchSteps (0.02, 18.0, 0.7) - 1);
            long counted = g.Estimator.StepsCounted;
            var h = new Flight { EngineFrom = 1000, EngineTo = 2000 };
            h.Run (1000);
            Check (counted == h.Estimator.StepsCounted, "the filter memory after the cut-off is not counted (" +
                counted + " against " + h.Estimator.StepsCounted + ")");
        }

        static void TestAuthorityAxes ()
        {
            // The roll axis without the surfaces' authority (folded fins): no step, no batch, no
            // verdict on it; the others unchanged.
            var f = new Flight { Axes = 5 };
            f.Run (3000);
            var e = f.Estimator;
            Check (e.Axes [1].Count == 0 && e.Axes [1].Steps == 0 && double.IsNaN (e.Axes [1].Kappa) &&
                e.Axes [1].State == SurfaceKappa.StateNotProven, "an axis without authority gathers nothing" + Describe (e));
            Check (e.Axes [0].Proven && e.Axes [2].Proven && (e.ProvenAxes () & 2) == 0,
                "the axes with authority are proven alone" + Describe (e));
        }

        static void TestNoAllocationAndReset ()
        {
            var e = new SurfaceKappaEstimator ();
            double[] w = new double[3], i = { 1e8, 1e7, 1e8 }, m = new double[3], d = new double[3], n = new double[3];
            var rng = new Random (3);
            // Warm the code paths (JIT) first.
            for (int t = 0; t < 400; t++) {
                for (int a = 0; a < 3; a++) {
                    w [a] = rng.NextDouble ();
                    d [a] = rng.NextDouble ();
                    n [a] = rng.NextDouble ();
                }
                e.Step (t, t, true, 0.02, 18.0, 0.7, w, w, i, m, false, true, d, n, 7);
            }
            long before = GC.GetAllocatedBytesForCurrentThread ();
            for (int t = 400; t < 20000; t++) {
                for (int a = 0; a < 3; a++) {
                    w [a] = rng.NextDouble ();
                    d [a] = rng.NextDouble ();
                    n [a] = rng.NextDouble ();
                }
                e.Step (t, t, true, 0.02, 18.0, 0.7, w, w, i, m, false, true, d, n, 7);
            }
            long allocated = GC.GetAllocatedBytesForCurrentThread () - before;
            Check (allocated == 0, "no allocation over 19600 steps (" + allocated + " bytes)");
            e.Reset ();
            Check (e.StepsCounted == 0 && e.Axes [0].Count == 0 && double.IsNaN (e.Axes [1].Kappa) &&
                !e.Axes [2].Tripped && e.BatchSteps == 0 && e.ProvenAxes () == 0, "Reset forgets everything");
        }

        static void TestCoverage ()
        {
            // 60 noisy correct flights: the 1e-3 interval misses 1 rarely, and never trips.
            int misses = 0, trips = 0, judged = 0;
            for (int s = 0; s < 60; s++) {
                var f = new Flight { Seed = 1000 + s, Noise = 2e4 };
                f.Run (3000);
                for (int a = 0; a < 3; a++) {
                    var x = f.Estimator.Axes [a];
                    if (double.IsNaN (x.Se) || double.IsInfinity (x.Se))
                        continue;
                    judged++;
                    if (!x.Consistent)
                        misses++;
                    if (x.Tripped)
                        trips++;
                }
            }
            Check (judged >= 150 && trips == 0 && misses <= judged / 20, "coverage: " + misses + " misses of 1 and " +
                trips + " trips over " + judged + " judged axes");
        }

        // ------------------------------------------------------------------ 

        static double Dof (double[] dd, double[] dn, double[] nn, bool partial)
        {
            var batches = new double[dd.Length * SurfaceKappa.BatchSums];
            double sdn = 0.0, snn = 0.0;
            for (int b = 0; b < dd.Length; b++) {
                batches [b * SurfaceKappa.BatchSums] = dd [b];
                batches [b * SurfaceKappa.BatchSums + 1] = dn == null ? 0.0 : dn [b];
                batches [b * SurfaceKappa.BatchSums + 2] = nn == null ? 0.0 : nn [b];
                sdn += dn == null ? 0.0 : dn [b];
                snn += nn == null ? 0.0 : nn [b];
            }
            return SurfaceKappa.JackknifeDof (batches, dd.Length, snn > 0.0 ? sdn / snn : 0.0, partial);
        }

        static void TestJackknifeDof ()
        {
            // Brute force (eigenvalues of the jackknife's quadratic form, Satterthwaite): equal shares
            // give batches - 1; one batch carrying most of the regressor gives about 1.
            Check (Near (Dof (new[] { 1.0, 1.0, 1.0, 1.0 }, null, null, false), 3.0, 1e-12), "dof: four equal batches, 3");
            Check (Near (Dof (new[] { 10.0, 1.0, 1.0, 1.0 }, null, null, false), 1.107379033, 1e-8), "dof: one batch ten times the others, 1.107");
            var ramp = new double[10];
            for (int b = 0; b < 10; b++)
                ramp [b] = b + 1.0;
            Check (Near (Dof (ramp, null, null, false), 6.634696507, 1e-8), "dof: weights 1..10, 6.635");
            Check (Near (Dof (new[] { 3.0, 1.0, 0.0 }, null, null, false), 1.0, 1e-12), "dof: a batch without weight adds nothing");
            Check (Dof (new[] { 1.0, 0.0, 0.0, 0.0 }, null, null, false) == 0.0, "dof: one batch carries everything, 0");
            Check (Dof (new[] { 0.0, 0.0, 0.0 }, null, null, false) == 0.0, "dof: no weight, 0");
            // Partialled (Frisch-Waugh): dd - 2 beta dn + beta^2 nn, beta = 3 / 3 = 1: weights 1, 2, 4.
            Check (Near (Dof (new[] { 2.0, 3.0, 5.0 }, new[] { 1.0, 1.0, 1.0 }, new[] { 1.0, 1.0, 1.0 }, true), 1.366338022, 1e-8),
                "dof: the deflection regressor partialled out of the nuisance");
            var big = new double[512];
            var rng = new Random (11);
            for (int b = 0; b < big.Length; b++)
                big [b] = 1.0 + rng.NextDouble ();
            double nu = Dof (big, null, null, false);
            Check (nu > 400.0 && nu <= 511.0, "dof: 512 similar batches, close to 511 (" + nu + ")");
        }

        static void TestGyroscopicFlip ()
        {
            // The flip of a recorded flight: the RCS turns the vessel at up to 1.2 rad/s in
            // yaw with a roll rate, the gyroscopic pitch torque (I_z - I_y) w_y w_z disturbs the pitch,
            // and the AutoPilot's pitch answers move the fins. With the rate, the estimator subtracts
            // that torque and finds the physics; without it (6015f9c), the AutoPilot's answers to it
            // look like the fins' own effect.
            var f = new Flight { Flip = true };
            f.Run (3000);
            Trace ("flip, rate given", f);
            var e = f.Estimator;
            Check (f.MaxYawRate > 1.0 && f.MaxRollRate > 0.02, "flip: yaw rate " + f.MaxYawRate.ToString ("G3") +
                " rad/s, roll rate " + f.MaxRollRate.ToString ("G3"));
            Check (Near (e.Axes [0].Kappa, 1.0, 0.05) && e.Axes [0].Proven && e.TrippedAxes () == 0,
                "flip with the rate: pitch kappa 1, proven, nothing out" + Describe (e));
            Check (e.Axes [0].RmsGyro > e.Axes [0].RmsFins, "flip: the gyroscopic increment outweighs the fins' (" +
                e.Axes [0].RmsGyro.ToString ("G3") + " against " + e.Axes [0].RmsFins.ToString ("G3") + ")");
            var g = new Flight { Flip = true, WithholdRate = true };
            g.Run (3000);
            Trace ("flip, rate withheld", g);
            Check (Math.Abs (g.Estimator.Axes [0].Kappa - 1.0) > 0.3, "flip without the rate: the pitch slope is biased (" +
                g.Estimator.Axes [0].Kappa.ToString ("G4") + ")" + Describe (g.Estimator));
            Check (g.Estimator.Axes [0].RmsGyro == 0.0, "flip without the rate: no gyroscopic increment");
        }

        static void TestOutNeedsDispersedEvidence ()
        {
            // The trip of a recorded flight (6015f9c): four batches all pulled to about -0.7, one of them
            // (the fins' deployment slew) carrying nearly all the deflections. Kish 1.3; the interval on
            // t(3) lies entirely under the band; on the jackknife's own dof it does not.
            var x = new SurfaceKappaAxis ();
            var rng = new Random (5);
            double[] amplitude = { 1.0, 12.0, 1.5, 1.2 };
            double[] slope = { -0.62, -0.70, -0.66, -0.75 };
            for (int b = 0; b < 4; b++) {
                for (int i = 0; i < 28; i++) {
                    double d = amplitude [b] * Math.Sin (0.7 * i + b) * 1e3;
                    double n = (rng.NextDouble () - 0.5) * 1e3;
                    x.Add (d, n, slope [b] * d + n + (rng.NextDouble () - 0.5) * 2e2, 0.0);
                }
                x.Close ();
                x.Evaluate ();
            }
            bool oldTrip = x.Kappa + x.T * x.Se < SurfaceKappa.ProofMin;
            Check (x.EffectiveBatches < 1.6 && oldTrip, "the 6015f9c interval would have tripped (kappa " +
                x.Kappa.ToString ("G4") + " +- " + (x.T * x.Se).ToString ("G3") + ", Kish " + x.EffectiveBatches.ToString ("G3") + ")");
            Check (!x.Tripped && x.State == SurfaceKappa.StateNotProven && x.OutDof < 1.5 && x.OutHigh >= SurfaceKappa.ProofMin,
                "one episode is not out of tolerance (dof " + x.OutDof.ToString ("G3") + ", out interval " +
                x.OutLow.ToString ("G4") + " .. " + x.OutHigh.ToString ("G4") + ")");
            Check (x.OutLow <= x.Kappa - x.T * x.Se && x.OutHigh >= x.Kappa + x.T * x.Se, "the out interval contains the proof interval");
            // A real defect (2.5) spread over ten similar batches: out of tolerance.
            var y = new SurfaceKappaAxis ();
            for (int b = 0; b < 10; b++) {
                for (int i = 0; i < 28; i++) {
                    double d = (1.0 + 0.2 * b) * Math.Sin (0.7 * i + b) * 1e3;
                    double n = (rng.NextDouble () - 0.5) * 1e3;
                    y.Add (d, n, 2.5 * d + n + (rng.NextDouble () - 0.5) * 2e2, 0.0);
                }
                y.Close ();
                y.Evaluate ();
            }
            Check (y.Tripped && y.OutDof > 5.0 && Near (y.Kappa, 2.5, 0.05), "a defect spread over ten batches is out (dof " +
                y.OutDof.ToString ("G3") + ", kappa " + y.Kappa.ToString ("G4") + ")");
        }

        // ------------------------------------------------------------------ the leak gate

        static void TestLeakGate ()
        {
            // The computed constant: an earlier change's LeakMax.
            Check (Near (SurfaceKappa.LeakMax, 0.5, 1e-15) && Near (SurfaceKappa.LeakMax, SurfaceKappaRoll.LeakMax, 0.0),
                "LeakMax = 1 / GainMargin^2 = 1/2, an earlier change's");
            Check (SurfaceKappa.LeakPasses (2.0, 0.99) && SurfaceKappa.LeakPasses (-2.0, -0.99) &&
                !SurfaceKappa.LeakPasses (2.0, 1.0000001) && !SurfaceKappa.LeakPasses (0.0, 1e-12) &&
                SurfaceKappa.LeakPasses (0.0, 0.0) && !SurfaceKappa.LeakPasses (double.NaN, 0.0) &&
                !SurfaceKappa.LeakPasses (1.0, double.PositiveInfinity), "the rule |m| <= LeakMax |d|, finite only");

            // Closed: the estimator of before, step for step, the default argument and false alike.
            var a0 = new Flight ();
            a0.Run (4000);
            var a1 = new Flight { LeakGateExplicit = true };
            a1.Run (4000);
            bool same = a0.Estimator.StepsCounted == a1.Estimator.StepsCounted;
            for (int a = 0; a < 3; a++) {
                var x = a0.Estimator.Axes [a];
                var y = a1.Estimator.Axes [a];
                same &= x.Kappa.Equals (y.Kappa) && x.Se.Equals (y.Se) && x.Count == y.Count && x.Steps == y.Steps &&
                    x.Proven == y.Proven && x.Tripped == y.Tripped && y.LeakRefused == 0;
            }
            Check (same, "gate off: identical to the estimator of before, bit for bit" + Describe (a1.Estimator));

            // The fins weak (x 0.1), a proportional RCS with the command measured 10 % low: the RCS
            // carries r = 3e5 / (60 kN.m/deg x 0.1 x 20 deg x 1000) = 2.5 times the fins' increment.
            const double rcs = 3.0e5, eps = 0.1;
            double r = rcs / (60.0 * 0.1 * 20.0 * 1000.0);
            var weak = new Flight { QScale = 0.1, RcsPerCommand = rcs, RcsError = eps, Noise = 2e2 };
            weak.Run (4000);
            Trace ("leak, weak fins, no gate", weak);
            var wx = weak.Estimator.Axes [0];
            // (beta is a little under r: the fins slew at 40 deg/s toward their target while the RCS follows
            // the command at once; the bias is eps beta, kappa_hat = 1 + eps beta.)
            Check (wx.LeakBeta > 0.5 * r && wx.LeakBeta <= 1.1 * r,
                "no gate: the published leak slope is about r = " + r.ToString ("G3") + " (" + wx.LeakBeta.ToString ("G4") + ")");
            Check (Near (wx.Kappa, 1.0 + eps * wx.LeakBeta, 0.03) && wx.Kappa > 1.15 && wx.Proven,
                "no gate: the leak is PROVEN as kappa 1 + eps beta (" + (1.0 + eps * wx.LeakBeta).ToString ("G4") + ")" +
                Describe (weak.Estimator));
            var weakGated = new Flight { QScale = 0.1, RcsPerCommand = rcs, RcsError = eps, Noise = 2e2, LeakGate = true };
            weakGated.Run (4000);
            Trace ("leak, weak fins, gate", weakGated);
            var gx = weakGated.Estimator.Axes [0];
            // The gate keeps only the steps the fins carry (their slew while the command, hence the RCS,
            // holds): kappa comes back to 1 within eps LeakMax.
            Check (gx.LeakRefused > 1000 && gx.Steps > 0 && !gx.Tripped &&
                Math.Abs (gx.Kappa - 1.0) <= eps * SurfaceKappa.LeakMax + 3.0 * gx.Se,
                "gate: the leaky steps refused (" + gx.LeakRefused + "), kappa within eps LeakMax of 1" +
                Describe (weakGated.Estimator));
            Check (double.IsNaN (gx.LeakBeta) || Math.Abs (gx.LeakBeta) <= SurfaceKappa.LeakMax + 1e-12,
                "gate: |beta| of the counted steps at most LeakMax (" + gx.LeakBeta.ToString ("G4") + ")");

            // The fins strong (x 2, r = 1/8): the gate counts; kappa within eps LeakMax of 1.
            var strong = new Flight { QScale = 2.0, RcsPerCommand = rcs, RcsError = eps, Noise = 2e2, LeakGate = true };
            strong.Run (4000);
            Trace ("leak, strong fins, gate", strong);
            var sx = strong.Estimator.Axes [0];
            Check (sx.Proven && !sx.Tripped && Math.Abs (sx.Kappa - 1.0) <= eps * SurfaceKappa.LeakMax + 3.0 * sx.Se,
                "gate, strong fins: proven within eps LeakMax of 1" + Describe (strong.Estimator));
            Check (sx.Steps > 2000 && Math.Abs (sx.LeakBeta) <= SurfaceKappa.LeakMax,
                "gate, strong fins: most steps counted (" + sx.Steps + "), |beta| " + sx.LeakBeta.ToString ("G4"));

            // an earlier change's unit under the gate: the gate compares in physical N.m (the law's unit is what kappa
            // judges), so the steps are counted as for a right replica and kappa reads 1000, out, never proven.
            var unit = new Flight { LawScale = 1.0, LeakGate = true };
            unit.Run (6000);
            Trace ("leak gate, an earlier change unit", unit);
            Check (!unit.EverProven && unit.Estimator.TrippedAxes () == 7 &&
                Near (unit.Estimator.Axes [0].Kappa / 1000.0, 1.0, 0.05) && unit.Estimator.Axes [0].Steps > 2000,
                "gate: an earlier change's unit still out of tolerance at 1000 on every axis, its steps counted" + Describe (unit.Estimator));

            // A real defect stays out under the gate (x 2.5 physics, strong fins).
            var defect = new Flight { QScale = 2.0, RcsPerCommand = rcs, RcsError = eps, Noise = 2e2, LeakGate = true,
                Physics = 2.5 };
            defect.Run (6000);
            Check (!defect.EverProven && (defect.Estimator.TrippedAxes () & 5) == 5,
                "gate: a real defect x2.5 still out of tolerance, never proven" + Describe (defect.Estimator));

            // No allocation per step with the gate.
            var e = new SurfaceKappaEstimator ();
            double[] w = new double[3], i = { 1e8, 1e7, 1e8 }, m = new double[3], d = new double[3], n = new double[3];
            var rng = new Random (5);
            for (int t = 0; t < 400; t++) {
                for (int k = 0; k < 3; k++) {
                    w [k] = rng.NextDouble ();
                    d [k] = rng.NextDouble ();
                    m [k] = rng.NextDouble ();
                }
                e.Step (t, t, true, 0.02, 18.0, 0.7, w, w, i, m, false, true, d, n, 7, true);
            }
            long before = GC.GetAllocatedBytesForCurrentThread ();
            for (int t = 400; t < 10000; t++) {
                for (int k = 0; k < 3; k++) {
                    w [k] = rng.NextDouble ();
                    d [k] = rng.NextDouble ();
                    m [k] = rng.NextDouble ();
                }
                e.Step (t, t, true, 0.02, 18.0, 0.7, w, w, i, m, false, true, d, n, 7, true);
            }
            Check (GC.GetAllocatedBytesForCurrentThread () - before == 0, "gate: no allocation per step");
            Check ((e.LastCountedAxes & ~7) == 0, "LastCountedAxes within the axes");
            e.Reset ();
            Check (e.Axes [0].LeakRefused == 0 && double.IsNaN (e.Axes [1].LeakBeta) && e.LastCountedAxes == 0,
                "Reset forgets the leak too");
        }

        // ------------------------------------------------------------------ the physical inertia

        static void TestPhysicalInertia ()
        {
            // One KRE titanium grid fin of the super hope: 0.7 t, its origin 1.81 m from the axis, its mass
            // centre 0.6 m further along the radius (CoMOffset), 20.6 m above the CoM. Vessel.MOI takes the
            // mass at the origin: the roll and the yaw axis miss m (2.41^2 - 1.81^2), the pitch axis (along
            // the offset) nothing.
            double[] off = new double[3];
            bool ok = SurfaceKappa.AddInertiaOffset (700.0, new[] { 2.41, 20.6, 0.0 }, new[] { 1.81, 20.6, 0.0 }, off);
            Check (ok && Near (off [0], 0.0, 1e-9) && Near (off [1], 700.0 * 2.532, 1e-9) && Near (off [2], 700.0 * 2.532, 1e-9),
                "one fin: (0, 1772.4, 1772.4) kg.m^2 (" + off [0] + ", " + off [1] + ", " + off [2] + ")");
            // A mass on its own origin adds nothing, wherever it is.
            double[] none = new double[3];
            SurfaceKappa.AddInertiaOffset (5000.0, new[] { 1.3, -7.0, 0.4 }, new[] { 1.3, -7.0, 0.4 }, none);
            Check (none [0] == 0.0 && none [1] == 0.0 && none [2] == 0.0, "a mass at its origin: nothing");
            // The super hope (FMRS save of a recorded flight, CoM on the axis, 21.87 m under the
            // root): four fins at 1.8103657 m, 20.6214 m above the CoM, 0.6 m out; four structural wings (0.1 t)
            // at 45 deg, 1.8972908 m out, 3.9914 m under it, CoMOffset 0.7 m out and 0.6 m down.
            double[] hope = new double[3];
            double rf = 1.8103657, rw = 1.8972908, yf = 20.6214, yw = -3.9914;
            for (int k = 0; k < 4; k++) {
                double cos = Math.Cos (k * Math.PI / 2.0), sin = Math.Sin (k * Math.PI / 2.0);
                SurfaceKappa.AddInertiaOffset (700.0, new[] { (rf + 0.6) * cos, yf, (rf + 0.6) * sin },
                    new[] { rf * cos, yf, rf * sin }, hope);
                double cw = Math.Cos (Math.PI / 4.0 + k * Math.PI / 2.0), sw = Math.Sin (Math.PI / 4.0 + k * Math.PI / 2.0);
                SurfaceKappa.AddInertiaOffset (100.0, new[] { (rw + 0.7) * cw, yw - 0.6, (rw + 0.7) * sw },
                    new[] { rw * cw, yw, rw * sw }, hope);
            }
            Check (Near (hope [1], 8349.3, 0.5) && Near (hope [0], hope [2], 1e-6) && Near (hope [0], 6234.5, 0.5),
                "the super hope: 8 349 kg.m^2 on roll, 6 234 on pitch and yaw (" + hope [0].ToString ("F1") + ", " +
                hope [1].ToString ("F1") + ", " + hope [2].ToString ("F1") + ")");
            // Refused inputs add nothing.
            double[] keep = { 1.0, 2.0, 3.0 };
            bool r1 = SurfaceKappa.AddInertiaOffset (double.NaN, new[] { 1.0, 0.0, 0.0 }, new double[3], keep);
            bool r2 = SurfaceKappa.AddInertiaOffset (-1.0, new[] { 1.0, 0.0, 0.0 }, new double[3], keep);
            bool r3 = SurfaceKappa.AddInertiaOffset (1.0, new[] { double.PositiveInfinity, 0.0, 0.0 }, new double[3], keep);
            bool r4 = SurfaceKappa.AddInertiaOffset (1.0, new double[3], new[] { 0.0, double.NaN, 0.0 }, keep);
            Check (!r1 && !r2 && !r3 && !r4 && keep [0] == 1.0 && keep [1] == 2.0 && keep [2] == 3.0,
                "non-finite or negative inputs refused, nothing added");

            // The mode rule: 32 only with 8 (and the model on).
            Check (SurfaceModel.ModeFlagKappaInertia == 32 && (SurfaceModel.ModeFlags & 32) == 32, "flag 32 among the mode flags");
            bool valid = true;
            foreach (int m in new[] { 41, 42, 45, 46, 57, 58, 61, 62 })
                valid &= SurfaceModel.ValidMode (m);
            foreach (int m in new[] { 32, 33, 34, 37, 38, 49, 50, 40, 56 })
                valid &= !SurfaceModel.ValidMode (m);
            Check (valid, "32 valid with 8 and the model on (41, 42, 45, 46, 57, 58, 61, 62), refused otherwise");

            // A body whose true inertia is the served one plus the offset (3.3 % on roll, 0.065 % on pitch and
            // yaw: the super hope in the coast). Same flight, same noise; only the inertia given changes.
            var served = new Flight ();
            served.InertiaLeftOut [0] = 1.3e5;
            served.InertiaLeftOut [1] = 3.3e5;
            served.InertiaLeftOut [2] = 1.3e5;
            served.Run (6000);
            var physical = new Flight { PhysicalInertia = true };
            physical.InertiaLeftOut [0] = 1.3e5;
            physical.InertiaLeftOut [1] = 3.3e5;
            physical.InertiaLeftOut [2] = 1.3e5;
            physical.Run (6000);
            Trace ("inertia served", served);
            Trace ("inertia physical", physical);
            double[] full = { 2.0e8, 1.0e7, 2.0e8 };
            for (int a = 0; a < 3; a++) {
                var s = served.Estimator.Axes [a];
                var p = physical.Estimator.Axes [a];
                double ratio = full [a] / (full [a] + served.InertiaLeftOut [a]);
                Check (Near (s.Kappa / p.Kappa, ratio, 2e-3) && Near (s.Nuisance / p.Nuisance, ratio, 5e-3),
                    "axis " + a + ": served inertia, kappa and the nuisance slope scaled by I_served / I_true = " +
                    ratio.ToString ("G5") + " (" + (s.Kappa / p.Kappa).ToString ("G5") + ", " +
                    (s.Nuisance / p.Nuisance).ToString ("G5") + ")");
                Check (p.Proven && !p.Tripped && Math.Abs (p.Kappa - 1.0) <= p.T * p.Se,
                    "axis " + a + ": physical inertia, kappa consistent with 1" + Describe (physical.Estimator));
                Check (s.Proven && !s.Tripped, "axis " + a + ": the served inertia's bias stays inside the band (proven)");
            }
            Check (Math.Abs (served.Estimator.Axes [1].Kappa - 1.0) > Math.Abs (physical.Estimator.Axes [1].Kappa - 1.0),
                "roll: the physical inertia brings kappa closer to 1 (" + served.Estimator.Axes [1].Kappa.ToString ("G5") +
                " -> " + physical.Estimator.Axes [1].Kappa.ToString ("G5") + ")");
        }
    }
}
