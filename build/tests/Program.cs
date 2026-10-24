// -----------------------------------------------------------------------------------
// AttitudeV1 pure-maths tests and golden vectors. See AttitudeTests.csproj.
//
// Usage:
//     dotnet run --project build/tests/AttitudeTests.csproj -c Release [-- <golden.json>]
// Without an argument the golden vectors go to build/tests/golden/attitude_v1_golden.json.
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using KRPC.SpaceCenter;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static class Program
    {
        static int failures;
        static int checks;

        public static int Main (string[] args)
        {
            string golden = args.Length > 0 ? args [0]
                : Path.Combine (AppContext.BaseDirectory, "..", "..", "golden",
                    "attitude_v1_golden.json");
            // bin/<config>/ is two levels below build/tests; normalise for the message.
            golden = Path.GetFullPath (golden);

            TestPointingError ();
            TestSqrtReference ();
            TestBiquad ();
            TestBoxLeastSquaresAgainstBruteForce ();
            TestWarmStartSameSolution ();
            TestStepAllocatesExactlyWhenAuthorityIsAmple ();
            TestHedgeGrowsUnderSaturationAndLeaks ();
            TestStepRejectsNonFiniteInput ();
            TestTickGapRestartsFilters ();
            TestFilteredWithoutReference ();

            // M3: per-axis guards, write rule, roll command, coupling, fly-by-wire chain.
            TestAxisGuards ();
            TestWriteRules ();
            TestRollCommandAndCoupling ();
            TestFlyByWireChain ();

            // QW6: cached reflection readers against verbatim copies of the legacy ones.
            int reflectionChecks;
            int reflectionFailures = ReflectionTests.Run (out reflectionChecks);
            checks += reflectionChecks;
            failures += reflectionFailures;

            // Lateral force (chantier 19 suite): the pure layer in front of the law. Writes
            // nothing into the golden vectors (the law and its mirror are unchanged); the free
            // authority's (option 256) go to their own file.
            int translationChecks;
            int translationFailures = TranslationTests.Run (out translationChecks);
            checks += translationChecks;
            failures += translationFailures;

            // the rules of the non-active vessel (AttitudeArmV2 options). Writes nothing
            // into the golden vectors.
            int inactiveChecks;
            int inactiveFailures = InactiveVesselTests.Run (out inactiveChecks);
            checks += inactiveChecks;
            failures += inactiveFailures;

            // the per-pivot mapping of multi-pivot gimbal modules. Writes nothing into the
            // golden vectors.
            int pivotChecks;
            int pivotFailures = PivotTests.Run (out pivotChecks);
            checks += pivotChecks;
            failures += pivotFailures;

            // Refactor lot 1: the options of AttitudeOptionsV1 (per-family blocking,
            // evidence aging, non-converged iterate served) and the property on a fake game
            // clock. Writes nothing into the golden vectors.
            int lot1Checks;
            int lot1Failures = FamilyEvidenceTests.Run (out lot1Checks);
            checks += lot1Checks;
            failures += lot1Failures;

            // Refactor R1 (plan 2.4): the effector registry's pure rules. Writes nothing into
            // the golden vectors.
            int effectorChecks;
            int effectorFailures = EffectorTests.Run (out effectorChecks);
            checks += effectorChecks;
            failures += effectorFailures;

            // Lot 3, 3.1: SurfaceModelV1's pure replica and guard. Writes nothing into the golden
            // vectors (the free torque of the law is born false).
            int surfaceChecks;
            int surfaceFailures = SurfaceTests.Run (out surfaceChecks);
            checks += surfaceChecks;
            failures += surfaceFailures;

            // the closed loop of the armed surfaces (the unit of the fins' torque in the law).
            // Writes nothing into the golden vectors.
            int loopChecks;
            int loopFailures = SurfaceLoopTests.Run (out loopChecks);
            checks += loopChecks;
            failures += loopFailures;

            // the physical slope kappa of the surfaces. Writes nothing into the golden vectors.
            int kappaChecks;
            int kappaFailures = SurfaceKappaTests.Run (out kappaChecks);
            checks += kappaChecks;
            failures += kappaFailures;

            // the roll excitation of kappa. Writes nothing into the golden vectors.
            int rollChecks;
            int rollFailures = SurfaceKappaRollTests.Run (out rollChecks);
            checks += rollChecks;
            failures += rollFailures;

            // a surface engaged only on axes the loop owns; the bumpless change of the law's M.
            // Writes nothing into the golden vectors.
            int ownershipChecks;
            int ownershipFailures = SurfaceOwnershipTests.Run (out ownershipChecks);
            checks += ownershipChecks;
            failures += ownershipFailures;

            // the contact hold of option 8 (pure rule). Writes nothing into the golden vectors.
            int holdChecks;
            int holdFailures = ContactHoldTests.Run (out holdChecks);
            checks += holdChecks;
            failures += holdFailures;

            // the coast write bits of option 16 (pure rules). Writes nothing into the golden vectors.
            int coastChecks;
            int coastFailures = CoastWriteTests.Run (out coastChecks);
            checks += coastChecks;
            failures += coastFailures;

            // the RCS torque in flight of option 32 (pure maths, and the closed loop on one axis with
            // the law and a stock RCS). Writes nothing into the golden vectors.
            int inFlightChecks;
            int inFlightFailures = RcsInFlightTests.Run (out inFlightChecks);
            checks += inFlightChecks;
            failures += inFlightFailures;

            // the boostback write bit of option 64 (pure rules, and the channel's sequence over a
            // boostback). Writes nothing into the golden vectors.
            int boostbackChecks;
            int boostbackFailures = BoostbackWriteTests.Run (out boostbackChecks);
            checks += boostbackChecks;
            failures += boostbackFailures;

            // option 128, the gimbals without thrust to neutral and their columns at the coming
            // thrust (pure rules, the spool, the law, a closed loop). Its golden vectors go to their own file.
            int trimChecks;
            int trimFailures = GimbalTrimTests.Run (out trimChecks);
            checks += trimChecks;
            failures += trimFailures;

            // the pure part of gimbal_authority_v1 (rest pose, per kN). Writes nothing into the golden vectors.
            int authorityChecks;
            int authorityFailures = GimbalAuthorityTests.Run (out authorityChecks);
            checks += authorityChecks;
            failures += authorityFailures;

            // option 512, the main throttle at the physics step (MainThrottle.cs: orders, the flip's calage,
            // an earlier change's guard, a closed loop). Its golden vectors go to their own file.
            int throttleChecks;
            int throttleFailures = MainThrottleTests.Run (out throttleChecks);
            checks += throttleChecks;
            failures += throttleFailures;

            // option 1024, the roll pointed (AttitudeMath.RollError, RollTargetValid). Its golden vectors go to
            // their own file.
            int rollTargetChecks;
            int rollTargetFailures = RollTargetTests.Run (out rollTargetChecks);
            checks += rollTargetChecks;
            failures += rollTargetFailures;

            // option 2048, the reference ring (ReferenceRing.cs: validation, interpolation, K5's ramp under the
            // ring). Its golden vectors go to their own file.
            int ringChecks;
            int ringFailures = ReferenceRingTests.Run (out ringChecks);
            checks += ringChecks;
            failures += ringFailures;

            // options 4096 and 8192, the engine-by-engine allocation in shadow and writing
            // (EngineAllocation.cs). Its golden vectors go to their own file (the law's do not change).
            int allocationChecks;
            int allocationFailures = EngineAllocationTests.Run (out allocationChecks);
            checks += allocationChecks;
            failures += allocationFailures;

            // option 16384, the RCS as ModuleRCS fires it (RcsCombined.cs). Its golden vectors go to their own file (the
            // law's do not change: the law is not touched).
            int rcsChecks;
            int rcsFailures = RcsCombinedTests.Run (out rcsChecks);
            checks += rcsChecks;
            failures += rcsFailures;

            Directory.CreateDirectory (Path.GetDirectoryName (golden));
            WriteGolden (golden);
            // the golden vectors of the law with TrimNullColumns, beside the others (which do not change).
            string goldenTrim = Path.Combine (Path.GetDirectoryName (golden), "attitude_v1_golden_trim.json");
            WriteGoldenTrim (goldenTrim);
            // the golden vectors of the free authority (option 256), beside the others (which do not change).
            string goldenFree = Path.Combine (Path.GetDirectoryName (golden), "attitude_v1_golden_free.json");
            File.WriteAllText (goldenFree, TranslationTests.GoldenFreeJson ());
            // the golden vectors of the main throttle (option 512), beside the others (which do not change).
            string goldenThrottle = Path.Combine (Path.GetDirectoryName (golden), "attitude_v1_golden_throttle.json");
            File.WriteAllText (goldenThrottle, MainThrottleTests.GoldenJson ());
            // the golden vectors of the law with a roll target (option 1024), beside the others (unchanged).
            string goldenRoll = Path.Combine (Path.GetDirectoryName (golden), "attitude_v1_golden_roll.json");
            WriteGoldenRoll (goldenRoll);
            // the golden vectors of the reference ring (option 2048), beside the others (unchanged).
            string goldenRing = Path.Combine (Path.GetDirectoryName (golden), "attitude_v1_golden_ring.json");
            File.WriteAllText (goldenRing, ReferenceRingTests.GoldenJson ());
            // the golden vectors of the engine-by-engine allocation (options 4096 and 8192), beside the others
            // (which do not change).
            string goldenAllocation = Path.Combine (Path.GetDirectoryName (golden), "attitude_v1_golden_allocation.json");
            File.WriteAllText (goldenAllocation, EngineAllocationTests.GoldenAllocationJson ());
            // the golden vectors of option 16384 (the combined RCS), beside the others (which do not change).
            string goldenRcs = Path.Combine (Path.GetDirectoryName (golden), "attitude_v1_golden_rcs.json");
            File.WriteAllText (goldenRcs, RcsCombinedTests.GoldenJson ());

            Console.WriteLine ();
            Console.WriteLine ("checks: " + checks + ", failures: " + failures);
            Console.WriteLine ("golden vectors: " + golden);
            Console.WriteLine ("golden vectors (option 128): " + goldenTrim);
            Console.WriteLine ("golden vectors (option 256): " + goldenFree);
            Console.WriteLine ("golden vectors (option 512): " + goldenThrottle);
            Console.WriteLine ("golden vectors (option 1024): " + goldenRoll);
            Console.WriteLine ("golden vectors (option 2048): " + goldenRing);
            Console.WriteLine ("golden vectors (options 4096, 8192): " + goldenAllocation);
            Console.WriteLine ("golden vectors (option 16384): " + goldenRcs);
            if (failures > 0) {
                Console.Error.WriteLine ("ATTITUDE MATHS TESTS FAILED");
                return 1;
            }
            Console.WriteLine ("OK - AttitudeV1 maths");
            return 0;
        }

        // ------------------------------------------------------------------ helpers

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.Error.WriteLine ("FAIL: " + what);
            }
        }

        static bool Near (double a, double b, double tol)
        {
            return Math.Abs (a - b) <= tol * Math.Max (1.0, Math.Max (Math.Abs (a), Math.Abs (b)));
        }

        // ------------------------------------------------------------ pointing error

        static void TestPointingError ()
        {
            var e = new double[3];
            Check (AttitudeMath.PointingError (0, 1, 0, e) && e [0] == 0 && e [1] == 0 && e [2] == 0,
                "nose on target: zero error");
            AttitudeMath.PointingError (1, 0, 0, e);
            Check (Near (e [0], 0, 1e-15) && e [1] == 0 && Near (e [2], -Math.PI / 2, 1e-15),
                "target to the right: rotation about -z (omega x nose = +x)");
            AttitudeMath.PointingError (0, 0, 1, e);
            Check (Near (e [0], Math.PI / 2, 1e-15) && e [1] == 0 && Near (e [2], 0, 1e-15),
                "target to the belly: rotation about +x (omega x nose = +z)");
            AttitudeMath.PointingError (0, -2, 0, e);
            Check (Near (e [0], Math.PI, 1e-15) && e [1] == 0 && e [2] == 0,
                "antipodal target: pi about pitch");
            Check (!AttitudeMath.PointingError (0, 0, 0, e), "zero direction rejected");
            // Kinematic sign: d(nose)/dt = e x nose must point toward the target.
            var rng = new Random (7);
            for (int k = 0; k < 200; k++) {
                double dx = rng.NextDouble () * 2 - 1, dy = rng.NextDouble () * 2 - 1,
                    dz = rng.NextDouble () * 2 - 1;
                if (!AttitudeMath.PointingError (dx, dy, dz, e))
                    continue;
                // e x (0,1,0) = (-e_z, 0, e_x)
                double vx = -e [2], vz = e [0];
                double dot = vx * dx + vz * dz;
                Check (dot >= -1e-12, "error rotation turns the nose toward the target");
                double n = Math.Sqrt (dx * dx + dy * dy + dz * dz);
                double angle = Math.Acos (Math.Max (-1, Math.Min (1, dy / n)));
                double mag = Math.Sqrt (e [0] * e [0] + e [2] * e [2]);
                Check (Near (mag, angle, 1e-9), "error magnitude is the angle to the target");
            }
        }

        // --------------------------------------------------------- sqrt reference

        static void TestSqrtReference ()
        {
            double eta = 0.6, kLin = 0.75, wMax = 0.2;
            var rng = new Random (11);
            for (int k = 0; k < 200; k++) {
                double alpha = 0.001 + rng.NextDouble () * 0.5;
                double prev = 0.0;
                for (int i = 0; i <= 400; i++) {
                    double err = i * 0.005;
                    double w = AttitudeMath.SqrtReference (err, alpha, eta, kLin, wMax);
                    Check (w >= prev - 1e-15, "sqrt reference monotone in |e|");
                    Check (w <= wMax + 1e-15 && w >= 0, "sqrt reference bounded by omega_max");
                    double wn = AttitudeMath.SqrtReference (-err, alpha, eta, kLin, wMax);
                    Check (wn == -w, "sqrt reference odd");
                    Check (w <= kLin * err + 1e-15 &&
                           w <= Math.Sqrt (2 * eta * alpha * err) + 1e-15,
                        "sqrt reference below both branches");
                    prev = w;
                }
            }
            Check (AttitudeMath.SqrtReference (0, 0.1, eta, kLin, wMax) == 0, "zero error, zero rate");
        }

        // ------------------------------------------------------------------ biquad

        static void TestBiquad ()
        {
            var f = new Biquad3 ();
            f.Configure (0.02, 18.0, 0.7);
            var x = new double[3];
            var y = new double[3];
            // DC: primed on the first sample, a constant input stays constant.
            x [0] = 2.5; x [1] = -1.0; x [2] = 0.0;
            for (int i = 0; i < 50; i++)
                f.Step (x, y);
            Check (Near (y [0], 2.5, 1e-12) && Near (y [1], -1.0, 1e-12) && y [2] == 0,
                "unit DC gain");
            Check (Near (f.B0 + f.B1 + f.B2, 1 + f.A1 + f.A2, 1e-12), "coefficients: H(1) = 1");
            // Step from zero: settles to 1, overshoot of a zeta 0.7 second order (< 10 %).
            f.Reset ();
            x [0] = 0; x [1] = 0; x [2] = 0;
            f.Step (x, y);
            x [0] = 1;
            double peak = 0;
            for (int i = 0; i < 100; i++) {
                f.Step (x, y);
                peak = Math.Max (peak, y [0]);
            }
            Check (Near (y [0], 1.0, 1e-6), "step settles to the input");
            Check (peak < 1.10, "step overshoot below 10 %");
            // Nyquist: an alternating sign is rejected.
            f.Reset ();
            double amp = 0;
            for (int i = 0; i < 200; i++) {
                x [0] = (i % 2 == 0) ? 1 : -1;
                f.Step (x, y);
                if (i > 100)
                    amp = Math.Max (amp, Math.Abs (y [0]));
            }
            Check (amp < 0.02, "Nyquist rate rejected (" + amp.ToString ("G4") + ")");
        }

        // -------------------------------------------------- bounded least squares

        static double Objective (double[] A, int m, int n, double[] b, double[] x)
        {
            double sum = 0;
            for (int i = 0; i < m; i++) {
                double s = -b [i];
                for (int j = 0; j < n; j++)
                    s += A [i * n + j] * x [j];
                sum += s * s;
            }
            return sum;
        }

        /// <summary>Every free/lower/upper assignment; the best feasible one.</summary>
        static double[] BruteForce (double[] A, int m, int n, double[] b, double[] lb, double[] ub,
            out double best)
        {
            best = double.PositiveInfinity;
            double[] bestX = null;
            int combos = 1;
            for (int j = 0; j < n; j++)
                combos *= 3;
            var x = new double[n];
            for (int c = 0; c < combos; c++) {
                int code = c;
                var freeList = new List<int> ();
                for (int j = 0; j < n; j++) {
                    int s = code % 3;
                    code /= 3;
                    if (s == 0)
                        freeList.Add (j);
                    else
                        x [j] = s == 1 ? lb [j] : ub [j];
                }
                int nf = freeList.Count;
                if (nf > 0) {
                    // r = b - A_fixed x_fixed ; solve (A_F^T A_F) x_F = A_F^T r
                    var r = new double[m];
                    for (int i = 0; i < m; i++) {
                        double s = b [i];
                        for (int j = 0; j < n; j++)
                            if (!freeList.Contains (j))
                                s -= A [i * n + j] * x [j];
                        r [i] = s;
                    }
                    var H = new double[nf * nf];
                    var g = new double[nf];
                    for (int p = 0; p < nf; p++) {
                        for (int i = 0; i < m; i++)
                            g [p] += A [i * n + freeList [p]] * r [i];
                        for (int q = 0; q < nf; q++)
                            for (int i = 0; i < m; i++)
                                H [p * nf + q] += A [i * n + freeList [p]] * A [i * n + freeList [q]];
                    }
                    var L = new double[nf * nf];
                    if (!AttitudeMath.CholeskySolve (H, L, g, nf, nf))
                        continue;
                    bool feasible = true;
                    for (int p = 0; p < nf; p++) {
                        int j = freeList [p];
                        if (g [p] < lb [j] - 1e-12 || g [p] > ub [j] + 1e-12)
                            feasible = false;
                        x [j] = g [p];
                    }
                    if (!feasible)
                        continue;
                }
                double obj = Objective (A, m, n, b, x);
                if (obj < best) {
                    best = obj;
                    bestX = (double[])x.Clone ();
                }
            }
            return bestX;
        }

        static AttitudeWork RandomProblem (Random rng, int n, out int m)
        {
            m = 3 + n;
            var w = new AttitudeWork ();
            w.Resize (m, n);
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < n; j++)
                    w.A [i * n + j] = (rng.NextDouble () * 2 - 1) * Math.Pow (10, rng.Next (-2, 2));
            double reg = Math.Pow (10, -rng.Next (1, 4));
            for (int j = 0; j < n; j++)
                w.A [(3 + j) * n + j] = reg * (0.5 + rng.NextDouble ());
            for (int i = 0; i < m; i++)
                w.Bv [i] = (rng.NextDouble () * 2 - 1) * (i < 3 ? 3.0 : 0.01);
            for (int j = 0; j < n; j++) {
                double lo = -rng.NextDouble () * 2;
                double hi = rng.NextDouble () * 2;
                int kind = rng.Next (10);
                if (kind == 0) {            // degenerate: fixed actuator
                    lo = 0.0;
                    hi = 0.0;
                } else if (kind == 1) {     // start outside: u0 beyond the box
                    lo = 0.1 + rng.NextDouble ();
                    hi = lo + rng.NextDouble ();
                }
                w.Lb [j] = lo;
                w.Ub [j] = hi;
                w.W [j] = AttitudeMath.SetFree;
            }
            return w;
        }

        static void TestBoxLeastSquaresAgainstBruteForce ()
        {
            var rng = new Random (2026);
            int worstIterations = 0;
            for (int trial = 0; trial < 1500; trial++) {
                int n = 1 + rng.Next (7);
                int m;
                var w = RandomProblem (rng, n, out m);
                double best;
                var reference = BruteForce ((double[])w.A.Clone (), m, n, (double[])w.Bv.Clone (),
                    (double[])w.Lb.Clone (), (double[])w.Ub.Clone (), out best);
                bool converged;
                int it = AttitudeMath.SolveBoxLeastSquares (w, m, n, 50, out converged);
                worstIterations = Math.Max (worstIterations, it);
                Check (converged, "active set converges (trial " + trial + ")");
                Check (reference != null, "brute force finds a feasible point");
                if (reference == null)
                    continue;
                double obj = Objective (w.A, m, n, w.Bv, w.X);
                Check (obj <= best + 1e-9 * Math.Max (1, best),
                    "active set objective equals the brute-force optimum (trial " + trial + ")");
                for (int j = 0; j < n; j++) {
                    Check (w.X [j] >= w.Lb [j] - 1e-12 && w.X [j] <= w.Ub [j] + 1e-12,
                        "solution inside the box");
                    Check (Near (w.X [j], reference [j], 1e-6),
                        "solution equals the brute-force point (trial " + trial + ")");
                }
            }
            Console.WriteLine ("  WLS vs brute force: 1500 draws, worst iteration count " + worstIterations);
        }

        static void TestWarmStartSameSolution ()
        {
            var rng = new Random (99);
            for (int trial = 0; trial < 500; trial++) {
                int n = 1 + rng.Next (7);
                int m;
                var cold = RandomProblem (rng, n, out m);
                var warm = new AttitudeWork ();
                warm.Resize (m, n);
                Array.Copy (cold.A, warm.A, cold.A.Length);
                Array.Copy (cold.Bv, warm.Bv, cold.Bv.Length);
                Array.Copy (cold.Lb, warm.Lb, n);
                Array.Copy (cold.Ub, warm.Ub, n);
                for (int j = 0; j < n; j++)
                    warm.W [j] = rng.Next (3) - 1;     // arbitrary, possibly wrong, guess
                bool c1, c2;
                AttitudeMath.SolveBoxLeastSquares (cold, m, n, 50, out c1);
                AttitudeMath.SolveBoxLeastSquares (warm, m, n, 50, out c2);
                Check (c1 && c2, "cold and warm starts converge");
                for (int j = 0; j < n; j++)
                    Check (Near (cold.X [j], warm.X [j], 1e-6), "warm start reaches the same optimum");
            }
        }

        // ------------------------------------------------------------ law, closed

        static void FillEngineLikeInput (AttitudeInput inp, double bScale)
        {
            // Four gimbals (8 columns) + three ctrlState channels, a booster-like geometry.
            inp.Resize (11);
            inp.Dt = 0.02;
            inp.Inertia [0] = 6.0e7; inp.Inertia [1] = 4.0e6; inp.Inertia [2] = 6.0e7;
            for (int j = 0; j < 11; j++) {
                inp.B [0 * 11 + j] = 0; inp.B [1 * 11 + j] = 0; inp.B [2 * 11 + j] = 0;
            }
            for (int g = 0; g < 4; g++) {
                // x-deflection: pitch torque ; y-deflection: yaw torque ; small roll coupling
                inp.B [0 * 11 + 2 * g] = bScale * 1.0e5;
                inp.B [1 * 11 + 2 * g] = bScale * ((g % 2 == 0) ? 2.0e3 : -2.0e3);
                inp.B [2 * 11 + 2 * g + 1] = bScale * 1.0e5;
                inp.UMin [2 * g] = -2; inp.UMax [2 * g] = 2;
                inp.UMin [2 * g + 1] = -2; inp.UMax [2 * g + 1] = 2;
                inp.Priority [2 * g] = 1.0; inp.Priority [2 * g + 1] = 1.0;
            }
            for (int c = 0; c < 3; c++) {
                int j = 8 + c;
                inp.B [c * 11 + j] = -bScale * 2.0e4;
                inp.UMin [j] = -1; inp.UMax [j] = 1;
                inp.Priority [j] = 3.0;
            }
        }

        static void TestStepAllocatesExactlyWhenAuthorityIsAmple ()
        {
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            FillEngineLikeInput (inp, 10.0);
            inp.OmegaDotValid = true;
            inp.ReferenceValid = true;
            inp.Error [0] = 0.02; inp.Error [2] = -0.01;
            for (int t = 0; t < 30; t++)
                AttitudeMath.Step (g, inp, st, work, o);
            Check (o.Computed && o.Converged, "law computes and converges");
            for (int a = 0; a < 3; a++) {
                Check (Near (o.TorqueAllocated [a], o.TorqueAllocTarget [a], 1e-3),
                    "ample authority: allocated torque equals the target (axis " + a + ")");
                Check (Math.Abs (o.NuHedge [a]) < 1e-4, "ample authority: no hedge");
            }
            Check (o.OmegaRef [0] > 0 && o.OmegaRef [2] < 0, "reference rate follows the error sign");
        }

        static void TestHedgeGrowsUnderSaturationAndLeaks ()
        {
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            FillEngineLikeInput (inp, 0.01);        // almost no authority
            // The actuators already sit near the bound that the demand needs: alpha
            // (computed from the full span) promises more than the box can still give.
            for (int gi = 0; gi < 4; gi++)
                inp.U0 [2 * gi] = 1.5;
            inp.U0 [8] = -0.9;
            inp.OmegaDotValid = true;
            inp.ReferenceValid = true;
            inp.Error [0] = 0.5;
            // Force a demand the actuators cannot meet: the vessel spins the wrong way.
            inp.Omega [0] = -0.05;
            for (int t = 0; t < 100; t++)
                AttitudeMath.Step (g, inp, st, work, o);
            Check (o.NuHedge [0] > 0, "saturated: positive unrealizable pseudo-control");
            Check (o.OmegaPch [0] > 0, "saturated: the hedge slows the reference (w_pch > 0)");
            Check (o.SaturationFraction > 0, "saturated: active bounds reported");
            // Remove the error and the spin: the hedge leaks back to zero.
            FillEngineLikeInput (inp, 10.0);
            inp.Error [0] = 0; inp.Omega [0] = 0;
            double before = o.OmegaPch [0];
            for (int t = 0; t < 400; t++)
                AttitudeMath.Step (g, inp, st, work, o);
            Check (Math.Abs (o.OmegaPch [0]) < 0.05 * Math.Abs (before) + 1e-9, "hedge leaks away");
        }

        static void TestStepRejectsNonFiniteInput ()
        {
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            FillEngineLikeInput (inp, 1.0);
            inp.OmegaDotValid = true;
            inp.ReferenceValid = true;
            inp.Torque [1] = double.NaN;
            AttitudeMath.Step (g, inp, st, work, o);
            Check (!o.Computed, "non-finite torque: nothing computed");
            inp.Torque [1] = 0;
            inp.B [5] = double.PositiveInfinity;
            AttitudeMath.Step (g, inp, st, work, o);
            Check (!o.Computed, "non-finite B: nothing computed");
        }

        static void TestTickGapRestartsFilters ()
        {
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            FillEngineLikeInput (inp, 1.0);
            inp.OmegaDotValid = true;
            inp.ReferenceValid = true;
            inp.OmegaDotMeasured [0] = 1.0;
            for (int t = 0; t < 10; t++)
                AttitudeMath.Step (g, inp, st, work, o);
            inp.OmegaDotValid = false;
            AttitudeMath.Step (g, inp, st, work, o);
            Check (!o.Computed && !st.FilterOmegaDot.Primed && !st.FilterTorque.Primed,
                "tick gap: both filters restart");
            inp.OmegaDotValid = true;
            inp.OmegaDotMeasured [0] = 5.0;
            AttitudeMath.Step (g, inp, st, work, o);
            Check (o.Computed && Near (o.OmegaDotF [0], 5.0, 1e-12),
                "after a gap the filter primes on the new sample");
        }

        static void TestFilteredWithoutReference ()
        {
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            FillEngineLikeInput (inp, 1.0);
            inp.OmegaDotValid = true;
            inp.ReferenceValid = false;
            inp.OmegaDotMeasured [2] = 0.3;
            AttitudeMath.Step (g, inp, st, work, o);
            Check (!o.Computed && o.Filtered && Near (o.OmegaDotF [2], 0.3, 1e-12),
                "no reference: the filters still run and publish");
            inp.OmegaDotValid = false;
            AttitudeMath.Step (g, inp, st, work, o);
            Check (!o.Computed && !o.Filtered, "tick gap: nothing filtered");
        }

        // ---------------------------------------------------------- M3: axis guards

        /// <summary>
        /// Deterministic plant for the per-axis guards (also a golden scenario): pitch fully
        /// saturated (every positive-pitch actuator at its bound, a 31 deg pitch error), a
        /// roll spin of 0.3 rad/s for ticks 10-69 against a roll limit of 0.1 rad/s, a yaw
        /// acceleration ramp that no torque explains for ticks 20-84, a tick gap at 95.
        /// </summary>
        static void GuardScenarioTick (AttitudeInput inp, int t)
        {
            FillEngineLikeInput (inp, 1.0);
            for (int gi = 0; gi < 4; gi++) {
                inp.U0 [2 * gi] = inp.UMax [2 * gi];
                inp.U0 [2 * gi + 1] = 0.0;
            }
            inp.U0 [8] = -1.0;           // ctrlState pitch: B < 0, at the bound of + torque
            inp.U0 [9] = 0.0;
            inp.U0 [10] = 0.0;
            for (int j = 0; j < 11; j++)
                inp.UPref [j] = 0.0;
            AttitudeMath.PointingError (0.0, 1.0, 0.6, inp.Error);
            inp.Omega [0] = -0.02;
            inp.Omega [1] = t >= 10 && t < 70 ? 0.3 : 0.001;
            inp.Omega [2] = 0.0;
            inp.OmegaDotMeasured [0] = 0.0;
            inp.OmegaDotMeasured [1] = 0.0;
            inp.OmegaDotMeasured [2] = t >= 20 && t < 85 ? 0.02 * (t - 20) : 0.0;
            inp.Torque [0] = 1.0e4;
            inp.Torque [1] = 0.0;
            inp.Torque [2] = 0.0;
            for (int a = 0; a < 3; a++) {
                inp.OmegaFf [a] = 0.0;
                inp.OmegaDotFf [a] = 0.0;
            }
            inp.OmegaMax = 0.2;
            inp.OmegaMaxAxis [0] = 0.2;
            inp.OmegaMaxAxis [1] = 0.1;
            inp.OmegaMaxAxis [2] = 0.2;
            inp.OmegaDotValid = t != 95;
            inp.ReferenceValid = true;
        }

        const int GuardScenarioTicks = 110;

        static void TestAxisGuards ()
        {
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            int firstRollOmega = -1, lastRollOmega = -1, firstPitchSat = -1, firstYawResidual = -1;
            bool rollOther = false;
            for (int t = 0; t < GuardScenarioTicks; t++) {
                GuardScenarioTick (inp, t);
                AttitudeMath.Step (g, inp, st, work, o);
                if (o.AxisOmegaGuard [1]) {
                    if (firstRollOmega < 0)
                        firstRollOmega = t;
                    lastRollOmega = t;
                }
                if (o.AxisSaturationGuard [0] && firstPitchSat < 0)
                    firstPitchSat = t;
                if (o.AxisResidualGuard [2] && firstYawResidual < 0)
                    firstYawResidual = t;
                if (o.AxisResidualGuard [1] || o.AxisSaturationGuard [1])
                    rollOther = true;
                Check (o.AxisGuardMask == AttitudeMath.AxisGuardMask (o), "axis guard mask coherent");
                if (t == 95)
                    Check (!o.Filtered && o.AxisGuardMask == 0, "tick gap: no axis guard published");
                if (t == 96)
                    Check (!o.AxisSaturationGuard [0], "tick gap: the saturation timer restarts");
                if (o.Computed)
                    Check (Near (o.Unrealized [0], Math.Abs (o.NuHedge [0]) / o.Alpha [0], 1e-15),
                        "unrealized = |nu_h| / alpha");
            }
            // Roll spin from tick 10: guard after more than 0.5 s, cleared when it stops.
            Check (firstRollOmega >= 10 + 24 && firstRollOmega <= 10 + 26,
                "roll omega guard after 0.5 s (" + firstRollOmega + ")");
            Check (lastRollOmega == 69, "roll omega guard clears with the spin (" + lastRollOmega + ")");
            Check (!rollOther, "roll: no residual nor saturation guard");
            Check (firstPitchSat >= 49 && firstPitchSat <= 53,
                "pitch saturation guard after 1 s (" + firstPitchSat + ")");
            Check (firstYawResidual > 20 + 49 && firstYawResidual < 85,
                "yaw residual guard after 1 s of unexplained acceleration (" + firstYawResidual + ")");
            // Without a reference, no saturation guard; omega and residual still watched.
            st.Reset ();
            for (int t = 0; t < 60; t++) {
                GuardScenarioTick (inp, t);
                inp.ReferenceValid = false;
                AttitudeMath.Step (g, inp, st, work, o);
                Check (!o.Computed && !o.AxisSaturationGuard [0] && double.IsNaN (o.Unrealized [0]),
                    "no reference: no saturation guard, unrealized NaN");
            }
            Check (o.AxisOmegaGuard [1], "no reference: the omega guard still watches");
            // Per-axis limit: <= 0 falls back to the law's omega_max.
            st.Reset ();
            GuardScenarioTick (inp, 0);
            inp.OmegaMaxAxis [1] = 0.0;
            AttitudeMath.Step (g, inp, st, work, o);
            Check (o.OmegaMaxAxis [1] == 0.2 && o.OmegaMaxAxis [0] == 0.2, "axis limit defaults to omega_max");
            // M4: RELATIVE limit. The AP limit drops from 0.3 to 0.1 while the vessel still
            // turns at 0.2: 1.5 x the PEAK of the last second (0.45) holds for 1 s, then the
            // current limit (0.15) applies and the guard fires 0.5 s later.
            st.Reset ();
            int premier = -1;
            for (int t = 0; t < 200; t++) {
                GuardScenarioTick (inp, 0);
                inp.Omega [1] = t < 20 ? 0.0 : 0.2;
                inp.OmegaMaxAxis [1] = t < 20 ? 0.3 : 0.1;
                AttitudeMath.Step (g, inp, st, work, o);
                if (t == 25)
                    Check (Near (o.OmegaGuardLimit [1], 1.5 * 0.3, 1e-12), "limit = 1.5 x the peak of the last second");
                if (o.AxisOmegaGuard [1] && premier < 0)
                    premier = t;
            }
            Check (premier >= 20 + 50 + 24 && premier <= 20 + 50 + 27,
                "relative omega guard: peak held 1 s, then 0.5 s over the current limit (" + premier + ")");
            // The law's own |w_r| raises the limit too (never below what it asks for).
            Check (o.OmegaGuardLimit [1] >= 1.5 * Math.Abs (st.PrevOmegaRef [1]) - 1e-15,
                "limit never below 1.5 |w_r|");
        }

        // -------------------------------------------------------- M3: write rules

        static readonly long[] InformationalMotifs = {
            AttitudeWrite.MotifResidualGuard, AttitudeWrite.MotifOmegaGuard,
            AttitudeWrite.MotifSaturationGuard, AttitudeWrite.MotifFlyByWireReordered,
            AttitudeWrite.MotifWriteFallback, AttitudeWrite.MotifFlyByWireMissed,
            // M4: the pitch/yaw-only motifs never stop the roll.
            AttitudeWrite.MotifPitchYawReferenceOld, AttitudeWrite.MotifGimbalAuthority,
            AttitudeWrite.MotifPitchYawFallback, AttitudeWrite.MotifPitchYawWriteOff,
            AttitudeWrite.MotifPitchYawGuardHold
        };

        // M4: what never stops pitch/yaw - the M2 guards, the roll-only motifs, the summaries.
        static readonly long[] InformationalMotifsPitchYaw = {
            AttitudeWrite.MotifResidualGuard, AttitudeWrite.MotifOmegaGuard,
            AttitudeWrite.MotifSaturationGuard, AttitudeWrite.MotifRollChannelUnmodeled,
            AttitudeWrite.MotifFlyByWireReordered, AttitudeWrite.MotifClientWriteOff,
            AttitudeWrite.MotifGuardHold, AttitudeWrite.MotifWriteFallback,
            AttitudeWrite.MotifFlyByWireMissed, AttitudeWrite.MotifPitchYawFallback
        };

        static void TestWriteRules ()
        {
            Check (AttitudeWrite.ArmedBodyAxes (0) == 0 && AttitudeWrite.ArmedBodyAxes (1) == 2 &&
                AttitudeWrite.ArmedBodyAxes (2) == 5 && AttitudeWrite.ArmedBodyAxes (3) == 7,
                "arm bits to body axes: roll = y, pitch/yaw = x and z");
            for (int mask = 0; mask < 512; mask++) {
                for (int axes = 0; axes < 4; axes++) {
                    long m = AttitudeWrite.ArmedAxisGuardMotifs (mask, axes);
                    int body = AttitudeWrite.ArmedBodyAxes (axes);
                    bool r = false, w = false, s = false;
                    for (int a = 0; a < 3; a++) {
                        if ((body & (1 << a)) == 0)
                            continue;
                        r |= (mask & (1 << (3 * a))) != 0;
                        w |= (mask & (1 << (3 * a + 1))) != 0;
                        s |= (mask & (1 << (3 * a + 2))) != 0;
                    }
                    Check (((m & AttitudeWrite.MotifAxisResidualGuard) != 0) == r &&
                        ((m & AttitudeWrite.MotifAxisOmegaGuard) != 0) == w &&
                        ((m & AttitudeWrite.MotifAxisSaturationGuard) != 0) == s &&
                        (m & ~(AttitudeWrite.MotifAxisResidualGuard | AttitudeWrite.MotifAxisOmegaGuard |
                            AttitudeWrite.MotifAxisSaturationGuard)) == 0,
                        "armed axis guards: only the armed axes count");
                }
            }
            // A guard of pitch/yaw never touches a roll-only arming: the M2 defect 3.
            Check (AttitudeWrite.ArmedAxisGuardMotifs ((1 << 2) | (1 << 7) | (1 << 0) | (1 << 8) | (1 << 1) | (1 << 6), 1) == 0,
                "roll armed: pitch and yaw guards ignored");
            Check (AttitudeWrite.MayWrite (1, 1, 1, true, 0L), "armed roll, clean step: write");
            for (int bit = 0; bit < 32; bit++) {
                long motif = 1L << bit;
                bool informational = Array.IndexOf (InformationalMotifs, motif) >= 0;
                Check (AttitudeWrite.MayWrite (1, 1, 1, true, motif) == informational,
                    "motif bit " + bit + (informational ? " does not block" : " blocks"));
                Check (((AttitudeWrite.BlockingMotifs & motif) != 0) == !informational,
                    "blocking set, bit " + bit);
                // M4: the pitch/yaw rule, each bit on its own.
                bool infoPy = Array.IndexOf (InformationalMotifsPitchYaw, motif) >= 0;
                Check (AttitudeWrite.MayWritePitchYaw (1, 2, 1, true, motif) == infoPy,
                    "pitch/yaw, motif bit " + bit + (infoPy ? " does not block" : " blocks"));
                Check (((AttitudeWrite.BlockingMotifsPitchYaw & motif) != 0) == !infoPy,
                    "pitch/yaw blocking set, bit " + bit);
            }
            Check (!AttitudeWrite.MayWrite (0, 1, 1, true, 0L), "shadow never writes");
            Check (!AttitudeWrite.MayWrite (1, 2, 1, true, 0L), "roll rule: pitch/yaw arming writes no roll");
            Check (!AttitudeWrite.MayWrite (1, 1, 2, true, 0L), "watcher fallback stage never writes");
            Check (!AttitudeWrite.MayWrite (1, 1, 1, false, 0L), "law not computed: no write");
            Check (AttitudeWrite.MayWritePitchYaw (1, 3, 1, true, 0L) && AttitudeWrite.MayWritePitchYaw (1, 2, 1, true, 0L),
                "armed pitch/yaw, clean step: write");
            Check (!AttitudeWrite.MayWritePitchYaw (1, 1, 1, true, 0L) && !AttitudeWrite.MayWritePitchYaw (0, 3, 1, true, 0L) &&
                !AttitudeWrite.MayWritePitchYaw (1, 2, 2, true, 0L) && !AttitudeWrite.MayWritePitchYaw (1, 2, 1, false, 0L),
                "pitch/yaw: roll-only arming, shadow, fallback stage, law not computed: no write");
            // Each channel on its own axes: a roll guard never stops pitch/yaw and back.
            Check (AttitudeWrite.ArmedAxisGuardMotifs ((1 << 4), 2) == 0 && AttitudeWrite.ArmedAxisGuardMotifs ((1 << 1) | (1 << 8), 1) == 0,
                "guards per channel");
            // Contact from the law.
            var ur = new double[] { 0.2, double.NaN, 6.0 };
            Check (AttitudeWrite.ImpossibleDemand (ur, 2) && !AttitudeWrite.ImpossibleDemand (ur, 1) &&
                !AttitudeWrite.ImpossibleDemand (new[] { 4.9, 4.9, 4.9 }, 3), "impossible demand = contact");
        }

        static void TestRollCommandAndCoupling ()
        {
            double u;
            Check (AttitudeWrite.RollCommand (0.0, 1000.0, -2.0e4, out u) && Near (u, -0.05, 1e-15),
                "roll command: u0 + target / column");
            Check (AttitudeWrite.RollCommand (0.9, -1.0e5, -2.0e4, out u) && u == 1.0, "clamped to +1");
            Check (AttitudeWrite.RollCommand (-0.9, 1.0e5, -2.0e4, out u) && u == -1.0, "clamped to -1");
            Check (!AttitudeWrite.RollCommand (0.0, 1.0, 0.0, out u) && double.IsNaN (u), "null column refused");
            Check (!AttitudeWrite.RollCommand (double.NaN, 1.0, 1.0, out u), "NaN u0 refused");
            Check (!AttitudeWrite.RollCommand (0.0, double.PositiveInfinity, 1.0, out u), "infinite target refused");
            // One gimbal (2 columns, span 2 deg) + 3 ctrlState columns.
            int n = 5;
            var b = new double[3 * n];
            var lo = new double[] { -2, -1.5, -1, -1, -1 };
            var hi = new double[] { 1.5, 2, 1, 1, 1 };
            b [1 * n + 0] = 1.0e3;
            b [1 * n + 1] = -2.0e3;
            b [1 * n + 3] = -4.0e4;
            double rho = AttitudeWrite.RollCoupling (b, lo, hi, n, 2);
            Check (Near (rho, (1.0e3 * 2 + 2.0e3 * 2) / 4.0e4, 1e-15), "coupling: gimbal roll authority / roll column");
            b [1 * n + 3] = 0.0;
            Check (double.IsPositiveInfinity (AttitudeWrite.RollCoupling (b, lo, hi, n, 2)),
                "no roll column: infinite coupling (no write)");
            b [1 * n + 3] = -4.0e4;
            b [1 * n + 0] = 0.0;
            b [1 * n + 1] = 0.0;
            Check (AttitudeWrite.RollCoupling (b, lo, hi, n, 2) == 0.0, "no thrust: no coupling");
        }

        static void TestFlyByWireChain ()
        {
            object vessel = new object ();
            PilotAddon.Callback kspEmpty = delegate { };
            PilotAddon.Callback krpc = PilotAddon.Register (vessel);
            PilotAddon.Callback own = s => { };
            PilotAddon.Callback other = s => { };
            Check (AttitudeWrite.IsKrpcPilot (krpc), "kRPC closure-through-Invoke recognised");
            Check (AttitudeWrite.IsKrpcPilot (PilotAddon.RegisterDirect ()), "kRPC method group recognised");
            Check (!AttitudeWrite.IsKrpcPilot (own) && !AttitudeWrite.IsKrpcPilot (kspEmpty) &&
                !AttitudeWrite.IsKrpcPilot (null), "other callbacks are not kRPC");
            int o, k, l;
            // KSP's empty delegate, kRPC (registered at the vessel's first FixedUpdate), ours.
            var chain = (PilotAddon.Callback)Delegate.Combine (Delegate.Combine (kspEmpty, krpc), own);
            AttitudeWrite.ClassifyChain (chain, own, out o, out k, out l);
            Check (l == 3 && k == 1 && o == 2 && AttitudeWrite.OrderVerified (o, k, l),
                "ours after kRPC and last: verified");
            // Ours before kRPC: refused, then the EnsureHook move (Remove + Combine) fixes it.
            chain = (PilotAddon.Callback)Delegate.Combine (Delegate.Combine (kspEmpty, own), krpc);
            AttitudeWrite.ClassifyChain (chain, own, out o, out k, out l);
            Check (o == 1 && k == 2 && !AttitudeWrite.OrderVerified (o, k, l), "ours before kRPC: not verified");
            chain = (PilotAddon.Callback)Delegate.Combine (Delegate.Remove (chain, own), own);
            AttitudeWrite.ClassifyChain (chain, own, out o, out k, out l);
            Check (o == 2 && k == 1 && AttitudeWrite.OrderVerified (o, k, l), "moved to the end: verified");
            // A third party registered after us: not last, not verified until moved.
            chain = (PilotAddon.Callback)Delegate.Combine (chain, other);
            AttitudeWrite.ClassifyChain (chain, own, out o, out k, out l);
            Check (o == 2 && l == 4 && !AttitudeWrite.OrderVerified (o, k, l), "not last: not verified");
            // kRPC absent (AutoPilot never flew this vessel): never verified.
            chain = (PilotAddon.Callback)Delegate.Combine (kspEmpty, own);
            AttitudeWrite.ClassifyChain (chain, own, out o, out k, out l);
            Check (k == -1 && !AttitudeWrite.OrderVerified (o, k, l), "kRPC absent: not verified");
            // Ours absent.
            chain = (PilotAddon.Callback)Delegate.Combine (kspEmpty, krpc);
            AttitudeWrite.ClassifyChain (chain, own, out o, out k, out l);
            Check (o == -1 && !AttitudeWrite.OrderVerified (o, k, l), "ours absent: not verified");
            AttitudeWrite.ClassifyChain (null, own, out o, out k, out l);
            Check (o == -1 && k == -1 && l == 0, "empty chain");
            // Invocation order is the chain order: ours runs after kRPC.
            var seen = new List<string> ();
            PilotAddon.Callback kr = s => seen.Add ("krpc");
            PilotAddon.Callback ow = s => seen.Add ("own");
            ((PilotAddon.Callback)Delegate.Combine (kr, ow)) (null);
            Check (seen.Count == 2 && seen [0] == "krpc" && seen [1] == "own", "multicast runs in chain order");
        }

        // ------------------------------------------------------------- golden vectors

        static void WriteGolden (string path)
        {
            var g = new AttitudeGains ();
            var sb = new StringBuilder ();
            sb.Append ("{\n  \"schema\": 2,\n  \"source\": \"krpc_bridge build/tests (AttitudeMath.cs, AttitudeWrite.cs)\",\n");
            sb.Append ("  \"gains\": {");
            sb.Append ("\"filter_omega_c\": " + D (g.FilterOmegaC) + ", \"filter_zeta\": " + D (g.FilterZeta));
            sb.Append (", \"k_omega_pitch_yaw\": " + D (g.KOmegaPitchYaw) + ", \"k_omega_roll\": " + D (g.KOmegaRoll));
            sb.Append (", \"k_lin\": " + D (g.KLin) + ", \"eta\": " + D (g.Eta) + ", \"tau_pch\": " + D (g.TauPch));
            sb.Append (", \"gamma\": " + D (g.Gamma) + ", \"effect_floor\": " + D (g.EffectFloor) + ", \"wv_pitch_yaw\": " + D (g.WvPitchYaw));
            sb.Append (", \"wv_roll\": " + D (g.WvRoll) + ", \"alpha_floor\": " + D (g.AlphaFloor));
            sb.Append (", \"omega_max_default\": " + D (g.OmegaMaxDefault) + ", \"tau_residual\": " + D (g.TauResidual));
            sb.Append (", \"max_iterations\": " + g.MaxIterations);
            sb.Append (", \"guard_residual_ratio\": " + D (g.GuardResidualRatio) + ", \"guard_residual_seconds\": " + D (g.GuardResidualSeconds));
            sb.Append (", \"guard_omega_ratio\": " + D (g.GuardOmegaRatio) + ", \"guard_omega_seconds\": " + D (g.GuardOmegaSeconds));
            sb.Append (", \"guard_saturation_ratio\": " + D (g.GuardSaturationRatio) + ", \"guard_saturation_seconds\": " + D (g.GuardSaturationSeconds));
            sb.Append (", \"guard_omega_peak_seconds\": " + D (g.GuardOmegaPeakSeconds));
            sb.Append ("},\n");
            sb.Append ("  \"scenarios\": [\n");
            string[] names = { "propulse_ample", "propulse_sature", "coast_rcs", "trou_et_reprise",
                "gardes_par_axe" };
            for (int s = 0; s < names.Length; s++) {
                sb.Append ("    {\"name\": \"" + names [s] + "\", \"ticks\": [\n");
                if (s < 4)
                    GoldenScenario (sb, g, s);
                else
                    GoldenGuardScenario (sb, g);
                sb.Append ("    ]}" + (s + 1 < names.Length ? "," : "") + "\n");
            }
            sb.Append ("  ],\n");
            GoldenWrite (sb);
            sb.Append ("}\n");
            File.WriteAllText (path, sb.ToString ());
        }

        /// <summary>
        /// the golden vectors of option 128's law, AttitudeInput.TrimNullColumns (and the free torque
        /// AttitudeV1 gives it), in their own file so that attitude_v1_golden.json does not move by a byte.
        /// Three scenarios: a burn that drives the gimbals to their bounds then a coast on null gimbal columns
        /// (they go to neutral), the same coast then an ignition whose columns grow with the spool (free torque
        /// of the spool at the realized deflections), and a burn where every column has an effect (the option
        /// changes nothing). The gimbals Lerp 0.16 a step toward the law's command.
        /// </summary>
        static void WriteGoldenTrim (string path)
        {
            var g = new AttitudeGains ();
            var sb = new StringBuilder ();
            sb.Append ("{\n  \"schema\": 1,\n  \"source\": \"krpc_bridge build/tests (AttitudeMath.cs, option 128: TrimNullColumns)\",\n");
            sb.Append ("  \"gains\": {");
            sb.Append ("\"filter_omega_c\": " + D (g.FilterOmegaC) + ", \"filter_zeta\": " + D (g.FilterZeta));
            sb.Append (", \"k_omega_pitch_yaw\": " + D (g.KOmegaPitchYaw) + ", \"k_omega_roll\": " + D (g.KOmegaRoll));
            sb.Append (", \"k_lin\": " + D (g.KLin) + ", \"eta\": " + D (g.Eta) + ", \"tau_pch\": " + D (g.TauPch));
            sb.Append (", \"gamma\": " + D (g.Gamma) + ", \"effect_floor\": " + D (g.EffectFloor) + ", \"wv_pitch_yaw\": " + D (g.WvPitchYaw));
            sb.Append (", \"wv_roll\": " + D (g.WvRoll) + ", \"alpha_floor\": " + D (g.AlphaFloor));
            sb.Append (", \"omega_max_default\": " + D (g.OmegaMaxDefault) + ", \"tau_residual\": " + D (g.TauResidual));
            sb.Append (", \"max_iterations\": " + g.MaxIterations);
            sb.Append (", \"guard_residual_ratio\": " + D (g.GuardResidualRatio) + ", \"guard_residual_seconds\": " + D (g.GuardResidualSeconds));
            sb.Append (", \"guard_omega_ratio\": " + D (g.GuardOmegaRatio) + ", \"guard_omega_seconds\": " + D (g.GuardOmegaSeconds));
            sb.Append (", \"guard_saturation_ratio\": " + D (g.GuardSaturationRatio) + ", \"guard_saturation_seconds\": " + D (g.GuardSaturationSeconds));
            sb.Append (", \"guard_omega_peak_seconds\": " + D (g.GuardOmegaPeakSeconds));
            sb.Append ("},\n");
            sb.Append ("  \"scenarios\": [\n");
            string[] names = { "cardans_gares_coast", "allumage_colonnes_croissantes", "trim_sans_colonne_nulle" };
            for (int s = 0; s < names.Length; s++) {
                sb.Append ("    {\"name\": \"" + names [s] + "\", \"ticks\": [\n");
                GoldenTrimScenario (sb, g, s);
                sb.Append ("    ]}" + (s + 1 < names.Length ? "," : "") + "\n");
            }
            sb.Append ("  ]\n}\n");
            File.WriteAllText (path, sb.ToString ());
        }

        /// <summary>
        /// the golden vectors of the law with a roll target (AttitudeInput.RollTargetValid), for the client's
        /// mirror: the gains, then three scenarios of 80 ticks -- the roll aligned from 0.7 rad on RCS and wheels alone,
        /// the roll with pitch and yaw errors and the engine-like gimbals of the trim file, and the roll target lost and
        /// found again (RollTargetValid toggling, the roll's own AutoPilot limit set and not). Deterministic.
        /// </summary>
        static void WriteGoldenRoll (string path)
        {
            var g = new AttitudeGains ();
            var sb = new StringBuilder ();
            sb.Append ("{\n  \"schema\": 1,\n  \"source\": \"krpc_bridge build/tests (AttitudeMath.cs, option 1024: RollTargetValid)\",\n");
            sb.Append ("  \"scenarios\": [\n");
            string[] names = { "roulis_seul", "roulis_tangage_lacet_cardans", "cible_perdue_retrouvee" };
            for (int s = 0; s < names.Length; s++) {
                sb.Append ("    {\"name\": \"" + names [s] + "\", \"ticks\": [\n");
                GoldenRollScenario (sb, g, s);
                sb.Append ("    ]}" + (s + 1 < names.Length ? "," : "") + "\n");
            }
            sb.Append ("  ]\n}\n");
            File.WriteAllText (path, sb.ToString ());
        }

        static void GoldenRollScenario (StringBuilder sb, AttitudeGains g, int scenario)
        {
            var rng = new Random (2528 + scenario);
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            int n;
            if (scenario == 1) {
                FillEngineLikeInput (inp, 1.0);
                n = 11;
            } else {
                inp.Resize (3);
                n = 3;
                for (int j = 0; j < 3; j++)
                    for (int a = 0; a < 3; a++)
                        inp.B [a * 3 + j] = a == j ? (a == 1 ? 3.0e5 : 7.9e5) : 1.0e4 * (rng.NextDouble () - 0.5);
                inp.Inertia [0] = 1.2e7;
                inp.Inertia [1] = 2.0e6;
                inp.Inertia [2] = 1.2e7;
                inp.Dt = 0.02;
            }
            var u = new double[n];
            double roll = 0.7, w = 0.0;
            int ticks = 80;
            for (int t = 0; t < ticks; t++) {
                for (int a = 0; a < 3; a++) {
                    inp.Omega [a] = a == 1 ? w : 0.03 * Math.Sin (0.07 * t + a);
                    inp.OmegaDotMeasured [a] = 0.01 * Math.Cos (0.11 * t + 2 * a) + 0.004 * (rng.NextDouble () - 0.5);
                    inp.Torque [a] = 2.0e4 * Math.Sin (0.05 * t + 3 * a) + 500 * (rng.NextDouble () - 0.5);
                    inp.OmegaFf [a] = 0.0;
                    inp.OmegaDotFf [a] = 0.0;
                    inp.OmegaMaxAxis [a] = 0.0;
                }
                double tilt = scenario == 1 ? 0.2 : 0.0;
                AttitudeMath.PointingError (tilt * Math.Sin (0.03 * t + 1.0), 1.0, tilt * Math.Cos (0.05 * t), inp.Error);
                for (int j = 0; j < n; j++) {
                    inp.U0 [j] = t == 0 ? 0.0 : u [j];
                    inp.UPref [j] = 0.0;
                    if (scenario != 1) {
                        inp.UMin [j] = -1.0;
                        inp.UMax [j] = 1.0;
                        inp.Priority [j] = 3.0;
                    }
                }
                inp.UseTorqueFree = false;
                inp.TrimNullColumns = 0;
                inp.OmegaMax = 0.25;
                if (scenario == 2 && t % 20 >= 10)
                    inp.OmegaMaxAxis [1] = 0.08;
                inp.OmegaDotValid = true;
                inp.ReferenceValid = true;
                inp.RollTargetValid = !(scenario == 2 && t >= 30 && t < 45);
                inp.Error [1] = inp.RollTargetValid ? AttitudeMath.RollError (-Math.Sin (roll), 0.1, -Math.Cos (roll)) : 0.0;
                AttitudeMath.Step (g, inp, st, work, o);
                AppendTick (sb, inp, o, t + 1 < ticks, false, true);
                for (int j = 0; j < n; j++)
                    u [j] = o.Computed ? o.U [j] : inp.U0 [j];
                // the plant on roll only: the ctrlState roll column's torque (index n - 2), the inertia about the nose
                double torque = inp.B [1 * n + (n - 2)] * u [n - 2];
                w += torque / inp.Inertia [1] * 0.02;
                roll -= w * 0.02;
            }
        }

        static void GoldenTrimScenario (StringBuilder sb, AttitudeGains g, int scenario)
        {
            var rng = new Random (2370 + scenario);
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            FillEngineLikeInput (inp, 1.0);
            var full = new double[3 * 11];
            Array.Copy (inp.B, full, 3 * 11);
            var u = new double[11];
            double c = scenario == 2 ? 1.0 : 0.05, horizon = GimbalSpool.HorizonSteps (0.16);
            int ticks = 60;
            for (int t = 0; t < ticks; t++) {
                // The thrust: a burn, then nothing (0, 1), then a spool (1) toward full thrust; full in 2.
                double requested = scenario == 2 ? 1.0 : (t < 20 ? 0.05 : (scenario == 1 && t >= 35 ? 1.0 : 0.0));
                double cExp = GimbalSpool.ExpectedThrottle (c, requested, true, false, 1.5, 50.0, 0.02, horizon);
                if (cExp < c)
                    cExp = c;
                bool spool = scenario == 1 && cExp != c;
                for (int j = 0; j < 11; j++)
                    for (int a = 0; a < 3; a++)
                        inp.B [a * 11 + j] = j < 8 ? full [a * 11 + j] * (scenario == 1 ? cExp : c) : full [a * 11 + j];
                for (int a = 0; a < 3; a++) {
                    inp.Omega [a] = 0.05 * Math.Sin (0.07 * t + a) + 0.002 * (rng.NextDouble () - 0.5);
                    inp.OmegaDotMeasured [a] = 0.01 * Math.Cos (0.11 * t + 2 * a) + 0.004 * (rng.NextDouble () - 0.5);
                    inp.Torque [a] = 1.0e4 * Math.Sin (0.05 * t + 3 * a) + 500 * (rng.NextDouble () - 0.5);
                    inp.OmegaFf [a] = 0.0;
                    inp.OmegaDotFf [a] = 0.0;
                }
                double big = t < 20 ? 0.6 : 0.02;
                AttitudeMath.PointingError (big * Math.Sin (0.03 * t + 1.0), 1.0, big * Math.Cos (0.05 * t), inp.Error);
                for (int j = 0; j < 11; j++) {
                    inp.U0 [j] = t == 0 ? 0.0 : u [j];
                    inp.UPref [j] = 0.0;
                }
                inp.UseTorqueFree = false;
                if (spool) {
                    for (int a = 0; a < 3; a++) {
                        double free = 0.0;
                        for (int j = 0; j < 8; j++)
                            free += full [a * 11 + j] * (cExp - c) * u [j];
                        inp.TorqueFree [a] = free;
                    }
                    inp.UseTorqueFree = true;
                } else {
                    inp.TorqueFree [0] = 0.0;
                    inp.TorqueFree [1] = 0.0;
                    inp.TorqueFree [2] = 0.0;
                }
                inp.TrimNullColumns = 8;
                inp.OmegaMax = 0.25;
                inp.OmegaDotValid = true;
                inp.ReferenceValid = true;
                AttitudeMath.Step (g, inp, st, work, o);
                AppendTick (sb, inp, o, t + 1 < ticks, true);
                // The plant: the gimbals Lerp toward the command, the ctrlState served at once, the spool.
                for (int j = 0; j < 11; j++) {
                    double cmd = o.Computed ? o.U [j] : inp.U0 [j];
                    u [j] = j < 8 ? u [j] + (cmd - u [j]) * 0.16 : cmd;
                }
                // The cut is immediate (a deceleration speed of 50 / s); the spool up 1.5 / s.
                c = requested == 0.0 ? 0.0 : c + (requested - c) * (1.5 * 0.02);
            }
        }

        static void GoldenGuardScenario (StringBuilder sb, AttitudeGains g)
        {
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            for (int t = 0; t < GuardScenarioTicks; t++) {
                GuardScenarioTick (inp, t);
                AttitudeMath.Step (g, inp, st, work, o);
                AppendTick (sb, inp, o, t + 1 < GuardScenarioTicks);
            }
        }

        /// <summary>
        /// The write rules of AttitudeWrite (M3) on exhaustive and random inputs, for the
        /// Python mirror: the decision, the armed-axis guard motifs, the roll command, the
        /// gimbal-roll coupling.
        /// </summary>
        static void GoldenWrite (StringBuilder sb)
        {
            sb.Append ("  \"write\": {\"roll_coupling_max\": " + D (AttitudeWrite.RollCouplingMax));
            sb.Append (", \"guard_hold_seconds\": " + D (AttitudeWrite.GuardHoldSeconds));
            sb.Append (", \"blocking_motifs\": " + AttitudeWrite.BlockingMotifs.ToString (CultureInfo.InvariantCulture));
            sb.Append (", \"blocking_motifs_pitch_yaw\": " + AttitudeWrite.BlockingMotifsPitchYaw.ToString (CultureInfo.InvariantCulture));
            sb.Append (", \"pitch_yaw_max_reference_age_s\": " + D (AttitudeWrite.PitchYawMaxReferenceAgeS));
            sb.Append (", \"contact_unrealized_ratio\": " + D (AttitudeWrite.ContactUnrealizedRatio));
            sb.Append (",\n    \"may_write\": [");
            bool first = true;
            for (int mode = 0; mode < 3; mode++)
                for (int axes = 0; axes < 4; axes++)
                    for (int stage = 1; stage <= 2; stage++)
                        for (int computed = 0; computed < 2; computed++)
                            for (int bit = -1; bit < 32; bit++) {
                                long motif = bit < 0 ? 0L : 1L << bit;
                                bool w = AttitudeWrite.MayWrite (mode, axes, stage, computed == 1, motif);
                                bool wpy = AttitudeWrite.MayWritePitchYaw (mode, axes, stage, computed == 1, motif);
                                sb.Append ((first ? "" : ", ") + "[" + mode + ", " + axes + ", " + stage + ", " +
                                    computed + ", " + motif.ToString (CultureInfo.InvariantCulture) + ", " +
                                    (w ? 1 : 0) + ", " + (wpy ? 1 : 0) + "]");
                                first = false;
                            }
            sb.Append ("],\n    \"impossible_demand\": [");
            var rngc = new Random (2711);
            first = true;
            for (int k = 0; k < 200; k++) {
                var ur = new double[3];
                for (int a = 0; a < 3; a++)
                    ur [a] = rngc.Next (6) == 0 ? double.NaN : rngc.NextDouble () * 8.0;
                int axes = rngc.Next (4);
                bool c = AttitudeWrite.ImpossibleDemand (ur, axes);
                sb.Append ((first ? "" : ", ") + "[" + VN (ur, 3) + ", " + axes + ", " + (c ? 1 : 0) + "]");
                first = false;
            }
            sb.Append ("],\n    \"armed_axis_guard_motifs\": [");
            first = true;
            for (int mask = 0; mask < 512; mask++)
                for (int axes = 0; axes < 4; axes++) {
                    long m = AttitudeWrite.ArmedAxisGuardMotifs (mask, axes);
                    sb.Append ((first ? "" : ", ") + "[" + mask + ", " + axes + ", " +
                        m.ToString (CultureInfo.InvariantCulture) + "]");
                    first = false;
                }
            sb.Append ("],\n    \"roll_command\": [");
            var rng = new Random (3131);
            first = true;
            for (int k = 0; k < 300; k++) {
                double u0 = k == 0 ? 0.0 : rng.NextDouble () * 2.0 - 1.0;
                double target = (rng.NextDouble () * 2.0 - 1.0) * Math.Pow (10, rng.Next (1, 6));
                double column = k == 1 ? 0.0 : -(rng.NextDouble () * 1.0e5 + 1.0) * (rng.Next (5) == 0 ? -1 : 1);
                double command;
                bool ok = AttitudeWrite.RollCommand (u0, target, column, out command);
                sb.Append ((first ? "" : ", ") + "[" + D (u0) + ", " + D (target) + ", " + D (column) + ", " +
                    (ok ? 1 : 0) + ", " + DN (command) + "]");
                first = false;
            }
            sb.Append ("],\n    \"roll_coupling\": [");
            first = true;
            for (int k = 0; k < 60; k++) {
                int gimbals = rng.Next (0, 7);
                int nG = 2 * gimbals;
                int n = nG + 3;
                var b = new double[3 * n];
                var lo = new double[n];
                var hi = new double[n];
                for (int i = 0; i < 3 * n; i++)
                    b [i] = (rng.NextDouble () * 2.0 - 1.0) * Math.Pow (10, rng.Next (2, 6));
                for (int j = 0; j < n; j++) {
                    lo [j] = j < nG ? -rng.NextDouble () * 3.0 : -1.0;
                    hi [j] = j < nG ? rng.NextDouble () * 3.0 : 1.0;
                }
                if (k == 0)
                    b [1 * n + nG + 1] = 0.0;
                double rho = AttitudeWrite.RollCoupling (b, lo, hi, n, nG);
                sb.Append ((first ? "" : ",\n      ") + "{\"n\": " + n + ", \"n_gimbal_columns\": " + nG +
                    ", \"b\": " + V (b, 3 * n) + ", \"u_min\": " + V (lo, n) + ", \"u_max\": " + V (hi, n) +
                    ", \"rho\": " + DN (rho) + "}");
                first = false;
            }
            sb.Append ("]}\n");
        }

        static void GoldenScenario (StringBuilder sb, AttitudeGains g, int scenario)
        {
            var rng = new Random (500 + scenario);
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            double scale = scenario == 1 ? 0.02 : 1.0;
            FillEngineLikeInput (inp, scale);
            if (scenario == 2) {
                // Coast: no thrust, the gimbal columns vanish; the ctrlState channel alone.
                for (int j = 0; j < 8; j++) {
                    inp.B [0 * 11 + j] = 0; inp.B [1 * 11 + j] = 0; inp.B [2 * 11 + j] = 0;
                }
            }
            int ticks = 60;
            for (int t = 0; t < ticks; t++) {
                // A random walk of the plant: every input moves every tick.
                for (int a = 0; a < 3; a++) {
                    inp.Omega [a] = 0.05 * Math.Sin (0.07 * t + a) + 0.002 * (rng.NextDouble () - 0.5);
                    inp.OmegaDotMeasured [a] = 0.01 * Math.Cos (0.11 * t + 2 * a) + 0.004 * (rng.NextDouble () - 0.5);
                    inp.Torque [a] = 1.0e4 * Math.Sin (0.05 * t + 3 * a) + 500 * (rng.NextDouble () - 0.5);
                    inp.OmegaFf [a] = a == 1 ? 0.0 : 0.001 * Math.Sin (0.02 * t);
                    inp.OmegaDotFf [a] = 0.0;
                }
                AttitudeMath.PointingError (0.3 * Math.Sin (0.03 * t), 1.0, 0.2 * Math.Cos (0.05 * t), inp.Error);
                for (int j = 0; j < 11; j++) {
                    double lo = inp.UMin [j], hi = inp.UMax [j];
                    inp.U0 [j] = lo + (hi - lo) * (0.3 + 0.4 * rng.NextDouble ());
                    inp.UPref [j] = 0.0;
                }
                inp.OmegaMax = scenario == 1 ? 0.08 : 0.25;
                inp.OmegaDotValid = !(scenario == 3 && (t == 20 || t == 41));
                inp.ReferenceValid = !(scenario == 3 && t >= 30 && t < 35);
                AttitudeMath.Step (g, inp, st, work, o);
                AppendTick (sb, inp, o, t + 1 < ticks);
            }
        }

        static void AppendTick (StringBuilder sb, AttitudeInput inp, AttitudeOutput o, bool more)
        {
            AppendTick (sb, inp, o, more, false);
        }

        /// <summary>
        /// One tick of a golden scenario. trim (the file of option 128 only): the input also carries
        /// trim_null_columns and the free torque, the output null_columns_trimmed; without it the line is the
        /// one the golden file has always had.
        /// </summary>
        static void AppendTick (StringBuilder sb, AttitudeInput inp, AttitudeOutput o, bool more, bool trim)
        {
            AppendTick (sb, inp, o, more, trim, false);
        }

        /// <summary>One tick; roll (the file of option 1024 only): the input also carries roll_target_valid.</summary>
        static void AppendTick (StringBuilder sb, AttitudeInput inp, AttitudeOutput o, bool more, bool trim, bool roll)
        {
            {
                sb.Append ("      {\"in\": {");
                sb.Append ("\"dt\": " + D (inp.Dt));
                sb.Append (", \"omega\": " + V (inp.Omega, 3));
                sb.Append (", \"omega_dot_measured\": " + V (inp.OmegaDotMeasured, 3));
                sb.Append (", \"omega_dot_valid\": " + (inp.OmegaDotValid ? "true" : "false"));
                sb.Append (", \"torque\": " + V (inp.Torque, 3));
                sb.Append (", \"inertia\": " + V (inp.Inertia, 3));
                sb.Append (", \"reference_valid\": " + (inp.ReferenceValid ? "true" : "false"));
                sb.Append (", \"error\": " + V (inp.Error, 3));
                sb.Append (", \"omega_ff\": " + V (inp.OmegaFf, 3));
                sb.Append (", \"omega_dot_ff\": " + V (inp.OmegaDotFf, 3));
                sb.Append (", \"omega_max\": " + D (inp.OmegaMax));
                sb.Append (", \"omega_max_axis\": " + V (inp.OmegaMaxAxis, 3));
                sb.Append (", \"n\": " + inp.N);
                sb.Append (", \"b\": " + V (inp.B, 3 * inp.N));
                sb.Append (", \"u0\": " + V (inp.U0, inp.N));
                sb.Append (", \"u_min\": " + V (inp.UMin, inp.N));
                sb.Append (", \"u_max\": " + V (inp.UMax, inp.N));
                sb.Append (", \"u_pref\": " + V (inp.UPref, inp.N));
                sb.Append (", \"priority\": " + V (inp.Priority, inp.N));
                if (roll)
                    sb.Append (", \"roll_target_valid\": " + (inp.RollTargetValid ? "true" : "false"));
                if (trim) {
                    sb.Append (", \"trim_null_columns\": " + inp.TrimNullColumns);
                    sb.Append (", \"use_torque_free\": " + (inp.UseTorqueFree ? "true" : "false"));
                    sb.Append (", \"torque_free\": " + V (inp.TorqueFree, 3));
                }
                sb.Append ("}, \"out\": {");
                sb.Append ("\"computed\": " + (o.Computed ? "true" : "false"));
                sb.Append (", \"filtered\": " + (o.Filtered ? "true" : "false"));
                sb.Append (", \"omega_dot_f\": " + V (o.OmegaDotF, 3));
                sb.Append (", \"torque_f\": " + V (o.TorqueF, 3));
                sb.Append (", \"alpha\": " + V (o.Alpha, 3));
                sb.Append (", \"omega_sqrt\": " + V (o.OmegaSqrt, 3));
                sb.Append (", \"omega_ref\": " + V (o.OmegaRef, 3));
                sb.Append (", \"omega_pch\": " + V (o.OmegaPch, 3));
                sb.Append (", \"nu\": " + V (o.Nu, 3));
                sb.Append (", \"delta_torque_desired\": " + V (o.DeltaTorqueDesired, 3));
                sb.Append (", \"torque_desired\": " + V (o.TorqueDesired, 3));
                sb.Append (", \"torque_alloc_target\": " + V (o.TorqueAllocTarget, 3));
                sb.Append (", \"torque_allocated\": " + V (o.TorqueAllocated, 3));
                sb.Append (", \"nu_hedge\": " + V (o.NuHedge, 3));
                sb.Append (", \"residual\": " + V (o.Residual, 3));
                sb.Append (", \"residual_f\": " + V (o.ResidualF, 3));
                sb.Append (", \"delta_u\": " + V (o.DeltaU, inp.N));
                sb.Append (", \"regularization_weight\": " + V (o.RegularizationWeight, inp.N));
                sb.Append (", \"active_set\": " + I (o.ActiveSet, inp.N));
                sb.Append (", \"iterations\": " + o.Iterations);
                sb.Append (", \"converged\": " + (o.Converged ? "true" : "false"));
                sb.Append (", \"saturation_fraction\": " + D (o.SaturationFraction));
                sb.Append (", \"residual_guard\": " + (o.ResidualGuard ? "true" : "false"));
                sb.Append (", \"saturation_guard\": " + (o.SaturationGuard ? "true" : "false"));
                sb.Append (", \"omega_guard\": " + (o.OmegaGuard ? "true" : "false"));
                sb.Append (", \"unrealized\": " + VN (o.Unrealized, 3));
                sb.Append (", \"omega_max_axis\": " + VN (o.OmegaMaxAxis, 3));
                sb.Append (", \"omega_guard_limit\": " + VN (o.OmegaGuardLimit, 3));
                sb.Append (", \"axis_guard_mask\": " + o.AxisGuardMask);
                if (trim)
                    sb.Append (", \"null_columns_trimmed\": " + o.NullColumnsTrimmed);
                sb.Append ("}}" + (more ? "," : "") + "\n");
            }
        }

        static string D (double v)
        {
            return v.ToString ("R", CultureInfo.InvariantCulture);
        }

        /// <summary>A double, or null when not finite (strict JSON; the mirror reads NaN).</summary>
        static string DN (double v)
        {
            return double.IsNaN (v) || double.IsInfinity (v) ? "null" : D (v);
        }

        static string VN (double[] v, int count)
        {
            var sb = new StringBuilder ("[");
            for (int i = 0; i < count; i++) {
                if (i > 0)
                    sb.Append (", ");
                sb.Append (DN (v [i]));
            }
            return sb.Append ("]").ToString ();
        }

        static string V (double[] v, int count)
        {
            var sb = new StringBuilder ("[");
            for (int i = 0; i < count; i++) {
                if (i > 0)
                    sb.Append (", ");
                sb.Append (D (v [i]));
            }
            return sb.Append ("]").ToString ();
        }

        static string I (int[] v, int count)
        {
            var sb = new StringBuilder ("[");
            for (int i = 0; i < count; i++) {
                if (i > 0)
                    sb.Append (", ");
                sb.Append (v [i].ToString (CultureInfo.InvariantCulture));
            }
            return sb.Append ("]").ToString ();
        }
    }
}
