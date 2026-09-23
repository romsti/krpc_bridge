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

namespace KRPC.Bridge.Actuators
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

            // QW6: cached reflection readers against verbatim copies of the legacy ones.
            int reflectionChecks;
            int reflectionFailures = ReflectionTests.Run (out reflectionChecks);
            checks += reflectionChecks;
            failures += reflectionFailures;

            Directory.CreateDirectory (Path.GetDirectoryName (golden));
            WriteGolden (golden);

            Console.WriteLine ();
            Console.WriteLine ("checks: " + checks + ", failures: " + failures);
            Console.WriteLine ("golden vectors: " + golden);
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

        // ------------------------------------------------------------- golden vectors

        static void WriteGolden (string path)
        {
            var g = new AttitudeGains ();
            var sb = new StringBuilder ();
            sb.Append ("{\n  \"schema\": 1,\n  \"source\": \"krpc_bridge build/tests (AttitudeMath.cs)\",\n");
            sb.Append ("  \"gains\": {");
            sb.Append ("\"filter_omega_c\": " + D (g.FilterOmegaC) + ", \"filter_zeta\": " + D (g.FilterZeta));
            sb.Append (", \"k_omega_pitch_yaw\": " + D (g.KOmegaPitchYaw) + ", \"k_omega_roll\": " + D (g.KOmegaRoll));
            sb.Append (", \"k_lin\": " + D (g.KLin) + ", \"eta\": " + D (g.Eta) + ", \"tau_pch\": " + D (g.TauPch));
            sb.Append (", \"gamma\": " + D (g.Gamma) + ", \"effect_floor\": " + D (g.EffectFloor) + ", \"wv_pitch_yaw\": " + D (g.WvPitchYaw));
            sb.Append (", \"wv_roll\": " + D (g.WvRoll) + ", \"alpha_floor\": " + D (g.AlphaFloor));
            sb.Append (", \"omega_max_default\": " + D (g.OmegaMaxDefault) + ", \"tau_residual\": " + D (g.TauResidual));
            sb.Append (", \"max_iterations\": " + g.MaxIterations + "},\n");
            sb.Append ("  \"scenarios\": [\n");
            string[] names = { "propulse_ample", "propulse_sature", "coast_rcs", "trou_et_reprise" };
            for (int s = 0; s < names.Length; s++) {
                sb.Append ("    {\"name\": \"" + names [s] + "\", \"ticks\": [\n");
                GoldenScenario (sb, g, s);
                sb.Append ("    ]}" + (s + 1 < names.Length ? "," : "") + "\n");
            }
            sb.Append ("  ]\n}\n");
            File.WriteAllText (path, sb.ToString ());
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
                sb.Append (", \"n\": " + inp.N);
                sb.Append (", \"b\": " + V (inp.B, 3 * inp.N));
                sb.Append (", \"u0\": " + V (inp.U0, inp.N));
                sb.Append (", \"u_min\": " + V (inp.UMin, inp.N));
                sb.Append (", \"u_max\": " + V (inp.UMax, inp.N));
                sb.Append (", \"u_pref\": " + V (inp.UPref, inp.N));
                sb.Append (", \"priority\": " + V (inp.Priority, inp.N));
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
                sb.Append ("}}" + (t + 1 < ticks ? "," : "") + "\n");
            }
        }

        static string D (double v)
        {
            return v.ToString ("R", CultureInfo.InvariantCulture);
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
