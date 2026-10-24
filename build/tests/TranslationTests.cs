// -----------------------------------------------------------------------------------
// Lateral force (chantier 19 suite): tests of AttitudeTranslation.cs, the pure layer that
// puts two differential-thrust motif columns and a common gimbal deflection in front of
// the UNCHANGED AttitudeMath law. The last test flies a barge-like hover (4 engines 750 kN
// in a +-0.6 m square, nozzles 5.96 m and pivots 3.8 m below the CoM, gimbal 2 deg at the
// stock ramp 8/s) through AttitudeMath.Step with the plugin's sequence of calls.
//
// Computed levers (bit 4, modes 5 and 7): the lever criterion (the heavy booster's 0.133 m
// accepted, coaxial engines refused), the RCS translation columns (signs, torques, the heavy
// booster's RCS ring fitted on its flight pitch column), the joint allocation (torque ~0,
// bounds, the gimbal-fraction scale), the authorities per lever (heavy booster at throttle
// 0.08 and 0.3, barge, medium) and a heavy-booster hover in mode 7 through the unchanged law.
// -----------------------------------------------------------------------------------

using System;
using System.Globalization;
using System.Text;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static class TranslationTests
    {
        static int checks;
        static int failures;
        const double Deg = Math.PI / 180.0;

        public static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestModes ();
            TestMotifs ();
            TestMotifColumns ();
            TestRealizedAmplitudes ();
            TestBoxes ();
            TestInradius ();
            TestCancelTorque ();
            TestLateralFromGap ();
            TestWriteScale ();
            TestRamp ();
            TestShiftedBounds ();
            TestBargeTiltAndCancellation ();
            TestLateralAuthorityFormula ();
            TestAllocationPrefersDifferential ();
            TestHoverClosedLoop ();
            // Computed levers (bit 4, modes 5 and 7).
            TestComputedModes ();
            TestComputedLeverCriterion ();
            TestRcsTranslationColumns ();
            TestRealizedShare ();
            TestJointAllocation ();
            TestComputedAuthorities ();
            TestHeavyJointHover ();
            TestHeavyClampFallback ();
            TestHeavyClampTiltedLevers ();
            // Honest translation (bits 16 and 32, modes 21-55).
            TestHonestModes ();
            TestRcsRotationFractions ();
            TestRcsHeadroomColumns ();
            TestHeavyHeadroomBalance ();
            TestMeasuredShare ();
            TestHeavySharedRcsHover ();
            // the free authority (option 256).
            TestFreeAuthorityRules ();
            TestFreeAuthorityHeavy ();
            checkCount = checks;
            Console.WriteLine ("translation checks: " + checks + ", failures: " + failures);
            return failures;
        }

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.Error.WriteLine ("FAIL (translation): " + what);
            }
        }

        static bool Near (double a, double b, double tol)
        {
            return Math.Abs (a - b) <= tol * Math.Max (1.0, Math.Max (Math.Abs (a), Math.Abs (b)));
        }

        static void Cross (double[] a, double[] b, double[] r)
        {
            double x = a [1] * b [2] - a [2] * b [1];
            double y = a [2] * b [0] - a [0] * b [2];
            double z = a [0] * b [1] - a [1] * b [0];
            r [0] = x;
            r [1] = y;
            r [2] = z;
        }

        // ----------------------------------------------------------- modes, validity

        static void TestModes ()
        {
            Check (AttitudeTranslation.ValidMode (0) && AttitudeTranslation.ValidMode (1) &&
                AttitudeTranslation.ValidMode (3), "modes 0, 1, 3 valid");
            Check (!AttitudeTranslation.ValidMode (2) && !AttitudeTranslation.ValidMode (-1) &&
                !AttitudeTranslation.ValidMode (4), "mode 2 alone (translation without differential) and others refused");
            Check (AttitudeTranslation.RequestLive (true, 3, 10.0, 10.0), "live up to ut_valid_until included");
            Check (!AttitudeTranslation.RequestLive (true, 3, 10.02, 10.0), "stale after ut_valid_until");
            Check (!AttitudeTranslation.RequestLive (true, 0, 9.0, 10.0), "mode 0 is never live");
            Check (!AttitudeTranslation.RequestLive (false, 3, 9.0, 10.0), "no request, nothing live");
            // an earlier change's phantom demand (a recorded flight): the client's mode 53 (bits 1, 4, 16, 32: the differential
            // alone, (0, 0, 0)) is live but asks no lateral force; 55 (bit 2 too) does; the layer's Translate is the same
            // rule (AttitudeV1Translation: live and bit 2).
            Check (AttitudeTranslation.RequestLive (true, 53, 9.0, 10.0) && !AttitudeTranslation.RequestTranslates (true, 53, 9.0, 10.0),
                "mode 53 (the differential alone) is live and asks no lateral force");
            Check (AttitudeTranslation.RequestTranslates (true, 55, 9.0, 10.0) && AttitudeTranslation.RequestTranslates (true, 3, 9.0, 10.0),
                "modes 55 and 3 (bit 2) ask a lateral force");
            Check (!AttitudeTranslation.RequestTranslates (true, 55, 10.02, 10.0) && !AttitudeTranslation.RequestTranslates (false, 55, 9.0, 10.0) &&
                !AttitudeTranslation.RequestTranslates (true, 0, 9.0, 10.0) && !AttitudeTranslation.RequestTranslates (true, 1, 9.0, 10.0),
                "stale, absent, a stop, the differential alone: no lateral force");
            for (int mode = 0; mode < 64; mode++)
                foreach (double ut in new[] { 9.0, 10.0, 10.02, double.NaN })
                    foreach (bool have in new[] { true, false })
                        Check (AttitudeTranslation.RequestTranslates (have, mode, ut, 10.0) ==
                            (AttitudeTranslation.RequestLive (have, mode, ut, 10.0) && (mode & AttitudeTranslation.BitTranslate) != 0),
                            "RequestTranslates is the layer's Translate (live and bit 2), mode " + mode);
            var p = new TranslationParams ();
            bool timeout;
            Check (!AttitudeTranslation.WindDownDone (0.0, 0.0, 0, p, out timeout),
                "wind-down: not done before one step at the main throttle");
            Check (AttitudeTranslation.WindDownDone (0.005, 0.1, 1, p, out timeout) && !timeout,
                "wind-down: done once the realized common deflection is back under 0.01 deg");
            Check (!AttitudeTranslation.WindDownDone (0.5, 0.5, 20, p, out timeout),
                "wind-down: not done while the common deflection is still there");
            Check (AttitudeTranslation.WindDownDone (0.5, 1.0, 20, p, out timeout) && timeout,
                "wind-down: ends by time after 1 s whatever the deflection");
            p.FracGimbal = 0.3;
            p.Reset ();
            Check (p.FracGimbal == 0.8 && p.FracDiff == 0.8 && p.DiffPriority == 0.5,
                "tunables reset to the bench values");
        }

        // ------------------------------------------------------------------ motifs

        static void Square (double half, double[] x, double[] z)
        {
            double[] sx = { 1, -1, -1, 1 }, sz = { 1, 1, -1, -1 };
            for (int i = 0; i < 4; i++) {
                x [i] = sx [i] * half;
                z [i] = sz [i] * half;
            }
        }

        static void TestMotifs ()
        {
            var x = new double[8];
            var z = new double[8];
            var s = new double[8];
            var mp = new double[8];
            var my = new double[8];
            // Barge / small booster: 4 engines in a +-0.6 m square.
            Square (0.6, x, z);
            for (int i = 0; i < 4; i++)
                s [i] = 750e3;
            Check (AttitudeTranslation.Motifs (4, s, x, z, 0.25, mp, my), "square: motifs exist");
            double sumP = 0, sumY = 0, maxP = 0, maxY = 0;
            for (int i = 0; i < 4; i++) {
                sumP += s [i] * mp [i];
                sumY += s [i] * my [i];
                maxP = Math.Max (maxP, Math.Abs (mp [i]));
                maxY = Math.Max (maxY, Math.Abs (my [i]));
                Check (Near (Math.Abs (mp [i]), 1.0, 1e-12) && Near (Math.Abs (my [i]), 1.0, 1e-12),
                    "square: every engine at +-1 of both motifs");
                Check (Math.Sign (mp [i]) == Math.Sign (z [i]) && Math.Sign (my [i]) == Math.Sign (x [i]),
                    "square: pitch motif follows z, yaw motif follows x");
            }
            Check (Math.Abs (sumP) < 1e-6 && Math.Abs (sumY) < 1e-6, "square: total thrust conserved (sum s m = 0)");
            Check (maxP == 1.0 && maxY == 1.0, "square: normalized to max |m| = 1");

            // Unequal slopes (one engine weaker): still zero weighted sum, bounded by 1.
            s [0] = 500e3;
            s [2] = 900e3;
            Check (AttitudeTranslation.Motifs (4, s, x, z, 0.25, mp, my), "unequal slopes: motifs exist");
            sumP = sumY = 0;
            for (int i = 0; i < 4; i++) {
                sumP += s [i] * mp [i];
                sumY += s [i] * my [i];
                Check (Math.Abs (mp [i]) <= 1.0 + 1e-12 && Math.Abs (my [i]) <= 1.0 + 1e-12,
                    "unequal slopes: |motif| <= 1");
            }
            Check (Math.Abs (sumP) < 1e-6 && Math.Abs (sumY) < 1e-6,
                "unequal slopes: weighted sum zero (thrust conserved)");

            // Medium booster: 6 engines in a ring r = 1.26 m.
            for (int i = 0; i < 6; i++) {
                double a = i * Math.PI / 3.0;
                x [i] = 1.26 * Math.Cos (a);
                z [i] = 1.26 * Math.Sin (a);
                s [i] = 900e3;
            }
            Check (AttitudeTranslation.Motifs (6, s, x, z, 0.25, mp, my), "ring of 6: motifs exist");
            sumP = sumY = 0;
            for (int i = 0; i < 6; i++) {
                sumP += s [i] * mp [i];
                sumY += s [i] * my [i];
            }
            Check (Math.Abs (sumP) < 1e-6 && Math.Abs (sumY) < 1e-6, "ring: thrust conserved");

            // Big booster: 8 engines whose centroids sit 0.13 m from the axis: refused.
            for (int i = 0; i < 8; i++) {
                double a = i * Math.PI / 4.0;
                x [i] = 0.13 * Math.Cos (a);
                z [i] = 0.13 * Math.Sin (a);
                s [i] = 1500e3;
            }
            Check (!AttitudeTranslation.Motifs (8, s, x, z, 0.25, mp, my), "big booster (0.13 m): refused");
            bool allZero = true;
            for (int i = 0; i < 8; i++)
                allZero &= mp [i] == 0.0 && my [i] == 0.0;
            Check (allZero, "refused: motifs left at zero");
            // One engine, or engines on a line (no pitch lever): refused.
            Check (!AttitudeTranslation.Motifs (1, s, x, z, 0.25, mp, my), "single engine: refused");
            double[] lx = { -1.0, 0.0, 1.0 }, lz = { 0.0, 0.0, 0.0 }, ls = { 1.0, 1.0, 1.0 };
            Check (!AttitudeTranslation.Motifs (3, ls, lx, lz, 0.25, mp, my), "engines on a line: refused");
            double[] bad = { 1.0, 0.0, 1.0 };
            Check (!AttitudeTranslation.Motifs (3, bad, lx, lz, 0.0, mp, my), "non-positive slope: refused");
        }

        static void TestMotifColumns ()
        {
            // The column is exactly the torque of the per-engine thrust changes.
            var rng = new Random (21);
            for (int k = 0; k < 50; k++) {
                int n = 4 + rng.Next (5);
                var x = new double[n];
                var z = new double[n];
                var s = new double[n];
                var tpn = new double[3 * n];
                var mp = new double[n];
                var my = new double[n];
                for (int i = 0; i < n; i++) {
                    double a = 2 * Math.PI * i / n + 0.3 * rng.NextDouble ();
                    double r = 0.5 + rng.NextDouble ();
                    x [i] = r * Math.Cos (a);
                    z [i] = r * Math.Sin (a);
                    s [i] = 200e3 + 800e3 * rng.NextDouble ();
                    // Thrust along the nose, slightly canted.
                    double[] p = { x [i], -6.0, z [i] };
                    double[] d = { 0.02 * (rng.NextDouble () - 0.5), 1.0, 0.02 * (rng.NextDouble () - 0.5) };
                    var t = new double[3];
                    Cross (p, d, t);
                    tpn [3 * i] = t [0];
                    tpn [3 * i + 1] = t [1];
                    tpn [3 * i + 2] = t [2];
                }
                if (!AttitudeTranslation.Motifs (n, s, x, z, 0.1, mp, my))
                    continue;
                var cp = new double[3];
                var cy = new double[3];
                AttitudeTranslation.MotifColumns (n, s, mp, my, tpn, cp, cy);
                double dp = rng.NextDouble () - 0.5, dy = rng.NextDouble () - 0.5;
                var delta = new double[n];
                AttitudeTranslation.ThrottleDeltas (n, mp, my, dp, dy, delta);
                double[] direct = new double[3];
                double thrust = 0.0;
                for (int i = 0; i < n; i++) {
                    double dT = s [i] * delta [i];
                    thrust += dT;
                    for (int r = 0; r < 3; r++)
                        direct [r] += dT * tpn [3 * i + r];
                }
                for (int r = 0; r < 3; r++)
                    Check (Near (direct [r], cp [r] * dp + cy [r] * dy, 1e-9),
                        "motif columns = torque of the per-engine thrust changes");
                Check (Math.Abs (thrust) < 1e-6 * 1e6, "motif amplitudes change no total thrust");
            }
        }

        static void TestRealizedAmplitudes ()
        {
            var rng = new Random (5);
            for (int k = 0; k < 50; k++) {
                int n = 4 + rng.Next (4);
                var x = new double[n];
                var z = new double[n];
                var s = new double[n];
                var mp = new double[n];
                var my = new double[n];
                for (int i = 0; i < n; i++) {
                    double a = 2 * Math.PI * i / n;
                    x [i] = Math.Cos (a);
                    z [i] = Math.Sin (a);
                    s [i] = 100e3 + 900e3 * rng.NextDouble ();
                }
                if (!AttitudeTranslation.Motifs (n, s, x, z, 0.25, mp, my))
                    continue;
                double c = rng.NextDouble (), dp = 0.1 * (rng.NextDouble () - 0.5),
                    dy = 0.1 * (rng.NextDouble () - 0.5);
                var tau = new double[n];
                for (int i = 0; i < n; i++)
                    tau [i] = c + dp * mp [i] + dy * my [i];
                double rp, ry;
                Check (AttitudeTranslation.RealizedAmplitudes (n, s, mp, my, tau, out rp, out ry) &&
                    Near (rp, dp, 1e-9) && Near (ry, dy, 1e-9),
                    "realized amplitudes recovered from the engines' throttles");
            }
            // Equal throttles: no differential.
            double[] s4 = { 1, 1, 1, 1 }, x4 = new double[4], z4 = new double[4];
            double[] mp4 = new double[4], my4 = new double[4], t4 = { 0.3, 0.3, 0.3, 0.3 };
            Square (0.6, x4, z4);
            AttitudeTranslation.Motifs (4, s4, x4, z4, 0.25, mp4, my4);
            double a0, a1;
            Check (AttitudeTranslation.RealizedAmplitudes (4, s4, mp4, my4, t4, out a0, out a1) &&
                Math.Abs (a0) < 1e-15 && Math.Abs (a1) < 1e-15, "equal throttles: zero amplitudes");
        }

        // ------------------------------------------------------------------- boxes

        static void TestBoxes ()
        {
            var x = new double[4];
            var z = new double[4];
            var s = new double[] { 750e3, 750e3, 750e3, 750e3 };
            var mp = new double[4];
            var my = new double[4];
            Square (0.6, x, z);
            AttitudeTranslation.Motifs (4, s, x, z, 0.25, mp, my);
            double main = 0.1;
            double m = AttitudeTranslation.ThrottleMargin (main, 0.0, 1.0);
            Check (Near (m, 0.1, 1e-15), "hover 0.1: margin 0.1 (down to 0)");
            Check (AttitudeTranslation.ThrottleMargin (0.0, 0.0, 1.0) == 0.0 &&
                AttitudeTranslation.ThrottleMargin (1.0, 0.0, 1.0) == 0.0 &&
                AttitudeTranslation.ThrottleMargin (double.NaN, 0.0, 1.0) == 0.0,
                "no margin at a cut or full main throttle");
            var margin = new double[] { m, m, m, m };
            double h = AttitudeTranslation.SymmetricHalfWidth (4, mp, my, margin);
            Check (Near (h, 0.05, 1e-12), "square at 0.1: symmetric half width 0.05");

            var rng = new Random (3);
            for (int k = 0; k < 300; k++) {
                double dffP = 0.12 * (rng.NextDouble () - 0.5), dffY = 0.12 * (rng.NextDouble () - 0.5);
                double p0 = dffP, y0 = dffY;
                double hp, hy, share;
                double scale = AttitudeTranslation.BoxHalfWidths (4, mp, my, margin, ref dffP,
                    ref dffY, out hp, out hy, out share);
                Check (scale <= 1.0 && scale >= 0.0, "feed-forward scale in [0, 1]");
                Check (Near (dffP, p0 * scale, 1e-15) && Near (dffY, y0 * scale, 1e-15),
                    "feed-forward scaled as a whole (direction kept)");
                Check (share >= 0.0 && hp >= Math.Abs (dffP) - 1e-15 && hy >= Math.Abs (dffY) - 1e-15,
                    "box contains the feed-forward, share >= 0");
                // Every corner of the box keeps every engine within [0, 1].
                for (int c = 0; c < 4; c++) {
                    double dp = (c & 1) != 0 ? hp : -hp, dy = (c & 2) != 0 ? hy : -hy;
                    for (int i = 0; i < 4; i++) {
                        double u = main + dp * mp [i] + dy * my [i];
                        Check (u >= -1e-12 && u <= 1.0 + 1e-12, "box corner keeps every engine in [0, 1]");
                    }
                }
            }
        }

        static void TestInradius ()
        {
            double[] cp = { -1.8e6, 0.0, 0.0 }, cy = { 0.0, 0.0, 1.8e6 };
            Check (Near (AttitudeTranslation.Inradius (cp, cy, 0.05), 90e3, 1e-12),
                "square at hover: 90 kN.m in any direction (173-180 on one axis alone)");
            var rng = new Random (17);
            for (int k = 0; k < 200; k++) {
                double[] a = { rng.NextDouble () - 0.5, 0.0, rng.NextDouble () - 0.5 };
                double[] b = { rng.NextDouble () - 0.5, 0.0, rng.NextDouble () - 0.5 };
                double hw = 0.01 + rng.NextDouble ();
                double rho = AttitudeTranslation.Inradius (a, b, hw);
                // Every direction: the torque rho u needs |d_k| <= hw.
                for (int q = 0; q < 16; q++) {
                    double ang = q * Math.PI / 8.0;
                    double dp, dy;
                    if (!AttitudeTranslation.CancelTorque (a, b, -rho * Math.Cos (ang),
                            -rho * Math.Sin (ang), out dp, out dy))
                        continue;
                    Check (Math.Abs (dp) <= hw * (1 + 1e-9) && Math.Abs (dy) <= hw * (1 + 1e-9),
                        "inradius reachable in every direction");
                }
            }
            Check (AttitudeTranslation.Inradius (new double[3], new double[3], 1.0) == 0.0,
                "null columns: no authority");
        }

        static void TestCancelTorque ()
        {
            var rng = new Random (9);
            for (int k = 0; k < 100; k++) {
                double[] a = { rng.NextDouble () - 0.5, rng.NextDouble (), rng.NextDouble () - 0.5 };
                double[] b = { rng.NextDouble () - 0.5, rng.NextDouble (), rng.NextDouble () - 0.5 };
                double mx = rng.NextDouble () - 0.5, mz = rng.NextDouble () - 0.5;
                double dp, dy;
                if (!AttitudeTranslation.CancelTorque (a, b, mx, mz, out dp, out dy))
                    continue;
                Check (Near (a [0] * dp + b [0] * dy, -mx, 1e-9) && Near (a [2] * dp + b [2] * dy, -mz, 1e-9),
                    "feed-forward cancels the pitch and yaw torque");
            }
            double o0, o1;
            Check (!AttitudeTranslation.CancelTorque (new double[] { 1, 0, 2 }, new double[] { 2, 0, 4 },
                1.0, 1.0, out o0, out o1), "collinear motifs: no cancellation");
        }

        // ------------------------------------------------------------- translation

        static void TestLateralFromGap ()
        {
            var gap = new double[3];
            double ax, az, det;
            // Vertical nose (up = body y): the gap is the horizontal demand.
            double[] up = { 0, 1, 0 };
            double[] demand = { 0.3, 5.0, -0.2 };   // the vertical part is ignored
            Check (AttitudeTranslation.LateralFromGap (demand, up, 9.8, 0.25, gap, out ax, out az, out det) &&
                Near (ax, 0.3, 1e-12) && Near (az, -0.2, 1e-12) && Near (det, 1.0, 1e-12),
                "vertical nose: lateral = horizontal demand");
            // Tilted nose: the horizontal projection of ax ex + az ez must be the gap.
            var rng = new Random (13);
            for (int k = 0; k < 200; k++) {
                double tilt = 50.0 * Deg * rng.NextDouble ();
                double az0 = 2 * Math.PI * rng.NextDouble ();
                double[] u = { Math.Sin (tilt) * Math.Cos (az0), Math.Cos (tilt), Math.Sin (tilt) * Math.Sin (az0) };
                double[] d = { rng.NextDouble () - 0.5, rng.NextDouble () - 0.5, rng.NextDouble () - 0.5 };
                double ta = 5 + 10 * rng.NextDouble ();
                if (!AttitudeTranslation.LateralFromGap (d, u, ta, 0.25, gap, out ax, out az, out det)) {
                    Check (false, "tilt below 50 deg: translation computed");
                    continue;
                }
                double un = Math.Sqrt (u [0] * u [0] + u [1] * u [1] + u [2] * u [2]);
                double[] v = { ax, 0.0, az };
                double vu = (v [0] * u [0] + v [2] * u [2]) / (un * un);
                double[] vh = { v [0] - vu * u [0], v [1] - vu * u [1], v [2] - vu * u [2] };
                for (int r = 0; r < 3; r++)
                    Check (Near (vh [r], gap [r], 1e-9), "horiz(ax ex + az ez) = gap");
                // gap = horiz(demand) - ta horiz(nose)
                double du = (d [0] * u [0] + d [1] * u [1] + d [2] * u [2]) / (un * un);
                double nu = u [1] / (un * un);
                for (int r = 0; r < 3; r++) {
                    double expect = (d [r] - du * u [r]) - ta * ((r == 1 ? 1.0 : 0.0) - nu * u [r]);
                    Check (Near (gap [r], expect, 1e-9), "gap = horiz(demand) - T/m horiz(nose)");
                }
                Check (Near (det, Math.Cos (tilt) * Math.Cos (tilt), 1e-9), "det = cos^2(tilt)");
            }
            double[] far = { Math.Sin (70 * Deg), Math.Cos (70 * Deg), 0.0 };
            Check (!AttitudeTranslation.LateralFromGap (demand, far, 9.8, 0.25, gap, out ax, out az, out det),
                "nose 70 deg from the vertical: no translation");
            Check (!AttitudeTranslation.LateralFromGap (demand, new double[3], 9.8, 0.25, gap, out ax, out az, out det),
                "null up: no translation");
        }

        static void TestWriteScale ()
        {
            double[] delta = { 0.05, -0.05, 0.05, -0.05 };
            Check (AttitudeTranslation.WriteScale (4, delta, 0.0, 0.0, 1.0) == 0.0,
                "main throttle cut: no engine lit by the differential");
            Check (AttitudeTranslation.WriteScale (4, delta, 0.5, 0.0, 1.0) == 1.0,
                "ample margin: offsets written whole");
            double k = AttitudeTranslation.WriteScale (4, delta, 0.02, 0.0, 1.0);
            Check (Near (k, 0.4, 1e-12), "main 0.02: offsets scaled to 0.4 (engine at 0)");
            k = AttitudeTranslation.WriteScale (4, delta, 0.98, 0.0, 1.0);
            Check (Near (k, 0.4, 1e-12), "main 0.98: offsets scaled to 0.4 (engine at 1)");
            Check (AttitudeTranslation.WriteScale (4, delta, double.NaN, 0.0, 1.0) == 0.0,
                "NaN main throttle: nothing");
            var rng = new Random (1);
            for (int q = 0; q < 200; q++) {
                double main = rng.NextDouble ();
                var d = new double[6];
                double sum = 0;
                for (int i = 0; i < 5; i++) {
                    d [i] = 0.3 * (rng.NextDouble () - 0.5);
                    sum += d [i];
                }
                d [5] = -sum;
                double kk = AttitudeTranslation.WriteScale (6, d, main, 0.0, 1.0);
                double total = 0;
                for (int i = 0; i < 6; i++) {
                    double u = main + kk * d [i];
                    total += kk * d [i];
                    Check (u >= -1e-12 && u <= 1 + 1e-12, "written throttles within [0, 1]");
                }
                Check (Math.Abs (total) < 1e-12, "write scale keeps the offsets zero-sum");
            }
        }

        static void TestRamp ()
        {
            double f = AttitudeTranslation.RampFactor (true, 8.0, 0.02);
            Check (Near (f, 0.16, 1e-12), "gimbalResponseSpeed 8 at 0.02 s: 0.16 per step (stock Lerp)");
            Check (AttitudeTranslation.RampFactor (false, 8.0, 0.02) == 1.0, "no response speed: immediate");
            Check (AttitudeTranslation.RampFactor (true, 100.0, 0.02) == 1.0, "factor capped at 1");
            // n steps of the model = the lease's 1 - (1 - f)^n (ActuatorsAddon.ApplyGimbal).
            double v = 0.3;
            for (int i = 0; i < 12; i++)
                v = AttitudeTranslation.Ramp (v, 1.7, f);
            double lease = 0.3 + (1.7 - 0.3) * (1.0 - Math.Pow (1.0 - f, 12));
            Check (Near (v, lease, 1e-12), "ramp model = lease ramp over several steps");
        }

        static void TestShiftedBounds ()
        {
            double u0, lo, hi;
            AttitudeTranslation.ShiftGimbal (1.2, 0.9, 1.4, -2.0, 2.0, out u0, out lo, out hi);
            Check (Near (u0, 0.3, 1e-15) && Near (lo, -3.4, 1e-15) && Near (hi, 0.6, 1e-15),
                "gimbal: realized attitude part, bounds shifted by the common target");
            Check (Near (1.4 + lo, -2.0, 1e-15) && Near (1.4 + hi, 2.0, 1e-15),
                "gimbal: common + attitude stays within the physical range");
            AttitudeTranslation.DiffBounds (0.03, 0.02, 0.025, 0.04, true, out u0, out lo, out hi);
            Check (Near (u0, 0.01, 1e-15) && Near (lo, -0.065, 1e-15) && Near (hi, 0.015, 1e-15),
                "motif: realized attitude part, box [-h - dff, h - dff]");
            AttitudeTranslation.DiffBounds (0.03, 0.02, 0.025, 0.04, false, out u0, out lo, out hi);
            Check (lo == 0.0 && hi == 0.0, "wind-down: no attitude share on the motifs");
        }

        // --------------------------------------------------- barge-like geometry

        sealed class Barge
        {
            internal readonly double[] Px = new double[4], Pz = new double[4];
            internal const double Py = -5.96, Qy = -3.8;
            internal const double Mass = 29300.0;
            internal readonly double[] Inertia = { 1.0e6, 5.0e4, 1.0e6 };
            internal const double Tmax = 750e3;
            internal const double RangeDeg = 2.0;

            internal Barge ()
            {
                Square (0.6, Px, Pz);
            }

            /// <summary>Torque and force columns per degree of gimbal i about body x and z.</summary>
            internal void Columns (int i, double thrust, double[] colX, double[] colY,
                double[] forceX, double[] forceY)
            {
                double[] p = { Px [i], Py, Pz [i] };
                double[] off = { 0.0, Py - Qy, 0.0 };
                double[] d = { 0.0, 1.0, 0.0 };
                double[] f = { 0.0, thrust, 0.0 };
                var axes = new[] { new double[] { 1, 0, 0 }, new double[] { 0, 0, 1 } };
                for (int k = 0; k < 2; k++) {
                    var a = axes [k];
                    var dF = new double[3];
                    Cross (a, d, dF);
                    for (int r = 0; r < 3; r++)
                        dF [r] *= thrust;
                    var dLever = new double[3];
                    Cross (a, off, dLever);
                    var t1 = new double[3];
                    var t2 = new double[3];
                    Cross (dLever, f, t1);
                    Cross (p, dF, t2);
                    var col = k == 0 ? colX : colY;
                    var force = k == 0 ? forceX : forceY;
                    for (int r = 0; r < 3; r++) {
                        col [r] = (t1 [r] + t2 [r]) * Deg;
                        force [r] = dF [r] * Deg;
                    }
                }
            }

            internal void TorquePerNewton (int i, double[] t)
            {
                Cross (new[] { Px [i], Py, Pz [i] }, new double[] { 0, 1, 0 }, t);
            }
        }

        static void TestBargeTiltAndCancellation ()
        {
            var b = new Barge ();
            double tEach = Barge.Mass * 9.81 / 4.0;
            // Gimbal model sanity: 3.8 m effective arm (0.64 x the nozzle lever).
            var cx = new double[3];
            var cy = new double[3];
            var fx = new double[3];
            var fy = new double[3];
            b.Columns (0, tEach, cx, cy, fx, fy);
            Check (Near (cx [0], -3.8 * tEach * Deg, 1e-9), "gimbal column: pivot 3.8 m below the CoM");
            // Full common deflection torque: ~38 kN.m at hover for 2 deg (the bench figure).
            double full = 4 * 3.8 * tEach * Deg * Barge.RangeDeg;
            Check (full > 36e3 && full < 40e3, "full common deflection torque ~38 kN.m");

            // Tilt matrix and torque of a body tilt (axes: gimbal x = body x, y = body z).
            double kxx = 0, kxz = 0, kzx = 0, kzz = 0, txx = 0, txz = 0, tzx = 0, tzz = 0;
            for (int i = 0; i < 4; i++) {
                b.Columns (i, tEach, cx, cy, fx, fy);
                kxx += fx [0];
                kxz += fy [0];
                kzx += fx [2];
                kzz += fy [2];
                txx += cx [0];
                txz += cy [0];
                tzx += cx [2];
                tzz += cy [2];
            }
            // Motifs.
            var s = new double[] { Barge.Tmax, Barge.Tmax, Barge.Tmax, Barge.Tmax };
            var mp = new double[4];
            var my = new double[4];
            var tpn = new double[12];
            var t3 = new double[3];
            for (int i = 0; i < 4; i++) {
                b.TorquePerNewton (i, t3);
                tpn [3 * i] = t3 [0];
                tpn [3 * i + 1] = t3 [1];
                tpn [3 * i + 2] = t3 [2];
            }
            AttitudeTranslation.Motifs (4, s, b.Px, b.Pz, 0.25, mp, my);
            var colP = new double[3];
            var colY = new double[3];
            AttitudeTranslation.MotifColumns (4, s, mp, my, tpn, colP, colY);
            double main = tEach / Barge.Tmax;
            var margin = new double[4];
            for (int i = 0; i < 4; i++)
                margin [i] = AttitudeTranslation.ThrottleMargin (main, 0, 1);

            // 0.2 m/s^2 lateral in body +z, then +x.
            for (int q = 0; q < 2; q++) {
                double ax = q == 0 ? 0.0 : 0.2, az = q == 0 ? 0.2 : 0.0;
                double phiX, phiZ;
                Check (AttitudeTranslation.SolveTilt (new[] { kxx, kxz, kzx, kzz }, ax * Barge.Mass,
                    az * Barge.Mass, out phiX, out phiZ), "barge: tilt matrix invertible");
                double forceX = kxx * phiX + kxz * phiZ, forceZ = kzx * phiX + kzz * phiZ;
                Check (Near (forceX / Barge.Mass, ax, 1e-9) && Near (forceZ / Barge.Mass, az, 1e-9),
                    "barge: the common tilt gives the wanted lateral acceleration");
                double tilt = Math.Sqrt (phiX * phiX + phiZ * phiZ);
                Check (tilt > 1.0 && tilt < 1.3, "barge: ~1.17 deg of common tilt for 0.2 m/s^2");
                double frac = AttitudeTranslation.GimbalFraction (phiX, phiZ, 2, 2, 2, 2);
                Check (frac < 0.8, "barge: 0.2 m/s^2 within 0.8 of the range");
                double mx = txx * phiX + txz * phiZ, mz = tzx * phiX + tzz * phiZ;
                Check (Math.Sqrt (mx * mx + mz * mz) > 20e3, "barge: the common tilt makes ~22 kN.m");
                double dp, dy;
                Check (AttitudeTranslation.CancelTorque (colP, colY, mx, mz, out dp, out dy),
                    "barge: the differential can cancel it");
                // Net pitch/yaw torque of the pair = 0, total thrust unchanged.
                Check (Math.Abs (mx + colP [0] * dp + colY [0] * dy) < 1e-6 &&
                    Math.Abs (mz + colP [2] * dp + colY [2] * dy) < 1e-6,
                    "barge: common deflection + feed-forward = pure translation (no pitch/yaw torque)");
                double cap = AttitudeTranslation.DiffCapacityScale (4, mp, my, margin, dp, dy, 0.8);
                Check (cap > 1.0, "barge: 0.2 m/s^2 needs less than 0.8 of the throttle margin");
                double hp, hy, share, dffP = dp, dffY = dy;
                double scale = AttitudeTranslation.BoxHalfWidths (4, mp, my, margin, ref dffP, ref dffY,
                    out hp, out hy, out share);
                Check (scale == 1.0 && share > 0.0, "barge: feed-forward fits, attitude share left");
            }
            // A demand beyond the range is capped by the gimbal fraction, a hover at 0.01
            // throttle by the differential capacity.
            double px, pz;
            AttitudeTranslation.SolveTilt (new[] { kxx, kxz, kzx, kzz }, 0.0, 1.0 * Barge.Mass, out px, out pz);
            double worst = AttitudeTranslation.GimbalFraction (px, pz, 2, 2, 2, 2);
            Check (worst > 0.8, "1 m/s^2: more than 0.8 of the range, so it will be capped");
            double m2 = txx * px + txz * pz, z2 = tzx * px + tzz * pz, d2p, d2y;
            AttitudeTranslation.CancelTorque (colP, colY, m2, z2, out d2p, out d2y);
            var low = new double[] { 0.01, 0.01, 0.01, 0.01 };
            double capLow = AttitudeTranslation.DiffCapacityScale (4, mp, my, low, d2p, d2y, 0.8);
            // 1 m/s^2: 111 kN.m to cancel, 0.062 of throttle per engine, 0.008 available.
            Check (capLow > 0.1 && capLow < 0.16,
                "throttle 0.01: the differential capacity caps the translation (" + capLow.ToString ("F3") + ")");
        }

        static void TestLateralAuthorityFormula ()
        {
            double a = AttitudeTranslation.LateralAuthority (9.8, 2.0, 0.8, 0.8, 90e3, 38e3);
            Check (Near (a, 9.8 * Math.Sin (2 * Deg) * 0.8, 1e-12), "barge: authority capped by the gimbal share (0.27 m/s^2)");
            a = AttitudeTranslation.LateralAuthority (9.8, 2.0, 0.8, 0.8, 20e3, 38e3);
            Check (Near (a, 9.8 * Math.Sin (2 * Deg) * 0.8 * 20.0 / 38.0, 1e-12),
                "low differential authority: capped by what it can cancel");
            Check (AttitudeTranslation.LateralAuthority (9.8, 2.0, 0.8, 0.8, 0.0, 38e3) == 0.0,
                "no differential authority: no lateral authority");
            Check (AttitudeTranslation.LateralAuthority (0.0, 2.0, 0.8, 0.8, 90e3, 38e3) == 0.0,
                "no thrust: no lateral authority");
            Check (Near (AttitudeTranslation.MaxSingular2 (3, 0, 0, -2), 3, 1e-12) &&
                Near (AttitudeTranslation.MaxSingular2 (0, -4, 4, 0), 4, 1e-12),
                "largest singular value of a 2x2");
        }

        // ------------------------------------------------- the law with the motifs

        /// <summary>
        /// Barge law input: 4 gimbals (8 columns), ctrlState (3, small RCS), and optionally
        /// the two motifs (priority p). Hover thrust, no error.
        /// </summary>
        static void FillBarge (AttitudeInput inp, bool motifs, double diffPriority,
            double[] colP, double[] colY, double half)
        {
            var b = new Barge ();
            int n = 8 + 3 + (motifs ? 2 : 0);
            inp.Resize (n);
            inp.Dt = 0.02;
            double tEach = Barge.Mass * 9.81 / 4.0;
            var cx = new double[3];
            var cy = new double[3];
            var fx = new double[3];
            var fy = new double[3];
            for (int i = 0; i < 4; i++) {
                b.Columns (i, tEach, cx, cy, fx, fy);
                for (int r = 0; r < 3; r++) {
                    inp.B [r * n + 2 * i] = cx [r];
                    inp.B [r * n + 2 * i + 1] = cy [r];
                }
                for (int k = 0; k < 2; k++) {
                    int j = 2 * i + k;
                    inp.U0 [j] = 0;
                    inp.UMin [j] = -2;
                    inp.UMax [j] = 2;
                    inp.UPref [j] = 0;
                    inp.Priority [j] = 1;
                }
            }
            for (int a = 0; a < 3; a++) {
                int j = 8 + a;
                for (int r = 0; r < 3; r++)
                    inp.B [r * n + j] = r == a ? -2000.0 : 0.0;
                inp.U0 [j] = 0;
                inp.UMin [j] = -1;
                inp.UMax [j] = 1;
                inp.UPref [j] = 0;
                inp.Priority [j] = 3;
            }
            if (motifs) {
                for (int r = 0; r < 3; r++) {
                    inp.B [r * n + 11] = colP [r];
                    inp.B [r * n + 12] = colY [r];
                }
                for (int j = 11; j < 13; j++) {
                    inp.U0 [j] = 0;
                    inp.UMin [j] = -half;
                    inp.UMax [j] = half;
                    inp.UPref [j] = 0;
                    inp.Priority [j] = diffPriority;
                }
            }
            for (int a = 0; a < 3; a++) {
                inp.Inertia [a] = new Barge ().Inertia [a];
                inp.Omega [a] = 0;
                inp.OmegaDotMeasured [a] = 0;
                inp.Torque [a] = 0;
                inp.Error [a] = 0;
                inp.OmegaFf [a] = 0;
                inp.OmegaDotFf [a] = 0;
            }
            inp.OmegaDotValid = true;
            inp.ReferenceValid = true;
            inp.OmegaMax = 0.2;
        }

        static void BargeMotifs (double[] colP, double[] colY, double[] mp, double[] my)
        {
            var b = new Barge ();
            var s = new double[] { Barge.Tmax, Barge.Tmax, Barge.Tmax, Barge.Tmax };
            var tpn = new double[12];
            var t3 = new double[3];
            for (int i = 0; i < 4; i++) {
                b.TorquePerNewton (i, t3);
                tpn [3 * i] = t3 [0];
                tpn [3 * i + 1] = t3 [1];
                tpn [3 * i + 2] = t3 [2];
            }
            AttitudeTranslation.Motifs (4, s, b.Px, b.Pz, 0.25, mp, my);
            AttitudeTranslation.MotifColumns (4, s, mp, my, tpn, colP, colY);
        }

        static void TestAllocationPrefersDifferential ()
        {
            var colP = new double[3];
            var colY = new double[3];
            var mp = new double[4];
            var my = new double[4];
            BargeMotifs (colP, colY, mp, my);
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            FillBarge (inp, true, 0.5, colP, colY, 0.05);
            inp.Error [0] = 0.01;            // a small pitch error
            for (int t = 0; t < 5; t++)
                AttitudeMath.Step (g, inp, st, work, o);
            Check (o.Computed && o.Converged, "with motifs: law computes and converges");
            double gimbalPitch = 0, diffPitch = inp.B [0 * inp.N + 11] * o.DeltaU [11] + inp.B [0 * inp.N + 12] * o.DeltaU [12];
            for (int j = 0; j < 8; j++)
                gimbalPitch += inp.B [0 * inp.N + j] * o.DeltaU [j];
            Check (Math.Abs (diffPitch) > Math.Abs (gimbalPitch),
                "priority 0.5 against 1: the differential carries most of the pitch torque");
            Check (Near (gimbalPitch + diffPitch + inp.B [0 * inp.N + 8] * o.DeltaU [8],
                o.TorqueAllocTarget [0], 1e-3), "pitch target allocated");
            Check (o.DeltaU [11] >= -0.05 - 1e-9 && o.DeltaU [11] <= 0.05 + 1e-9, "motif within its box");
            // Same input without motifs: the law of c03d5de on the gimbals alone.
            var st2 = new AttitudeLawState ();
            var o2 = new AttitudeOutput ();
            var inp2 = new AttitudeInput ();
            FillBarge (inp2, false, 0.5, colP, colY, 0.05);
            inp2.Error [0] = 0.01;
            for (int t = 0; t < 5; t++)
                AttitudeMath.Step (g, inp2, st2, new AttitudeWork (), o2);
            Check (o2.Computed && inp2.N == 11, "without motifs: 11 columns, as c03d5de");
            Check (o.Alpha [0] > o2.Alpha [0], "the motifs add pitch authority");
        }

        // ------------------------------------------------------ closed-loop hover

        sealed class HoverResult
        {
            internal double MaxTiltDeg, FinalTiltDeg, FinalAccelZ, MaxAbsGimbal, MinThrottle,
                MaxThrottle, MaxAttitudeRate, SettleAccelZ, FinalThrottleSpread, FinalCommon;
            internal bool AllComputed = true;
            internal int WindDownSteps = -1;
            internal bool WindDownTimeout;
        }

        /// <summary>
        /// The plugin's sequence per physics step on the barge model: plan (motifs, common
        /// tilt, realized ramp, feed-forward, box, shifted bounds), the unchanged law, the
        /// commands (common + attitude, feed-forward + attitude), then one physics step with
        /// the TRUE plant (gimbal torque x gimbalErr, differential torque x slopeErr, engine
        /// throttle lag, a constant unmeasured disturbance torque). The realized torque the
        /// law receives is the true actuator torque, as the plugin measures it (finalThrust at
        /// the real nozzles); the columns are the nominal model. The attitude reference is the
        /// vertical, the demand 0.2 m/s^2 of horizontal acceleration along world z.
        /// </summary>
        static HoverResult Hover (bool feedForward, bool translate, double engineLag,
            double gimbalErr, double slopeErr, double disturbance, int steps, int staleAt = -1)
        {
            // staleAt >= 0: the request goes stale at that step; the plugin's wind-down
            // follows (common target 0, no attitude share, then the layer is removed).
            bool windDown = false, layerOn = true;
            int windDownStart = 0, windDownSteps = 0;
            var b = new Barge ();
            var p = new TranslationParams ();
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            const double dt = 0.02;
            double main = Barge.Mass * 9.81 / (4.0 * Barge.Tmax);
            double f = AttitudeTranslation.RampFactor (true, 8.0, dt);
            int n = 13;
            inp.Resize (n);
            inp.Dt = dt;
            for (int a = 0; a < 3; a++)
                inp.Inertia [a] = b.Inertia [a];
            var theta = new double[3];
            var omega = new double[3];
            var omegaPrev = new double[3];
            bool havePrev = false;
            var uReal = new double[8];
            var uCmd = new double[8];
            var tauReal = new double[] { main, main, main, main };
            var tauCmd = new double[] { main, main, main, main };
            var rcsCmd = new double[3];
            const double RcsColumn = -2000.0;
            var common = new double[8];
            var commonReal = new double[8];
            var commonNext = new double[8];
            double dffPrevP = 0, dffPrevY = 0;
            var slope = new double[] { Barge.Tmax, Barge.Tmax, Barge.Tmax, Barge.Tmax };
            var mp = new double[4];
            var my = new double[4];
            var tpn = new double[12];
            var t3 = new double[3];
            for (int i = 0; i < 4; i++) {
                b.TorquePerNewton (i, t3);
                tpn [3 * i] = t3 [0];
                tpn [3 * i + 1] = t3 [1];
                tpn [3 * i + 2] = t3 [2];
            }
            AttitudeTranslation.Motifs (4, slope, b.Px, b.Pz, p.MinLeverArmM, mp, my);
            var colP = new double[3];
            var colY = new double[3];
            AttitudeTranslation.MotifColumns (4, slope, mp, my, tpn, colP, colY);
            var margin = new double[4];
            for (int i = 0; i < 4; i++)
                margin [i] = AttitudeTranslation.ThrottleMargin (main, 0, 1);
            var cx = new double[3];
            var cy = new double[3];
            var fx = new double[3];
            var fy = new double[3];
            var gap = new double[3];
            var delta = new double[4];
            var res = new HoverResult { MinThrottle = 1, MaxThrottle = 0 };
            double accelZ = 0;
            double accelSum = 0;
            int accelCount = 0;
            for (int step = 0; step < steps; step++) {
                bool live = staleAt < 0 || step < staleAt;
                if (!live && layerOn) {
                    if (!windDown) {
                        windDown = true;
                        windDownStart = step;
                        windDownSteps = 0;
                    } else {
                        double maxCr = 0;
                        for (int j = 0; j < 8; j++)
                            maxCr = Math.Max (maxCr, Math.Abs (commonReal [j]));
                        bool timeout;
                        if (AttitudeTranslation.WindDownDone (maxCr, (step - windDownStart) * dt,
                                windDownSteps, p, out timeout)) {
                            layerOn = false;
                            res.WindDownSteps = step - windDownStart;
                            res.WindDownTimeout = timeout;
                            for (int j = 0; j < 8; j++)
                                commonReal [j] = 0;
                            dffPrevP = dffPrevY = 0;
                        }
                    }
                }
                // ---- measurement (Earlyish): realized torque from the realized actuators.
                var thrust = new double[4];
                for (int i = 0; i < 4; i++)
                    thrust [i] = slope [i] * tauReal [i];
                var torque = new double[3];
                for (int i = 0; i < 4; i++) {
                    b.Columns (i, thrust [i], cx, cy, fx, fy);
                    for (int r = 0; r < 3; r++) {
                        inp.B [r * n + 2 * i] = cx [r];
                        inp.B [r * n + 2 * i + 1] = cy [r];
                        // Measured as the plugin does: the REAL engine torque (true plant).
                        torque [r] += gimbalErr * (cx [r] * uReal [2 * i] + cy [r] * uReal [2 * i + 1]);
                        torque [r] += thrust [i] * slopeErr * tpn [3 * i + r];
                    }
                }
                for (int a = 0; a < 3; a++)
                    torque [a] += RcsColumn * rcsCmd [a];
                for (int a = 0; a < 3; a++) {
                    inp.Omega [a] = omega [a];
                    inp.OmegaDotMeasured [a] = havePrev ? (omega [a] - omegaPrev [a]) / dt : 0.0;
                    inp.Torque [a] = torque [a];
                    inp.OmegaFf [a] = 0;
                    inp.OmegaDotFf [a] = 0;
                }
                inp.OmegaDotValid = havePrev;
                havePrev = true;
                Array.Copy (omega, omegaPrev, 3);
                // Pointing error toward the vertical, as the plugin (target in body frame).
                inp.ReferenceValid = AttitudeMath.PointingError (theta [2], 1.0, -theta [0], inp.Error);
                inp.OmegaMax = 0.2;
                for (int a = 0; a < 3; a++)
                    inp.OmegaMaxAxis [a] = 0.2;
                for (int j = 0; j < 8; j++) {
                    inp.U0 [j] = uReal [j];
                    inp.UMin [j] = -Barge.RangeDeg;
                    inp.UMax [j] = Barge.RangeDeg;
                    inp.UPref [j] = 0;
                    inp.Priority [j] = 1;
                }
                for (int a = 0; a < 3; a++) {
                    int j = 8 + a;
                    for (int r = 0; r < 3; r++)
                        inp.B [r * n + j] = r == a ? RcsColumn : 0.0;
                    inp.U0 [j] = rcsCmd [a];
                    inp.UMin [j] = -1;
                    inp.UMax [j] = 1;
                    inp.UPref [j] = 0;
                    inp.Priority [j] = 3;
                }

                // ---- plan: realized amplitudes, translation, ramp, feed-forward, box.
                double dRealP, dRealY;
                AttitudeTranslation.RealizedAmplitudes (4, slope, mp, my, tauReal, out dRealP, out dRealY);
                for (int j = 0; j < 8; j++)
                    common [j] = 0;
                double totalThrust = thrust [0] + thrust [1] + thrust [2] + thrust [3];
                if (translate && live && layerOn) {
                    // Up in body (small angles): up_world = y, body = y - theta x y.
                    double[] up = { theta [2], 1.0, -theta [0] };
                    double[] demand = { 0.0, 0.0, 0.2 };
                    double[] demandBody = { demand [0] + theta [2] * demand [1] - 0.0,
                        demand [1] - theta [2] * demand [0] + theta [0] * demand [2],
                        demand [2] - theta [0] * demand [1] };
                    double ax, az, det;
                    if (AttitudeTranslation.LateralFromGap (demandBody, up, totalThrust / Barge.Mass,
                            p.MinProjectionDet, gap, out ax, out az, out det)) {
                        double kxx = 0, kxz = 0, kzx = 0, kzz = 0, txx = 0, txz = 0, tzx = 0, tzz = 0;
                        for (int i = 0; i < 4; i++) {
                            b.Columns (i, thrust [i], cx, cy, fx, fy);
                            kxx += fx [0];
                            kxz += fy [0];
                            kzx += fx [2];
                            kzz += fy [2];
                            txx += cx [0];
                            txz += cy [0];
                            tzx += cx [2];
                            tzz += cy [2];
                        }
                        double phiX, phiZ;
                        if (AttitudeTranslation.SolveTilt (new[] { kxx, kxz, kzx, kzz }, ax * Barge.Mass,
                                az * Barge.Mass, out phiX, out phiZ)) {
                            double worst = AttitudeTranslation.GimbalFraction (phiX, phiZ, 2, 2, 2, 2);
                            double scale = worst > p.FracGimbal ? p.FracGimbal / worst : 1.0;
                            double dp0, dy0;
                            if (AttitudeTranslation.CancelTorque (colP, colY, txx * phiX + txz * phiZ,
                                    tzx * phiX + tzz * phiZ, out dp0, out dy0))
                                scale = Math.Min (scale, AttitudeTranslation.DiffCapacityScale (4, mp, my,
                                    margin, dp0, dy0, p.FracDiff));
                            for (int i = 0; i < 4; i++) {
                                common [2 * i] = scale * phiX;
                                common [2 * i + 1] = scale * phiZ;
                            }
                        }
                    }
                }
                double mx = 0, mz = 0;
                for (int j = 0; j < 8; j++) {
                    commonNext [j] = AttitudeTranslation.Ramp (commonReal [j], common [j], f);
                    mx += inp.B [0 * n + j] * commonNext [j];
                    mz += inp.B [2 * n + j] * commonNext [j];
                }
                double dffP = 0, dffY = 0;
                if (feedForward)
                    AttitudeTranslation.CancelTorque (colP, colY, mx, mz, out dffP, out dffY);
                double hp, hy, share;
                AttitudeTranslation.BoxHalfWidths (4, mp, my, margin, ref dffP, ref dffY, out hp, out hy, out share);
                for (int j = 0; j < 8; j++) {
                    double u0, lo, hi;
                    AttitudeTranslation.ShiftGimbal (uReal [j], commonReal [j], common [j], inp.UMin [j],
                        inp.UMax [j], out u0, out lo, out hi);
                    inp.U0 [j] = u0;
                    inp.UMin [j] = lo;
                    inp.UMax [j] = hi;
                }
                for (int r = 0; r < 3; r++) {
                    inp.B [r * n + 11] = colP [r];
                    inp.B [r * n + 12] = colY [r];
                }
                double u0p, lop, hip, u0y, loy, hiy;
                AttitudeTranslation.DiffBounds (dRealP, dffPrevP, dffP, hp, live, out u0p, out lop, out hip);
                AttitudeTranslation.DiffBounds (dRealY, dffPrevY, dffY, hy, live, out u0y, out loy, out hiy);
                if (!layerOn) {
                    // Layer removed: no motif (the plugin drops the two columns), no offsets.
                    for (int r = 0; r < 3; r++) {
                        inp.B [r * n + 11] = 0;
                        inp.B [r * n + 12] = 0;
                    }
                    u0p = lop = hip = u0y = loy = hiy = 0;
                    dffP = dffY = 0;
                    for (int j = 0; j < 8; j++)
                        commonNext [j] = 0;
                }
                inp.U0 [11] = u0p;
                inp.UMin [11] = lop;
                inp.UMax [11] = hip;
                inp.U0 [12] = u0y;
                inp.UMin [12] = loy;
                inp.UMax [12] = hiy;
                inp.UPref [11] = 0;
                inp.UPref [12] = 0;
                inp.Priority [11] = p.DiffPriority;
                inp.Priority [12] = p.DiffPriority;

                // ---- law (unchanged) and commands.
                AttitudeMath.Step (g, inp, st, work, o);
                if (step > 0 && !o.Computed)
                    res.AllComputed = false;
                if (o.Computed) {
                    for (int j = 0; j < 8; j++) {
                        uCmd [j] = common [j] + inp.U0 [j] + o.DeltaU [j];
                        if (uCmd [j] > 2.0)
                            uCmd [j] = 2.0;
                        if (uCmd [j] < -2.0)
                            uCmd [j] = -2.0;
                        res.MaxAbsGimbal = Math.Max (res.MaxAbsGimbal, Math.Abs (uCmd [j]));
                    }
                    double dP = dffP + inp.U0 [11] + o.DeltaU [11];
                    double dY = dffY + inp.U0 [12] + o.DeltaU [12];
                    AttitudeTranslation.ThrottleDeltas (4, mp, my, dP, dY, delta);
                    if (!layerOn)
                        for (int i = 0; i < 4; i++)
                            delta [i] = 0;
                    double kappa = AttitudeTranslation.WriteScale (4, delta, main, 0, 1);
                    for (int i = 0; i < 4; i++) {
                        tauCmd [i] = main + kappa * delta [i];
                        res.MinThrottle = Math.Min (res.MinThrottle, tauCmd [i]);
                        res.MaxThrottle = Math.Max (res.MaxThrottle, tauCmd [i]);
                    }
                    for (int a = 0; a < 3; a++)
                        rcsCmd [a] = Math.Max (-1.0, Math.Min (1.0, inp.U0 [8 + a] + o.DeltaU [8 + a]));
                    Array.Copy (commonNext, commonReal, 8);
                    dffPrevP = dffP;
                    dffPrevY = dffY;
                    if (windDown && layerOn)
                        windDownSteps++;
                }

                // ---- physics step with the TRUE plant.
                for (int j = 0; j < 8; j++)
                    uReal [j] = AttitudeTranslation.Ramp (uReal [j], uCmd [j], f);
                for (int i = 0; i < 4; i++)
                    tauReal [i] = engineLag >= 1.0 ? tauCmd [i]
                        : tauReal [i] + (tauCmd [i] - tauReal [i]) * engineLag;
                var trueTorque = new double[3];
                double lateralX = 0, lateralZ = 0, thrustSum = 0;
                for (int i = 0; i < 4; i++) {
                    double ti = slope [i] * tauReal [i];
                    thrustSum += ti;
                    b.Columns (i, ti, cx, cy, fx, fy);
                    for (int r = 0; r < 3; r++) {
                        trueTorque [r] += gimbalErr * (cx [r] * uReal [2 * i] + cy [r] * uReal [2 * i + 1]);
                        trueTorque [r] += ti * slopeErr * tpn [3 * i + r];
                    }
                    lateralX += fx [0] * uReal [2 * i] + fy [0] * uReal [2 * i + 1];
                    lateralZ += fx [2] * uReal [2 * i] + fy [2] * uReal [2 * i + 1];
                }
                for (int a = 0; a < 3; a++)
                    trueTorque [a] += RcsColumn * rcsCmd [a];
                trueTorque [0] += disturbance;
                trueTorque [2] -= 0.5 * disturbance;
                for (int a = 0; a < 3; a++) {
                    omega [a] += trueTorque [a] / b.Inertia [a] * dt;
                    theta [a] += omega [a] * dt;
                }
                // World horizontal acceleration along z: nose tilt (theta_x) + gimbal force.
                accelZ = (thrustSum * theta [0] + lateralZ) / Barge.Mass;
                double tilt = Math.Sqrt (theta [0] * theta [0] + theta [2] * theta [2]) / Deg;
                res.MaxTiltDeg = Math.Max (res.MaxTiltDeg, tilt);
                res.MaxAttitudeRate = Math.Max (res.MaxAttitudeRate,
                    Math.Sqrt (omega [0] * omega [0] + omega [2] * omega [2]));
                if (step >= steps - 25) {
                    accelSum += accelZ;
                    accelCount++;
                }
            }
            res.FinalTiltDeg = Math.Sqrt (theta [0] * theta [0] + theta [2] * theta [2]) / Deg;
            for (int i = 0; i < 4; i++)
                res.FinalThrottleSpread = Math.Max (res.FinalThrottleSpread, Math.Abs (tauCmd [i] - main));
            for (int j = 0; j < 8; j++)
                res.FinalCommon = Math.Max (res.FinalCommon, Math.Abs (commonReal [j]));
            res.FinalAccelZ = accelZ;
            res.SettleAccelZ = accelCount > 0 ? accelSum / accelCount : 0;
            return res;
        }

        static void TestHoverClosedLoop ()
        {
            // Ideal plant, instant engines: the translation is delivered, the attitude holds.
            var r = Hover (true, true, 1.0, 1.0, 1.0, 0.0, 150);
            Check (r.AllComputed, "hover: the law computes every step");
            Check (Math.Abs (r.SettleAccelZ - 0.2) < 0.02,
                "hover: 0.2 m/s^2 of lateral acceleration delivered (" + r.SettleAccelZ.ToString ("F4") + ")");
            Check (r.MaxTiltDeg < 0.2, "hover: attitude held within 0.2 deg (" + r.MaxTiltDeg.ToString ("F4") + ")");
            Check (r.MaxAbsGimbal <= 2.0 + 1e-9, "hover: gimbal commands within the range");
            Check (r.MinThrottle >= -1e-12 && r.MaxThrottle <= 1 + 1e-12, "hover: throttles within [0, 1]");

            // Without the feed-forward the law cancels the common torque by itself, late:
            // the attitude moves more. The feed-forward is what makes it a pure translation.
            var noFf = Hover (false, true, 1.0, 1.0, 1.0, 0.0, 150);
            Console.WriteLine ("  hover 0.2 m/s^2, ideal plant: accel " + r.SettleAccelZ.ToString ("F4") +
                ", max tilt " + r.MaxTiltDeg.ToString ("F4") + " deg (without feed-forward " +
                noFf.MaxTiltDeg.ToString ("F4") + " deg, accel " + noFf.SettleAccelZ.ToString ("F4") +
                "), max gimbal " + r.MaxAbsGimbal.ToString ("F3") + " deg, throttles " +
                r.MinThrottle.ToString ("F4") + "-" + r.MaxThrottle.ToString ("F4"));
            Check (r.MaxTiltDeg < noFf.MaxTiltDeg,
                "hover: the feed-forward reduces the attitude excursion (" + r.MaxTiltDeg.ToString ("F4") +
                " < " + noFf.MaxTiltDeg.ToString ("F4") + " deg)");

            // Degraded plant: gimbal torque 15 % weaker and differential torque 10 % stronger
            // than the columns, throttle lag 0.2
            // per step (engineAccelerationSpeed 10), a 5 kN.m disturbance (CoM offset).
            var d = Hover (true, true, 0.2, 0.85, 1.1, 5000.0, 250);
            Console.WriteLine ("  hover 0.2 m/s^2, degraded plant: accel " + d.SettleAccelZ.ToString ("F4") +
                ", max tilt " + d.MaxTiltDeg.ToString ("F4") + " deg, final tilt " +
                d.FinalTiltDeg.ToString ("F4") + " deg, throttles " + d.MinThrottle.ToString ("F4") +
                "-" + d.MaxThrottle.ToString ("F4"));
            Check (d.AllComputed, "degraded: the law computes every step");
            Check (Math.Abs (d.SettleAccelZ - 0.2) < 0.05,
                "degraded: lateral acceleration within 25 % (" + d.SettleAccelZ.ToString ("F4") + ")");
            Check (d.MaxTiltDeg < 0.6, "degraded: attitude within 0.6 deg (" + d.MaxTiltDeg.ToString ("F4") + ")");
            Check (d.FinalTiltDeg < 0.3, "degraded: attitude back under 0.3 deg (" + d.FinalTiltDeg.ToString ("F4") + ")");
            Check (d.MinThrottle >= -1e-12 && d.MaxThrottle <= 1 + 1e-12, "degraded: throttles within [0, 1]");

            // Stale request after 2 s: wind-down (common back to 0 at the gimbal ramp, the
            // feed-forward following it), then the throttles are exactly the main throttle.
            var w = Hover (true, true, 1.0, 1.0, 1.0, 0.0, 200, 100);
            Console.WriteLine ("  wind-down after a stale request: " + w.WindDownSteps + " steps" +
                (w.WindDownTimeout ? " (timeout)" : "") + ", max tilt " + w.MaxTiltDeg.ToString ("F4") +
                " deg, final lateral accel " + w.SettleAccelZ.ToString ("F4"));
            Check (w.WindDownSteps > 0 && w.WindDownSteps <= 50 && !w.WindDownTimeout,
                "stale: wound down within 1 s by the deflection rule (" + w.WindDownSteps + " steps)");
            Check (w.FinalThrottleSpread == 0.0 && w.FinalCommon == 0.0,
                "stale: throttles back to the main throttle, no common offset left");
            Check (w.MaxTiltDeg < 0.05, "stale: attitude held through the wind-down");
            Check (Math.Abs (w.SettleAccelZ) < 0.01, "stale: no lateral acceleration left");

            // Motifs without translation (mode 1): the attitude holds with the disturbance.
            var m1 = Hover (true, false, 0.2, 0.85, 1.1, 5000.0, 250);
            Check (m1.FinalTiltDeg < 0.3 && Math.Abs (m1.SettleAccelZ) < 0.05,
                "mode 1: attitude held, no lateral acceleration (" + m1.FinalTiltDeg.ToString ("F4") + " deg)");
        }

        // ================================================= computed levers (bit 4, modes 5/7)

        static void TestComputedModes ()
        {
            Check (AttitudeTranslation.ValidMode (5) && AttitudeTranslation.ValidMode (7),
                "bit 4: modes 5 (1 | 4) and 7 (1 | 2 | 4) valid");
            Check (!AttitudeTranslation.ValidMode (2) && !AttitudeTranslation.ValidMode (4) &&
                !AttitudeTranslation.ValidMode (6) && !AttitudeTranslation.ValidMode (8) &&
                !AttitudeTranslation.ValidMode (15), "bit 4: 2, 4, 6 (no differential), 8, 15 refused");
            Check (AttitudeTranslation.Computed (5) && AttitudeTranslation.Computed (7) &&
                !AttitudeTranslation.Computed (0) && !AttitudeTranslation.Computed (1) &&
                !AttitudeTranslation.Computed (3), "computed levers on 5 and 7 only");
            Check (AttitudeTranslation.ReturnVersion == 1 && AttitudeTranslation.ReturnVersionComputed == 2,
                "return layout 1 for modes 1/3, 2 with bit 4");
            var p = new TranslationParams ();
            Check (p.FracRcs == 0.5 && p.MinLeverArmM == 0.25, "FracRcs 0.5; the 0.25 m of modes 1/3 kept");
            p.FracRcs = 0.1;
            p.Reset ();
            Check (p.FracRcs == 0.5, "FracRcs reset with the tunables");
            long[] bits = {
                AttitudeTranslation.ReasonNoRequest, AttitudeTranslation.ReasonNotArmedPitchYaw,
                AttitudeTranslation.ReasonPitchYawNotPermitted, AttitudeTranslation.ReasonNotWriting,
                AttitudeTranslation.ReasonStale, AttitudeTranslation.ReasonStopRequested,
                AttitudeTranslation.ReasonNoDiffAuthority, AttitudeTranslation.ReasonNoThrottleMargin,
                AttitudeTranslation.ReasonNoTranslationGeometry, AttitudeTranslation.ReasonTiltTooLarge,
                AttitudeTranslation.ReasonLimitedByGimbal, AttitudeTranslation.ReasonLimitedByDiff,
                AttitudeTranslation.ReasonFeedForwardClamped, AttitudeTranslation.ReasonThrottleWriteMissed,
                AttitudeTranslation.ReasonWindDownTimeout, AttitudeTranslation.ReasonNotActiveVessel,
                AttitudeTranslation.ReasonNoLateralAuthority, AttitudeTranslation.ReasonLimitedByRcs,
                AttitudeTranslation.ReasonRcsModelApprox };
            bool distinct = true;
            for (int i = 0; i < bits.Length; i++)
                distinct &= bits [i] == 1L << i;
            Check (distinct, "reason bits 0-18 distinct, the old ones unmoved");
        }

        static double[] Column (double[,] table, int c)
        {
            var v = new double[table.GetLength (0)];
            for (int i = 0; i < v.Length; i++)
                v [i] = table [i, c];
            return v;
        }

        static void TestComputedLeverCriterion ()
        {
            const int n = 8;
            var slope = new double[n];
            var x = Column (PivotTests.HeavyCentroids, 0);
            var z = Column (PivotTests.HeavyCentroids, 1);
            var y = new double[n];
            var mp = new double[n];
            var my = new double[n];
            for (int i = 0; i < n; i++) {
                slope [i] = 2.5e6 / 0.08 / n;
                y [i] = -10.83;
            }
            double crit = AttitudeTranslation.LeverResolution (n, x, y, z);
            Check (crit > 1.08e-4 && crit < 1.09e-4, "heavy: criterion 1e-5 x |r| = 1.08e-4 m (" + crit.ToString ("E3") + ")");
            double maxP, maxY;
            bool old = AttitudeTranslation.Motifs (n, slope, x, z, 0.25, mp, my, out maxP, out maxY);
            Console.WriteLine ("  computed levers: heavy booster motif levers " + maxP.ToString ("F4") + " / " +
                maxY.ToString ("F4") + " m, criterion " + crit.ToString ("E3") + " m (modes 1/3: 0.25 m)");
            Check (!old && maxP > 0.12 && maxP < 0.14 && maxY > 0.12 && maxY < 0.14,
                "heavy: refused by the 0.25 m of modes 1/3, its levers 0.12-0.14 m read");
            bool acc = AttitudeTranslation.Motifs (n, slope, x, z, crit, mp, my, out maxP, out maxY);
            double sp = 0.0, sy = 0.0, mmax = 0.0;
            for (int i = 0; i < n; i++) {
                sp += slope [i] * mp [i];
                sy += slope [i] * my [i];
                mmax = Math.Max (mmax, Math.Max (Math.Abs (mp [i]), Math.Abs (my [i])));
            }
            Check (acc && Math.Abs (sp) < 1e-6 && Math.Abs (sy) < 1e-6 && Near (mmax, 1.0, 1e-12),
                "heavy: accepted with the computed criterion, thrust conserved, |motif| <= 1");
            // Degenerate: every centroid on the axis, exactly or within float rounding.
            var zero = new double[n];
            Check (!AttitudeTranslation.Motifs (n, slope, zero, zero, crit, mp, my, out maxP, out maxY),
                "coaxial engines (lever 0): refused, no division by zero");
            bool allZero = true;
            for (int i = 0; i < n; i++)
                allZero &= mp [i] == 0.0 && my [i] == 0.0;
            Check (allZero, "coaxial: motifs zero");
            var nx = new double[n];
            var nz = new double[n];
            for (int i = 0; i < n; i++) {
                nx [i] = 10.83 * 1.2e-7 * ((i & 1) == 0 ? 1 : -1);
                nz [i] = 10.83 * 1.2e-7 * ((i & 2) == 0 ? 1 : -1);
            }
            Check (!AttitudeTranslation.Motifs (n, slope, nx, nz, AttitudeTranslation.LeverResolution (n, nx, y, nz),
                mp, my, out maxP, out maxY), "levers at float rounding (1.2e-7 |r|): refused");
            // The overload with the levers is the old function: same verdict, same motifs.
            var rng = new Random (44);
            bool same = true;
            var mp2 = new double[n];
            var my2 = new double[n];
            for (int k = 0; k < 200; k++) {
                int m = 1 + rng.Next (8);
                for (int i = 0; i < m; i++) {
                    x [i] = (rng.NextDouble () - 0.5) * (k % 3 == 0 ? 0.3 : 3.0);
                    z [i] = (rng.NextDouble () - 0.5) * (k % 3 == 0 ? 0.3 : 3.0);
                    slope [i] = rng.NextDouble () * 1e6;
                }
                bool a = AttitudeTranslation.Motifs (m, slope, x, z, 0.25, mp, my);
                bool b = AttitudeTranslation.Motifs (m, slope, x, z, 0.25, mp2, my2, out maxP, out maxY);
                same &= a == b;
                for (int i = 0; i < m; i++)
                    same &= mp [i] == mp2 [i] && my [i] == my2 [i];
            }
            Check (same, "Motifs with levers = Motifs (200 draws, bit for bit)");
        }

        static void TestRcsTranslationColumns ()
        {
            var fp = new double[3];
            var fn = new double[3];
            var tp = new double[3];
            var tn = new double[3];
            // Two nozzles v = +x at (0, 10, +-1), 1000 N.
            double[] lever = { 0, 10, 1, 0, 10, -1 };
            double[] dir = { 1, 0, 0, 1, 0, 0 };
            double[] power = { 1000, 1000 };
            AttitudeTranslation.RcsTranslationColumns (2, lever, dir, power, 1, 0, 0, fp, fn, tp, tn);
            Check (fp [0] == -2000.0 && fp [1] == 0.0 && fp [2] == 0.0,
                "ctrlState.X +1: nozzles with v.x > 0 fire, force along -v (-2000 N x)");
            Check (tp [0] == 0.0 && tp [1] == 0.0 && tp [2] == 20000.0, "torque r x F about the CoM (0, 0, 20 kN.m)");
            Check (fn [0] == 0.0 && tn [2] == 0.0, "X -1: no nozzle with v.x < 0, nothing (asymmetric module)");
            // The opposite pair: X -1 gives +2000 x, i.e. -2000 per unit of the negative command.
            for (int k = 0; k < 3; k++)
                fp [k] = fn [k] = tp [k] = tn [k] = 0.0;
            double[] lever4 = { 0, 10, 1, 0, 10, -1, 0, 10, 1, 0, 10, -1 };
            double[] dir4 = { 1, 0, 0, 1, 0, 0, -1, 0, 0, -1, 0, 0 };
            double[] power4 = { 1000, 1000, 1000, 1000 };
            AttitudeTranslation.RcsTranslationColumns (4, lever4, dir4, power4, 1, 0, 0, fp, fn, tp, tn);
            Check (fp [0] == -2000.0 && fn [0] == -2000.0 && tp [2] == 20000.0 && tn [2] == 20000.0,
                "symmetric module: + and - columns equal, force per unit of X = -2000 N x either way");
            // Oblique nozzle: fraction v . axis, force -v p f.
            for (int k = 0; k < 3; k++)
                fp [k] = fn [k] = tp [k] = tn [k] = 0.0;
            AttitudeTranslation.RcsTranslationColumns (1, new double[] { 0, 5, 0 }, new double[] { 0.6, 0.8, 0 },
                new double[] { 500 }, 1, 0, 0, fp, fn, tp, tn);
            Check (Near (fp [0], -180.0, 1e-12) && Near (fp [1], -240.0, 1e-12) && Near (tp [2], 900.0, 1e-12),
                "oblique nozzle: fraction 0.6, force -(0.6, 0.8) x 500 x 0.6, torque 900 N.m");
            // ctrlState.Y acts along body z (ReferenceTransform.forward).
            for (int k = 0; k < 3; k++)
                fp [k] = fn [k] = tp [k] = tn [k] = 0.0;
            AttitudeTranslation.RcsTranslationColumns (1, new double[] { 2, 0, 0 }, new double[] { 0, 0, 1 },
                new double[] { 100 }, 0, 0, 1, fp, fn, tp, tn);
            Check (fp [2] == -100.0 && tp [1] == 200.0 && fn [2] == 0.0, "ctrlState.Y +1 (body z): -100 N z, 200 N.m roll");
            // No power, NaN power: skipped.
            for (int k = 0; k < 3; k++)
                fp [k] = fn [k] = tp [k] = tn [k] = 0.0;
            AttitudeTranslation.RcsTranslationColumns (2, lever, dir, new double[] { 0.0, double.NaN }, 1, 0, 0, fp, fn, tp, tn);
            Check (fp [0] == 0.0 && tp [2] == 0.0, "no power or NaN power: no column");

            // The heavy booster's RCS: 8 modules at +36.9 / +37.6 m, radius 2.53 m, azimuths +-x,
            // +-z, four nozzles each (axial +-y, tangential). The same geometry through the
            // ROTATION rule of AttitudeGeometry.AddRcsColumns gives the pitch column published in
            // flight (1.464 MN.m per unit), which fixes the nozzle power.
            var h = new HeavyModel (0.08);
            double pitch = h.RcsRotationColumn (0);
            Console.WriteLine ("  computed levers: heavy RCS pitch column " + (pitch / 1e6).ToString ("F3") +
                " MN.m per unit at " + HeavyModel.RcsPower.ToString ("F0") + " N per nozzle (flight: 1.464)");
            Check (Math.Abs (-pitch / 1.464e6 - 1.0) < 0.01, "heavy RCS model: pitch column 1.464 MN.m within 1 %");
            var j = new JointTranslation ();
            h.Fill (j, 0.8, 0.8, 0.5, true, true);
            double fX = j.Rf [0], tZ = j.Rt [4];
            Console.WriteLine ("  computed levers: heavy RCS translation X column " + (fX / 1e3).ToString ("F2") +
                " kN, " + (tZ / 1e6).ToString ("F3") + " MN.m (yaw); tilt: " + (j.K [1] / 1e3).ToString ("F2") +
                " kN/deg, " + (j.Kt [5] / 1e6).ToString ("F3") + " MN.m/deg");
            Check (fX < -38e3 && fX > -41e3, "heavy: ctrlState.X +1 gives ~-39 kN along x (four tangential nozzles)");
            // Same force direction from the RCS (top) and the tilt (bottom): opposite torques.
            double tiltForceX = j.K [1], tiltTorqueZ = j.Kt [5];
            Check (Math.Sign (fX) == Math.Sign (tiltForceX) && Math.Sign (tZ) == -Math.Sign (tiltTorqueZ),
                "heavy: RCS force and tilt force in the same direction have OPPOSITE torques");
            Check (Near (tZ / fX, -(36.9 + 37.6) / 2.0, 1e-3), "heavy: RCS lever ~37.25 m above the CoM");
        }

        static void TestRealizedShare ()
        {
            var c = new double[] { 1.0, -0.5, 0.0, 2.0 };
            var r = new double[] { 0.16, -0.08, 0.0, 0.32 };
            Check (Near (AttitudeTranslation.RealizedShare (c, r, 4), 0.16, 1e-12), "realized share: ramp 0.16");
            Check (AttitudeTranslation.RealizedShare (c, c, 4) == 1.0, "realized = commanded: 1");
            var over = new double[] { 2.0, -1.0, 0.0, 4.0 };
            Check (AttitudeTranslation.RealizedShare (c, over, 4) == 1.0, "bounded at 1 (the command came down)");
            var back = new double[] { -0.1, 0.05, 0.0, -0.2 };
            Check (Near (AttitudeTranslation.RealizedShare (c, back, 4), -0.1, 1e-12),
                "opposite realized (a reversal): negative, the RCS keeps balancing the old tilt");
            var farBack = new double[] { -2.0, 1.0, 0.0, -4.0 };
            Check (AttitudeTranslation.RealizedShare (c, farBack, 4) == -1.0, "bounded at -1");
            Check (AttitudeTranslation.RealizedShare (new double[4], r, 4) == 1.0,
                "nothing commanded (RCS alone): 1, no deflection to wait for");
        }

        // ------------------------------------------------------------ heavy booster model

        /// <summary>
        /// The heavy booster in PF (flight, 28/09): 8 AR-1 Cluster parts whose nozzle
        /// centroids are PivotTests.HeavyCentroids, 10.83 m below the CoM, thrust along the nose
        /// (2.5 MN at throttle 0.08, so 3.9 MN per part at full), 260 t, gimbal range 2 deg
        /// (V1 axes on body x and z); 8 RCS modules as TestRcsTranslationColumns describes.
        /// </summary>
        sealed class HeavyModel
        {
            internal const int Engines = 8;
            internal const double L = 10.83, Mass = 260e3, RangeDeg = 2.0;
            internal const double RcsPower = 9825.0;
            internal readonly double Throttle, SlopeEach;
            internal readonly double[] X = new double[Engines], Y = new double[Engines], Z = new double[Engines];
            internal readonly double[] Tpn = new double[3 * Engines];
            internal readonly double[] RcsLever, RcsDir, RcsPowers;
            internal readonly int RcsCount;
            internal readonly double[] MotifP = new double[Engines], MotifY = new double[Engines];
            internal readonly double[] ColP = new double[3], ColY = new double[3];
            internal double Half;
            // Nozzles below the CoM (m) and mass (kg): L and Mass, or the stack in the clamps.
            internal readonly double Lever, MassKg;

            internal HeavyModel (double throttle) : this (throttle, L, Mass, 0.0, 2.5e6 / 0.08)
            {
            }

            /// The same booster with another CoM: nozzles `lever` below it, RCS rings shifted by
            /// `rcsShift` (m, along the nose), `fullThrustN` at full throttle (the stack in the clamps).
            internal HeavyModel (double throttle, double lever, double massKg, double rcsShift, double fullThrustN)
            {
                Throttle = throttle;
                Lever = lever;
                MassKg = massKg;
                SlopeEach = fullThrustN / Engines;
                for (int i = 0; i < Engines; i++) {
                    X [i] = PivotTests.HeavyCentroids [i, 0];
                    Z [i] = PivotTests.HeavyCentroids [i, 1];
                    Y [i] = -lever;
                    // r x dir, dir = +y (thrust along the nose): (-z, 0, x)
                    Tpn [3 * i] = -Z [i];
                    Tpn [3 * i + 1] = 0.0;
                    Tpn [3 * i + 2] = X [i];
                }
                double[] rings = { 36.9 + rcsShift, 37.6 + rcsShift };
                RcsCount = 2 * 4 * 4;
                RcsLever = new double[3 * RcsCount];
                RcsDir = new double[3 * RcsCount];
                RcsPowers = new double[RcsCount];
                int k = 0;
                foreach (double yr in rings)
                    for (int a = 0; a < 4; a++) {
                        double px = a == 0 ? 2.53 : (a == 1 ? -2.53 : 0.0);
                        double pz = a == 2 ? 2.53 : (a == 3 ? -2.53 : 0.0);
                        // Axial +-y, tangential: +-z on the x azimuths, +-x on the z azimuths.
                        double[][] vs = a < 2
                            ? new[] { new double[] { 0, 1, 0 }, new double[] { 0, -1, 0 }, new double[] { 0, 0, 1 }, new double[] { 0, 0, -1 } }
                            : new[] { new double[] { 0, 1, 0 }, new double[] { 0, -1, 0 }, new double[] { 1, 0, 0 }, new double[] { -1, 0, 0 } };
                        foreach (var v in vs) {
                            RcsLever [3 * k] = px;
                            RcsLever [3 * k + 1] = yr;
                            RcsLever [3 * k + 2] = pz;
                            RcsDir [3 * k] = v [0];
                            RcsDir [3 * k + 1] = v [1];
                            RcsDir [3 * k + 2] = v [2];
                            RcsPowers [k] = RcsPower;
                            k++;
                        }
                    }
                var s = new double[Engines];
                for (int i = 0; i < Engines; i++)
                    s [i] = SlopeEach;
                AttitudeTranslation.Motifs (Engines, s, X, Z, AttitudeTranslation.LeverResolution (Engines, X, Y, Z),
                    MotifP, MotifY);
                AttitudeTranslation.MotifColumns (Engines, s, MotifP, MotifY, Tpn, ColP, ColY);
                var margin = new double[Engines];
                for (int i = 0; i < Engines; i++)
                    margin [i] = AttitudeTranslation.ThrottleMargin (throttle, 0.0, 1.0);
                Half = AttitudeTranslation.SymmetricHalfWidth (Engines, MotifP, MotifY, margin);
            }

            internal double Thrust { get { return Engines * SlopeEach * Throttle; } }

            /// The ctrlState ROTATION column of axis a (0 pitch, 1 roll, 2 yaw), N.m per unit,
            /// by the rule of AttitudeGeometry.AddRcsColumns (command +1).
            internal double RcsRotationColumn (int a)
            {
                double[] axis = { a == 0 ? 1 : 0, a == 1 ? 1 : 0, a == 2 ? 1 : 0 };
                double torque = 0.0;
                for (int i = 0; i < RcsCount; i++) {
                    double[] r = { RcsLever [3 * i], RcsLever [3 * i + 1], RcsLever [3 * i + 2] };
                    double rn = r [0] * axis [0] + r [1] * axis [1] + r [2] * axis [2];
                    double[] perp = { r [0] - rn * axis [0], r [1] - rn * axis [1], r [2] - rn * axis [2] };
                    double pn = Math.Sqrt (perp [0] * perp [0] + perp [1] * perp [1] + perp [2] * perp [2]);
                    var rot = new double[3];
                    Cross (axis, new[] { perp [0] / pn, perp [1] / pn, perp [2] / pn }, rot);
                    double f = RcsDir [3 * i] * rot [0] + RcsDir [3 * i + 1] * rot [1] + RcsDir [3 * i + 2] * rot [2];
                    if (!(f > 0.0))
                        continue;
                    if (f > 1.0)
                        f = 1.0;
                    double[] force = { -RcsDir [3 * i] * RcsPowers [i] * f, -RcsDir [3 * i + 1] * RcsPowers [i] * f,
                        -RcsDir [3 * i + 2] * RcsPowers [i] * f };
                    var t = new double[3];
                    Cross (r, force, t);
                    torque += t [a];
                }
                return torque;
            }

            /// Honest translation (bit 16): the rotation fraction of every RCS nozzle for the body
            /// command (pitch, roll, yaw), the ModuleRCS rule.
            internal void RotationFractions (double pitch, double roll, double yaw, double[] rho)
            {
                AttitudeTranslation.RcsRotationFractions (RcsCount, RcsLever, RcsDir, pitch, roll, yaw, rho);
            }

            /// Bit 16: ModuleRCS's pitch (0) or yaw (2) answer nonlinear at the served rotation?
            /// (the heavy RCS as one module, every axis enabled)
            internal bool RotationNonlinear (double pitch, double roll, double yaw, int axis)
            {
                return AttitudeTranslation.RcsRotationNonlinear (1, new[] { RcsCount }, new[] { RcsLever },
                    new[] { RcsDir }, new[] { RcsPowers }, new double[] { 1, 1, 1 }, pitch, roll, yaw, axis,
                    new double[RcsCount], new double[3]);
            }

            /// The PLANT: the force and torque (body, about the CoM) ModuleRCS.FixedUpdate gives for
            /// the rotation (pitch, roll, yaw) and the translation (X along body x, Y along body z)
            /// together: each nozzle at min(1, max(v . rot, 0) + max(v . lin, 0)).
            internal void RcsPlant (double pitch, double roll, double yaw, double cx, double cy,
                double[] force, double[] torque)
            {
                var rho = new double[RcsCount];
                RotationFractions (pitch, roll, yaw, rho);
                for (int k = 0; k < 3; k++)
                    force [k] = torque [k] = 0.0;
                for (int i = 0; i < RcsCount; i++) {
                    double vx = RcsDir [3 * i], vy = RcsDir [3 * i + 1], vz = RcsDir [3 * i + 2];
                    double lin = vx * cx + vz * cy;
                    double f = rho [i] + (lin > 0.0 ? lin : 0.0);
                    if (f > 1.0)
                        f = 1.0;
                    if (!(f > 0.0))
                        continue;
                    double[] F = { -vx * RcsPowers [i] * f, -vy * RcsPowers [i] * f, -vz * RcsPowers [i] * f };
                    double[] r = { RcsLever [3 * i], RcsLever [3 * i + 1], RcsLever [3 * i + 2] };
                    var t = new double[3];
                    Cross (r, F, t);
                    for (int k = 0; k < 3; k++) {
                        force [k] += F [k];
                        torque [k] += t [k];
                    }
                }
            }

            /// Fills a joint allocation as AttitudeV1Translation.PrepareJoint does, then prepares it.
            /// rho (bit 16): the held rotation fractions, the RCS columns with the headroom and the
            /// weaker sign (PrepareRcsHeadroom); null: the bit-4 columns, unchanged.
            internal bool Fill (JointTranslation j, double fracGimbal, double fracDiff, double fracRcs,
                bool rcs, bool diff, double[] rho = null)
            {
                j.Clear ();
                j.Resize (Engines);
                double d = Deg * Thrust;
                j.Mass = MassKg;
                j.K [0] = 0.0;
                j.K [1] = -d;
                j.K [2] = d;
                j.K [3] = 0.0;
                j.Kt [0] = -Lever * d;
                j.Kt [5] = -Lever * d;
                var fp = new double[6];
                var fn = new double[6];
                var tp = new double[6];
                var tn = new double[6];
                for (int a = 0; a < 2; a++) {
                    var f1 = new double[3];
                    var f2 = new double[3];
                    var t1 = new double[3];
                    var t2 = new double[3];
                    if (rho == null)
                        AttitudeTranslation.RcsTranslationColumns (RcsCount, RcsLever, RcsDir, RcsPowers,
                            a == 0 ? 1 : 0, 0, a == 0 ? 0 : 1, f1, f2, t1, t2);
                    else
                        AttitudeTranslation.RcsTranslationColumnsHeadroom (RcsCount, RcsLever, RcsDir, RcsPowers,
                            rho, fracRcs, a == 0 ? 1 : 0, 0, a == 0 ? 0 : 1, f1, f2, t1, t2);
                    for (int k = 0; k < 3; k++) {
                        fp [3 * a + k] = f1 [k];
                        fn [3 * a + k] = f2 [k];
                        tp [3 * a + k] = t1 [k];
                        tn [3 * a + k] = t2 [k];
                    }
                }
                if (rho != null) {
                    double sx, sy;
                    AttitudeTranslation.WeakerSign (fp, fn, tp, tn, out sx, out sy);
                }
                j.Rf [0] = 0.5 * (fp [0] + fn [0]);
                j.Rf [1] = 0.5 * (fp [3] + fn [3]);
                j.Rf [2] = 0.5 * (fp [2] + fn [2]);
                j.Rf [3] = 0.5 * (fp [5] + fn [5]);
                for (int r = 0; r < 3; r++) {
                    j.Rt [2 * r] = 0.5 * (tp [r] + tn [r]);
                    j.Rt [2 * r + 1] = 0.5 * (tp [3 + r] + tn [3 + r]);
                    j.Bd [2 * r] = ColP [r];
                    j.Bd [2 * r + 1] = ColY [r];
                }
                j.PhiMax = fracGimbal * RangeDeg;
                j.RcsMax = rcs ? fracRcs : 0.0;
                j.DiffMax = diff ? fracDiff * Half : 0.0;
                j.DiffWeight = 1.0 / new TranslationParams ().DiffPriority;
                j.FracGimbal = fracGimbal;
                for (int g = 0; g < Engines; g++) {
                    j.GimbalUsable [g] = true;
                    j.GimbalAxes [4 * g] = 1.0;
                    j.GimbalAxes [4 * g + 1] = 0.0;
                    j.GimbalAxes [4 * g + 2] = 0.0;
                    j.GimbalAxes [4 * g + 3] = 1.0;
                    for (int q = 0; q < 4; q++)
                        j.GimbalRanges [4 * g + q] = RangeDeg;
                }
                return j.Prepare ();
            }

            /// Differential torque authority (inradius) and the formula of modes 1/3.
            internal double FormulaAuthority (double fracGimbal, double fracDiff, out double inradius)
            {
                inradius = AttitudeTranslation.Inradius (ColP, ColY, Half);
                double full = RangeDeg * AttitudeTranslation.MaxSingular2 (-L * Deg * Thrust, 0, 0, -L * Deg * Thrust);
                return AttitudeTranslation.LateralAuthority (Thrust / Mass, RangeDeg, fracGimbal, fracDiff,
                    inradius, full);
            }
        }

        static void TestJointAllocation ()
        {
            var h = new HeavyModel (0.08);
            var j = new JointTranslation ();
            Check (h.Fill (j, 0.8, 0.8, 0.5, true, true), "heavy: joint allocation prepared");
            Check (Near (j.Lever, HeavyModel.L, 1e-12), "torque rows weighed at the gimbals' lever (10.83 m)");
            var rng = new Random (77);
            double worstTorque = 0.0, worstMiss = 0.0;
            bool bounds = true, converged = true;
            int iterMax = 0;
            for (int k = 0; k < 300; k++) {
                double ang = 2 * Math.PI * rng.NextDouble ();
                double a = k < 150 ? 0.25 * rng.NextDouble () : 0.5 + 3.0 * rng.NextDouble ();
                double fx = a * HeavyModel.Mass * Math.Cos (ang), fz = a * HeavyModel.Mass * Math.Sin (ang);
                j.Solve (fx, fz);
                converged &= j.Converged;
                iterMax = Math.Max (iterMax, j.Iterations);
                double f = Math.Sqrt (j.Force [0] * j.Force [0] + j.Force [1] * j.Force [1]);
                double tq = Math.Sqrt (j.Torque [0] * j.Torque [0] + j.Torque [1] * j.Torque [1] + j.Torque [2] * j.Torque [2]);
                if (f > 0.0)
                    worstTorque = Math.Max (worstTorque, tq / (HeavyModel.L * f));
                if (k < 150) {
                    double miss = Math.Sqrt (Math.Pow (j.Force [0] - fx, 2) + Math.Pow (j.Force [1] - fz, 2));
                    worstMiss = Math.Max (worstMiss, miss / Math.Sqrt (fx * fx + fz * fz));
                }
                bounds &= Math.Abs (j.X [0]) <= j.PhiMax * (1 + 1e-12) && Math.Abs (j.X [1]) <= j.PhiMax * (1 + 1e-12) &&
                    Math.Abs (j.X [2]) <= j.RcsMax * (1 + 1e-12) && Math.Abs (j.X [3]) <= j.RcsMax * (1 + 1e-12) &&
                    Math.Abs (j.X [4]) <= j.DiffMax * (1 + 1e-12) && Math.Abs (j.X [5]) <= j.DiffMax * (1 + 1e-12);
            }
            Console.WriteLine ("  computed levers: joint allocation, 300 heavy targets: residual torque <= " +
                worstTorque.ToString ("E2") + " x L|F|, reachable force missed by <= " + worstMiss.ToString ("E2") +
                ", <= " + iterMax + " iterations");
            Check (converged, "joint allocation converges (300 targets)");
            Check (worstTorque < 1e-5, "residual torque ~0: under 1e-5 of the force at the gimbals' lever");
            Check (worstMiss < 1e-4, "a reachable force (0-0.25 m/s^2) is met within 1e-4");
            Check (bounds, "every variable within its bound");
            // A reachable target: the RCS pushes the SAME way as the tilt (their torques cancel).
            j.Solve (0.2 * HeavyModel.Mass, 0.0);
            double tiltFx = j.K [0] * j.X [0] + j.K [1] * j.X [1];
            double rcsFx = j.Rf [0] * j.X [2] + j.Rf [1] * j.X [3];
            Console.WriteLine ("  computed levers: heavy 0.2 m/s^2 along +x: tilt " + (tiltFx / 1e3).ToString ("F1") +
                " kN (" + j.X [1].ToString ("F3") + " deg), RCS " + (rcsFx / 1e3).ToString ("F1") + " kN (X " +
                j.X [2].ToString ("F3") + "), motifs " + j.X [4].ToString ("E2") + " / " + j.X [5].ToString ("E2"));
            Check (tiltFx > 0.0 && rcsFx > 0.0, "0.2 m/s^2: tilt and RCS both push along +x");
            Check (Math.Abs (j.X [4]) <= 0.5 * j.DiffMax && Math.Abs (j.X [5]) <= 0.5 * j.DiffMax,
                "0.2 m/s^2: the differential spared (weight 1 / DiffPriority): under half its bound, the RCS balances first");
            // The worst gimbal fraction scales the whole solution: module axes turned 45 deg,
            // a diagonal target at the box corner.
            for (int g = 0; g < HeavyModel.Engines; g++) {
                double c = Math.Cos (Math.PI / 4), s = Math.Sin (Math.PI / 4);
                j.GimbalAxes [4 * g] = c;
                j.GimbalAxes [4 * g + 1] = -s;
                j.GimbalAxes [4 * g + 2] = s;
                j.GimbalAxes [4 * g + 3] = c;
            }
            j.Solve (5.0 * HeavyModel.Mass, 5.0 * HeavyModel.Mass);
            double worstFrac = 0.0;
            for (int g = 0; g < HeavyModel.Engines; g++) {
                double cx = j.X [0] * j.GimbalAxes [4 * g] + j.X [1] * j.GimbalAxes [4 * g + 1];
                double cy = j.X [0] * j.GimbalAxes [4 * g + 2] + j.X [1] * j.GimbalAxes [4 * g + 3];
                worstFrac = Math.Max (worstFrac, AttitudeTranslation.GimbalFraction (cx, cy, 2, 2, 2, 2));
            }
            double tqs = Math.Sqrt (j.Torque [0] * j.Torque [0] + j.Torque [1] * j.Torque [1] + j.Torque [2] * j.Torque [2]);
            double fs = Math.Sqrt (j.Force [0] * j.Force [0] + j.Force [1] * j.Force [1]);
            Check (j.Scale < 1.0 && worstFrac <= 0.8 + 1e-12 && tqs < 1e-5 * HeavyModel.L * fs,
                "gimbals turned 45 deg, diagonal: whole solution scaled to 0.8 of the range, torque still ~0 (scale " +
                j.Scale.ToString ("F3") + ")");
            // Nothing usable: not prepared, zero authority.
            var empty = new JointTranslation ();
            empty.Mass = 1000.0;
            Check (!empty.Prepare () && empty.Authority () == 0.0 && !empty.Solve (1.0, 0.0),
                "no column: not prepared, no authority, no solve");
        }

        /// Engine i of a ring/square below the CoM, gimbal pivot at qy: torque and force columns
        /// per degree about body x and z (the Barge.Columns derivation, any position).
        static void EngineColumns (double px, double py, double pz, double qy, double thrust,
            double[] colX, double[] colY, double[] forceX, double[] forceY)
        {
            double[] p = { px, py, pz };
            double[] off = { 0.0, py - qy, 0.0 };
            double[] d = { 0.0, 1.0, 0.0 };
            double[] f = { 0.0, thrust, 0.0 };
            var axes = new[] { new double[] { 1, 0, 0 }, new double[] { 0, 0, 1 } };
            for (int k = 0; k < 2; k++) {
                var dF = new double[3];
                Cross (axes [k], d, dF);
                for (int r = 0; r < 3; r++)
                    dF [r] *= thrust;
                var dLever = new double[3];
                Cross (axes [k], off, dLever);
                var t1 = new double[3];
                var t2 = new double[3];
                Cross (dLever, f, t1);
                Cross (p, dF, t2);
                var col = k == 0 ? colX : colY;
                var force = k == 0 ? forceX : forceY;
                for (int r = 0; r < 3; r++) {
                    col [r] = (t1 [r] + t2 [r]) * Deg;
                    force [r] = dF [r] * Deg;
                }
            }
        }

        /// A booster without RCS translation (barge, medium): today's announced authority and
        /// the joint allocation's, both at the hover throttle.
        static void Compare (string name, double[] x, double[] z, double py, double qy, double tmax,
            double mass, out double today, out double joint)
        {
            int n = x.Length;
            double main = mass * 9.81 / (n * tmax);
            double thrust = mass * 9.81;
            var s = new double[n];
            var tpn = new double[3 * n];
            var mp = new double[n];
            var my = new double[n];
            var margin = new double[n];
            var j = new JointTranslation ();
            j.Resize (n);
            var cx = new double[3];
            var cy = new double[3];
            var fx = new double[3];
            var fy = new double[3];
            double tEach = thrust / n;
            for (int i = 0; i < n; i++) {
                s [i] = tmax;
                var t3 = new double[3];
                Cross (new[] { x [i], py, z [i] }, new double[] { 0, 1, 0 }, t3);
                for (int r = 0; r < 3; r++)
                    tpn [3 * i + r] = t3 [r];
                margin [i] = AttitudeTranslation.ThrottleMargin (main, 0, 1);
                EngineColumns (x [i], py, z [i], qy, tEach, cx, cy, fx, fy);
                // V1 axes: gimbal x = body x, y = body z (as the Barge model).
                j.K [0] += fx [0];
                j.K [1] += fy [0];
                j.K [2] += fx [2];
                j.K [3] += fy [2];
                for (int r = 0; r < 3; r++) {
                    j.Kt [2 * r] += cx [r];
                    j.Kt [2 * r + 1] += cy [r];
                }
                j.GimbalUsable [i] = true;
                j.GimbalAxes [4 * i] = 1.0;
                j.GimbalAxes [4 * i + 3] = 1.0;
                for (int q = 0; q < 4; q++)
                    j.GimbalRanges [4 * i + q] = 2.0;
            }
            AttitudeTranslation.Motifs (n, s, x, z, 0.25, mp, my);
            var colP = new double[3];
            var colY = new double[3];
            AttitudeTranslation.MotifColumns (n, s, mp, my, tpn, colP, colY);
            double half = AttitudeTranslation.SymmetricHalfWidth (n, mp, my, margin);
            double diffAuth = AttitudeTranslation.Inradius (colP, colY, half);
            double full = 2.0 * AttitudeTranslation.MaxSingular2 (j.Kt [0], j.Kt [1], j.Kt [4], j.Kt [5]);
            today = AttitudeTranslation.LateralAuthority (thrust / mass, 2.0, 0.8, 0.8, diffAuth, full);
            j.Mass = mass;
            for (int r = 0; r < 3; r++) {
                j.Bd [2 * r] = colP [r];
                j.Bd [2 * r + 1] = colY [r];
            }
            j.PhiMax = 0.8 * 2.0;
            j.RcsMax = 0.5;           // no RCS translation column: absent
            j.DiffMax = 0.8 * half;
            j.DiffWeight = 2.0;
            j.FracGimbal = 0.8;
            joint = j.Prepare () ? j.Authority () : 0.0;
            Console.WriteLine ("  computed levers: " + name + " at hover (throttle " + main.ToString ("F3") +
                "): today's authority " + today.ToString ("F4") + " m/s^2, joint " + joint.ToString ("F4") +
                " (+x " + j.Directions [0].ToString ("F4") + ", -x " + j.Directions [1].ToString ("F4") + ", +z " +
                j.Directions [2].ToString ("F4") + ", -z " + j.Directions [3].ToString ("F4") + ")");
        }

        static void TestComputedAuthorities ()
        {
            foreach (double throttle in new[] { 0.08, 0.3 }) {
                var h = new HeavyModel (throttle);
                double inradius;
                double formula = h.FormulaAuthority (0.8, 0.8, out inradius);
                double rho = 0.0;
                for (int i = 0; i < HeavyModel.Engines; i++)
                    rho += Math.Sqrt (h.X [i] * h.X [i] + h.Z [i] * h.Z [i]) / HeavyModel.Engines;
                double margin = Math.Min (throttle, 1 - throttle);
                double spec = 0.8 * 2.83 * margin * h.SlopeEach * rho / HeavyModel.L / HeavyModel.Mass;
                var j = new JointTranslation ();
                h.Fill (j, 0.8, 0.8, 0.5, false, true);
                double diffTilt = j.Authority ();
                h.Fill (j, 0.8, 0.8, 0.5, true, false);
                double rcsTilt = j.Authority ();
                h.Fill (j, 0.8, 0.8, 0.5, true, true);
                double all = j.Authority ();
                double rcsForce = 0.5 * Math.Abs (j.Rf [0]);
                Console.WriteLine ("  computed levers: HEAVY throttle " + throttle.ToString ("F2") + " (T " +
                    (h.Thrust / 1e6).ToString ("F2") + " MN, 260 t): differential inradius " + (inradius / 1e3).ToString ("F1") +
                    " kN.m; authority m/s^2: formula of modes 1/3 (differential + tilt) " + formula.ToString ("F4") +
                    " [spec 0.8 x 2.83 x margin x s x rho / L / m = " + spec.ToString ("F4") + "], joint differential + tilt " +
                    diffTilt.ToString ("F4") + ", joint RCS + tilt " + rcsTilt.ToString ("F4") + ", joint ALL " +
                    all.ToString ("F4") + " (+x " + j.Directions [0].ToString ("F4") + ", -x " + j.Directions [1].ToString ("F4") +
                    ", +z " + j.Directions [2].ToString ("F4") + ", -z " + j.Directions [3].ToString ("F4") + "); RCS alone at 0.5: " +
                    (rcsForce / 1e3).ToString ("F1") + " kN = " + (rcsForce / HeavyModel.Mass).ToString ("F4") + " m/s^2 WITH its torque");
                Check (formula > 0.9 * spec && formula < 1.15 * spec,
                    "heavy " + throttle + ": the formula's differential authority ~ 0.8 x 2.83 x margin x s x rho / L / m (" +
                    (formula / spec).ToString ("F3") + ")");
                Check (diffTilt >= 0.9 * formula, "heavy " + throttle + ": joint differential + tilt ~ the formula (" +
                    (diffTilt / formula).ToString ("F3") + ")");
                Check (all >= rcsTilt - 1e-9 && all >= diffTilt - 1e-9, "heavy " + throttle + ": all levers >= any subset");
                if (throttle == 0.08)
                    Check (all > 0.3 && all < 0.4, "heavy 0.08: ~0.3 m/s^2 of torque-free translation (" + all.ToString ("F4") + ")");
                else
                    Check (all > 0.4, "heavy 0.3: more tilt, more authority (" + all.ToString ("F4") + ")");
            }
            // Barge (4 engines in a +-0.6 m square) and medium (6 on a 1.26 m ring): pivots 3.8 m
            // below the CoM, no RCS translation: bit 4 gives at least today's authority.
            double today, joint;
            var bx = new double[4];
            var bz = new double[4];
            Square (0.6, bx, bz);
            Compare ("barge", bx, bz, -5.96, -3.8, 750e3, 29300.0, out today, out joint);
            Check (joint >= today * (1 - 1e-6) && joint > 0.0, "barge: bit 4 authority >= today's");
            var mx = new double[6];
            var mz = new double[6];
            for (int i = 0; i < 6; i++) {
                mx [i] = 1.26 * Math.Cos (i * Math.PI / 3);
                mz [i] = 1.26 * Math.Sin (i * Math.PI / 3);
            }
            Compare ("medium", mx, mz, -5.96, -3.8, 900e3, 60000.0, out today, out joint);
            Check (joint >= today * (1 - 1e-6) && joint > 0.0, "medium: bit 4 authority >= today's");
        }

        // --------------------------------------------- heavy booster, closed loop (mode 7)

        sealed class HeavyHoverResult
        {
            internal double SettleAccelZ, MaxTiltDeg, FinalTiltDeg, MaxRcs, MaxResidualFf, MinThrottle = 1,
                MaxThrottle, MaxGimbal;
            // Step-to-step change over the last 100 steps: tilt (deg), RCS, engine throttle.
            internal double LateTiltJump, LateRcsJump, LateThrottleJump;
            internal bool AllComputed = true, AllConverged = true;
        }

        /// <summary>
        /// The plugin's mode-7 sequence on the heavy booster model through the UNCHANGED law:
        /// joint allocation (tilt + RCS + differential), realized ramp, RCS written x realized
        /// share, feed-forward of the realized tilt AND the written RCS by the differential,
        /// shifted bounds, AttitudeMath.Step, commands; then one physics step. rcsDelay: the RCS
        /// (rotation and translation) acts one step after it is written (ModuleRCS reads
        /// ctrlState in Update). Demand: 0.2 m/s^2 of horizontal acceleration along world z.
        /// </summary>
        static HeavyHoverResult HeavyHover (double throttle, int rcsDelay, double engineLag, double gimbalErr,
            double disturbance, int steps, double demandZ)
        {
            var h = new HeavyModel (throttle);
            var p = new TranslationParams ();
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            var j = new JointTranslation ();
            const int E = HeavyModel.Engines;
            const double dt = 0.02;
            double[] inertia = { 5.4e7, 8.0e5, 5.4e7 };
            double f = AttitudeTranslation.RampFactor (true, 8.0, dt);
            int nG = 2 * E, n = nG + 3 + 2;
            inp.Resize (n);
            inp.Dt = dt;
            for (int a = 0; a < 3; a++)
                inp.Inertia [a] = inertia [a];
            double[] rcsRot = { h.RcsRotationColumn (0), h.RcsRotationColumn (1), h.RcsRotationColumn (2) };
            var theta = new double[3];
            var omega = new double[3];
            var omegaPrev = new double[3];
            bool havePrev = false;
            var uReal = new double[nG];
            var uCmd = new double[nG];
            var common = new double[nG];
            var commonReal = new double[nG];
            var commonNext = new double[nG];
            var tauReal = new double[E];
            var tauCmd = new double[E];
            for (int i = 0; i < E; i++)
                tauReal [i] = tauCmd [i] = throttle;
            var slope = new double[E];
            for (int i = 0; i < E; i++)
                slope [i] = h.SlopeEach;
            var margin = new double[E];
            for (int i = 0; i < E; i++)
                margin [i] = AttitudeTranslation.ThrottleMargin (throttle, 0, 1);
            // RCS commands written (rotation pitch/roll/yaw, translation X/Y) and applied.
            var rotCmd = new double[3];
            var linCmd = new double[2];
            var rotApplied = new double[3];
            var linApplied = new double[2];
            var rotQueue = new double[3];
            var linQueue = new double[2];
            double dffPrevP = 0, dffPrevY = 0;
            var gap = new double[3];
            var delta = new double[E];
            var res = new HeavyHoverResult ();
            double prevPhiX = double.NaN, prevPhiZ = double.NaN, prevRx = 0, prevRy = 0, prevTau0 = double.NaN;
            double accelSum = 0;
            int accelCount = 0;
            var gx = new double[3];
            var gy = new double[3];
            var fx = new double[3];
            var fy = new double[3];
            for (int step = 0; step < steps; step++) {
                // ---- measurement: columns at the realized thrust, realized torque (true plant).
                var thrust = new double[E];
                double thrustSum = 0;
                for (int i = 0; i < E; i++) {
                    thrust [i] = slope [i] * tauReal [i];
                    thrustSum += thrust [i];
                }
                // Fill the joint as the plugin does each tick: RCS, motifs, bounds from the model,
                // then K and Kt summed from the gimbal columns at the REALIZED thrust (V1 axes:
                // gimbal x = body x, y = body z).
                h.Fill (j, p.FracGimbal, p.FracDiff, p.FracRcs, true, true);
                for (int k = 0; k < 4; k++)
                    j.K [k] = 0.0;
                for (int k = 0; k < 6; k++)
                    j.Kt [k] = 0.0;
                var torque = new double[3];
                for (int i = 0; i < E; i++) {
                    EngineColumns (h.X [i], -HeavyModel.L, h.Z [i], -HeavyModel.L, thrust [i], gx, gy, fx, fy);
                    for (int r = 0; r < 3; r++) {
                        inp.B [r * n + 2 * i] = gx [r];
                        inp.B [r * n + 2 * i + 1] = gy [r];
                        // Measured as the plugin does: the REAL engine torque (true plant).
                        torque [r] += gimbalErr * (gx [r] * uReal [2 * i] + gy [r] * uReal [2 * i + 1]);
                        torque [r] += thrust [i] * h.Tpn [3 * i + r];
                        j.Kt [2 * r] += gx [r];
                        j.Kt [2 * r + 1] += gy [r];
                    }
                    j.K [0] += fx [0];
                    j.K [1] += fy [0];
                    j.K [2] += fx [2];
                    j.K [3] += fy [2];
                }
                j.Prepare ();
                // The RCS the last FixedUpdate applied (rotation and translation).
                for (int r = 0; r < 3; r++)
                    torque [r] += rcsRot [r] * rotApplied [r] + j.Rt [2 * r] * linApplied [0] +
                        j.Rt [2 * r + 1] * linApplied [1];
                for (int a = 0; a < 3; a++) {
                    inp.Omega [a] = omega [a];
                    inp.OmegaDotMeasured [a] = havePrev ? (omega [a] - omegaPrev [a]) / dt : 0.0;
                    inp.Torque [a] = torque [a];
                    inp.OmegaFf [a] = 0;
                    inp.OmegaDotFf [a] = 0;
                }
                inp.OmegaDotValid = havePrev;
                havePrev = true;
                Array.Copy (omega, omegaPrev, 3);
                inp.ReferenceValid = AttitudeMath.PointingError (theta [2], 1.0, -theta [0], inp.Error);
                inp.OmegaMax = 0.2;
                for (int a = 0; a < 3; a++)
                    inp.OmegaMaxAxis [a] = 0.2;
                for (int k = 0; k < nG; k++) {
                    inp.U0 [k] = uReal [k];
                    inp.UMin [k] = -HeavyModel.RangeDeg;
                    inp.UMax [k] = HeavyModel.RangeDeg;
                    inp.UPref [k] = 0;
                    inp.Priority [k] = 1;
                }
                for (int a = 0; a < 3; a++) {
                    int c = nG + a;
                    for (int r = 0; r < 3; r++)
                        inp.B [r * n + c] = r == a ? rcsRot [a] : 0.0;
                    inp.U0 [c] = rotCmd [a];
                    inp.UMin [c] = -1;
                    inp.UMax [c] = 1;
                    inp.UPref [c] = 0;
                    inp.Priority [c] = 3;
                }

                // ---- plan: realized amplitudes, joint translation, ramp, realized share, feed-forward.
                double dRealP, dRealY;
                AttitudeTranslation.RealizedAmplitudes (E, slope, h.MotifP, h.MotifY, tauReal, out dRealP, out dRealY);
                for (int k = 0; k < nG; k++)
                    common [k] = 0;
                double rX = 0, rY = 0;
                double[] up = { theta [2], 1.0, -theta [0] };
                double[] demandBody = { theta [2] * 0.0, theta [0] * demandZ, demandZ };
                double ax, az, det;
                if (AttitudeTranslation.LateralFromGap (demandBody, up, thrustSum / HeavyModel.Mass,
                        p.MinProjectionDet, gap, out ax, out az, out det) &&
                        j.Solve (ax * HeavyModel.Mass, az * HeavyModel.Mass)) {
                    res.AllConverged &= j.Converged;
                    for (int i = 0; i < E; i++) {
                        common [2 * i] = j.X [0];
                        common [2 * i + 1] = j.X [1];
                    }
                    rX = j.X [2];
                    rY = j.X [3];
                    if (step >= steps - 100 && !double.IsNaN (prevPhiX)) {
                        res.LateTiltJump = Math.Max (res.LateTiltJump, Math.Max (Math.Abs (j.X [0] - prevPhiX),
                            Math.Abs (j.X [1] - prevPhiZ)));
                        res.LateRcsJump = Math.Max (res.LateRcsJump, Math.Max (Math.Abs (rX - prevRx), Math.Abs (rY - prevRy)));
                    }
                    prevPhiX = j.X [0];
                    prevPhiZ = j.X [1];
                    prevRx = rX;
                    prevRy = rY;
                }
                double mx = 0, my = 0, mz = 0;
                for (int k = 0; k < nG; k++) {
                    commonNext [k] = AttitudeTranslation.Ramp (commonReal [k], common [k], f);
                    mx += inp.B [0 * n + k] * commonNext [k];
                    my += inp.B [1 * n + k] * commonNext [k];
                    mz += inp.B [2 * n + k] * commonNext [k];
                }
                double kappa = AttitudeTranslation.RealizedShare (common, commonNext, nG);
                double wX = rX * kappa, wY = rY * kappa;
                mx += j.Rt [0] * wX + j.Rt [1] * wY;
                my += j.Rt [2] * wX + j.Rt [3] * wY;
                mz += j.Rt [4] * wX + j.Rt [5] * wY;
                double dffP = 0, dffY = 0;
                AttitudeTranslation.CancelTorque (h.ColP, h.ColY, mx, mz, out dffP, out dffY);
                double hp, hy, share;
                AttitudeTranslation.BoxHalfWidths (E, h.MotifP, h.MotifY, margin, ref dffP, ref dffY, out hp, out hy, out share);
                double rxr = mx + h.ColP [0] * dffP + h.ColY [0] * dffY, rzr = mz + h.ColP [2] * dffP + h.ColY [2] * dffY;
                if (step > 5)
                    res.MaxResidualFf = Math.Max (res.MaxResidualFf, Math.Sqrt (rxr * rxr + rzr * rzr));
                for (int k = 0; k < nG; k++) {
                    double u0, lo, hi;
                    AttitudeTranslation.ShiftGimbal (uReal [k], commonReal [k], common [k], inp.UMin [k],
                        inp.UMax [k], out u0, out lo, out hi);
                    inp.U0 [k] = u0;
                    inp.UMin [k] = lo;
                    inp.UMax [k] = hi;
                }
                int jp = nG + 3, jyw = nG + 4;
                for (int r = 0; r < 3; r++) {
                    inp.B [r * n + jp] = h.ColP [r];
                    inp.B [r * n + jyw] = h.ColY [r];
                }
                double u0p, lop, hip, u0y, loy, hiy;
                AttitudeTranslation.DiffBounds (dRealP, dffPrevP, dffP, hp, true, out u0p, out lop, out hip);
                AttitudeTranslation.DiffBounds (dRealY, dffPrevY, dffY, hy, true, out u0y, out loy, out hiy);
                inp.U0 [jp] = u0p;
                inp.UMin [jp] = lop;
                inp.UMax [jp] = hip;
                inp.U0 [jyw] = u0y;
                inp.UMin [jyw] = loy;
                inp.UMax [jyw] = hiy;
                inp.UPref [jp] = inp.UPref [jyw] = 0;
                inp.Priority [jp] = inp.Priority [jyw] = p.DiffPriority;

                // ---- law and commands.
                AttitudeMath.Step (g, inp, st, work, o);
                if (step > 0 && !o.Computed)
                    res.AllComputed = false;
                if (o.Computed) {
                    for (int k = 0; k < nG; k++) {
                        uCmd [k] = Math.Max (-HeavyModel.RangeDeg, Math.Min (HeavyModel.RangeDeg,
                            common [k] + inp.U0 [k] + o.DeltaU [k]));
                        res.MaxGimbal = Math.Max (res.MaxGimbal, Math.Abs (uCmd [k]));
                    }
                    double dP = dffP + inp.U0 [jp] + o.DeltaU [jp];
                    double dY = dffY + inp.U0 [jyw] + o.DeltaU [jyw];
                    AttitudeTranslation.ThrottleDeltas (E, h.MotifP, h.MotifY, dP, dY, delta);
                    double kw = AttitudeTranslation.WriteScale (E, delta, throttle, 0, 1);
                    for (int i = 0; i < E; i++) {
                        tauCmd [i] = throttle + kw * delta [i];
                        res.MinThrottle = Math.Min (res.MinThrottle, tauCmd [i]);
                        res.MaxThrottle = Math.Max (res.MaxThrottle, tauCmd [i]);
                    }
                    if (step >= steps - 100 && !double.IsNaN (prevTau0))
                        res.LateThrottleJump = Math.Max (res.LateThrottleJump, Math.Abs (tauCmd [0] - prevTau0));
                    prevTau0 = tauCmd [0];
                    for (int a = 0; a < 3; a++)
                        rotCmd [a] = Math.Max (-1.0, Math.Min (1.0, inp.U0 [nG + a] + o.DeltaU [nG + a]));
                    linCmd [0] = wX;
                    linCmd [1] = wY;
                    res.MaxRcs = Math.Max (res.MaxRcs, Math.Max (Math.Abs (wX), Math.Abs (wY)));
                    Array.Copy (commonNext, commonReal, nG);
                    dffPrevP = dffP;
                    dffPrevY = dffY;
                }

                // ---- physics step (true plant).
                if (rcsDelay > 0) {
                    Array.Copy (rotQueue, rotApplied, 3);
                    Array.Copy (linQueue, linApplied, 2);
                    Array.Copy (rotCmd, rotQueue, 3);
                    Array.Copy (linCmd, linQueue, 2);
                } else {
                    Array.Copy (rotCmd, rotApplied, 3);
                    Array.Copy (linCmd, linApplied, 2);
                }
                for (int k = 0; k < nG; k++)
                    uReal [k] = AttitudeTranslation.Ramp (uReal [k], uCmd [k], f);
                for (int i = 0; i < E; i++)
                    tauReal [i] = engineLag >= 1.0 ? tauCmd [i] : tauReal [i] + (tauCmd [i] - tauReal [i]) * engineLag;
                var trueTorque = new double[3];
                double latX = 0, latZ = 0, tSum = 0;
                for (int i = 0; i < E; i++) {
                    double ti = slope [i] * tauReal [i];
                    tSum += ti;
                    EngineColumns (h.X [i], -HeavyModel.L, h.Z [i], -HeavyModel.L, ti, gx, gy, fx, fy);
                    for (int r = 0; r < 3; r++) {
                        trueTorque [r] += gimbalErr * (gx [r] * uReal [2 * i] + gy [r] * uReal [2 * i + 1]);
                        trueTorque [r] += ti * h.Tpn [3 * i + r];
                    }
                    latX += fx [0] * uReal [2 * i] + fy [0] * uReal [2 * i + 1];
                    latZ += fx [2] * uReal [2 * i] + fy [2] * uReal [2 * i + 1];
                }
                for (int a = 0; a < 3; a++)
                    trueTorque [a] += rcsRot [a] * rotApplied [a];
                for (int r = 0; r < 3; r++)
                    trueTorque [r] += j.Rt [2 * r] * linApplied [0] + j.Rt [2 * r + 1] * linApplied [1];
                latX += j.Rf [0] * linApplied [0] + j.Rf [1] * linApplied [1];
                latZ += j.Rf [2] * linApplied [0] + j.Rf [3] * linApplied [1];
                trueTorque [0] += disturbance;
                trueTorque [2] -= 0.5 * disturbance;
                for (int a = 0; a < 3; a++) {
                    omega [a] += trueTorque [a] / inertia [a] * dt;
                    theta [a] += omega [a] * dt;
                }
                double accelZ = (tSum * theta [0] + latZ) / HeavyModel.Mass;
                double tilt = Math.Sqrt (theta [0] * theta [0] + theta [2] * theta [2]) / Deg;
                res.MaxTiltDeg = Math.Max (res.MaxTiltDeg, tilt);
                if (step >= steps - 50) {
                    accelSum += accelZ;
                    accelCount++;
                }
            }
            res.FinalTiltDeg = Math.Sqrt (theta [0] * theta [0] + theta [2] * theta [2]) / Deg;
            res.SettleAccelZ = accelCount > 0 ? accelSum / accelCount : 0;
            return res;
        }

        static void TestHeavyJointHover ()
        {
            var r = HeavyHover (0.08, 0, 1.0, 1.0, 0.0, 300, 0.2);
            Console.WriteLine ("  computed levers: heavy hover 0.2 m/s^2 (mode 7), ideal plant: accel " +
                r.SettleAccelZ.ToString ("F4") + ", max tilt " + r.MaxTiltDeg.ToString ("F4") + " deg, max RCS " +
                r.MaxRcs.ToString ("F3") + ", max gimbal " + r.MaxGimbal.ToString ("F3") + " deg, throttles " +
                r.MinThrottle.ToString ("F4") + "-" + r.MaxThrottle.ToString ("F4") + ", feed-forward residual " +
                (r.MaxResidualFf / 1e3).ToString ("F2") + " kN.m");
            Check (r.AllComputed && r.AllConverged, "heavy hover: law and joint allocation run every step");
            Check (Math.Abs (r.SettleAccelZ - 0.2) < 0.02, "heavy hover: 0.2 m/s^2 delivered (" + r.SettleAccelZ.ToString ("F4") + ")");
            Check (r.MaxTiltDeg < 0.2, "heavy hover: attitude held within 0.2 deg (" + r.MaxTiltDeg.ToString ("F4") + ")");
            Check (r.MaxRcs <= 0.5 + 1e-12 && r.MaxRcs > 0.05, "heavy hover: RCS used, within FracRcs");
            Check (r.MinThrottle >= -1e-12 && r.MaxThrottle <= 1 + 1e-12, "heavy hover: throttles within [0, 1]");
            Check (r.MaxResidualFf < 1e3, "heavy hover: feed-forward leaves < 1 kN.m (tilt + RCS torque cancelled)");
            Console.WriteLine ("  computed levers: heavy hover, last 100 steps, largest step-to-step change: tilt " +
                r.LateTiltJump.ToString ("E2") + " deg, RCS " + r.LateRcsJump.ToString ("E2") + ", throttle " +
                r.LateThrottleJump.ToString ("E2"));
            Check (r.LateTiltJump < 1e-3 && r.LateRcsJump < 1e-3 && r.LateThrottleJump < 1e-3,
                "heavy hover: settled, no step-to-step cycle (tilt, RCS, throttles)");
            // Beyond the differential alone (0.035 m/s^2): 0.3 m/s^2 asked.
            var big = HeavyHover (0.08, 0, 1.0, 1.0, 0.0, 300, 0.3);
            Console.WriteLine ("  computed levers: heavy hover 0.3 m/s^2: accel " + big.SettleAccelZ.ToString ("F4") +
                ", max tilt " + big.MaxTiltDeg.ToString ("F4") + " deg, max RCS " + big.MaxRcs.ToString ("F3"));
            Check (Math.Abs (big.SettleAccelZ - 0.3) < 0.03 && big.MaxTiltDeg < 0.3,
                "heavy hover: 0.3 m/s^2 delivered, attitude held (" + big.SettleAccelZ.ToString ("F4") + ")");
            // Degraded: RCS one step late (ModuleRCS reads ctrlState in Update), engines lagging,
            // gimbal torque 15 % weaker than its columns, 50 kN.m of disturbance.
            var d = HeavyHover (0.08, 1, 0.2, 0.85, 5.0e4, 400, 0.2);
            Console.WriteLine ("  computed levers: heavy hover 0.2 m/s^2, degraded plant: accel " + d.SettleAccelZ.ToString ("F4") +
                ", max tilt " + d.MaxTiltDeg.ToString ("F4") + " deg, final tilt " + d.FinalTiltDeg.ToString ("F4") +
                " deg, throttles " + d.MinThrottle.ToString ("F4") + "-" + d.MaxThrottle.ToString ("F4"));
            Check (d.AllComputed, "heavy degraded: the law computes every step");
            Check (Math.Abs (d.SettleAccelZ - 0.2) < 0.05, "heavy degraded: lateral acceleration within 25 % (" +
                d.SettleAccelZ.ToString ("F4") + ")");
            Check (d.MaxTiltDeg < 0.6 && d.FinalTiltDeg < 0.3, "heavy degraded: attitude within 0.6 deg, back under 0.3 (" +
                d.MaxTiltDeg.ToString ("F4") + ", " + d.FinalTiltDeg.ToString ("F4") + ")");
        }

        // ------------------------------------------ heavy stack in the launch clamps (ground check)

        sealed class ClampPhase
        {
            // Medians over the tool's window (last 60 % of the phase), N.m about body x and z.
            internal double EngineX, EngineZ, RcsX, RcsZ, TotalX, TotalZ;
            internal double CommonPhys, CommonModel, Kappa, RcsWrite;
            // Largest step-to-step change of an engine's throttle command in the window.
            internal double ThrottleChatter;
            // Largest change of the total torque (body x) from the phase's first step, before the window.
            internal double PeakTransient;
            internal double MaxTiltDeg;         // free flight: the largest attitude excursion in the phase
            internal int Fallbacks, GuardSteps;
        }

        static double Median (System.Collections.Generic.List<double> xs)
        {
            if (xs.Count == 0)
                return double.NaN;
            var s = xs.ToArray ();
            Array.Sort (s);
            return s [s.Length / 2];
        }

        /// <summary>
        /// The ground check's sequence (ksp the client
        /// --leviers) on the heavy stack held by the launch clamps: omega and its derivative stay
        /// 0, the reference is the nose; mode 5 for m1Steps, then mode 7 at +demand and at -demand
        /// for m3Steps each, through the plugin's sequence of calls (as HeavyHover) and its
        /// pitch/yaw FALLBACK (the law's pitch/yaw axis guards, held GuardHoldSeconds, or
        /// `inject`): gimbals handed to the stock Lerp toward ctrlState (0 here),
        /// ResetTranslationDynamics, X/Y and throttles released. `keepRealized`: the fallback
        /// keeps the common deflection the gimbals still hold in CommonRealized (the fix).
        /// Returns the three phases (m1, m3p, m3m).
        /// </summary>
        static ClampPhase[] HeavyClamp (HeavyModel h, double[] inertia, double gimbalSpeed, int rcsDelay,
            int m1Steps, int m3Steps, double demand, Func<int, bool> inject, bool guards, bool keepRealized,
            bool tilt = false, double driftNm = 0.0, bool commonDir = false, bool kappaPositive = false,
            bool free = false, double disturbanceNm = 0.0)
        {
            var p = new TranslationParams ();
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            var j = new JointTranslation ();
            const int E = HeavyModel.Engines;
            const double dt = 0.02;
            double f = AttitudeTranslation.RampFactor (true, gimbalSpeed, dt);
            var ramps = new double[E];
            for (int i = 0; i < E; i++)
                ramps [i] = f;
            var tpn = new double[3 * E];
            var dirs = new double[3 * E];
            var levY = new double[E];
            var colP = new double[3];
            var colY = new double[3];
            int nG = 2 * E, n = nG + 3 + 2;
            inp.Resize (n);
            inp.Dt = dt;
            for (int a = 0; a < 3; a++)
                inp.Inertia [a] = inertia [a];
            double[] rcsRot = { h.RcsRotationColumn (0), h.RcsRotationColumn (1), h.RcsRotationColumn (2) };
            var uReal = new double[nG];
            var uCmd = new double[nG];
            var common = new double[nG];
            var commonReal = new double[nG];
            var commonNext = new double[nG];
            var tauReal = new double[E];
            var tauCmd = new double[E];
            for (int i = 0; i < E; i++)
                tauReal [i] = tauCmd [i] = h.Throttle;
            var slope = new double[E];
            for (int i = 0; i < E; i++)
                slope [i] = h.SlopeEach;
            var margin = new double[E];
            for (int i = 0; i < E; i++)
                margin [i] = AttitudeTranslation.ThrottleMargin (h.Throttle, 0, 1);
            var rotCmd = new double[3];
            var linCmd = new double[2];
            var rotApplied = new double[3];
            var linApplied = new double[2];
            var rotQueue = new double[3];
            var linQueue = new double[2];
            double dffPrevP = 0, dffPrevY = 0;
            var gap = new double[3];
            var delta = new double[E];
            var gx = new double[3];
            var gy = new double[3];
            var fx = new double[3];
            var fy = new double[3];
            int total = m1Steps + 2 * m3Steps;
            int holdUntil = -1;
            double phaseStartTorque = 0.0;
            var omega = new double[3];
            var omegaPrev = new double[3];
            var theta = new double[3];
            const int pitchYawGuardBits = 0x1C7;     // body x (bits 0-2) and z (bits 6-8)
            int holdSteps = (int)Math.Round (AttitudeWrite.GuardHoldSeconds / dt);
            var phases = new ClampPhase[3];
            var lists = new System.Collections.Generic.List<double>[3, 10];
            for (int ph = 0; ph < 3; ph++) {
                phases [ph] = new ClampPhase ();
                for (int k = 0; k < 10; k++)
                    lists [ph, k] = new System.Collections.Generic.List<double> ();
            }
            for (int step = 0; step < total; step++) {
                int phase = step < m1Steps ? 0 : (step < m1Steps + m3Steps ? 1 : 2);
                int start = phase == 0 ? 0 : m1Steps + (phase - 1) * m3Steps;
                int length = phase == 0 ? m1Steps : m3Steps;
                bool window = step - start >= (int)(0.4 * length);
                bool translate = phase > 0;
                double sign = phase == 2 ? -1.0 : 1.0;

                // ---- measurement (true plant): columns and torque at the realized thrust.
                double thrustSum = 0;
                var thrust = new double[E];
                for (int i = 0; i < E; i++) {
                    thrust [i] = slope [i] * tauReal [i];
                    thrustSum += thrust [i];
                }
                // The motif columns the plugin publishes. `tilt` false: at rest (the model before
                // the third ground check's reading). `tilt` true: CollectMotifEngines reads each
                // engine's torque per newton at its REALIZED nozzle direction (EngineLever): a
                // tilted nozzle adds its lever along the nose, L x tilt, 0.41 m per degree here,
                // three times the heavy booster's 0.133 m motif lever; with `commonDir` the fix,
                // CommonDirectionTorques along the engines' common direction. The physics
                // (engine torque below) is the tilted one in every case.
                for (int i = 0; i < E; i++) {
                    double tx = uReal [2 * i] * Deg, tz = uReal [2 * i + 1] * Deg;
                    double dn = Math.Sqrt (tx * tx + 1.0 + tz * tz);
                    dirs [3 * i] = -tz / dn;
                    dirs [3 * i + 1] = 1.0 / dn;
                    dirs [3 * i + 2] = tx / dn;
                    levY [i] = -h.Lever;
                    for (int r = 0; r < 3; r++)
                        tpn [3 * i + r] = h.Tpn [3 * i + r];
                    if (tilt && !commonDir) {
                        tpn [3 * i] -= h.Lever * tx;
                        tpn [3 * i + 1] -= h.Z [i] * tz + h.X [i] * tx;
                        tpn [3 * i + 2] -= h.Lever * tz;
                    }
                }
                if (tilt && commonDir)
                    AttitudeTranslation.CommonDirectionTorques (E, slope, h.X, levY, h.Z, dirs, tpn);
                AttitudeTranslation.MotifColumns (E, slope, h.MotifP, h.MotifY, tpn, colP, colY);
                h.Fill (j, p.FracGimbal, p.FracDiff, p.FracRcs, true, true);
                for (int r = 0; r < 3; r++) {
                    j.Bd [2 * r] = colP [r];
                    j.Bd [2 * r + 1] = colY [r];
                }
                for (int k = 0; k < 4; k++)
                    j.K [k] = 0.0;
                for (int k = 0; k < 6; k++)
                    j.Kt [k] = 0.0;
                var engine = new double[3];
                for (int i = 0; i < E; i++) {
                    EngineColumns (h.X [i], -h.Lever, h.Z [i], -h.Lever, thrust [i], gx, gy, fx, fy);
                    for (int r = 0; r < 3; r++) {
                        inp.B [r * n + 2 * i] = gx [r];
                        inp.B [r * n + 2 * i + 1] = gy [r];
                        engine [r] += gx [r] * uReal [2 * i] + gy [r] * uReal [2 * i + 1];
                        engine [r] += thrust [i] * h.Tpn [3 * i + r];
                        j.Kt [2 * r] += gx [r];
                        j.Kt [2 * r + 1] += gy [r];
                    }
                    j.K [0] += fx [0];
                    j.K [1] += fy [0];
                    j.K [2] += fx [2];
                    j.K [3] += fy [2];
                }
                j.Prepare ();
                var rcs = new double[3];
                for (int r = 0; r < 3; r++)
                    rcs [r] = rcsRot [r] * rotApplied [r] + j.Rt [2 * r] * linApplied [0] +
                        j.Rt [2 * r + 1] * linApplied [1];
                for (int a = 0; a < 3; a++) {
                    // The clamps (free: the vessel turns, as in flight).
                    inp.Omega [a] = free ? omega [a] : 0.0;
                    inp.OmegaDotMeasured [a] = free && step > 0 ? (omega [a] - omegaPrev [a]) / dt : 0.0;
                    inp.Torque [a] = engine [a] + rcs [a];
                    inp.OmegaFf [a] = 0;
                    // The open-loop drift measured in the clamps (INDI integrating a bias, ~ -1 to -6
                    // kN.m per step on the ground checks): a constant angular-acceleration bias.
                    inp.OmegaDotFf [a] = a == 1 ? 0.0 : driftNm / inertia [a];
                }
                if (step == start)
                    phaseStartTorque = inp.Torque [0];
                else if (!window)
                    phases [phase].PeakTransient = Math.Max (phases [phase].PeakTransient,
                        Math.Abs (inp.Torque [0] - phaseStartTorque));
                inp.OmegaDotValid = step > 0;
                Array.Copy (omega, omegaPrev, 3);
                inp.ReferenceValid = AttitudeMath.PointingError (theta [2], 1.0, -theta [0], inp.Error);
                inp.OmegaMax = 0.2;
                for (int a = 0; a < 3; a++)
                    inp.OmegaMaxAxis [a] = 0.2;
                for (int k = 0; k < nG; k++) {
                    inp.U0 [k] = uReal [k];
                    inp.UMin [k] = -HeavyModel.RangeDeg;
                    inp.UMax [k] = HeavyModel.RangeDeg;
                    inp.UPref [k] = 0;
                    inp.Priority [k] = 1;
                }
                for (int a = 0; a < 3; a++) {
                    int c = nG + a;
                    for (int r = 0; r < 3; r++)
                        inp.B [r * n + c] = r == a ? rcsRot [a] : 0.0;
                    inp.U0 [c] = rotCmd [a];
                    inp.UMin [c] = -1;
                    inp.UMax [c] = 1;
                    inp.UPref [c] = 0;
                    inp.Priority [c] = 3;
                }

                // ---- plan (ApplyTranslationColumns, mode 5 then 7).
                double dRealP, dRealY;
                AttitudeTranslation.RealizedAmplitudes (E, slope, h.MotifP, h.MotifY, tauReal, out dRealP, out dRealY);
                for (int k = 0; k < nG; k++)
                    common [k] = 0;
                double rX = 0, rY = 0;
                double[] up = { 0.0, 1.0, 0.0 };
                double[] demandBody = { 0.0, 0.0, sign * demand };
                double ax, az, det;
                if (translate && AttitudeTranslation.LateralFromGap (demandBody, up, thrustSum / h.MassKg,
                        p.MinProjectionDet, gap, out ax, out az, out det) &&
                        j.Solve (ax * h.MassKg, az * h.MassKg) && j.Converged) {
                    for (int i = 0; i < E; i++) {
                        common [2 * i] = j.X [0];
                        common [2 * i + 1] = j.X [1];
                    }
                    rX = j.X [2];
                    rY = j.X [3];
                }
                double mx = 0, my = 0, mz = 0;
                for (int k = 0; k < nG; k++) {
                    commonNext [k] = AttitudeTranslation.Ramp (commonReal [k], common [k], f);
                    mx += inp.B [0 * n + k] * commonNext [k];
                    my += inp.B [1 * n + k] * commonNext [k];
                    mz += inp.B [2 * n + k] * commonNext [k];
                }
                double kappa = AttitudeTranslation.RealizedShare (common, commonNext, nG);
                if (kappaPositive && kappa < 0.0)
                    kappa = 0.0;                    // the share before the third ground check
                double wX = rX * kappa, wY = rY * kappa;
                mx += j.Rt [0] * wX + j.Rt [1] * wY;
                my += j.Rt [2] * wX + j.Rt [3] * wY;
                mz += j.Rt [4] * wX + j.Rt [5] * wY;
                double dffP = 0, dffY = 0;
                if ((mx != 0.0 || mz != 0.0) && !AttitudeTranslation.CancelTorque (colP, colY, mx, mz,
                        out dffP, out dffY)) {
                    dffP = 0.0;
                    dffY = 0.0;
                }
                double hp, hy, share;
                AttitudeTranslation.BoxHalfWidths (E, h.MotifP, h.MotifY, margin, ref dffP, ref dffY, out hp, out hy, out share);
                for (int k = 0; k < nG; k++) {
                    double u0, lo, hi;
                    AttitudeTranslation.ShiftGimbal (uReal [k], commonReal [k], common [k], inp.UMin [k],
                        inp.UMax [k], out u0, out lo, out hi);
                    inp.U0 [k] = u0;
                    inp.UMin [k] = lo;
                    inp.UMax [k] = hi;
                }
                int jp = nG + 3, jyw = nG + 4;
                for (int r = 0; r < 3; r++) {
                    inp.B [r * n + jp] = colP [r];
                    inp.B [r * n + jyw] = colY [r];
                }
                double u0p, lop, hip, u0y, loy, hiy;
                AttitudeTranslation.DiffBounds (dRealP, dffPrevP, dffP, hp, true, out u0p, out lop, out hip);
                AttitudeTranslation.DiffBounds (dRealY, dffPrevY, dffY, hy, true, out u0y, out loy, out hiy);
                inp.U0 [jp] = u0p;
                inp.UMin [jp] = lop;
                inp.UMax [jp] = hip;
                inp.U0 [jyw] = u0y;
                inp.UMin [jyw] = loy;
                inp.UMax [jyw] = hiy;
                inp.UPref [jp] = inp.UPref [jyw] = 0;
                inp.Priority [jp] = inp.Priority [jyw] = p.DiffPriority;

                // ---- law, then write or fall back (AttitudeV1.TickEntry, pitch/yaw channel).
                AttitudeMath.Step (g, inp, st, work, o);
                bool guard = (o.AxisGuardMask & pitchYawGuardBits) != 0;
                if (guards && guard)
                    holdUntil = step + holdSteps;
                bool fallback = (inject != null && inject (step)) || (guards && (guard || step < holdUntil));
                if (fallback)
                    phases [phase].Fallbacks++;
                if (guard)
                    phases [phase].GuardSteps++;
                if (!fallback && o.Computed) {
                    for (int k = 0; k < nG; k++)
                        uCmd [k] = Math.Max (-HeavyModel.RangeDeg, Math.Min (HeavyModel.RangeDeg,
                            common [k] + inp.U0 [k] + o.DeltaU [k]));
                    double dP = dffP + inp.U0 [jp] + o.DeltaU [jp];
                    double dY = dffY + inp.U0 [jyw] + o.DeltaU [jyw];
                    AttitudeTranslation.ThrottleDeltas (E, h.MotifP, h.MotifY, dP, dY, delta);
                    double kw = AttitudeTranslation.WriteScale (E, delta, h.Throttle, 0, 1);
                    for (int i = 0; i < E; i++) {
                        double before = tauCmd [i];
                        tauCmd [i] = h.Throttle + kw * delta [i];
                        if (window)
                            phases [phase].ThrottleChatter = Math.Max (phases [phase].ThrottleChatter,
                                Math.Abs (tauCmd [i] - before));
                    }
                    for (int a = 0; a < 3; a++)
                        rotCmd [a] = Math.Max (-1.0, Math.Min (1.0, inp.U0 [nG + a] + o.DeltaU [nG + a]));
                    linCmd [0] = translate ? wX : 0.0;
                    linCmd [1] = translate ? wY : 0.0;
                    Array.Copy (commonNext, commonReal, nG);
                    dffPrevP = dffP;
                    dffPrevY = dffY;
                } else if (fallback) {
                    // ReleaseAuthority: the stock Lerp toward ctrlState (0), every lease back,
                    // ResetTranslationDynamics: CommonRealized to 0 (before) or following the
                    // gimbals (keepRealized, computed levers).
                    for (int k = 0; k < nG; k++)
                        uCmd [k] = 0.0;
                    if (keepRealized)
                        AttitudeTranslation.ReleasedCommonFollows (commonReal, ramps);
                    else
                        Array.Clear (commonReal, 0, nG);
                    for (int i = 0; i < E; i++)
                        tauCmd [i] = h.Throttle;
                    for (int a = 0; a < 3; a++)
                        rotCmd [a] = 0.0;
                    linCmd [0] = linCmd [1] = 0.0;
                    dffPrevP = dffPrevY = 0.0;
                }

                // ---- physics (clamped: torques only; free: the measured torque turns the vessel).
                if (free) {
                    for (int a = 0; a < 3; a++) {
                        omega [a] += (inp.Torque [a] + (a == 0 ? disturbanceNm : (a == 2 ? -0.5 * disturbanceNm : 0.0))) / inertia [a] * dt;
                        theta [a] += omega [a] * dt;
                        phases [phase].MaxTiltDeg = Math.Max (phases [phase].MaxTiltDeg, Math.Abs (theta [a]) / Deg);
                    }
                }
                if (rcsDelay > 0) {
                    Array.Copy (rotQueue, rotApplied, 3);
                    Array.Copy (linQueue, linApplied, 2);
                    Array.Copy (rotCmd, rotQueue, 3);
                    Array.Copy (linCmd, linQueue, 2);
                } else {
                    Array.Copy (rotCmd, rotApplied, 3);
                    Array.Copy (linCmd, linApplied, 2);
                }
                for (int k = 0; k < nG; k++)
                    uReal [k] = AttitudeTranslation.Ramp (uReal [k], uCmd [k], f);
                for (int i = 0; i < E; i++)
                    tauReal [i] = tauCmd [i];
                if (window) {
                    // The torques the next step measures (what the ring and the frames record).
                    var eng = new double[3];
                    for (int i = 0; i < E; i++) {
                        double ti = slope [i] * tauReal [i];
                        EngineColumns (h.X [i], -h.Lever, h.Z [i], -h.Lever, ti, gx, gy, fx, fy);
                        for (int r = 0; r < 3; r++)
                            eng [r] += gx [r] * uReal [2 * i] + gy [r] * uReal [2 * i + 1] + ti * h.Tpn [3 * i + r];
                    }
                    var rc = new double[3];
                    for (int r = 0; r < 3; r++)
                        rc [r] = rcsRot [r] * rotApplied [r] + j.Rt [2 * r] * linApplied [0] +
                            j.Rt [2 * r + 1] * linApplied [1];
                    double phys = 0, model = 0;
                    for (int i = 0; i < E; i++) {
                        phys += uReal [2 * i] / E;
                        model += commonReal [2 * i] / E;
                    }
                    lists [phase, 0].Add (eng [0]);
                    lists [phase, 1].Add (eng [2]);
                    lists [phase, 2].Add (rc [0]);
                    lists [phase, 3].Add (rc [2]);
                    lists [phase, 4].Add (eng [0] + rc [0]);
                    lists [phase, 5].Add (eng [2] + rc [2]);
                    lists [phase, 6].Add (phys);
                    lists [phase, 7].Add (model);
                    lists [phase, 8].Add (kappa);
                    lists [phase, 9].Add (linCmd [0]);
                }
            }
            for (int ph = 0; ph < 3; ph++) {
                var c = phases [ph];
                c.EngineX = Median (lists [ph, 0]);
                c.EngineZ = Median (lists [ph, 1]);
                c.RcsX = Median (lists [ph, 2]);
                c.RcsZ = Median (lists [ph, 3]);
                c.TotalX = Median (lists [ph, 4]);
                c.TotalZ = Median (lists [ph, 5]);
                c.CommonPhys = Median (lists [ph, 6]);
                c.CommonModel = Median (lists [ph, 7]);
                c.Kappa = Median (lists [ph, 8]);
                c.RcsWrite = Median (lists [ph, 9]);
            }
            return phases;
        }

        /// <summary>
        /// Third ground check (sol_leviers3, 28/09, the stack in the clamps): total torque 2 033
        /// kN.m at -e, engines NOT reversed, throttles swinging 0 / 0.16 every step from 12 steps
        /// after the reversal, the law's modelled gimbal torque (+2 303 kN.m) against the realized
        /// (-1 255). Reproduced here: each engine's torque per newton read at its REALIZED nozzle
        /// (L x its own tilt, 0.41 m per degree against the 0.133 m motif lever) ties the
        /// differential to the gimbals' attitude part; with the clamps' open-loop drift (the law
        /// integrating a bias, -1.2 kN.m per step as measured in m1) that part grows until the
        /// loop chatters. Fixed by CommonDirectionTorques: the same drift then gives what the rest
        /// geometry gives. Also the realized share, signed through a reversal.
        /// </summary>
        static void TestHeavyClampTiltedLevers ()
        {
            var lev = new double[] { 1.0, 0.0, -1.0, -1.0, 0.0, 0.0 };
            var dir = new double[] { 0.0, 1.0, 0.0, 0.02, 1.0, 0.0 };
            var tq = new double[6];
            Check (AttitudeTranslation.CommonDirectionTorques (2, new double[] { 1.0, 1.0 }, new double[] { 1.0, -1.0 },
                    new double[] { -10.0, -10.0 }, new double[] { 0.0, 0.0 }, dir, tq) &&
                Near (tq [0], 0.0, 1e-12) && Near (tq [2], 1.1 / Math.Sqrt (1.0001), 1e-12) &&
                Near (tq [5], -0.9 / Math.Sqrt (1.0001), 1e-12),
                "common direction: every engine along the slope-weighted mean direction");
            Check (!AttitudeTranslation.CommonDirectionTorques (1, new double[] { 1.0 }, lev, lev, lev,
                new double[] { 0.0, 0.0, 0.0 }, tq), "common direction: none, torques untouched");

            var h = new HeavyModel (0.08, 23.66, 1.0535e6, 26.31 - 37.25, 2.609e6 / 0.08);
            double[] inertia = { 4.9e8, 3.0e6, 4.9e8 };
            const double drift = -2e4;
            var rest = HeavyClamp (h, inertia, 8.0, 1, 100, 125, 0.15, null, true, true, false, drift);
            var tilted = HeavyClamp (h, inertia, 8.0, 1, 100, 125, 0.15, null, true, true, true, drift);
            var fixedRun = HeavyClamp (h, inertia, 8.0, 1, 100, 125, 0.15, null, true, true, true, drift, true);
            Console.WriteLine (ClampLine ("open-loop drift, motif columns at rest (reference)", rest));
            Console.WriteLine (ClampLine ("same drift, columns at the tilted nozzles (before)", tilted));
            Console.WriteLine (ClampLine ("same drift, columns along the common direction (fixed)", fixedRun));
            double dEngP = tilted [1].EngineX - tilted [0].EngineX, dEngM = tilted [2].EngineX - tilted [0].EngineX;
            Check (Math.Sign (dEngP) == Math.Sign (dEngM) && Math.Abs (tilted [2].TotalX - tilted [0].TotalX) >
                    3.0 * Math.Abs (rest [2].TotalX - rest [0].TotalX) && tilted [2].ThrottleChatter > 0.05,
                "clamps, tilted columns: engines not reversed at -e, total 3 x the drift's, throttles chattering (as measured)");
            for (int ph = 1; ph < 3; ph++) {
                double ref0 = rest [ph].TotalX - rest [0].TotalX, got = fixedRun [ph].TotalX - fixedRun [0].TotalX;
                Check (Math.Abs (got - ref0) <= 0.1 * Math.Abs (ref0) + 5e4 && fixedRun [ph].ThrottleChatter < 0.01,
                    "clamps, common direction: the drift's torque only, no chatter (phase " + ph + ": " +
                    (got / 1e3).ToString ("F0") + " kN.m for " + (ref0 / 1e3).ToString ("F0") + ")");
            }
            // The reversal without drift, RCS applied the same step (the share alone): the old
            // tilt's torque unbalanced during the half ramp, then balanced by the signed share.
            var positive = HeavyClamp (h, inertia, 8.0, 0, 100, 125, 0.15, null, true, true, true, 0.0, true, true);
            var signed = HeavyClamp (h, inertia, 8.0, 0, 100, 125, 0.15, null, true, true, true, 0.0, true, false);
            Console.WriteLine ("  computed levers, clamps, reversal transient: share in [0, 1] " +
                (positive [2].PeakTransient / 1e3).ToString ("F0") + " kN.m, signed " +
                (signed [2].PeakTransient / 1e3).ToString ("F0") + " kN.m");
            Check (positive [2].PeakTransient > 2e5 && signed [2].PeakTransient < 5e4,
                "reversal: the signed share keeps the old tilt balanced (transient < 50 kN.m)");
            double t1, t2;
            Check (ClampNetOk (signed, 1, out t1) && ClampNetOk (signed, 2, out t2),
                "clamps without drift, fixed: total torque held at +e and -e");

            // In flight (PF: the booster alone, free to turn, 0.3 m/s^2 at +e then -e): omega-dot
            // closes the loop, the attitude part stays small and nothing couples; it does when the
            // gimbals hold a large one (1 MN.m of steady torque, their authority's order).
            var pf = new HeavyModel (0.08);
            double[] inertiaPf = { 5.4e7, 8.0e5, 5.4e7 };
            var calmBefore = HeavyClamp (pf, inertiaPf, 8.0, 1, 100, 125, 0.3, null, true, true, true, 0.0, false,
                false, true);
            var heldBefore = HeavyClamp (pf, inertiaPf, 8.0, 1, 100, 125, 0.3, null, true, true, true, 0.0, false,
                false, true, 1e6);
            var heldAfter = HeavyClamp (pf, inertiaPf, 8.0, 1, 100, 125, 0.3, null, true, true, true, 0.0, true,
                false, true, 1e6);
            Console.WriteLine ("  computed levers, in flight (PF), reversal: tilted columns, calm: throttle jump " +
                calmBefore [2].ThrottleChatter.ToString ("F3") + ", max tilt " + calmBefore [2].MaxTiltDeg.ToString ("F4") +
                " deg; 1 MN.m held: tilted " + heldBefore [2].ThrottleChatter.ToString ("F3") + " / " +
                heldBefore [2].MaxTiltDeg.ToString ("F4") + " deg, common direction " +
                heldAfter [2].ThrottleChatter.ToString ("F3") + " / " + heldAfter [2].MaxTiltDeg.ToString ("F4") + " deg");
            Check (calmBefore [2].ThrottleChatter < 0.01 && heldBefore [2].ThrottleChatter > 0.05 &&
                heldAfter [2].ThrottleChatter < 0.01 && heldAfter [2].MaxTiltDeg < heldBefore [2].MaxTiltDeg,
                "in flight: the tilted columns chatter only with a large held attitude part; common direction: none");
        }

        /// The ground check's couple_net rule on phase ph (1 m3p, 2 m3m) against m1: the total
        /// change within 0.25 x the larger of the engine and RCS changes + 1 kN.m. Translation
        /// torques are about body x here (demand along z).
        static bool ClampNetOk (ClampPhase[] r, int ph, out double total)
        {
            double eng = Math.Abs (r [ph].EngineX - r [0].EngineX), rcs = Math.Abs (r [ph].RcsX - r [0].RcsX);
            total = Math.Abs (r [ph].TotalX - r [0].TotalX);
            return total <= 0.25 * Math.Max (eng, rcs) + 1e3 && rcs >= 1e3;
        }

        static string ClampLine (string name, ClampPhase[] r)
        {
            var s = new System.Text.StringBuilder ("  computed levers, clamps, " + name + ":");
            string[] names = { "m1", "m3p", "m3m" };
            for (int ph = 1; ph < 3; ph++) {
                var c = r [ph];
                var b = r [0];
                s.Append (" " + names [ph] + " engines " + ((c.EngineX - b.EngineX) / 1e3).ToString ("F0") + " kN.m, RCS " +
                    ((c.RcsX - b.RcsX) / 1e3).ToString ("F0") + ", total " + ((c.TotalX - b.TotalX) / 1e3).ToString ("F0") +
                    ", tilt real/model " + c.CommonPhys.ToString ("F2") + "/" + c.CommonModel.ToString ("F2") +
                    " deg, kappa " + c.Kappa.ToString ("F2") + ", throttle jump " + c.ThrottleChatter.ToString ("F3") + ", transient " +
                    (c.PeakTransient / 1e3).ToString ("F0") + " kN.m" + ", fallbacks " +
                    c.Fallbacks + ";");
            }
            return s.ToString ();
        }

        /// <summary>
        /// Second ground check (sol_leviers2.json, the stack in the clamps, mode 7 at +e then -e):
        /// the total torque must stay put. Without a pitch/yaw fallback it does, the law holding
        /// it (omega-dot stays 0: INDI only sees torque changes). Each fallback handed the gimbals
        /// to the stock Lerp and, before, dropped CommonRealized to 0 while they still held the
        /// tilt: the next writing step counted it as the law's attitude part and kept it (tilt
        /// beyond the command, total torque of hundreds of kN.m, RCS scaled down by kappa).
        /// ReleasedCommonFollows keeps the split right.
        /// </summary>
        static void TestHeavyClampFallback ()
        {
            var c = new double[] { 0.6, -0.2, 0.6, -0.2, 0.0 };
            AttitudeTranslation.ReleasedCommonFollows (c, new double[] { 0.25, 1.0 });
            Check (Near (c [0], 0.45, 1e-12) && Near (c [1], -0.15, 1e-12) && c [2] == 0.0 && c [3] == 0.0 && c [4] == 0.0,
                "released common: x (1 - f) per module, snap (f = 1) and a column without module go to 0");
            var d = new double[] { 0.6, 0.6 };
            AttitudeTranslation.ReleasedCommonFollows (d, new double[] { double.NaN });
            Check (d [0] == 0.0 && d [1] == 0.0, "released common: a non-finite ramp factor drops it at once");

            // The stack of the ground check: 1 053 t, nozzles 23.66 m below the CoM, RCS 26.31 m
            // above, 2.61 MN at throttle 0.08; stock ramp 8/s; RCS one step late.
            var h = new HeavyModel (0.08, 23.66, 1.0535e6, 26.31 - 37.25, 2.609e6 / 0.08);
            double[] inertia = { 4.9e8, 3.0e6, 4.9e8 };
            const int m1 = 100, m3 = 125;
            var clean = HeavyClamp (h, inertia, 8.0, 1, m1, m3, 0.15, null, true, true);
            Console.WriteLine (ClampLine ("no fallback", clean));
            double t1, t2;
            Check (ClampNetOk (clean, 1, out t1) && ClampNetOk (clean, 2, out t2) && clean [1].Fallbacks == 0 &&
                clean [2].Fallbacks == 0, "clamps: no guard in the sequence, total torque held at +e and -e");
            Check (Math.Sign (clean [1].EngineX - clean [0].EngineX) == -Math.Sign (clean [1].RcsX - clean [0].RcsX) &&
                Math.Sign (clean [2].EngineX - clean [0].EngineX) == -Math.Sign (clean [2].RcsX - clean [0].RcsX) &&
                Math.Sign (clean [1].EngineX - clean [0].EngineX) == -Math.Sign (clean [2].EngineX - clean [0].EngineX),
                "clamps: engine and RCS torques opposite, both reversed between +e and -e");
            int start = m1 + m3;
            Func<int, bool> every12 = k => k > start && (k - start) % 12 == 0;
            var before = HeavyClamp (h, inertia, 8.0, 1, m1, m3, 0.15, every12, false, false);
            Console.WriteLine (ClampLine ("one-step fallback every 12 steps at -e, realized common dropped (before)", before));
            Check (!ClampNetOk (before, 2, out t2) && t2 > 3e5 && before [2].Kappa < 0.9,
                "clamps, before: fallbacks at -e leave hundreds of kN.m and cut the RCS (kappa < 0.9)");
            var after = HeavyClamp (h, inertia, 8.0, 1, m1, m3, 0.15, every12, false, true);
            Console.WriteLine (ClampLine ("same fallbacks, realized common followed", after));
            Check (ClampNetOk (after, 1, out t1) && ClampNetOk (after, 2, out t2) && after [2].Kappa > 0.9,
                "clamps, followed: the same fallbacks leave the total torque held, kappa > 0.9");
        }

        // ------------------------------------------------ honest translation (bits 16, 32)

        static void TestHonestModes ()
        {
            Check (AttitudeTranslation.ValidMode (21) && AttitudeTranslation.ValidMode (23) &&
                AttitudeTranslation.ValidMode (37) && AttitudeTranslation.ValidMode (39) &&
                AttitudeTranslation.ValidMode (53) && AttitudeTranslation.ValidMode (55),
                "bits 16/32: 5 and 7 with 16, 32 or both valid (21, 23, 37, 39, 53, 55)");
            Check (!AttitudeTranslation.ValidMode (16) && !AttitudeTranslation.ValidMode (32) &&
                !AttitudeTranslation.ValidMode (48) && !AttitudeTranslation.ValidMode (17) &&
                !AttitudeTranslation.ValidMode (19) && !AttitudeTranslation.ValidMode (20) &&
                !AttitudeTranslation.ValidMode (22) && !AttitudeTranslation.ValidMode (63) &&
                !AttitudeTranslation.ValidMode (-16) && !AttitudeTranslation.ValidMode (8 | 5) &&
                !AttitudeTranslation.ValidMode (64 | 7), "bits 16/32: alone, on 1/3 (no bit 4), with 8 or 64: refused");
            Check (AttitudeTranslation.ValidMode (0) && AttitudeTranslation.ValidMode (1) &&
                AttitudeTranslation.ValidMode (3) && AttitudeTranslation.ValidMode (5) &&
                AttitudeTranslation.ValidMode (7) && !AttitudeTranslation.ValidMode (15),
                "bits 16/32: modes 0-7 as before (15 still refused)");
            Check (AttitudeTranslation.Honest (21) && AttitudeTranslation.Honest (39) && !AttitudeTranslation.Honest (7) &&
                !AttitudeTranslation.Honest (5) && AttitudeTranslation.Computed (55),
                "honest: bits 16/32 read, bit 4 still read under them");
            Check (AttitudeTranslation.ReturnVersionHonest == 3, "return layout 3 with bit 16 or 32");
            Check (AttitudeTranslation.ReasonRcsShared == 1L << 19 && AttitudeTranslation.ReasonRcsModelApprox == 1L << 18,
                "reason bit 19 (law's RCS pitch/yaw left out) after the unmoved 0-18");
            var p = new TranslationParams ();
            Check (p.RcsHoldSeconds == 0.5 && p.ShareTauSeconds == 1.0 && p.ShareMinDegrees == 0.05,
                "honest tunables: hold 0.5 s, share 1 s, evidence 0.05 deg per column");
        }

        static void TestRcsRotationFractions ()
        {
            var h = new HeavyModel (0.08);
            var rho = new double[h.RcsCount];
            // The rotation rule = AddRcsColumns' (HeavyModel.RcsRotationColumn) on every axis.
            for (int a = 0; a < 3; a++) {
                h.RotationFractions (a == 0 ? 1 : 0, a == 1 ? 1 : 0, a == 2 ? 1 : 0, rho);
                double torque = 0.0;
                for (int i = 0; i < h.RcsCount; i++) {
                    double f = Math.Min (1.0, rho [i]);
                    double[] F = { -h.RcsDir [3 * i] * HeavyModel.RcsPower * f, -h.RcsDir [3 * i + 1] * HeavyModel.RcsPower * f,
                        -h.RcsDir [3 * i + 2] * HeavyModel.RcsPower * f };
                    var t = new double[3];
                    Cross (new[] { h.RcsLever [3 * i], h.RcsLever [3 * i + 1], h.RcsLever [3 * i + 2] }, F, t);
                    torque += t [a];
                }
                Check (Near (torque, h.RcsRotationColumn (a), 1e-6 * Math.Abs (h.RcsRotationColumn (a))),
                    "rotation fractions: the column of axis " + a + " rebuilt (" + (torque / 1e3).ToString ("F1") + " kN.m)");
            }
            // Roll +1 on the heavy ring: exactly the 8 tangential nozzles of one sense, at 1.
            h.RotationFractions (0, 1, 0, rho);
            int ones = 0, zeros = 0;
            for (int i = 0; i < h.RcsCount; i++) {
                if (Near (rho [i], 1.0, 1e-12))
                    ones++;
                else if (rho [i] == 0.0)
                    zeros++;
            }
            Check (ones == 8 && zeros == h.RcsCount - 8, "roll +1: 8 tangential nozzles at 1, the others at 0");
            h.RotationFractions (0, 0, 0, rho);
            bool none = true;
            for (int i = 0; i < h.RcsCount; i++)
                none &= rho [i] == 0.0;
            Check (none, "no rotation command: every fraction 0");
            // ModuleRCS's COMBINED rotation: with the roll saturated, a 0.1 pitch fires the whole
            // ring (the flight's x 4.8); without roll, linear.
            var tq = new double[3];
            AttitudeTranslation.RcsRotationTorque (1, new[] { h.RcsCount }, new[] { h.RcsLever }, new[] { h.RcsDir },
                new[] { h.RcsPowers }, new double[] { 1, 1, 1 }, 0.1, 1.0, 0.0, rho, tq);
            var tq0 = new double[3];
            AttitudeTranslation.RcsRotationTorque (1, new[] { h.RcsCount }, new[] { h.RcsLever }, new[] { h.RcsDir },
                new[] { h.RcsPowers }, new double[] { 1, 1, 1 }, 0.0, 1.0, 0.0, rho, tq0);
            double col = h.RcsRotationColumn (0);
            Console.WriteLine ("  honest: heavy RCS, roll 1 + pitch 0.1: pitch torque " + ((tq [0] - tq0 [0]) / 1e6).ToString ("F3") +
                " MN.m against " + (0.1 * col / 1e6).ToString ("F3") + " for the per-axis column (x " +
                ((tq [0] - tq0 [0]) / (0.1 * col)).ToString ("F1") + ")");
            Check (Math.Abs (tq [0] - tq0 [0]) > 3.0 * Math.Abs (0.1 * col),
                "combined rule: roll 1 + pitch 0.1 answers more than 3 x the column's 0.1");
            Check (h.RotationNonlinear (0, 1, 0, 0) && h.RotationNonlinear (0, -1, 0, 2) && h.RotationNonlinear (0.05, 0.95, 0, 0),
                "nonlinear criterion: roll saturated, pitch and yaw flagged");
            Check (!h.RotationNonlinear (0, 0, 0, 0) && !h.RotationNonlinear (0.3, 0, 0, 0) && !h.RotationNonlinear (-1, 0, 0.5, 2) &&
                !h.RotationNonlinear (0, 0.05, 0, 0), "nonlinear criterion: no roll (or 0.05), linear");
            Check (!h.RotationNonlinear (0, 1, 0, 1), "nonlinear criterion: roll axis never asked");
            // A lever along the command (no perpendicular part): 0, as Unity's normalized of zero.
            var r1 = new double[1];
            AttitudeTranslation.RcsRotationFractions (1, new double[] { 0, 5, 0 }, new double[] { 1, 0, 0 }, 0, 1, 0, r1);
            Check (r1 [0] == 0.0, "lever along the rotation axis: fraction 0");
            // Hold: max(now, hold - drop), a NaN hold restarts.
            var hold = new double[] { 1.0, 0.2, double.NaN };
            AttitudeTranslation.HoldFractions (3, new double[] { 0.0, 0.5, 0.3 }, hold, 0.04);
            Check (Near (hold [0], 0.96, 1e-15) && hold [1] == 0.5 && hold [2] == 0.3,
                "hold: decays 0.04 per step, a larger fraction takes over, NaN restarts");
        }

        static void TestRcsHeadroomColumns ()
        {
            // rho 0: bit for bit the bit-4 columns (random geometry, 200 draws).
            var rng = new Random (61);
            bool same = true;
            for (int k = 0; k < 200; k++) {
                int n = 1 + rng.Next (12);
                var lever = new double[3 * n];
                var dir = new double[3 * n];
                var power = new double[n];
                for (int i = 0; i < 3 * n; i++) {
                    lever [i] = (rng.NextDouble () - 0.5) * 60.0;
                    dir [i] = rng.NextDouble () - 0.5;
                }
                for (int i = 0; i < n; i++) {
                    double vn = Math.Sqrt (dir [3 * i] * dir [3 * i] + dir [3 * i + 1] * dir [3 * i + 1] + dir [3 * i + 2] * dir [3 * i + 2]);
                    for (int c = 0; c < 3; c++)
                        dir [3 * i + c] /= vn;
                    power [i] = k % 7 == 0 && i == 0 ? 0.0 : rng.NextDouble () * 1e4;
                }
                double ax = rng.Next (2), az = 1 - ax;
                var a = new double[12];
                var b = new double[12];
                AttitudeTranslation.RcsTranslationColumns (n, lever, dir, power, ax, 0, az, a, new double[3], a, new double[3]);
                var fp1 = new double[3]; var fn1 = new double[3]; var tp1 = new double[3]; var tn1 = new double[3];
                var fp2 = new double[3]; var fn2 = new double[3]; var tp2 = new double[3]; var tn2 = new double[3];
                AttitudeTranslation.RcsTranslationColumns (n, lever, dir, power, ax, 0, az, fp1, fn1, tp1, tn1);
                AttitudeTranslation.RcsTranslationColumnsHeadroom (n, lever, dir, power, new double[n], 0.5, ax, 0, az,
                    fp2, fn2, tp2, tn2);
                for (int c = 0; c < 3; c++)
                    same &= fp1 [c] == fp2 [c] && fn1 [c] == fn2 [c] && tp1 [c] == tp2 [c] && tn1 [c] == tn2 [c];
                var fp3 = new double[3]; var fn3 = new double[3]; var tp3 = new double[3]; var tn3 = new double[3];
                AttitudeTranslation.RcsTranslationColumnsHeadroom (n, lever, dir, power, null, 0.5, ax, 0, az,
                    fp3, fn3, tp3, tn3);
                for (int c = 0; c < 3; c++)
                    same &= fp1 [c] == fp3 [c] && fn1 [c] == fn3 [c] && tp1 [c] == tp3 [c] && tn1 [c] == tn3 [c];
            }
            Check (same, "headroom columns with rho 0 (or null) = the bit-4 columns, bit for bit (200 draws)");
            // rho 1 everywhere: nothing left.
            var h = new HeavyModel (0.08);
            var ones = new double[h.RcsCount];
            for (int i = 0; i < h.RcsCount; i++)
                ones [i] = 1.0;
            var f = new double[3]; var g = new double[3]; var t = new double[3]; var u = new double[3];
            AttitudeTranslation.RcsTranslationColumnsHeadroom (h.RcsCount, h.RcsLever, h.RcsDir, h.RcsPowers, ones, 0.5,
                1, 0, 0, f, g, t, u);
            Check (f [0] == 0.0 && g [0] == 0.0 && t [2] == 0.0 && u [2] == 0.0, "rho 1: no translation column");
            // Roll +1 (the AutoPilot's capture alignment): X keeps the nozzle the roll does not use.
            var rho = new double[h.RcsCount];
            h.RotationFractions (0, 1, 0, rho);
            var full = new double[3]; var fullN = new double[3]; var fullT = new double[3]; var fullTN = new double[3];
            AttitudeTranslation.RcsTranslationColumns (h.RcsCount, h.RcsLever, h.RcsDir, h.RcsPowers, 1, 0, 0,
                full, fullN, fullT, fullTN);
            for (int c = 0; c < 3; c++)
                f [c] = g [c] = t [c] = u [c] = 0.0;
            AttitudeTranslation.RcsTranslationColumnsHeadroom (h.RcsCount, h.RcsLever, h.RcsDir, h.RcsPowers, rho, 0.5,
                1, 0, 0, f, g, t, u);
            Console.WriteLine ("  honest: heavy RCS X column under roll +1: " + (f [0] / 1e3).ToString ("F2") + " kN (+), " +
                (g [0] / 1e3).ToString ("F2") + " kN (-) of " + (full [0] / 1e3).ToString ("F2") + "; yaw torque " +
                (t [2] / 1e6).ToString ("F3") + " of " + (fullT [2] / 1e6).ToString ("F3") + " MN.m");
            Check (Near (f [0], 0.5 * full [0], 1e-9 * Math.Abs (full [0])) && Near (g [0], 0.5 * fullN [0], 1e-9 * Math.Abs (full [0])),
                "roll +1: half of the X force left either way (one tangential pair saturated)");
            // Weaker sign: an asymmetric channel keeps its weaker column for both signs.
            var fp = new double[] { -100, 0, 0, 0, 0, -30 };
            var fn = new double[] { -60, 0, 0, 0, 0, -30 };
            var tp = new double[] { 0, 0, 1000, -300, 0, 0 };
            var tn = new double[] { 0, 0, 600, -300, 0, 0 };
            double sx, sy;
            AttitudeTranslation.WeakerSign (fp, fn, tp, tn, out sx, out sy);
            Check (fp [0] == -60 && tp [2] == 600 && fn [0] == -60 && tn [2] == 600 && Near (sx, 0.6, 1e-15) && sy == 1.0,
                "weaker sign: X keeps the -60 N / 600 N.m column both ways (share 0.6), Y symmetric untouched");
        }

        /// The realized pitch/yaw torque of a joint solution on the heavy booster when the RCS is
        /// the PLANT's (nozzles bounded at 1 with the rotation (pitch, roll, yaw) they also carry):
        /// tilt + differential + RCS translation increment over the rotation alone.
        static double RealizedResidual (HeavyModel h, JointTranslation j, double pitch, double roll, double yaw,
            out double forceX, out double forceZ)
        {
            var f0 = new double[3]; var t0 = new double[3];
            var f1 = new double[3]; var t1 = new double[3];
            h.RcsPlant (pitch, roll, yaw, 0, 0, f0, t0);
            h.RcsPlant (pitch, roll, yaw, j.X [2], j.X [3], f1, t1);
            double tx = j.Kt [0] * j.X [0] + j.Kt [1] * j.X [1] + j.Bd [0] * j.X [4] + j.Bd [1] * j.X [5] + t1 [0] - t0 [0];
            double tz = j.Kt [4] * j.X [0] + j.Kt [5] * j.X [1] + j.Bd [4] * j.X [4] + j.Bd [5] * j.X [5] + t1 [2] - t0 [2];
            forceX = j.K [0] * j.X [0] + j.K [1] * j.X [1] + f1 [0] - f0 [0];
            forceZ = j.K [2] * j.X [0] + j.K [3] * j.X [1] + f1 [2] - f0 [2];
            return Math.Sqrt (tx * tx + tz * tz);
        }

        static void TestHeavyHeadroomBalance ()
        {
            var h = new HeavyModel (0.08);
            var j4 = new JointTranslation ();
            var j16 = new JointTranslation ();
            var rho = new double[h.RcsCount];
            // Calm (no rotation): bit 16 = bit 4.
            h.Fill (j4, 0.8, 0.8, 0.5, true, true);
            h.RotationFractions (0, 0, 0, rho);
            h.Fill (j16, 0.8, 0.8, 0.5, true, true, rho);
            double a4 = j4.Authority (), a16 = j16.Authority ();
            Check (Near (a16, a4, 1e-12 * a4), "calm: bit 16 authority = bit 4 (" + a16.ToString ("F4") + ")");
            foreach (var cmd in new[] { new double[] { 0, 1, 0 }, new double[] { 0, -1, 0 }, new double[] { 0.8, 1, 0 },
                    new double[] { -1, 0, 1 } }) {
                h.RotationFractions (cmd [0], cmd [1], cmd [2], rho);
                h.Fill (j4, 0.8, 0.8, 0.5, true, true);
                h.Fill (j16, 0.8, 0.8, 0.5, true, true, rho);
                a4 = j4.Authority ();
                a16 = j16.Authority ();
                // The same demand, near each one's authority: 0.9 x along +x and +z.
                double worst4 = 0, worst16 = 0, wantForce = 0, got4 = 0, got16 = 0;
                foreach (var dir in new[] { new double[] { 1, 0 }, new double[] { 0, 1 }, new double[] { -1, 0 }, new double[] { 0, -1 } }) {
                    double fx, fz;
                    j4.Solve (dir [0] * 0.9 * a4 * HeavyModel.Mass, dir [1] * 0.9 * a4 * HeavyModel.Mass);
                    double modelForce4 = j4.Force [0] * dir [0] + j4.Force [1] * dir [1];
                    double r4 = RealizedResidual (h, j4, cmd [0], cmd [1], cmd [2], out fx, out fz);
                    worst4 = Math.Max (worst4, r4 / Math.Max (1.0, modelForce4 * HeavyModel.L));
                    got4 += (fx * dir [0] + fz * dir [1]) / Math.Max (1e-9, modelForce4);
                    j16.Solve (dir [0] * 0.9 * a16 * HeavyModel.Mass, dir [1] * 0.9 * a16 * HeavyModel.Mass);
                    double modelForce16 = j16.Force [0] * dir [0] + j16.Force [1] * dir [1];
                    double r16 = RealizedResidual (h, j16, cmd [0], cmd [1], cmd [2], out fx, out fz);
                    worst16 = Math.Max (worst16, r16 / Math.Max (1.0, modelForce16 * HeavyModel.L));
                    got16 += (fx * dir [0] + fz * dir [1]) / Math.Max (1e-9, modelForce16);
                    wantForce += 1;
                }
                Console.WriteLine ("  honest: heavy at 0.08, rotation (" + cmd [0] + ", " + cmd [1] + ", " + cmd [2] +
                    "): authority bit 4 " + a4.ToString ("F4") + " / bit 16 " + a16.ToString ("F4") +
                    " m/s^2; realized torque left / (model force x L): bit 4 " + worst4.ToString ("F3") + ", bit 16 " +
                    worst16.ToString ("F4") + "; realized force / model: bit 4 " + (got4 / wantForce).ToString ("F3") +
                    ", bit 16 " + (got16 / wantForce).ToString ("F3"));
                Check (a16 < a4 && a16 > 0.0, "rotation " + cmd [1] + ": bit 16 announces less than bit 4, still > 0");
                Check (worst4 > 0.2, "rotation: bit 4's solution leaves > 20 % of its force's torque unbalanced in the plant (" +
                    worst4.ToString ("F3") + ")");
                Check (worst16 < 0.02, "rotation: bit 16's solution is balanced in the plant (" + worst16.ToString ("F4") + ")");
            }
        }

        static void TestMeasuredShare ()
        {
            double num = 0, den = 0;
            var c = new double[] { 0.5, -0.2, 0.0, 0.3 };
            var real = new double[] { 0.5, -0.2, 0.0, 0.3 };
            for (int k = 0; k < 200; k++)
                AttitudeTranslation.AccumulateShare (real, c, 4, 0.02, ref num, ref den);
            Check (Near (num / den, 1.0, 1e-12) && Near (AttitudeTranslation.MeasuredFactor (num, den, 4 * 0.0025), 1.0, 1e-12),
                "share: gimbals at the common, 1");
            num = den = 0;
            var rng = new Random (5);
            for (int k = 0; k < 2000; k++) {
                for (int q = 0; q < 4; q++)
                    real [q] = 0.15 * c [q] + 0.3 * (rng.NextDouble () - 0.5);
                AttitudeTranslation.AccumulateShare (real, c, 4, 0.02, ref num, ref den);
            }
            Check (Math.Abs (num / den - 0.15) < 0.05, "share: attitude part cancelling 85 % plus noise, ~0.15 (" +
                (num / den).ToString ("F3") + ")");
            double before = num / den, d0 = den;
            Check (!AttitudeTranslation.AccumulateShare (real, new double[4], 4, 0.02, ref num, ref den) &&
                num / den == before && den == d0, "share: a step without common changes nothing");
            Check (!AttitudeTranslation.AccumulateShare (new[] { double.NaN, 0, 0, 0 }, c, 4, 0.02, ref num, ref den),
                "share: not finite, nothing");
            Check (AttitudeTranslation.MeasuredFactor (0.1, 1e-4, 0.01) == 1.0, "factor: no evidence, 1");
            Check (AttitudeTranslation.MeasuredFactor (-0.3, 1.0, 0.01) == 0.0 && AttitudeTranslation.MeasuredFactor (2.0, 1.0, 0.01) == 1.0,
                "factor: bounded to [0, 1]");
            Check (AttitudeTranslation.MeasuredFactor (double.NaN, 1.0, 0.01) == 1.0, "factor: NaN, 1");
        }

        sealed class SharedHoverResult
        {
            internal double Accel, Announced, Share, MaxTiltDeg, MaxRcs;
            internal int Excluded, RollDropped;
            internal bool AllComputed = true;
        }

        /// <summary>
        /// HeavyHover's mode-7 sequence through the unchanged law, with the RCS of the PLANT at
        /// nozzle level (HeavyModel.RcsPlant: rotation and translation share each nozzle, bounded
        /// at 1, force AND torque of every nozzle, the rotation's included) and an external ROLL
        /// command (the AutoPilot's, which the law does not write in pitch/yaw-only arming):
        /// rollPeriod 0 = constant rollCmd, else a square wave +-rollCmd of that period (steps).
        /// headroom: bit 16 (rotation fractions of the applied rotation, held 0.5 s, in the joint
        /// columns). Returns the lateral acceleration delivered (last 100 steps), the one the joint
        /// allocation announced (its Force / m, times the realized share), and the measured share
        /// of the common the gimbals hold (AccumulateShare).
        /// </summary>
        static SharedHoverResult HeavySharedHover (double throttle, bool headroom, double rollCmd, int rollPeriod,
            int steps, double demandZ)
        {
            var h = new HeavyModel (throttle);
            var p = new TranslationParams ();
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            var j = new JointTranslation ();
            const int E = HeavyModel.Engines;
            const double dt = 0.02;
            double[] inertia = { 5.4e7, 8.0e5, 5.4e7 };
            double f = AttitudeTranslation.RampFactor (true, 8.0, dt);
            int nG = 2 * E, n = nG + 3 + 2;
            inp.Resize (n);
            inp.Dt = dt;
            for (int a = 0; a < 3; a++)
                inp.Inertia [a] = inertia [a];
            double[] rcsRot = { h.RcsRotationColumn (0), h.RcsRotationColumn (1), h.RcsRotationColumn (2) };
            var theta = new double[3];
            var omega = new double[3];
            var omegaPrev = new double[3];
            bool havePrev = false;
            var uReal = new double[nG];
            var uCmd = new double[nG];
            var common = new double[nG];
            var commonReal = new double[nG];
            var commonNext = new double[nG];
            var slope = new double[E];
            var tau = new double[E];
            var margin = new double[E];
            for (int i = 0; i < E; i++) {
                slope [i] = h.SlopeEach;
                tau [i] = throttle;
                margin [i] = AttitudeTranslation.ThrottleMargin (throttle, 0, 1);
            }
            var rotCmd = new double[3];
            var linCmd = new double[2];
            var rotApplied = new double[3];
            var linApplied = new double[2];
            var rho = new double[h.RcsCount];
            var hold = new double[h.RcsCount];
            double dffPrevP = 0, dffPrevY = 0;
            var gap = new double[3];
            var delta = new double[E];
            var gx = new double[3];
            var gy = new double[3];
            var fx = new double[3];
            var fy = new double[3];
            var rf = new double[3];
            var rt = new double[3];
            var res = new SharedHoverResult ();
            double rollHeld = 0.0;
            double accelSum = 0, annSum = 0, shareNum = 0, shareDen = 0;
            int count = 0;
            bool committed = false;
            for (int step = 0; step < steps; step++) {
                var thrust = new double[E];
                double thrustSum = 0;
                for (int i = 0; i < E; i++) {
                    thrust [i] = slope [i] * tau [i];
                    thrustSum += thrust [i];
                }
                // Bit 16: the rotation the step finds applied (law pitch/yaw, external roll), held.
                if (headroom) {
                    h.RotationFractions (rotApplied [0], rotApplied [1], rotApplied [2], rho);
                    AttitudeTranslation.HoldFractions (h.RcsCount, rho, hold, dt / p.RcsHoldSeconds);
                }
                h.Fill (j, p.FracGimbal, p.FracDiff, p.FracRcs, true, true, headroom ? hold : null);
                for (int k = 0; k < 4; k++)
                    j.K [k] = 0.0;
                for (int k = 0; k < 6; k++)
                    j.Kt [k] = 0.0;
                var torque = new double[3];
                for (int i = 0; i < E; i++) {
                    EngineColumns (h.X [i], -HeavyModel.L, h.Z [i], -HeavyModel.L, thrust [i], gx, gy, fx, fy);
                    for (int r = 0; r < 3; r++) {
                        inp.B [r * n + 2 * i] = gx [r];
                        inp.B [r * n + 2 * i + 1] = gy [r];
                        torque [r] += gx [r] * uReal [2 * i] + gy [r] * uReal [2 * i + 1];
                        torque [r] += thrust [i] * h.Tpn [3 * i + r];
                        j.Kt [2 * r] += gx [r];
                        j.Kt [2 * r + 1] += gy [r];
                    }
                    j.K [0] += fx [0];
                    j.K [1] += fy [0];
                    j.K [2] += fx [2];
                    j.K [3] += fy [2];
                }
                j.Prepare ();
                // Measured RCS torque: the plant's (thrustForces), rotation and translation together.
                h.RcsPlant (rotApplied [0], rotApplied [1], rotApplied [2], linApplied [0], linApplied [1], rf, rt);
                for (int r = 0; r < 3; r++)
                    torque [r] += rt [r];
                for (int a = 0; a < 3; a++) {
                    inp.Omega [a] = omega [a];
                    inp.OmegaDotMeasured [a] = havePrev ? (omega [a] - omegaPrev [a]) / dt : 0.0;
                    inp.Torque [a] = torque [a];
                    inp.OmegaFf [a] = 0;
                    inp.OmegaDotFf [a] = 0;
                }
                inp.OmegaDotValid = havePrev;
                havePrev = true;
                Array.Copy (omega, omegaPrev, 3);
                inp.ReferenceValid = AttitudeMath.PointingError (theta [2], 1.0, -theta [0], inp.Error);
                inp.OmegaMax = 0.2;
                for (int a = 0; a < 3; a++)
                    inp.OmegaMaxAxis [a] = 0.2;
                // Bit 32's measurement: realized deflection against the previous step's realized
                // common, summed over the last 100 steps.
                if (committed && step >= steps - 100) {
                    for (int k = 0; k < nG; k++) {
                        shareNum += uReal [k] * commonReal [k];
                        shareDen += commonReal [k] * commonReal [k];
                    }
                }
                for (int k = 0; k < nG; k++) {
                    inp.U0 [k] = uReal [k];
                    inp.UMin [k] = -HeavyModel.RangeDeg;
                    inp.UMax [k] = HeavyModel.RangeDeg;
                    inp.UPref [k] = 0;
                    inp.Priority [k] = 1;
                }
                for (int a = 0; a < 3; a++) {
                    int c = nG + a;
                    for (int r = 0; r < 3; r++)
                        inp.B [r * n + c] = r == a ? rcsRot [a] : 0.0;
                    inp.U0 [c] = rotApplied [a];
                    inp.UMin [c] = -1;
                    inp.UMax [c] = 1;
                    inp.UPref [c] = 0;
                    inp.Priority [c] = 3;
                    // Bit 16: the law's RCS pitch / yaw left out (written 0) while ModuleRCS's
                    // combined rotation makes them nonlinear at the served point.
                    if (headroom && a != 1 && h.RotationNonlinear (rotApplied [0], rotApplied [1], rotApplied [2], a)) {
                        inp.UMin [c] = 0.0;
                        inp.UMax [c] = 0.0;
                        res.Excluded++;
                    }
                }
                double dRealP, dRealY;
                AttitudeTranslation.RealizedAmplitudes (E, slope, h.MotifP, h.MotifY, tau, out dRealP, out dRealY);
                for (int k = 0; k < nG; k++)
                    common [k] = 0;
                double rX = 0, rY = 0;
                double[] up = { theta [2], 1.0, -theta [0] };
                double[] demandBody = { 0.0, theta [0] * demandZ, demandZ };
                double ax, az, det;
                double annForce = 0;
                if (AttitudeTranslation.LateralFromGap (demandBody, up, thrustSum / HeavyModel.Mass,
                        p.MinProjectionDet, gap, out ax, out az, out det) &&
                        j.Solve (ax * HeavyModel.Mass, az * HeavyModel.Mass) && j.Converged) {
                    for (int i = 0; i < E; i++) {
                        common [2 * i] = j.X [0];
                        common [2 * i + 1] = j.X [1];
                    }
                    rX = j.X [2];
                    rY = j.X [3];
                }
                double mx = 0, mz = 0;
                for (int k = 0; k < nG; k++) {
                    commonNext [k] = AttitudeTranslation.Ramp (commonReal [k], common [k], f);
                    mx += inp.B [0 * n + k] * commonNext [k];
                    mz += inp.B [2 * n + k] * commonNext [k];
                }
                double kappa = AttitudeTranslation.RealizedShare (common, commonNext, nG);
                double wX = rX * kappa, wY = rY * kappa;
                mx += j.Rt [0] * wX + j.Rt [1] * wY;
                mz += j.Rt [4] * wX + j.Rt [5] * wY;
                // The announced (model) lateral force: tilt realized + RCS written.
                double lzModel = 0;
                for (int i = 0; i < E; i++)
                    lzModel += j.K [2] / E * commonNext [2 * i] + j.K [3] / E * commonNext [2 * i + 1];
                lzModel += j.Rf [2] * wX + j.Rf [3] * wY;
                annForce = lzModel;
                double dffP = 0, dffY = 0;
                AttitudeTranslation.CancelTorque (h.ColP, h.ColY, mx, mz, out dffP, out dffY);
                double hp, hy, share;
                AttitudeTranslation.BoxHalfWidths (E, h.MotifP, h.MotifY, margin, ref dffP, ref dffY, out hp, out hy, out share);
                for (int k = 0; k < nG; k++) {
                    double u0, lo, hi;
                    AttitudeTranslation.ShiftGimbal (uReal [k], commonReal [k], common [k], inp.UMin [k],
                        inp.UMax [k], out u0, out lo, out hi);
                    inp.U0 [k] = u0;
                    inp.UMin [k] = lo;
                    inp.UMax [k] = hi;
                }
                int jp = nG + 3, jyw = nG + 4;
                for (int r = 0; r < 3; r++) {
                    inp.B [r * n + jp] = h.ColP [r];
                    inp.B [r * n + jyw] = h.ColY [r];
                }
                double u0p, lop, hip, u0y, loy, hiy;
                AttitudeTranslation.DiffBounds (dRealP, dffPrevP, dffP, hp, true, out u0p, out lop, out hip);
                AttitudeTranslation.DiffBounds (dRealY, dffPrevY, dffY, hy, true, out u0y, out loy, out hiy);
                inp.U0 [jp] = u0p;
                inp.UMin [jp] = lop;
                inp.UMax [jp] = hip;
                inp.U0 [jyw] = u0y;
                inp.UMin [jyw] = loy;
                inp.UMax [jyw] = hiy;
                inp.UPref [jp] = inp.UPref [jyw] = 0;
                inp.Priority [jp] = inp.Priority [jyw] = p.DiffPriority;
                // Bit 16 (ShareRcsWithAttitude): the law does not write the roll (the AutoPilot
                // does); while that roll is commanded, the gimbal and motif columns carry no roll.
                rollHeld = Math.Max (Math.Abs (rotApplied [1]), rollHeld - dt / p.RcsHoldSeconds);
                if (headroom && rollHeld > AttitudeTranslation.RollActiveThreshold) {
                    for (int k = 0; k < n; k++)
                        if (k < nG || k >= nG + 3)
                            inp.B [1 * n + k] = 0.0;
                    res.RollDropped++;
                }
                AttitudeMath.Step (g, inp, st, work, o);
                if (step > 0 && !o.Computed)
                    res.AllComputed = false;
                if (o.Computed) {
                    for (int k = 0; k < nG; k++)
                        uCmd [k] = Math.Max (-HeavyModel.RangeDeg, Math.Min (HeavyModel.RangeDeg,
                            common [k] + inp.U0 [k] + o.DeltaU [k]));
                    double dP = dffP + inp.U0 [jp] + o.DeltaU [jp];
                    double dY = dffY + inp.U0 [jyw] + o.DeltaU [jyw];
                    AttitudeTranslation.ThrottleDeltas (E, h.MotifP, h.MotifY, dP, dY, delta);
                    double kw = AttitudeTranslation.WriteScale (E, delta, throttle, 0, 1);
                    for (int i = 0; i < E; i++)
                        tau [i] = throttle + kw * delta [i];
                    rotCmd [0] = Math.Max (-1.0, Math.Min (1.0, inp.U0 [nG] + o.DeltaU [nG]));
                    rotCmd [2] = Math.Max (-1.0, Math.Min (1.0, inp.U0 [nG + 2] + o.DeltaU [nG + 2]));
                    linCmd [0] = wX;
                    linCmd [1] = wY;
                    res.MaxRcs = Math.Max (res.MaxRcs, Math.Max (Math.Abs (wX), Math.Abs (wY)));
                    Array.Copy (commonNext, commonReal, nG);
                    dffPrevP = dffP;
                    dffPrevY = dffY;
                    committed = true;
                }
                // The AutoPilot's roll (not written by the law).
                rotCmd [1] = rollPeriod > 0 && (step / rollPeriod) % 2 == 1 ? -rollCmd : rollCmd;
                // Physics: the RCS written this step acts in this step's physics (measured at the
                // next), gimbals at the stock ramp, engines instant.
                Array.Copy (rotCmd, rotApplied, 3);
                Array.Copy (linCmd, linApplied, 2);
                for (int k = 0; k < nG; k++)
                    uReal [k] = AttitudeTranslation.Ramp (uReal [k], uCmd [k], f);
                var trueTorque = new double[3];
                double latZ = 0, tSum = 0;
                for (int i = 0; i < E; i++) {
                    double ti = slope [i] * tau [i];
                    tSum += ti;
                    EngineColumns (h.X [i], -HeavyModel.L, h.Z [i], -HeavyModel.L, ti, gx, gy, fx, fy);
                    for (int r = 0; r < 3; r++)
                        trueTorque [r] += gx [r] * uReal [2 * i] + gy [r] * uReal [2 * i + 1] + ti * h.Tpn [3 * i + r];
                    latZ += fx [2] * uReal [2 * i] + fy [2] * uReal [2 * i + 1];
                }
                h.RcsPlant (rotApplied [0], rotApplied [1], rotApplied [2], linApplied [0], linApplied [1], rf, rt);
                for (int r = 0; r < 3; r++)
                    trueTorque [r] += rt [r];
                latZ += rf [2];
                for (int a = 0; a < 3; a++) {
                    omega [a] += trueTorque [a] / inertia [a] * dt;
                    theta [a] += omega [a] * dt;
                }
                double accelZ = (tSum * theta [0] + latZ) / HeavyModel.Mass;
                res.MaxTiltDeg = Math.Max (res.MaxTiltDeg, Math.Sqrt (theta [0] * theta [0] + theta [2] * theta [2]) / Deg);
                if (step >= steps - 100) {
                    accelSum += accelZ;
                    annSum += annForce / HeavyModel.Mass;
                    count++;
                }
            }
            res.Accel = count > 0 ? accelSum / count : 0;
            res.Announced = count > 0 ? annSum / count : 0;
            res.Share = shareDen > 0 ? shareNum / shareDen : double.NaN;
            return res;
        }

        static void TestHeavySharedRcsHover ()
        {
            // Calm (no roll): the plant's nozzles agree with the model, bit 4 and bit 16 deliver.
            var calm4 = HeavySharedHover (0.08, false, 0.0, 0, 400, 0.2);
            var calm16 = HeavySharedHover (0.08, true, 0.0, 0, 400, 0.2);
            // The capture: the AutoPilot's roll saturated (constant, then reversing every 0.3 s).
            var roll4 = HeavySharedHover (0.08, false, 1.0, 0, 400, 0.2);
            var roll16 = HeavySharedHover (0.08, true, 1.0, 0, 400, 0.2);
            var sq4 = HeavySharedHover (0.08, false, 1.0, 15, 400, 0.2);
            var sq16 = HeavySharedHover (0.08, true, 1.0, 15, 400, 0.2);
            foreach (var x in new[] { Tuple.Create ("calm, bit 4", calm4), Tuple.Create ("calm, bit 16", calm16),
                    Tuple.Create ("roll +1, bit 4", roll4), Tuple.Create ("roll +1, bit 16", roll16),
                    Tuple.Create ("roll +-1 / 0.3 s, bit 4", sq4), Tuple.Create ("roll +-1 / 0.3 s, bit 16", sq16) })
                Console.WriteLine ("  honest: heavy shared-RCS hover 0.2 m/s^2, " + x.Item1 + ": delivered " +
                    x.Item2.Accel.ToString ("F4") + ", announced " + x.Item2.Announced.ToString ("F4") + " m/s^2, gimbals hold " +
                    x.Item2.Share.ToString ("F3") + " of the common, max tilt " + x.Item2.MaxTiltDeg.ToString ("F3") +
                    " deg, max RCS " + x.Item2.MaxRcs.ToString ("F3"));
            Check (calm4.AllComputed && calm16.AllComputed && roll4.AllComputed && roll16.AllComputed,
                "shared-RCS hover: the law runs every step");
            Check (Math.Abs (calm4.Accel - 0.2) < 0.03 && Math.Abs (calm16.Accel - 0.2) < 0.03,
                "calm: 0.2 m/s^2 delivered with bit 4 and bit 16 (" + calm4.Accel.ToString ("F4") + ", " + calm16.Accel.ToString ("F4") + ")");
            Check (calm4.Share > 0.8 && calm16.Share > 0.8, "calm: the gimbals hold the common");
            Check (calm16.Accel == calm4.Accel && calm16.Share == calm4.Share && calm16.RollDropped == 0 &&
                calm16.Excluded == 0, "calm: bit 16 = bit 4 bit for bit (no roll commanded, no RCS left out)");
            Check (roll4.Accel < 0.6 * roll4.Announced && sq4.Accel < 0.6 * sq4.Announced,
                "roll saturated, bit 4: delivers well under what it announces (" + roll4.Accel.ToString ("F4") + " of " +
                roll4.Announced.ToString ("F4") + ", " + sq4.Accel.ToString ("F4") + " of " + sq4.Announced.ToString ("F4") +
                ") -- the flight's symptom");
            Check (roll4.Share < 0.5 && sq4.Share < 0.5, "roll saturated, bit 4: the law cancels most of the common");
            Check (roll16.Accel > 0.8 * roll16.Announced && roll16.Accel > 2.0 * roll4.Accel,
                "roll +1, bit 16: delivers what it announces (" + roll16.Accel.ToString ("F4") + " of " +
                roll16.Announced.ToString ("F4") + ")");
            Check (sq16.Accel > 0.7 * sq16.Announced && sq16.Announced < 0.5 * sq4.Announced,
                "roll +-1, bit 16: announces less (the held roll keeps the RCS), delivers most of it (" +
                sq16.Accel.ToString ("F4") + " of " + sq16.Announced.ToString ("F4") + ")");
            Check (roll16.Share > 0.8 && sq16.Share > sq4.Share, "roll saturated, bit 16: the gimbals keep the common");
            Check (roll16.Excluded > 0 && calm16.Excluded == 0, "bit 16: the law's RCS pitch/yaw left out under the roll only");
        }

        // ------------------------------------------------- the free authority (option 256)

        static void TestFreeAuthorityRules ()
        {
            Check (AttitudeWrite.OptionFreeAuthority == 256 && (AttitudeWrite.OptionsSupported & 511) == 511 &&
                AttitudeWrite.ValidOptions (256) && AttitudeWrite.ValidOptions (511) &&
                !AttitudeWrite.ValidOptions (AttitudeWrite.OptionsSupported + 1),
                "option 256 supported (511 and above), the next bit refused");
            var r = new double[2];
            AttitudeTranslation.FreeGimbalRanges (0.0, -2.0, 2.0, 0.8, r, 0);
            Check (r [0] == 2.0 && r [1] == 2.0, "free ranges: no attitude part, the range itself (bit for bit)");
            AttitudeTranslation.FreeGimbalRanges (-1.5, -2.0, 2.0, 0.8, r, 0);
            Check (r [1] == 2.0 && Near (r [0], 0.5 / 0.8, 1e-15),
                "free ranges: the attitude part at -1.5 leaves the positive side whole, 0.5 deg on the negative one");
            AttitudeTranslation.FreeGimbalRanges (0.3, -2.0, 2.0, 0.8, r, 0);
            Check (r [0] == 2.0 && r [1] == 2.0, "free ranges: an attitude part inside the reserve takes nothing");
            AttitudeTranslation.FreeGimbalRanges (2.0, -2.0, 2.0, 0.8, r, 0);
            Check (r [1] == 0.0 && r [0] == 2.0, "free ranges: at its stop, nothing on that side");
            AttitudeTranslation.FreeGimbalRanges (2.3, -2.0, 2.0, 0.8, r, 0);
            Check (r [1] == 0.0, "free ranges: beyond its stop (realized common shifted), never below 0");
            double lo, hi;
            AttitudeTranslation.FreeDiffBounds (0.0, 0.1, 0.08, out lo, out hi);
            Check (lo == -1.0 && hi == 1.0, "free differential: no attitude part, -1 / 1 (bit for bit)");
            AttitudeTranslation.FreeDiffBounds (0.1, 0.1, 0.08, out lo, out hi);
            Check (hi == 0.0 && lo == -1.0, "free differential: the attitude at the half width, nothing on its side");
            AttitudeTranslation.FreeDiffBounds (-0.06, 0.1, 0.08, out lo, out hi);
            Check (hi == 1.0 && Near (lo, -0.5, 1e-12), "free differential: half the span left on the attitude's side");
            AttitudeTranslation.FreeDiffBounds (0.3, 0.1, 0.08, out lo, out hi);
            Check (hi == 0.0 && lo == -1.0, "free differential: 0 always inside");
            AttitudeTranslation.FreeDiffBounds (0.05, 0.1, 0.0, out lo, out hi);
            Check (lo == -1.0 && hi == 1.0, "free differential: no span, -1 / 1");
            int src;
            Check (AttitudeTranslation.FreeRatio (0.3, 0.15, out src) == 0.5 && src == AttitudeTranslation.FreeAlongGap,
                "free ratio: along free / along model");
            Check (AttitudeTranslation.FreeRatio (0.3, 0.31, out src) == 1.0, "free ratio: bounded at 1");
            Check (AttitudeTranslation.FreeRatio (0.0, 0.0, out src) == 1.0 && src == AttitudeTranslation.FreeSolveFailed,
                "free ratio: the model reaches nothing, 1");
            Check (AttitudeTranslation.FreeRatio (0.3, double.NaN, out src) == 1.0 && src == AttitudeTranslation.FreeSolveFailed,
                "free ratio: a failed solve, 1");
            Check (AttitudeTranslation.FreeFields.Length == 12 && AttitudeTranslation.FreeFields [0] == "free_authority_ms2",
                "free block: 12 named fields");
        }

        /// One case of the free authority on the heavy booster model: the attitude parts of its gimbals
        /// (2 per gimbal) and of the motifs; returns the ratio, with the reach along the gap.
        static double HeavyFree (HeavyModel h, JointTranslation j, JointTranslation f, double ux, double uz,
            double[] att, double attP, double attY, out double alongModel, out double alongFree, out int source)
        {
            int n = 2 * HeavyModel.Engines;
            var lo = new double[n];
            var hi = new double[n];
            for (int k = 0; k < n; k++) {
                lo [k] = -HeavyModel.RangeDeg;
                hi [k] = HeavyModel.RangeDeg;
            }
            return AttitudeTranslation.FreeAuthorityRatio (j, f, ux, uz, att, lo, hi, attP, attY, h.Half,
                out alongModel, out alongFree, out source);
        }

        static void TestFreeAuthorityHeavy ()
        {
            var h = new HeavyModel (0.08);
            var j = new JointTranslation ();
            var f = new JointTranslation ();
            Check (h.Fill (j, 0.8, 0.8, 0.5, true, true), "free, heavy: joint prepared");
            double model = j.Authority ();
            int n = 2 * HeavyModel.Engines;
            var zero = new double[n];
            // 1. No attitude part: the free reach IS the model's, bit for bit, along any direction.
            var rng = new Random (246);
            bool same = true;
            double am = 0, af = 0;
            int src = 0;
            for (int k = 0; k < 40; k++) {
                double ang = 2 * Math.PI * rng.NextDouble ();
                double ratio = HeavyFree (h, j, f, Math.Cos (ang), Math.Sin (ang), zero, 0.0, 0.0, out am, out af, out src);
                same &= ratio == 1.0 && am == af && src == AttitudeTranslation.FreeAlongGap && am > 0.0;
            }
            Check (same, "free, heavy: no attitude part, along_free == along_model and ratio 1 (40 directions)");
            // 2. The model joint's outputs are not touched by the free solves.
            j.Solve (0.1 * HeavyModel.Mass, 0.05 * HeavyModel.Mass);
            var x0 = (double[])j.X.Clone ();
            double f0 = j.Force [0], s0 = j.Scale;
            HeavyFree (h, j, f, 1.0, 0.0, zero, 0.0, 0.0, out am, out af, out src);
            bool untouched = j.Force [0] == f0 && j.Scale == s0;
            for (int k = 0; k < 6; k++)
                untouched &= j.X [k] == x0 [k];
            Check (untouched && j.Authority () == model, "free, heavy: the model joint untouched (outputs, authority)");
            // 3. The holes of the heavy booster's pad landings: the common along +x (gap +x), the law's
            // attitude part OPPOSED to it on every gimbal (the nose turning toward the demand pushes the tail
            // the other way), projected at -1.3 times the common: the measured share sample is 1 - 1.3 < 0
            // (the published factor 0), the free authority is the model's.
            j.Solve (0.2 * HeavyModel.Mass, 0.0);
            var common = new double[n];
            for (int g = 0; g < HeavyModel.Engines; g++) {
                common [2 * g] = j.X [0];
                common [2 * g + 1] = j.X [1];
            }
            var opposed = new double[n];
            var total = new double[n];
            for (int k = 0; k < n; k++) {
                opposed [k] = -1.3 * common [k];
                total [k] = common [k] + opposed [k];
            }
            double num = 0, den = 0;
            for (int s = 0; s < 100; s++)
                AttitudeTranslation.AccumulateShare (total, common, n, 0.02, ref num, ref den);
            double factor = AttitudeTranslation.MeasuredFactor (num, den, n * 0.0025);
            double rOpp = HeavyFree (h, j, f, 1.0, 0.0, opposed, 0.0, 0.0, out am, out af, out src);
            Console.WriteLine ("  free: heavy 0.08, attitude part -1.3 x the common: measured factor " +
                factor.ToString ("F3") + ", free ratio " + rOpp.ToString ("F3") + " (model " + model.ToString ("F4") +
                " m/s^2, along +x " + am.ToString ("F4") + ")");
            Check (factor == 0.0 && rOpp == 1.0, "free, heavy: the hole -- measured factor 0, free ratio 1");
            // 4. The attitude part ALONG the gap, growing to the stops: the ratio falls monotonically.
            double prev = 1.0;
            bool monotone = true;
            double atStop = double.NaN;
            for (int q = 0; q <= 20; q++) {
                double a = q * 0.1;
                var along = new double[n];
                for (int g = 0; g < HeavyModel.Engines; g++)
                    along [2 * g + 1] = j.X [1] >= 0.0 ? a : -a;
                double r = HeavyFree (h, j, f, 1.0, 0.0, along, 0.0, 0.0, out am, out af, out src);
                monotone &= r <= prev + 1e-12 && r >= 0.0 && src == AttitudeTranslation.FreeAlongGap;
                prev = r;
                if (q == 20)
                    atStop = r;
            }
            Console.WriteLine ("  free: heavy 0.08, attitude part along the gap up to the stops: ratio at the stops " +
                atStop.ToString ("F3"));
            Check (monotone, "free, heavy: the ratio falls monotonically as the attitude takes the room along the gap");
            Check (atStop < 1.0, "free, heavy: gimbals at their stops along the gap, the ratio below 1");
            // 5. The differential: its attitude part at the half width, on both motifs and either sign.
            double rdPlus = HeavyFree (h, j, f, 1.0, 0.0, zero, h.Half, h.Half, out am, out af, out src);
            double rdMinus = HeavyFree (h, j, f, 1.0, 0.0, zero, -h.Half, -h.Half, out am, out af, out src);
            Console.WriteLine ("  free: heavy 0.08, the motifs' attitude part at +-the half width: ratio " +
                rdPlus.ToString ("F3") + " / " + rdMinus.ToString ("F3"));
            Check (rdPlus <= 1.0 && rdMinus <= 1.0 && rdPlus >= 0.0 && rdMinus >= 0.0,
                "free, heavy: the differential's attitude part bounded in [0, 1]");
            // 6. Random attitude parts: always in [0, 1], and never above the model's reach.
            bool bounded = true;
            for (int k = 0; k < 200; k++) {
                var att = new double[n];
                for (int q = 0; q < n; q++)
                    att [q] = (2 * rng.NextDouble () - 1) * HeavyModel.RangeDeg;
                double ang = 2 * Math.PI * rng.NextDouble ();
                double r = HeavyFree (h, j, f, Math.Cos (ang), Math.Sin (ang), att,
                    (2 * rng.NextDouble () - 1) * h.Half, (2 * rng.NextDouble () - 1) * h.Half, out am, out af, out src);
                bounded &= r >= 0.0 && r <= 1.0 && (!(af > am) || r == 1.0);
            }
            Check (bounded, "free, heavy: 200 random attitude parts, ratio in [0, 1]");
        }

        // ------------------------------------------------- an earlier change golden vectors (the client's mirror)

        static string GD (double v)
        {
            return double.IsNaN (v) || double.IsInfinity (v) ? "null" : v.ToString ("R", CultureInfo.InvariantCulture);
        }

        static string GV (double[] v, int count)
        {
            var sb = new StringBuilder ("[");
            for (int i = 0; i < count; i++) {
                if (i > 0)
                    sb.Append (", ");
                sb.Append (GD (v [i]));
            }
            return sb.Append ("]").ToString ();
        }

        static void GoldenFreeCase (StringBuilder sb, string name, JointTranslation j, JointTranslation f, double ux,
            double uz, double[] att, double[] lo, double[] hi, double attP, double attY, double half, bool more)
        {
            int g = j.Gimbals;
            double authority = j.Authority ();
            var dirs = (double[])j.Directions.Clone ();
            double am, af;
            int src;
            double ratio = AttitudeTranslation.FreeAuthorityRatio (j, f, ux, uz, att, lo, hi, attP, attY, half,
                out am, out af, out src);
            var usable = new double[g];
            for (int k = 0; k < g; k++)
                usable [k] = j.GimbalUsable [k] ? 1.0 : 0.0;
            sb.Append ("    {\"name\": \"" + name + "\", \"in\": {\"mass\": " + GD (j.Mass) + ", \"k\": " + GV (j.K, 4) +
                ", \"kt\": " + GV (j.Kt, 6) + ", \"rf\": " + GV (j.Rf, 4) + ", \"rt\": " + GV (j.Rt, 6) + ", \"bd\": " +
                GV (j.Bd, 6) + ", \"phi_max\": " + GD (j.PhiMax) + ", \"rcs_max\": " + GD (j.RcsMax) + ", \"diff_max\": " +
                GD (j.DiffMax) + ", \"diff_weight\": " + GD (j.DiffWeight) + ", \"frac_gimbal\": " + GD (j.FracGimbal) +
                ", \"gimbals\": " + g + ", \"gimbal_usable\": " + GV (usable, g) + ", \"gimbal_axes\": " +
                GV (j.GimbalAxes, 4 * g) + ", \"gimbal_ranges\": " + GV (j.GimbalRanges, 4 * g) + ", \"ux\": " + GD (ux) +
                ", \"uz\": " + GD (uz) + ", \"attitude\": " + GV (att, 2 * g) + ", \"u_min\": " + GV (lo, 2 * g) +
                ", \"u_max\": " + GV (hi, 2 * g) + ", \"attitude_diff_p\": " + GD (attP) + ", \"attitude_diff_y\": " +
                GD (attY) + ", \"half_width\": " + GD (half) + "},\n");
            sb.Append ("     \"out\": {\"authority\": " + GD (authority) + ", \"directions\": " + GV (dirs, 4) +
                ", \"along_model\": " + GD (am) + ", \"along_free\": " + GD (af) + ", \"ratio\": " + GD (ratio) +
                ", \"source\": " + src + ", \"free_gimbal_ranges\": " + GV (f.GimbalRanges, 4 * g) + ", \"free_lower\": " +
                GV (f.Lower, 6) + ", \"free_upper\": " + GV (f.Upper, 6) + "}}" + (more ? "," : "") + "\n");
        }

        /// <summary>
        /// the golden vectors of the free authority (build/tests/golden/attitude_v1_golden_free.json), for
        /// the client's mirror (the client): the heavy booster model at throttle 0.08 and 0.3 with
        /// its gimbals' axes as they are and turned (as the heavy booster's local frames), attitude parts none,
        /// opposed to the gap, along it, at the stops and random, the motifs' attitude parts, and a booster
        /// without RCS translation (the barge's square). Deterministic.
        /// </summary>
        internal static string GoldenFreeJson ()
        {
            var sb = new StringBuilder ();
            sb.Append ("{\n  \"schema\": 1,\n  \"source\": \"krpc_bridge build/tests (AttitudeTranslation.cs, option 256: " +
                "FreeAuthorityRatio)\",\n  \"cases\": [\n");
            var rng = new Random (2460);
            var f = new JointTranslation ();
            int count = 0, total = 2 * 2 * 7 + 4;
            foreach (double throttle in new[] { 0.08, 0.3 }) {
                foreach (bool turned in new[] { false, true }) {
                    var h = new HeavyModel (throttle);
                    var j = new JointTranslation ();
                    h.Fill (j, 0.8, 0.8, 0.5, true, true);
                    if (turned)
                        for (int g = 0; g < HeavyModel.Engines; g++) {
                            double a = g * Math.PI / 4 + 0.3;
                            j.GimbalAxes [4 * g] = Math.Cos (a);
                            j.GimbalAxes [4 * g + 1] = -Math.Sin (a);
                            j.GimbalAxes [4 * g + 2] = Math.Sin (a);
                            j.GimbalAxes [4 * g + 3] = Math.Cos (a);
                        }
                    j.Prepare ();
                    int n = 2 * HeavyModel.Engines;
                    var lo = new double[n];
                    var hi = new double[n];
                    for (int k = 0; k < n; k++) {
                        lo [k] = -HeavyModel.RangeDeg;
                        hi [k] = HeavyModel.RangeDeg;
                    }
                    string tag = "heavy_" + throttle.ToString ("F2", CultureInfo.InvariantCulture) + (turned ? "_turned" : "");
                    for (int c = 0; c < 7; c++) {
                        double ang = 2 * Math.PI * rng.NextDouble ();
                        double ux = Math.Cos (ang), uz = Math.Sin (ang);
                        j.Solve (0.2 * HeavyModel.Mass * ux, 0.2 * HeavyModel.Mass * uz);
                        var att = new double[n];
                        double attP = 0.0, attY = 0.0;
                        string what;
                        for (int g = 0; g < HeavyModel.Engines; g++) {
                            double cx = j.X [0] * j.GimbalAxes [4 * g] + j.X [1] * j.GimbalAxes [4 * g + 1];
                            double cy = j.X [0] * j.GimbalAxes [4 * g + 2] + j.X [1] * j.GimbalAxes [4 * g + 3];
                            double sc = c == 1 ? -1.3 : (c == 2 ? 6.0 : (c == 3 ? 40.0 : 0.0));
                            att [2 * g] = sc * cx;
                            att [2 * g + 1] = sc * cy;
                        }
                        if (c == 0) {
                            what = "none";
                        } else if (c == 1) {
                            what = "opposed";
                        } else if (c == 2) {
                            what = "along";
                        } else if (c == 3) {
                            what = "stops";
                            for (int k = 0; k < n; k++)
                                att [k] = Math.Max (lo [k], Math.Min (hi [k], att [k]));
                        } else if (c == 4) {
                            what = "diff";
                            attP = 0.9 * h.Half;
                            attY = -1.2 * h.Half;
                        } else {
                            what = "random" + c;
                            for (int k = 0; k < n; k++)
                                att [k] = (2 * rng.NextDouble () - 1) * HeavyModel.RangeDeg;
                            attP = (2 * rng.NextDouble () - 1) * h.Half;
                            attY = (2 * rng.NextDouble () - 1) * h.Half;
                        }
                        GoldenFreeCase (sb, tag + "_" + what, j, f, ux, uz, att, lo, hi, attP, attY, h.Half, ++count < total);
                    }
                }
            }
            // A booster without RCS translation (the barge's 4 engines in a +-0.6 m square, 2 deg): the
            // differential and the tilt alone.
            var bx = new double[4];
            var bz = new double[4];
            Square (0.6, bx, bz);
            for (int c = 0; c < 4; c++) {
                var jb = BargeJoint (bx, bz);
                var att = new double[8];
                var lo = new double[8];
                var hi = new double[8];
                for (int k = 0; k < 8; k++) {
                    lo [k] = -2.0;
                    hi [k] = 2.0;
                    att [k] = c == 0 ? 0.0 : (2 * rng.NextDouble () - 1) * 2.0;
                }
                double ang = 2 * Math.PI * rng.NextDouble ();
                double half = jb.DiffMax / 0.8;
                double attP = c >= 2 ? (2 * rng.NextDouble () - 1) * half : 0.0;
                double attY = c >= 2 ? (2 * rng.NextDouble () - 1) * half : 0.0;
                GoldenFreeCase (sb, "barge_" + c, jb, f, Math.Cos (ang), Math.Sin (ang), att, lo, hi, attP, attY, half,
                    ++count < total);
            }
            sb.Append ("  ]\n}\n");
            return sb.ToString ();
        }

        /// The barge's joint allocation (Compare's columns at hover, no RCS translation), prepared.
        static JointTranslation BargeJoint (double[] x, double[] z)
        {
            int n = x.Length;
            double mass = 29300.0, tmax = 750e3, py = -5.96, qy = -3.8;
            double main = mass * 9.81 / (n * tmax);
            double thrust = mass * 9.81;
            var s = new double[n];
            var tpn = new double[3 * n];
            var mp = new double[n];
            var my = new double[n];
            var margin = new double[n];
            var j = new JointTranslation ();
            j.Resize (n);
            var cx = new double[3];
            var cy = new double[3];
            var fx = new double[3];
            var fy = new double[3];
            for (int i = 0; i < n; i++) {
                s [i] = tmax;
                var t3 = new double[3];
                Cross (new[] { x [i], py, z [i] }, new double[] { 0, 1, 0 }, t3);
                for (int r = 0; r < 3; r++)
                    tpn [3 * i + r] = t3 [r];
                margin [i] = AttitudeTranslation.ThrottleMargin (main, 0, 1);
                EngineColumns (x [i], py, z [i], qy, thrust / n, cx, cy, fx, fy);
                j.K [0] += fx [0];
                j.K [1] += fy [0];
                j.K [2] += fx [2];
                j.K [3] += fy [2];
                for (int r = 0; r < 3; r++) {
                    j.Kt [2 * r] += cx [r];
                    j.Kt [2 * r + 1] += cy [r];
                }
                j.GimbalUsable [i] = true;
                j.GimbalAxes [4 * i] = 1.0;
                j.GimbalAxes [4 * i + 3] = 1.0;
                for (int q = 0; q < 4; q++)
                    j.GimbalRanges [4 * i + q] = 2.0;
            }
            AttitudeTranslation.Motifs (n, s, x, z, 0.25, mp, my);
            var colP = new double[3];
            var colY = new double[3];
            AttitudeTranslation.MotifColumns (n, s, mp, my, tpn, colP, colY);
            double half = AttitudeTranslation.SymmetricHalfWidth (n, mp, my, margin);
            j.Mass = mass;
            for (int r = 0; r < 3; r++) {
                j.Bd [2 * r] = colP [r];
                j.Bd [2 * r + 1] = colY [r];
            }
            j.PhiMax = 0.8 * 2.0;
            j.RcsMax = 0.5;
            j.DiffMax = 0.8 * half;
            j.DiffWeight = 2.0;
            j.FracGimbal = 0.8;
            j.Prepare ();
            return j;
        }
    }
}
