// -----------------------------------------------------------------------------------
// (option 1024 of AttitudeOptionsV1, born closed): the roll pointed (AttitudeMath.RollError and
// AttitudeInput.RollTargetValid). Checks:
//   * option 1024 is a new bit, no older option holds it;
//   * the roll error's geometry: the roof on the target, a quarter turn each way, a target along the nose, and its
//     sign against a rotation about +y (a positive rate reduces a positive error);
//   * born closed: without RollTargetValid the law is bit for bit the same whatever Error[1] holds (2 000 draws);
//   * with it, the roll's square-root reference on its own authority and its own AutoPilot limit (or the scalar);
//   * a closed loop on the roll of the super hope (RCS + wheels, 1.2e7 kg.m^2 about the nose, one sixth of it about
//     the roll axis): 40 deg aligned under 0.5 deg, no overshoot beyond 2 deg; with the RCS column overestimated x20
//     (the M3 measure: x15-40 on the roll) the error still converges, slower, but the profile brakes on the modelled
//     authority and overshoots (reported, the ground check measures the column before arming); without the target the
//     roll is only damped.
// The golden vectors (golden/attitude_v1_golden_roll.json) are replayed by the client's mirror
// (the client, the client).
// -----------------------------------------------------------------------------------

using System;
using System.Globalization;
using System.Text;

namespace KRPC.Bridge.Model.Tests
{
    internal static class RollTargetTests
    {
        static int checks;
        static int failures;

        internal static int Run (out int checkCount)
        {
            TestBits ();
            TestGeometry ();
            TestBornClosed ();
            TestReference ();
            TestClosedLoop ();
            checkCount = checks;
            Console.WriteLine ("roll target checks: " + checks + ", failures: " + failures);
            return failures;
        }

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.Error.WriteLine ("FAIL (roll target): " + what);
            }
        }

        static void TestBits ()
        {
            Check (AttitudeWrite.OptionRollTarget == 1024 && (AttitudeWrite.OptionsSupported & 2047) == 2047,
                "option 1024, supported 2047 and above (mirrored by the client)");
            for (int o = 0; o < 1024; o++)
                Check ((o & AttitudeWrite.OptionRollTarget) == 0, "no older option holds bit 1024 (" + o + ")");
        }

        static void TestGeometry ()
        {
            Check (AttitudeMath.RollError (0, 0, -1) == 0.0, "the roof on the target: 0");
            Check (AttitudeMath.RollError (0, 0.7, -1) == 0.0, "only the part across the nose counts");
            Check (Math.Abs (AttitudeMath.RollError (1, 0, 0) + Math.PI / 2) < 1e-15, "target to the right: -pi/2");
            Check (Math.Abs (AttitudeMath.RollError (-1, 0, 0) - Math.PI / 2) < 1e-15, "target to the left: +pi/2");
            Check (Math.Abs (Math.Abs (AttitudeMath.RollError (0, 0, 1)) - Math.PI) < 1e-15, "target below: pi");
            Check (double.IsNaN (AttitudeMath.RollError (0, 1, 0)) && double.IsNaN (AttitudeMath.RollError (1e-12, 1, 0)),
                "a target along the nose: NaN");
            // The ground test of 15/10: an up vector along the nose reaches the body 1e-7 across it (float transforms):
            // no roll target, not a random one; the smallest usable part across is RollTargetMinAcross.
            Check (double.IsNaN (AttitudeMath.RollError (1e-7, 1, -1e-7)) &&
                double.IsNaN (AttitudeMath.RollError (0.9e-3, 1, 0)) && !double.IsNaN (AttitudeMath.RollError (1.1e-3, 1, 0)),
                "a target within RollTargetMinAcross of the nose: NaN");
            Check (double.IsNaN (AttitudeMath.RollError (double.NaN, 0, -1)), "non finite: NaN");
            // the target is fixed in the world; the body turns by +d about +y: in the body the target turns by -d
            var rng = new Random (2526);
            for (int k = 0; k < 500; k++) {
                double e = (rng.NextDouble () - 0.5) * 2.5, d = 0.01 * (rng.NextDouble () - 0.2);
                // target = R_y(e) roof, roof = (0, 0, -1): R_y(t) v = (x cos t + z sin t, y, -x sin t + z cos t)
                double tx = -Math.Sin (e), tz = -Math.Cos (e), ty = 0.3 * (rng.NextDouble () - 0.5);
                double e0 = AttitudeMath.RollError (tx, ty, tz);
                double cx = tx * Math.Cos (-d) + tz * Math.Sin (-d), cz = -tx * Math.Sin (-d) + tz * Math.Cos (-d);
                double e1 = AttitudeMath.RollError (cx, ty, cz);
                Check (Math.Abs (e0 - e) < 1e-12 && Math.Abs (e1 - (e - d)) < 1e-12,
                    "a rate about +y reduces the error: " + e0 + " " + e1 + " " + e + " " + d);
            }
        }

        static void Fill (AttitudeInput inp, Random rng, double i0, double i1, double i2)
        {
            inp.Resize (3);
            inp.Dt = 0.02;
            inp.Inertia [0] = i0;
            inp.Inertia [1] = i1;
            inp.Inertia [2] = i2;
            for (int a = 0; a < 3; a++) {
                inp.Omega [a] = 0.05 * (rng.NextDouble () - 0.5);
                inp.OmegaDotMeasured [a] = 0.02 * (rng.NextDouble () - 0.5);
                inp.Torque [a] = 1e4 * (rng.NextDouble () - 0.5);
                inp.OmegaFf [a] = 0.0;
                inp.OmegaDotFf [a] = 0.0;
                inp.OmegaMaxAxis [a] = 0.0;
            }
            for (int j = 0; j < 3; j++) {
                for (int a = 0; a < 3; a++)
                    inp.B [a * 3 + j] = a == j ? 4e5 * (0.5 + rng.NextDouble ()) : 2e4 * (rng.NextDouble () - 0.5);
                inp.U0 [j] = 0.2 * (rng.NextDouble () - 0.5);
                inp.UMin [j] = -1.0;
                inp.UMax [j] = 1.0;
                inp.UPref [j] = 0.0;
                inp.Priority [j] = 3.0;
            }
            inp.OmegaDotValid = true;
            inp.ReferenceValid = true;
            inp.OmegaMax = 0.3;
            AttitudeMath.PointingError (0.3 * (rng.NextDouble () - 0.5), 1.0, 0.3 * (rng.NextDouble () - 0.5), inp.Error);
        }

        static void TestBornClosed ()
        {
            var g = new AttitudeGains ();
            for (int k = 0; k < 2000; k++) {
                var rng = new Random (4000 + k);
                var a = new AttitudeInput ();
                Fill (a, rng, 1.2e7, 2e6, 1.2e7);
                var b = new AttitudeInput ();
                Fill (b, new Random (4000 + k), 1.2e7, 2e6, 1.2e7);
                b.Error [1] = 0.4 * (rng.NextDouble () - 0.5);
                var oa = new AttitudeOutput ();
                var ob = new AttitudeOutput ();
                AttitudeMath.Step (g, a, new AttitudeLawState (), new AttitudeWork (), oa);
                AttitudeMath.Step (g, b, new AttitudeLawState (), new AttitudeWork (), ob);
                bool same = oa.Computed == ob.Computed;
                for (int i = 0; i < 3 && same; i++)
                    same = oa.Nu [i] == ob.Nu [i] && oa.OmegaRef [i] == ob.OmegaRef [i] && oa.DeltaU [i] == ob.DeltaU [i]
                        && oa.TorqueAllocTarget [i] == ob.TorqueAllocTarget [i];
                Check (same, "without RollTargetValid, Error[1] changes nothing (" + k + ")");
            }
        }

        static void TestReference ()
        {
            var g = new AttitudeGains ();
            var rng = new Random (2527);
            for (int k = 0; k < 200; k++) {
                var inp = new AttitudeInput ();
                Fill (inp, rng, 1.2e7, 2e6, 1.2e7);
                inp.Error [1] = 1.2 * (rng.NextDouble () - 0.5);
                inp.RollTargetValid = true;
                if (k % 2 == 0)
                    inp.OmegaMaxAxis [1] = 0.05 + 0.1 * rng.NextDouble ();
                var o = new AttitudeOutput ();
                AttitudeMath.Step (g, inp, new AttitudeLawState (), new AttitudeWork (), o);
                // the first tick primes the filters: no reference law yet
                Check (!o.Computed || o.OmegaSqrt [1] != 0.0, "primed or computed");
                var st = new AttitudeLawState ();
                AttitudeMath.Step (g, inp, st, new AttitudeWork (), o);
                AttitudeMath.Step (g, inp, st, new AttitudeWork (), o);
                double wmax = inp.OmegaMaxAxis [1] > 0.0 ? inp.OmegaMaxAxis [1] : inp.OmegaMax;
                double expected = AttitudeMath.SqrtReference (inp.Error [1], o.Alpha [1], g.Eta, g.KLin, wmax);
                Check (o.Computed && o.OmegaSqrt [1] == expected, "the roll's square-root reference: " + o.OmegaSqrt [1] +
                    " " + expected);
                Check (o.Computed && Math.Sign (o.OmegaRef [1] + o.OmegaPch [1]) == Math.Sign (inp.Error [1]) ||
                    Math.Abs (inp.Error [1]) < 1e-9, "the roll reference toward the target");
            }
        }

        /// <summary>
        /// The roll of the super hope (study 4: RCS + wheels 0.79 MN.m pitch/yaw; the roll column of the RCS modelled,
        /// its true effect `factor` of it), one axis driven through the law, the realized torque fed back.
        /// </summary>
        static double[] RollLoop (bool target, double factor, double e0, int steps, out double overshoot, out bool monotonic)
        {
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            inp.Resize (3);
            double iy = 2.0e6, by = 3.0e5;
            double e = e0, w = 0.0, u = 0.0, prevWdot = 0.0;
            var trace = new double[steps];
            overshoot = 0.0;
            monotonic = true;
            double prevAbs = Math.Abs (e);
            for (int t = 0; t < steps; t++) {
                inp.Dt = 0.02;
                inp.Inertia [0] = 1.2e7;
                inp.Inertia [1] = iy;
                inp.Inertia [2] = 1.2e7;
                inp.Omega [0] = 0.0;
                inp.Omega [1] = w;
                inp.Omega [2] = 0.0;
                inp.OmegaDotMeasured [0] = 0.0;
                inp.OmegaDotMeasured [1] = prevWdot;
                inp.OmegaDotMeasured [2] = 0.0;
                inp.OmegaDotValid = t > 0;
                inp.Torque [0] = 0.0;
                inp.Torque [1] = factor * by * u;
                inp.Torque [2] = 0.0;
                for (int j = 0; j < 3; j++) {
                    for (int a = 0; a < 3; a++)
                        inp.B [a * 3 + j] = a == j ? (a == 1 ? by : 7.9e5) : 0.0;
                    inp.U0 [j] = j == 1 ? u : 0.0;
                    inp.UMin [j] = -1.0;
                    inp.UMax [j] = 1.0;
                    inp.UPref [j] = 0.0;
                    inp.Priority [j] = 3.0;
                    inp.OmegaFf [j] = 0.0;
                    inp.OmegaDotFf [j] = 0.0;
                    inp.OmegaMaxAxis [j] = 0.0;
                }
                inp.OmegaMax = 0.2;
                inp.ReferenceValid = true;
                inp.Error [0] = 0.0;
                inp.Error [1] = target ? e : 0.0;
                inp.Error [2] = 0.0;
                inp.RollTargetValid = target;
                AttitudeMath.Step (g, inp, st, work, o);
                if (o.Computed)
                    u = Math.Max (-1.0, Math.Min (1.0, inp.U0 [1] + o.DeltaU [1]));
                double wdot = factor * by * u / iy;
                prevWdot = wdot;
                w += wdot * 0.02;
                e -= w * 0.02;
                trace [t] = e;
                if (Math.Sign (e) != Math.Sign (e0) && Math.Abs (e) > overshoot)
                    overshoot = Math.Abs (e);
                if (Math.Sign (e) == Math.Sign (e0) && Math.Abs (e) > prevAbs + 1e-9)
                    monotonic = false;
                prevAbs = Math.Abs (e);
            }
            return trace;
        }

        static void TestClosedLoop ()
        {
            double deg = Math.PI / 180.0, over, slowOvershoot;
            bool mono;
            var aligned = RollLoop (true, 1.0, 40 * deg, 1500, out over, out mono);
            Check (Math.Abs (aligned [aligned.Length - 1]) < 0.5 * deg && over < 2.0 * deg,
                "40 deg aligned: " + aligned [aligned.Length - 1] / deg + " deg, overshoot " + over / deg);
            int at = 0;
            while (at < aligned.Length && Math.Abs (aligned [at]) > 0.5 * deg)
                at++;
            var slow = RollLoop (true, 1.0 / 20.0, 40 * deg, 6000, out over, out mono);
            // the effect 20 times smaller than modelled: the incremental law still converges (0 < b / b_hat < 2), slower;
            // but the square-root profile brakes on the modelled authority, 20 times too large: it overshoots (measured
            // here, reported: the ground check of K6 measures the roll column before arming, the client)
            slowOvershoot = over;
            Check (Math.Abs (slow [slow.Length - 1]) < 0.5 * deg && Math.Abs (slow [at]) > 0.5 * deg && over < 40 * deg,
                "RCS roll column overestimated x20: converges, slower, never diverges: " + slow [at] / deg + " deg when " +
                "the nominal is aligned, " + slow [slow.Length - 1] / deg + " deg at the end, overshoot " + over / deg);
            var damped = RollLoop (false, 1.0, 40 * deg, 1500, out over, out mono);
            Check (Math.Abs (damped [damped.Length - 1] - 40 * deg) < 1e-9, "without the target the roll is only damped");
            Console.WriteLine ("roll target: 40 deg under 0.5 deg after " + (at * 0.02).ToString ("F2", CultureInfo.InvariantCulture) +
                " s; with the RCS roll column x20 over: " + (slow [slow.Length - 1] / deg).ToString ("F1", CultureInfo.InvariantCulture) +
                " deg after 120 s, overshoot " + (slowOvershoot / deg).ToString ("F1", CultureInfo.InvariantCulture) + " deg");
        }
    }
}
