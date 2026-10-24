// -----------------------------------------------------------------------------------
// (option 2048 of AttitudeOptionsV1, born closed): the reference ring (src/Plugins/Model/ReferenceRing.cs).
// Checks:
//   * option 2048 is a new bit, no older option holds it; the stride, the capacity, the channels;
//   * the validation: every refusal code on its own case, the channels found, the boundaries (2 and 64 samples,
//     a throttle of 0 and 1, two directions just under and at 90 deg);
//   * the storage: directions normalized, the other channels as given;
//   * the sampling: nothing outside [ut_first, ut_last] nor before two samples; every node found exactly; the
//     lateral and the throttle linear; the direction the normalized linear combination, unit, on the arc between
//     its two samples and moving forward along it; the first segment at a shared node;
//   * a ring of a turning nose (40 deg/s sampled at 10 Hz, served at 50 Hz): the served nose stays within
//     the chord's sag of the true one (1 - cos(2 deg) of the arc's half angle);
//   * K5 under the ring: a ramp order with a ring throttle decides the ring's value (motif 18), clamped, under its
//     validity and an earlier change's guard; without one (NaN), exactly u0 + slope (t - t0).
// The golden vectors (golden/attitude_v1_golden_ring.json) are replayed by the client's mirror
// (the client, the client).
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace KRPC.Bridge.Model.Tests
{
    internal static class ReferenceRingTests
    {
        static int checks;
        static int failures;

        internal static int Run (out int checkCount)
        {
            TestBits ();
            TestValidate ();
            TestStoreAndSample ();
            TestTurningNose ();
            TestThrottleUnderRing ();
            checkCount = checks;
            Console.WriteLine ("reference ring checks: " + checks + ", failures: " + failures);
            return failures;
        }

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.Error.WriteLine ("FAIL (reference ring): " + what);
            }
        }

        static List<double> Ring (params double[][] samples)
        {
            var l = new List<double> ();
            foreach (var s in samples)
                l.AddRange (s);
            return l;
        }

        static double[] S (double ut, double dx, double dy, double dz, double ax, double ay, double az, double u)
        {
            return new[] { ut, dx, dy, dz, ax, ay, az, u };
        }

        // ------------------------------------------------------------------ bits

        static void TestBits ()
        {
            Check (AttitudeWrite.OptionReferenceRing == 2048 && (AttitudeWrite.OptionsSupported & 4095) == 4095,
                "option 2048 supported, with every older one");
            Check ((AttitudeWrite.OptionReferenceRing & (AttitudeWrite.OptionReferenceRing - 1)) == 0 &&
                (AttitudeWrite.OptionReferenceRing & 2047) == 0, "a single new bit");
            Check (ReferenceRing.Stride == 8 && ReferenceRing.Capacity == 64 && ReferenceRing.ChannelDirection == 1 &&
                ReferenceRing.ChannelLateral == 2 && ReferenceRing.ChannelThrottle == 4, "stride, capacity, channels");
            Check (MainThrottle.MotifRing == 18 && MainThrottle.MotifDeclenche == 17, "motif 18 after K5's last");
        }

        // ------------------------------------------------------------------ validation

        static int V (List<double> s, out int count, out int channels)
        {
            return ReferenceRing.Validate (s, out count, out channels);
        }

        static void TestValidate ()
        {
            double nan = double.NaN;
            int n, c;
            Check (V (Ring (S (1, 0, 1, 0, 0, 0, 0, 0.5), S (2, 0.1, 1, 0, 1, 0, 0, 0.6)), out n, out c) == 0 && n == 2 &&
                c == 7, "two full samples: valid, three channels");
            Check (V (Ring (S (1, 0, 1, 0, nan, nan, nan, nan), S (2, 0, 1, 0.1, nan, nan, nan, nan)), out n, out c) == 0 &&
                c == 1, "direction only");
            Check (V (Ring (S (1, nan, nan, nan, 1, 2, 3, nan), S (2, nan, nan, nan, 1, 2, 3, nan)), out n, out c) == 0 &&
                c == 2, "lateral only");
            Check (V (Ring (S (1, nan, nan, nan, nan, nan, nan, 0), S (2, nan, nan, nan, nan, nan, nan, 1)), out n, out c) == 0 &&
                c == 4, "throttle only, 0 and 1 accepted");
            Check (V (null, out n, out c) == ReferenceRing.BadLength, "null");
            var bad = Ring (S (1, 0, 1, 0, 0, 0, 0, 0.5), S (2, 0, 1, 0, 0, 0, 0, 0.5));
            bad.RemoveAt (bad.Count - 1);
            Check (V (bad, out n, out c) == ReferenceRing.BadLength, "not a multiple of the stride");
            Check (V (Ring (S (1, 0, 1, 0, 0, 0, 0, 0.5)), out n, out c) == ReferenceRing.BadCount, "one sample");
            var full = new List<double> ();
            for (int k = 0; k < 65; k++)
                full.AddRange (S (k, 0, 1, 0, 0, 0, 0, 0.5));
            Check (V (full, out n, out c) == ReferenceRing.BadCount, "65 samples");
            full.RemoveRange (64 * 8, 8);
            Check (V (full, out n, out c) == 0 && n == 64, "64 samples");
            Check (V (Ring (S (nan, 0, 1, 0, 0, 0, 0, 0.5), S (2, 0, 1, 0, 0, 0, 0, 0.5)), out n, out c) == ReferenceRing.BadUt,
                "ut NaN");
            Check (V (Ring (S (1, 0, 1, 0, 0, 0, 0, 0.5), S (double.PositiveInfinity, 0, 1, 0, 0, 0, 0, 0.5)), out n, out c) ==
                ReferenceRing.BadUt, "ut infinite");
            Check (V (Ring (S (2, 0, 1, 0, 0, 0, 0, 0.5), S (2, 0, 1, 0, 0, 0, 0, 0.5)), out n, out c) ==
                ReferenceRing.NotIncreasing, "ut equal");
            Check (V (Ring (S (2, 0, 1, 0, 0, 0, 0, 0.5), S (1, 0, 1, 0, 0, 0, 0, 0.5)), out n, out c) ==
                ReferenceRing.NotIncreasing, "ut decreasing");
            Check (V (Ring (S (1, 0, 1, nan, 0, 0, 0, 0.5), S (2, 0, 1, 0, 0, 0, 0, 0.5)), out n, out c) ==
                ReferenceRing.MixedChannel, "a direction partly NaN");
            Check (V (Ring (S (1, 0, 1, 0, 0, 0, 0, 0.5), S (2, 0, 1, 0, nan, nan, nan, 0.5)), out n, out c) ==
                ReferenceRing.MixedChannel, "a channel in one sample only");
            Check (V (Ring (S (1, 0, 1, 0, 0, 0, 0, double.PositiveInfinity), S (2, 0, 1, 0, 0, 0, 0, 0.5)), out n, out c) ==
                ReferenceRing.MixedChannel, "an infinite throttle");
            Check (V (Ring (S (1, nan, nan, nan, nan, nan, nan, nan), S (2, nan, nan, nan, nan, nan, nan, nan)), out n, out c) ==
                ReferenceRing.NoChannel, "no channel");
            Check (V (Ring (S (1, 0, 0, 0, 0, 0, 0, 0.5), S (2, 0, 1, 0, 0, 0, 0, 0.5)), out n, out c) ==
                ReferenceRing.NullDirection, "a null direction");
            double a = Math.PI / 2 - 1e-6;
            Check (V (Ring (S (1, 0, 1, 0, 0, 0, 0, 0.5), S (2, Math.Sin (a), Math.Cos (a), 0, 0, 0, 0, 0.5)), out n, out c) == 0,
                "two directions just under 90 deg");
            Check (V (Ring (S (1, 0, 1, 0, 0, 0, 0, 0.5), S (2, 1, 0, 0, 0, 0, 0, 0.5)), out n, out c) ==
                ReferenceRing.DirectionTurn, "two directions at 90 deg");
            Check (V (Ring (S (1, 0, 1, 0, 0, 0, 0, 0.5), S (2, 0, -1, 0.01, 0, 0, 0, 0.5)), out n, out c) ==
                ReferenceRing.DirectionTurn, "two directions opposite");
            Check (V (Ring (S (1, 0, 1, 0, 0, 0, 0, 1.0000001), S (2, 0, 1, 0, 0, 0, 0, 0.5)), out n, out c) ==
                ReferenceRing.BadThrottle, "a throttle above 1");
            Check (V (Ring (S (1, 0, 1, 0, 0, 0, 0, 0.5), S (2, 0, 1, 0, 0, 0, 0, -1e-9)), out n, out c) ==
                ReferenceRing.BadThrottle, "a throttle below 0");
            for (int code = 1; code < ReferenceRing.CodeCount; code++)
                Check (ReferenceRing.Refusal (code).StartsWith ("anneau"), "a refusal message for code " + code);
        }

        // ------------------------------------------------------------------ store and sample

        static void TestStoreAndSample ()
        {
            var d = new RingSamples ();
            var o = new RingSample ();
            var s = Ring (S (10.0, 0, 2, 0, 1, 0, -1, 0.2), S (10.5, 0, 3, 3, 3, 2, 1, 0.6), S (11.0, 0, 0, 5, -1, 0, 0, 0.0));
            int n, c;
            Check (ReferenceRing.Validate (s, out n, out c) == 0 && n == 3 && c == 7, "the test ring is valid");
            ReferenceRing.Store (s, n, c, d);
            Check (d.Count == 3 && d.Channels == 7 && d.Dir [1] == 1.0 && d.Dir [8] == 1.0 &&
                Math.Abs (d.Dir [4] - Math.Sqrt (0.5)) < 1e-15 && Math.Abs (d.Dir [5] - Math.Sqrt (0.5)) < 1e-15 &&
                d.A [3] == 3 && d.U [2] == 0.0, "stored, normalized");
            ReferenceRing.Sample (d, 9.999, o);
            Check (o.Served == 0 && o.Segment == -1 && double.IsNaN (o.U) && double.IsNaN (o.Dir [0]), "before: nothing");
            ReferenceRing.Sample (d, 11.0001, o);
            Check (o.Served == 0, "after: nothing");
            ReferenceRing.Sample (d, double.NaN, o);
            Check (o.Served == 0, "ut NaN: nothing");
            var one = new RingSamples ();
            ReferenceRing.Store (Ring (S (1, 0, 1, 0, 0, 0, 0, 0.5)), 1, 7, one);
            ReferenceRing.Sample (one, 1.0, o);
            Check (o.Served == 0, "one sample: nothing");
            ReferenceRing.Sample (d, 10.0, o);
            Check (o.Served == 7 && o.Segment == 0 && o.Fraction == 0.0 && o.Dir [1] == 1.0 && o.A [0] == 1 && o.U == 0.2,
                "the first node exactly");
            ReferenceRing.Sample (d, 10.5, o);
            Check (o.Segment == 0 && o.Fraction == 1.0 && o.A [1] == 2 && o.U == 0.6 &&
                Math.Abs (o.Dir [2] - Math.Sqrt (0.5)) < 1e-15, "a shared node: the first segment, its end");
            ReferenceRing.Sample (d, 11.0, o);
            Check (o.Segment == 1 && o.Fraction == 1.0 && o.Dir [2] == 1.0 && o.U == 0.0, "the last node exactly");
            ReferenceRing.Sample (d, 10.75, o);
            Check (o.Segment == 1 && Math.Abs (o.Fraction - 0.5) < 1e-15 && Math.Abs (o.U - 0.3) < 1e-15 &&
                Math.Abs (o.A [0] - 1.0) < 1e-15 && Math.Abs (o.A [1] - 1.0) < 1e-15 && Math.Abs (o.A [2] - 0.5) < 1e-15,
                "the middle of the second segment: linear");
            var rng = new Random (2527);
            for (int k = 0; k < 2000; k++) {
                double ut = 10.0 + 1.0 * rng.NextDouble ();
                ReferenceRing.Sample (d, ut, o);
                int seg = o.Segment;
                double f = o.Fraction;
                Check (o.Served == 7 && seg >= 0 && seg <= 1 && f >= 0.0 && f <= 1.0 && Math.Abs (d.Ut [seg] + f *
                    (d.Ut [seg + 1] - d.Ut [seg]) - ut) < 1e-12, "the segment and its fraction");
                double nx = o.Dir [0], ny = o.Dir [1], nz = o.Dir [2];
                Check (Math.Abs (nx * nx + ny * ny + nz * nz - 1.0) < 1e-14, "the served nose is unit");
                double c0 = nx * d.Dir [3 * seg] + ny * d.Dir [3 * seg + 1] + nz * d.Dir [3 * seg + 2];
                double c1 = nx * d.Dir [3 * seg + 3] + ny * d.Dir [3 * seg + 4] + nz * d.Dir [3 * seg + 5];
                double cab = d.Dir [3 * seg] * d.Dir [3 * seg + 3] + d.Dir [3 * seg + 1] * d.Dir [3 * seg + 4] +
                    d.Dir [3 * seg + 2] * d.Dir [3 * seg + 5];
                Check (c0 >= cab - 1e-12 && c1 >= cab - 1e-12, "on the arc between its two samples");
                double ut2 = Math.Min (d.Ut [seg + 1], ut + 0.01);
                var o2 = new RingSample ();
                ReferenceRing.Sample (d, ut2, o2);
                if (o2.Segment == seg) {
                    double c02 = o2.Dir [0] * d.Dir [3 * seg] + o2.Dir [1] * d.Dir [3 * seg + 1] + o2.Dir [2] * d.Dir [3 * seg + 2];
                    Check (c02 <= c0 + 1e-12, "forward along the arc");
                }
            }
        }

        // ------------------------------------------------------------------ a turning nose

        static void TestTurningNose ()
        {
            // 40 deg/s about x, sampled at 10 Hz over 1 s, served at 50 Hz: nlerp's error is the chord's sag
            double w = 40.0 * Math.PI / 180.0, t0 = 100.0;
            var s = new List<double> ();
            for (int k = 0; k <= 10; k++) {
                double t = 0.1 * k;
                s.AddRange (S (t0 + t, 0, Math.Cos (w * t), Math.Sin (w * t), double.NaN, double.NaN, double.NaN,
                    double.NaN));
            }
            int n, c;
            Check (ReferenceRing.Validate (s, out n, out c) == 0 && n == 11 && c == 1, "the turning ring is valid");
            var d = new RingSamples ();
            ReferenceRing.Store (s, n, c, d);
            var o = new RingSample ();
            double worst = 0.0;
            for (int k = 0; k <= 50; k++) {
                double t = 0.02 * k;
                ReferenceRing.Sample (d, t0 + t, o);
                double dot = o.Dir [1] * Math.Cos (w * t) + o.Dir [2] * Math.Sin (w * t);
                double err = Math.Acos (Math.Min (1.0, dot));
                if (err > worst)
                    worst = err;
            }
            // nlerp at the middle of a 4 deg chord is exact in direction (symmetry); away from it the angle error is
            // below the half arc's (1 - cos) times its half angle: under 0.002 deg here
            Check (worst < 0.002 * Math.PI / 180.0, "the served nose within 0.002 deg of the true one: " +
                (worst * 180.0 / Math.PI).ToString ("G4", CultureInfo.InvariantCulture));
        }

        // ------------------------------------------------------------------ K5 under the ring

        static void TestThrottleUnderRing ()
        {
            var g = new AttitudeGains ();
            var o = new ThrottleOrder ();
            o.Kind = MainThrottle.KindSchedule;
            o.Sequence = 1;
            o.Count = 4;
            o.P [0] = 0.2;
            o.P [1] = 0.5;
            o.P [2] = 0.05;
            o.P [3] = 4;
            o.UtStamp = 10.0;
            o.UtValidUntil = 11.0;
            o.ResetRuntime ();
            var outp = new CalageOutput ();
            var i = new ThrottleStepInput ();
            i.Dt = 0.02;
            i.CanWrite = true;
            i.FullThrustN = 9e6;
            i.FiniteResponse = false;
            i.AccelerationSpeed = double.NaN;
            int s, m;
            i.Ut = 10.2;
            i.ThrustNowN = 0.0;
            double u = MainThrottle.Decide (o, i, g, outp, out s, out m);
            Check (u == MainThrottle.Schedule (0.2, 0.5, 10.0, 10.2) && m == MainThrottle.MotifSchedule,
                "no ring: u0 + slope (t - t0), motif 14");
            i.Ut = 10.22;
            i.RingThrottle = 0.37;
            i.ThrustNowN = u * 9e6;
            u = MainThrottle.Decide (o, i, g, outp, out s, out m);
            Check (u == 0.37 && m == MainThrottle.MotifRing && s == MainThrottle.StateWriting, "the ring's value, motif 18");
            i.Ut = 10.24;
            i.RingThrottle = 1.5;
            i.ThrustNowN = u * 9e6;
            u = MainThrottle.Decide (o, i, g, outp, out s, out m);
            Check (u == 1.0, "clamped to 1");
            i.Ut = 11.5;
            i.RingThrottle = 0.4;
            u = MainThrottle.Decide (o, i, g, outp, out s, out m);
            Check (double.IsNaN (u) && m != MainThrottle.MotifRing, "past the order's validity: nothing, ring or not");
            // the guard: an episode lit at 0.4 whose thrust never shows trips past the chain, ring or not
            var o2 = new ThrottleOrder ();
            o2.Kind = MainThrottle.KindSchedule;
            o2.Sequence = 2;
            o2.Count = 4;
            o2.P [0] = 0.4;
            o2.P [1] = 0.0;
            o2.P [2] = 0.05;
            o2.P [3] = 4;
            o2.UtStamp = 20.0;
            o2.UtValidUntil = 25.0;
            o2.ResetRuntime ();
            bool tripped = false;
            for (int k = 0; k < 30 && !tripped; k++) {
                i.Ut = 20.0 + 0.02 * k;
                i.RingThrottle = 0.4;
                i.ThrustNowN = 0.0;
                u = MainThrottle.Decide (o2, i, g, outp, out s, out m);
                tripped = s == MainThrottle.StateGuard;
            }
            Check (tripped && double.IsNaN (u), "an earlier change's guard trips under the ring's throttle");
            // the other kinds ignore the ring
            var o3 = new ThrottleOrder ();
            o3.Kind = MainThrottle.KindPulse;
            o3.Sequence = 3;
            o3.Count = 4;
            o3.P [0] = 0.7;
            o3.P [1] = 3;
            o3.P [2] = 0.05;
            o3.P [3] = 4;
            o3.UtStamp = 30.0;
            o3.UtValidUntil = 31.0;
            o3.ResetRuntime ();
            i.Ut = 30.0;
            i.RingThrottle = 0.1;
            i.ThrustNowN = 0.0;
            u = MainThrottle.Decide (o3, i, g, outp, out s, out m);
            Check (u == 0.7 && m == MainThrottle.MotifPulse, "a pulse ignores the ring");
        }

        // ------------------------------------------------------------------ golden vectors

        static string GD (double v)
        {
            if (double.IsNaN (v))
                return "NaN";
            if (double.IsPositiveInfinity (v))
                return "Infinity";
            if (double.IsNegativeInfinity (v))
                return "-Infinity";
            return v.ToString ("R", CultureInfo.InvariantCulture);
        }

        static string List (IList<double> v)
        {
            var parts = new string[v.Count];
            for (int k = 0; k < v.Count; k++)
                parts [k] = GD (v [k]);
            return "[" + string.Join (", ", parts) + "]";
        }

        /// <summary>
        /// The golden vectors of the ring: validation cases (each refusal and valid rings of every channel set) with
        /// their code, count and channels; for the valid ones, the samples at random and node UTs (served, segment,
        /// fraction, direction, lateral, throttle). Deterministic.
        /// </summary>
        internal static string GoldenJson ()
        {
            double nan = double.NaN;
            var rng = new Random (2528);
            var cases = new List<List<double>> {
                Ring (S (1, 0, 1, 0, 0, 0, 0, 0.5)),
                Ring (S (2, 0, 1, 0, 0, 0, 0, 0.5), S (1, 0, 1, 0, 0, 0, 0, 0.5)),
                Ring (S (1, 0, 1, nan, 0, 0, 0, 0.5), S (2, 0, 1, 0, 0, 0, 0, 0.5)),
                Ring (S (1, nan, nan, nan, nan, nan, nan, nan), S (2, nan, nan, nan, nan, nan, nan, nan)),
                Ring (S (1, 0, 0, 0, 0, 0, 0, 0.5), S (2, 0, 1, 0, 0, 0, 0, 0.5)),
                Ring (S (1, 0, 1, 0, 0, 0, 0, 0.5), S (2, 1, 0, 0, 0, 0, 0, 0.5)),
                Ring (S (1, 0, 1, 0, 0, 0, 0, 1.5), S (2, 0, 1, 0, 0, 0, 0, 0.5)),
                Ring (S (10.0, 0, 2, 0, 1, 0, -1, 0.2), S (10.5, 0, 3, 3, 3, 2, 1, 0.6), S (11.0, 0, 0, 5, -1, 0, 0, 0.0)),
                Ring (S (1, nan, nan, nan, 1, 2, 3, nan), S (2.5, nan, nan, nan, -1, 0, 3, nan)),
                Ring (S (1, nan, nan, nan, nan, nan, nan, 0), S (1.25, nan, nan, nan, nan, nan, nan, 1)),
            };
            for (int r = 0; r < 6; r++) {
                var l = new List<double> ();
                int n = 2 + rng.Next (20);
                double ut = 100.0 * rng.NextDouble ();
                double x = rng.NextDouble () - 0.5, y = 1.0, z = rng.NextDouble () - 0.5;
                for (int k = 0; k < n; k++) {
                    ut += 0.01 + 0.1 * rng.NextDouble ();
                    x += 0.2 * (rng.NextDouble () - 0.5);
                    z += 0.2 * (rng.NextDouble () - 0.5);
                    l.AddRange (S (ut, x, y * (0.5 + rng.NextDouble ()), z, 3 * (rng.NextDouble () - 0.5),
                        r % 2 == 0 ? 0.0 : rng.NextDouble (), 3 * (rng.NextDouble () - 0.5), rng.NextDouble ()));
                }
                cases.Add (l);
            }
            var sb = new StringBuilder ();
            sb.Append ("{\n  \"schema\": 1,\n  \"source\": \"krpc_bridge build/tests (ReferenceRing.cs, option 2048)\",\n");
            sb.Append ("  \"rings\": [\n");
            var d = new RingSamples ();
            var o = new RingSample ();
            for (int q = 0; q < cases.Count; q++) {
                var s = cases [q];
                int n, c;
                int code = ReferenceRing.Validate (s, out n, out c);
                sb.Append ("    {\"samples\": " + List (s) + ", \"code\": " + code + ", \"count\": " + n + ", \"channels\": " + c +
                    ", \"at\": [\n");
                var ats = new List<double> ();
                if (code == 0) {
                    ReferenceRing.Store (s, n, c, d);
                    double a = d.Ut [0], b = d.Ut [n - 1];
                    ats.Add (a - 0.001);
                    for (int k = 0; k < n; k++)
                        ats.Add (d.Ut [k]);
                    for (int k = 0; k < 12; k++)
                        ats.Add (a + (b - a) * rng.NextDouble ());
                    ats.Add (b + 0.001);
                }
                for (int k = 0; k < ats.Count; k++) {
                    ReferenceRing.Sample (d, ats [k], o);
                    sb.Append ("      {\"ut\": " + GD (ats [k]) + ", \"served\": " + o.Served + ", \"segment\": " + o.Segment +
                        ", \"fraction\": " + GD (o.Fraction) + ", \"dir\": " + List (o.Dir) + ", \"a\": " + List (o.A) +
                        ", \"u\": " + GD (o.U) + "}" + (k + 1 < ats.Count ? "," : "") + "\n");
                }
                sb.Append ("    ]}" + (q + 1 < cases.Count ? "," : "") + "\n");
            }
            sb.Append ("  ]\n}\n");
            return sb.ToString ();
        }
    }
}
