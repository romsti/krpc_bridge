// -----------------------------------------------------------------------------------
// (option 512 of AttitudeOptionsV1, born closed): the main throttle decided at the physics step
// (src/Plugins/Model/MainThrottle.cs). Checks:
//   * option 512 is a new bit, no older option holds it, the kinds and their parameter counts;
//   * the dated ramp, the ignition margin, an earlier change's required throttle, lost fraction and cost cap against hand
//     computations (the formulas of the client, recopied in the comments);
//   * the flip's calage, every motif of the client, its order of tests, and the
//     saturation of the law's REAL pseudo-command;
//   * an earlier change's guard (the client): wait, lit, absent, the spool's delay;
//   * an order's life: idle, stop, validity, blocked, ramp, pulse, cut when s = 0, ignition, the calage's hold
//     (chain + 1 steps then 0), the guard tripped and latched, the episode's smallest throttle;
//   * the closed loop of a stalled flip on one axis, the super hope's numbers: without calage the aero holds the
//     nose, with the step's calage it closes, and a nose swung out of the burn hemisphere cuts the push at once.
// The golden vectors (golden/attitude_v1_golden_throttle.json) are replayed by the client's mirror
// (the client, the client).
// -----------------------------------------------------------------------------------

using System;
using System.Globalization;
using System.Text;

namespace KRPC.Bridge.Model.Tests
{
    internal static class MainThrottleTests
    {
        static int checks;
        static int failures;

        internal static int Run (out int checkCount)
        {
            TestBits ();
            TestFormulas ();
            TestCalageMotifs ();
            TestGuard ();
            TestOrderLife ();
            TestClosedLoopStall ();
            checkCount = checks;
            Console.WriteLine ("main throttle checks: " + checks + ", failures: " + failures);
            return failures;
        }

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.Error.WriteLine ("FAIL (main throttle): " + what);
            }
        }

        static bool Near (double a, double b, double tol)
        {
            if (double.IsPositiveInfinity (a) && double.IsPositiveInfinity (b))
                return true;
            return Math.Abs (a - b) <= tol * Math.Max (1.0, Math.Max (Math.Abs (a), Math.Abs (b)));
        }

        // ------------------------------------------------------------------ bits

        static void TestBits ()
        {
            Check (AttitudeWrite.OptionMainThrottle == 512 && (AttitudeWrite.OptionsSupported & 1023) == 1023,
                "option 512, supported 1023 and above (mirrored by the client)");
            Check (AttitudeWrite.ValidOptions (512) && AttitudeWrite.ValidOptions (1023), "512 valid");
            for (int o = 0; o < 512; o++)
                Check ((o & AttitudeWrite.OptionMainThrottle) == 0, "no older option holds bit 512 (" + o + ")");
            Check (MainThrottle.ParameterCounts.Length == MainThrottle.KindCount, "a parameter count per kind");
            Check (MainThrottle.ParameterCounts [MainThrottle.KindStop] == 0 &&
                MainThrottle.ParameterCounts [MainThrottle.KindCalage] == MainThrottle.MaxParameters,
                "stop has none, the calage the most");
            var p = new double[MainThrottle.MaxParameters];
            Check (!MainThrottle.ValidParameters (MainThrottle.KindSchedule, p, 3), "a wrong count is refused");
            Check (MainThrottle.ValidParameters (MainThrottle.KindSchedule, p, 4), "a ramp of zeros is valid");
            p [1] = double.NaN;
            Check (!MainThrottle.ValidParameters (MainThrottle.KindSchedule, p, 4), "a NaN slope is refused");
            p [1] = 0.0;
            Check (!MainThrottle.ValidParameters (MainThrottle.KindPulse, p, 4), "a pulse of 0 steps is refused");
            p [1] = 3.0;
            Check (MainThrottle.ValidParameters (MainThrottle.KindPulse, p, 4), "a pulse of 3 steps");
            for (int k = 0; k < p.Length; k++)
                p [k] = 0.0;
            p [6] = 2.0;
            Check (!MainThrottle.ValidParameters (MainThrottle.KindCrossing, p, 9), "velocity kind 0 or 1 only");
            p [6] = 1.0;
            Check (MainThrottle.ValidParameters (MainThrottle.KindCrossing, p, 9), "orbital velocity");
            for (int k = 0; k < p.Length; k++)
                p [k] = 0.0;
            p [6] = double.NaN;
            p [7] = double.NaN;
            p [8] = double.NaN;
            p [9] = 1.0;
            Check (MainThrottle.ValidParameters (MainThrottle.KindCalage, p, 12),
                "the calage takes a NaN slope and NaN RCS authority");
            p [9] = 0.5;
            Check (!MainThrottle.ValidParameters (MainThrottle.KindCalage, p, 12), "the verdict is -1, 0 or 1");
            Check (MainThrottle.ValidParameters (MainThrottle.KindStop, null, 0), "stop without parameters");
            Check (!MainThrottle.ValidParameters (MainThrottle.KindCount, p, 0), "unknown kind refused");
        }

        // ------------------------------------------------------------------ formulas

        static void TestFormulas ()
        {
            Check (MainThrottle.Schedule (0.2, 0.1, 10.0, 12.0) == 0.2 + 0.1 * 2.0, "ramp");
            Check (MainThrottle.Schedule (0.9, 1.0, 0.0, 5.0) == 1.0 && MainThrottle.Schedule (0.1, -1.0, 0.0, 5.0) == 0.0,
                "ramp clamped to [0, 1]");
            Check (double.IsNaN (MainThrottle.Schedule (double.NaN, 0.0, 0.0, 1.0)), "ramp NaN");
            Check (MainThrottle.IgniteMargin (500.0, -100.0, 50.0, 0.0, 1.0 / (2.0 * 20.0)) ==
                500.0 - (50.0 + 0.0 * -100.0 + 1.0 / (2.0 * 20.0) * 10000.0), "h - h_ign(v)");
            Check (MainThrottle.RequiredThrottle (2e6, -1e6, 4e6, 5e6) == 0.5, "required: max(|Mx|/Gx, |Mz|/Gz)");
            Check (double.IsNaN (MainThrottle.RequiredThrottle (1.0, 1.0, 0.0, 1.0)), "required without gimbals");
            // nose on b: nothing lost.
            Check (MainThrottle.FractionLost (0, 1, 0, 0, 0, 1, 0, 2, 0) == 0.0, "nose on the boostback");
            // nose d above b in the vertical plane: 1 - cos d + sin d cos(elevation of b) (flip.py's docstring).
            double elev = 0.5, d = 0.3;
            double bx = Math.Cos (elev), bz = Math.Sin (elev);
            double nx = Math.Cos (elev + d), nz = Math.Sin (elev + d);
            double w = MainThrottle.FractionLost (nx, 0.0, nz, 0.0, 0.0, 1.0, bx, 0.0, bz);
            Check (Near (w, 1.0 - Math.Cos (d) + Math.Sin (d) * Math.Cos (elev), 1e-12), "lost fraction above b: " + w);
            Check (double.IsNaN (MainThrottle.FractionLost (0, 0, 0, 0, 0, 1, 1, 0, 0)), "degenerate nose");
            Check (double.IsPositiveInfinity (MainThrottle.CostCap (3.0, 1e5, 9e6, 0.0)), "nothing lost: no cap");
            Check (MainThrottle.CostCap (3.0, 1e5, 9e6, 0.5) == 3.0 * 1e5 / (9e6 * 0.5), "cap = slope m / (T w)");
            Check (double.IsNaN (MainThrottle.CostCap (double.NaN, 1e5, 9e6, 0.5)) &&
                double.IsNaN (MainThrottle.CostCap (3.0, 0.0, 9e6, 0.5)) && double.IsNaN (MainThrottle.CostCap (-1.0, 1e5, 9e6, 0.5)),
                "cap unreadable");
        }

        // ------------------------------------------------------------------ the calage

        /// <summary>
        /// The super hope at the flip (study 4 sec. 3.1): RCS + wheels 0.79 MN.m, gimbals 14.5 MN.m, 1.2e7 kg.m^2; a
        /// boostback slope of 8 m/s^2 and a lost fraction of 0.3 put the cost cap at 0.35.
        /// </summary>
        static CalageInput Hope ()
        {
            return new CalageInput {
                Ex = 0.8, Ez = 0.0, Mx = -2.0e6, Mz = 0.0, Rx = 0.79e6, Rz = 0.79e6, Gx = 14.5e6, Gz = 14.5e6,
                LawComputed = true, NuX = 0.05, NuZ = 0.0, Ix = 1.2e7, Iz = 1.2e7, GuardRatio = 0.9, DotTarget = 0.5,
                DotAxis = 0.5, FractionLost = 0.3, Slope = 8.0, MassKg = 1.18e5, ThrustN = 9.0e6
            };
        }

        static void TestCalageMotifs ()
        {
            var o = new CalageOutput ();
            var i = Hope ();
            MainThrottle.Calage (i, o);
            // aero against the correction, 2.0 > 0.79 MN.m: the gimbals cover it, 2.0 / 14.5.
            Check (o.Motif == MainThrottle.MotifCalage && Near (o.Throttle, 2.0 / 14.5, 1e-15), "calage: " + o.Throttle);
            Check (Near (o.Required, 2.0 / 14.5, 1e-15) && Near (o.Cap, 8.0 * 1.18e5 / (9.0e6 * 0.3), 1e-15),
                "required and cap published");
            i = Hope ();
            i.Mx = 2.0e6;
            MainThrottle.Calage (i, o);
            Check (o.Motif == MainThrottle.MotifAide && o.Throttle == 0.0, "aero helps: 0");
            i = Hope ();
            i.Mx = -0.5e6;
            MainThrottle.Calage (i, o);
            Check (o.Motif == MainThrottle.MotifSousRcs && o.Throttle == 0.0, "aero under the RCS: 0");
            // saturation: the law asks I nu - M beyond 1.9 R on x, the aero itself under the RCS.
            i = Hope ();
            i.Mx = -0.5e6;
            i.NuX = (i.Mx - 1.9 * 0.79e6 - 1e5) / 1.2e7;
            MainThrottle.Calage (i, o);
            double dx = i.Ix * i.NuX - i.Mx;
            Check (o.Saturated && o.Motif == MainThrottle.MotifSaturation && Near (o.Unrealized, (Math.Abs (dx) - 0.79e6) / 0.79e6, 1e-12) &&
                Near (o.Throttle, 0.5 / 14.5, 1e-15), "saturation of the real pseudo-command: " + o.Unrealized);
            i.LawComputed = false;
            MainThrottle.Calage (i, o);
            Check (!o.Saturated && double.IsNaN (o.Unrealized) && o.Motif == MainThrottle.MotifSousRcs,
                "no law: an earlier change's rule alone");
            i = Hope ();
            i.Gz = 0.0;
            MainThrottle.Calage (i, o);
            Check (o.Motif == MainThrottle.MotifSansCardan && double.IsNaN (o.Throttle), "no gimbal on an axis");
            i = Hope ();
            i.DotTarget = -0.1;
            MainThrottle.Calage (i, o);
            Check (o.Motif == MainThrottle.MotifHemisphere && o.Throttle == 0.0, "nose away from the boostback");
            i = Hope ();
            i.DotAxis = double.NaN;
            MainThrottle.Calage (i, o);
            Check (o.Motif == MainThrottle.MotifHemisphere, "degenerate burn axis: hemisphere");
            i = Hope ();
            i.Rx = double.NaN;
            MainThrottle.Calage (i, o);
            Check (o.Motif == MainThrottle.MotifSansMesure && double.IsNaN (o.Throttle), "RCS authority unreadable");
            i = Hope ();
            i.Mx = -20.0e6;
            MainThrottle.Calage (i, o);
            Check (o.Motif == MainThrottle.MotifHorsAutorite && o.Throttle == 0.0 && o.Required > 1.0,
                "required beyond full thrust: 0");
            i = Hope ();
            i.Slope = 0.05;
            MainThrottle.Calage (i, o);
            Check (o.Motif == MainThrottle.MotifCout && o.Throttle == 0.0, "above the cost cap: 0 (cap " + o.Cap + ")");
            i = Hope ();
            i.Slope = double.NaN;
            MainThrottle.Calage (i, o);
            Check (o.Motif == MainThrottle.MotifSansCout && o.Throttle == 0.0, "no slope: no push");
            i = Hope ();
            i.FractionLost = 0.0;
            MainThrottle.Calage (i, o);
            Check (o.Motif == MainThrottle.MotifCalage && double.IsPositiveInfinity (o.Cap) && o.Throttle > 0.0,
                "nose on the boostback: no cap");
            i = Hope ();
            i.Ex = 0.0;
            MainThrottle.Calage (i, o);
            Check (o.Motif == MainThrottle.MotifNul && o.Throttle == 0.0, "no error: 0");
            // the transverse part: aero across the error also covered (flip.py's t = -(m_par eh + m_perp ep) = -M).
            i = Hope ();
            i.Mz = 4.0e6;
            MainThrottle.Calage (i, o);
            Check (Near (o.Throttle, 4.0 / 14.5, 1e-15) && Near (o.MPerp, 4.0e6, 1e-15),
                "transverse aero covered: " + o.Throttle + ", m_perp " + o.MPerp);
            // order of tests: unreadable RCS before the hemisphere (flip.py)
            i = Hope ();
            i.Rz = 0.0;
            i.DotTarget = -1.0;
            MainThrottle.Calage (i, o);
            Check (o.Motif == MainThrottle.MotifSansMesure, "sans_mesure before hemisphere");
        }

        // ------------------------------------------------------------------ an earlier change's guard

        static void TestGuard ()
        {
            double d;
            Check (MainThrottle.ThrustGuard (10.0, double.NaN, 0.0, 1e6, 0.02, 4, 0.05, false, 0.0, out d) ==
                MainThrottle.GuardWait && double.IsNaN (d), "nothing written: wait");
            Check (MainThrottle.ThrustGuard (10.05, 10.0, 0.0, 1e6, 0.02, 4, 0.05, false, 0.0, out d) ==
                MainThrottle.GuardWait && Near (d, 0.1, 1e-15), "within the chain: wait");
            Check (MainThrottle.ThrustGuard (10.11, 10.0, 0.0, 1e6, 0.02, 4, 0.05, false, 0.0, out d) ==
                MainThrottle.GuardAbsent, "past the chain without thrust: absent");
            Check (MainThrottle.ThrustGuard (10.11, 10.0, 0.05e6, 1e6, 0.02, 4, 0.05, false, 0.0, out d) ==
                MainThrottle.GuardLit, "5 % of the demand: lit");
            Check (MainThrottle.ThrustGuard (10.11, 10.0, 0.0, 1e6, 0.02, 4, 0.05, true, 2.0, out d) ==
                MainThrottle.GuardWait && Near (d, 0.1 - Math.Log (0.95) / 2.0, 1e-15), "a spool's delay");
            Check (MainThrottle.ThrustGuard (10.11, 10.0, 0.0, 1e6, 0.02, 4, 0.05, true, 0.0, out d) ==
                MainThrottle.GuardWait, "a spool of unknown speed: wait");
            Check (MainThrottle.ThrustGuard (10.11, 10.0, 0.0, 1e6, 0.02, 4, double.NaN, false, 0.0, out d) ==
                MainThrottle.GuardWait, "no threshold: no guard");
            // The ground test of 15/10: engines starved of propellant (ignited, flamed out) leave the calage's full thrust
            // at 0, not the guard's: an order's guard trips past the chain; engines shut down (none ignited) are no
            // thrust asked of anyone, for the client's (kRPC's max_thrust over the active engines) as for the loop.
            var g = new AttitudeGains ();
            var c = new CalageOutput ();
            for (int variant = 0; variant < 3; variant++) {
                var o = Order (MainThrottle.KindSchedule, 10.0, 12.0, 0.05, 0.0, 0.05, 4);
                o.ResetRuntime ();
                int state = -1, motif;
                for (int k = 0; k < 20; k++) {
                    var i = Step (10.0 + 0.02 * k);
                    i.FullThrustN = 0.0;
                    i.GuardFullThrustN = variant == 0 ? 8.4e6 : (variant == 1 ? 0.0 : double.NaN);
                    MainThrottle.Decide (o, i, g, c, out state, out motif);
                }
                Check (variant == 0 ? state == MainThrottle.StateGuard && o.GuardTripped
                    : state == MainThrottle.StateWriting && !o.GuardTripped,
                    "guard full thrust " + variant + ": state " + state);
            }
        }

        // ------------------------------------------------------------------ an order's life

        static ThrottleOrder Order (int kind, double from, double until, params double[] p)
        {
            var o = new ThrottleOrder ();
            o.Kind = kind;
            o.Sequence = 1;
            o.UtStamp = from;
            o.UtValidUntil = until;
            o.Count = p.Length;
            for (int k = 0; k < p.Length; k++)
                o.P [k] = p [k];
            return o;
        }

        static ThrottleStepInput Step (double ut)
        {
            var i = new ThrottleStepInput ();
            i.Ut = ut;
            i.Dt = 0.02;
            i.CanWrite = true;
            i.CrossingValue = double.NaN;
            i.Height = double.NaN;
            i.VerticalSpeed = double.NaN;
            i.ThrustNowN = 0.0;
            i.FullThrustN = 9.0e6;
            return i;
        }

        static void TestOrderLife ()
        {
            var g = new AttitudeGains ();
            var c = new CalageOutput ();
            int s, m;
            var none = new ThrottleOrder ();
            Check (double.IsNaN (MainThrottle.Decide (none, Step (0.0), g, c, out s, out m)) && s == MainThrottle.StateIdle,
                "no order: idle");
            var stop = Order (MainThrottle.KindStop, 0.0, 10.0);
            Check (double.IsNaN (MainThrottle.Decide (stop, Step (1.0), g, c, out s, out m)) && s == MainThrottle.StateEnded,
                "stop: ended");
            var ramp = Order (MainThrottle.KindSchedule, 10.0, 11.0, 0.2, 0.5, 0.05, 4);
            double u = MainThrottle.Decide (ramp, Step (10.4), g, c, out s, out m);
            Check (u == MainThrottle.Schedule (0.2, 0.5, 10.0, 10.4) && s == MainThrottle.StateWriting &&
                m == MainThrottle.MotifSchedule, "ramp written");
            var st = Step (10.5);
            st.CanWrite = false;
            Check (double.IsNaN (MainThrottle.Decide (ramp, st, g, c, out s, out m)) && s == MainThrottle.StateBlocked &&
                !ramp.EpisodeOn, "blocked: the client's throttle, the episode ends");
            Check (double.IsNaN (MainThrottle.Decide (ramp, Step (11.01), g, c, out s, out m)) &&
                s == MainThrottle.StateEnded, "validity passed: ended");
            var pulse = Order (MainThrottle.KindPulse, 0.0, 10.0, 0.6, 3, 0.05, 4);
            int written = 0;
            for (int k = 0; k < 6; k++) {
                var sp = Step (0.02 * k);
                sp.ThrustNowN = 0.6 * 9.0e6;
                if (!double.IsNaN (MainThrottle.Decide (pulse, sp, g, c, out s, out m)))
                    written++;
            }
            Check (written == 3 && s == MainThrottle.StateEnded, "pulse: 3 steps then the client's throttle");
            // cut when s = 0: 1 while s > 0, 0 from the first s <= 0, latched.
            var cut = Order (MainThrottle.KindCrossing, 0.0, 10.0, 1.0, 0.0, 1, 0, 0, 0, 0, 0.05, 4);
            double[] values = { 3.0, 2.0, 0.5, -0.1, 0.4, 2.0 };
            var outs = new double[values.Length];
            for (int k = 0; k < values.Length; k++) {
                var sp = Step (0.02 * k);
                sp.CrossingValue = values [k];
                sp.ThrustNowN = 9.0e6;
                outs [k] = MainThrottle.Decide (cut, sp, g, c, out s, out m);
            }
            Check (outs [0] == 1.0 && outs [2] == 1.0 && outs [3] == 0.0 && outs [4] == 0.0 && outs [5] == 0.0 &&
                cut.Fired && m == MainThrottle.MotifDeclenche, "cut at the first s <= 0, latched");
            var late = Order (MainThrottle.KindCrossing, 0.0, 10.0, 1.0, 0.0, 1, 0, 0, 0, 0, 0.05, 4);
            var sl = Step (0.0);
            sl.CrossingValue = -2.0;
            Check (MainThrottle.Decide (late, sl, g, c, out s, out m) == 0.0 && late.Fired,
                "an event already past at the first look fires at once");
            // ignition when h <= v^2 / (2 a) + h0.
            var ign = Order (MainThrottle.KindIgnite, 0.0, 100.0, 0.0, 0.7, 100.0, 50.0, 0.0, 1.0 / 40.0, 0.05, 4);
            double h = 1200.0, v = -120.0;
            int fired = -1;
            for (int k = 0; k < 400 && fired < 0; k++) {
                var sp = Step (0.02 * k);
                sp.Height = h;
                sp.VerticalSpeed = v;
                double w = MainThrottle.Decide (ign, sp, g, c, out s, out m);
                if (w > 0.0)
                    fired = k;
                h += v * 0.02;
            }
            double hAt = 1200.0 - 120.0 * 0.02 * fired, hIgn = 50.0 + 120.0 * 120.0 / 40.0;
            Check (fired > 0 && hAt <= hIgn && hAt + 120.0 * 0.02 > hIgn, "ignition at the first step below h_ign: " + fired);
            // the calage: a step without the law holds the last value chain + 1 steps, then 0.
            var cal = Order (MainThrottle.KindCalage, 0.0, 100.0, 0, 1, 0, 0, 1, 0, 3.0, 0.79e6, 0.79e6, 1, 0.05, 4);
            var si = Step (1.0);
            CopyHope (si.Calage);
            si.ThrustNowN = 9.0e6;
            u = MainThrottle.Decide (cal, si, g, c, out s, out m);
            Check (Near (u, 2.0 / 14.5, 1e-15) && cal.EpisodeOn && cal.EpisodeMin == u, "calage written, episode on");
            int held = 0;
            for (int k = 1; k <= 7; k++) {
                var sk = Step (1.0 + 0.02 * k);
                CopyHope (sk.Calage);
                sk.Calage.LawComputed = false;
                sk.ThrustNowN = 9.0e6;
                double w = MainThrottle.Decide (cal, sk, g, c, out s, out m);
                if (s == MainThrottle.StateHeld && w == u)
                    held++;
                else
                    Check (w == 0.0 && m == MainThrottle.MotifSansLoi, "past the hold: 0 (" + k + ")");
            }
            Check (held == 5, "held chain + 1 = 5 steps: " + held);
            // the guard: a push without thrust past the chain trips and latches.
            var gd = Order (MainThrottle.KindCalage, 0.0, 100.0, 0, 1, 0, 0, 1, 0, 3.0, 0.79e6, 0.79e6, 1, 0.05, 4);
            int trip = -1;
            for (int k = 0; k < 20; k++) {
                var sk = Step (2.0 + 0.02 * k);
                CopyHope (sk.Calage);
                sk.ThrustNowN = 0.0;
                double w = MainThrottle.Decide (gd, sk, g, c, out s, out m);
                if (s == MainThrottle.StateGuard && trip < 0)
                    trip = k;
                if (trip >= 0)
                    Check (double.IsNaN (w) && m == MainThrottle.MotifGardePoussee, "latched guard (" + k + ")");
            }
            Check (trip == 6, "guard at the first step past 0.1 s: " + trip);
            // a renewal keeps the latch; a new kind clears it.
            Check (gd.GuardTripped, "guard latched for the order");
            gd.ResetRuntime ();
            Check (!gd.GuardTripped && !gd.EpisodeOn, "a new order starts afresh");
        }

        static void CopyHope (CalageInput i)
        {
            var h = Hope ();
            i.Ex = h.Ex; i.Ez = h.Ez; i.Mx = h.Mx; i.Mz = h.Mz; i.Rx = h.Rx; i.Rz = h.Rz; i.Gx = h.Gx; i.Gz = h.Gz;
            i.LawComputed = h.LawComputed; i.NuX = h.NuX; i.NuZ = h.NuZ; i.Ix = h.Ix; i.Iz = h.Iz;
            i.GuardRatio = h.GuardRatio; i.DotTarget = h.DotTarget; i.DotAxis = h.DotAxis;
            i.FractionLost = h.FractionLost; i.Slope = h.Slope; i.MassKg = h.MassKg; i.ThrustN = h.ThrustN;
        }

        // ------------------------------------------------------------------ closed loop

        /// <summary>
        /// One pitch axis of the super hope in a stall: the aero holds the nose 40 deg from the target with
        /// 1.2 MN.m against an RCS + wheels authority of 0.79 MN.m. The law's pseudo-command is a P-D on the error
        /// bounded by the RCS; the gimbals add u 14.5 MN.m. Without calage the nose stays stalled; with the step's
        /// calage it closes; a nose leaving the hemisphere cuts the push the same step.
        /// </summary>
        static void TestClosedLoopStall ()
        {
            double I = 1.2e7, R = 0.79e6, G = 14.5e6, Ma = 1.2e6, dt = 0.02;
            for (int withCalage = 0; withCalage < 2; withCalage++) {
                double e = 0.7, w = 0.0;
                var order = Order (MainThrottle.KindCalage, 0.0, 100.0, 0, 1, 0, 0, 1, 0, 3.0, R, R, 1, 0.05, 4);
                var g = new AttitudeGains ();
                var c = new CalageOutput ();
                double uPrev = 0.0, maxU = 0.0;
                for (int k = 0; k < 1500; k++) {
                    // the law: nu = 3 (sqrt reference - w), bounded by its own authority (RCS + gimbals at the thrust now)
                    double alpha = (R + uPrev * G) / I;
                    double wr = Math.Sign (e) * Math.Min (0.75 * Math.Abs (e), Math.Sqrt (2.0 * 0.6 * alpha * Math.Abs (e)));
                    double nu = Math.Max (-alpha, Math.Min (alpha, 3.0 * (wr - w)));
                    double u = 0.0;
                    if (withCalage == 1) {
                        var si = Step (k * dt);
                        si.FullThrustN = 9.0e6;
                        si.ThrustNowN = uPrev * 9.0e6;
                        var ci = si.Calage;
                        ci.Ex = e; ci.Ez = 0.0; ci.Mx = -Ma; ci.Mz = 0.0; ci.Rx = R; ci.Rz = R; ci.Gx = G; ci.Gz = G;
                        ci.LawComputed = true; ci.NuX = nu; ci.NuZ = 0.0; ci.Ix = I; ci.Iz = I;
                        ci.DotTarget = Math.Cos (e); ci.DotAxis = Math.Cos (e); ci.FractionLost = 1.0 - Math.Cos (e);
                        ci.Slope = 3.0; ci.MassKg = 1.18e5; ci.ThrustN = 9.0e6;
                        int s, m;
                        double v = MainThrottle.Decide (order, si, g, c, out s, out m);
                        u = double.IsNaN (v) ? 0.0 : v;
                    }
                    // the plant: actuators give I nu within RCS + u G, the aero -Ma (against the closing of e > 0)
                    double act = Math.Max (-(R + uPrev * G), Math.Min (R + uPrev * G, I * nu + Ma));
                    double wdot = (act - Ma) / I;
                    w += wdot * dt;
                    e -= w * dt;
                    uPrev = u;
                    maxU = Math.Max (maxU, u);
                }
                if (withCalage == 0)
                    Check (e > 0.6, "without calage the aero holds the nose: " + e);
                else
                    Check (Math.Abs (e) < 0.05 && maxU <= Ma / G + 1e-12, "with the step's calage it closes: " + e + " u " + maxU);
            }
            // the hemisphere cuts at once: a push, then the nose past 90 deg from the boostback.
            var o2 = Order (MainThrottle.KindCalage, 0.0, 100.0, 0, 1, 0, 0, 1, 0, 3.0, 0.79e6, 0.79e6, 1, 0.05, 4);
            var g2 = new AttitudeGains ();
            var c2 = new CalageOutput ();
            int s2, m2;
            var a = Step (0.0);
            CopyHope (a.Calage);
            a.ThrustNowN = 9.0e6;
            double u1 = MainThrottle.Decide (o2, a, g2, c2, out s2, out m2);
            var b = Step (0.02);
            CopyHope (b.Calage);
            b.Calage.DotTarget = -0.01;
            b.ThrustNowN = 9.0e6;
            double u2 = MainThrottle.Decide (o2, b, g2, c2, out s2, out m2);
            Check (u1 > 0.0 && u2 == 0.0 && m2 == MainThrottle.MotifHemisphere && !o2.EpisodeOn,
                "nose out of the hemisphere: 0 the same step");
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

        static string CalageJson (CalageInput i)
        {
            return "{\"ex\": " + GD (i.Ex) + ", \"ez\": " + GD (i.Ez) + ", \"mx\": " + GD (i.Mx) + ", \"mz\": " + GD (i.Mz) +
                ", \"rx\": " + GD (i.Rx) + ", \"rz\": " + GD (i.Rz) + ", \"gx\": " + GD (i.Gx) + ", \"gz\": " + GD (i.Gz) +
                ", \"law\": " + (i.LawComputed ? "true" : "false") + ", \"nux\": " + GD (i.NuX) + ", \"nuz\": " +
                GD (i.NuZ) + ", \"ix\": " + GD (i.Ix) + ", \"iz\": " + GD (i.Iz) + ", \"guard_ratio\": " +
                GD (i.GuardRatio) + ", \"dot_target\": " + GD (i.DotTarget) + ", \"dot_axis\": " + GD (i.DotAxis) +
                ", \"fraction_lost\": " + GD (i.FractionLost) + ", \"slope\": " + GD (i.Slope) + ", \"mass\": " +
                GD (i.MassKg) + ", \"thrust\": " + GD (i.ThrustN) + "}";
        }

        static string CalageOutJson (CalageOutput o)
        {
            return "{\"throttle\": " + GD (o.Throttle) + ", \"motif\": " + o.Motif + ", \"unrealized\": " +
                GD (o.Unrealized) + ", \"saturated\": " + (o.Saturated ? "true" : "false") + ", \"m_par\": " +
                GD (o.MPar) + ", \"m_perp\": " + GD (o.MPerp) + ", \"rcs_par\": " + GD (o.RcsPar) + ", \"required\": " +
                GD (o.Required) + ", \"cap\": " + GD (o.Cap) + "}";
        }

        /// <summary>
        /// The golden vectors of option 512 (golden/attitude_v1_golden_throttle.json): calage cases (hand cases and
        /// 300 random draws around the super hope's, the moyen's and the heavy booster's flips), lost fractions and
        /// cost caps, guard cases, and order sequences (a step list per order, inputs and decisions), for the
        /// client's mirror (the client). Deterministic.
        /// </summary>
        internal static string GoldenJson ()
        {
            var sb = new StringBuilder ();
            sb.Append ("{\n  \"schema\": 1,\n  \"source\": \"krpc_bridge build/tests (MainThrottle.cs, option 512)\",\n");
            sb.Append ("  \"calage\": [\n");
            var rng = new Random (2525);
            var o = new CalageOutput ();
            int total = 312;
            for (int k = 0; k < total; k++) {
                var i = Hope ();
                if (k < 12) {
                    switch (k) {
                    case 1: i.Mx = 2.0e6; break;
                    case 2: i.Mx = -0.5e6; break;
                    case 3: i.Mx = -0.5e6; i.NuX = -0.17; break;
                    case 4: i.Gz = 0.0; break;
                    case 5: i.DotTarget = -0.1; break;
                    case 6: i.Rx = double.NaN; break;
                    case 7: i.Mx = -20.0e6; break;
                    case 8: i.Slope = 0.05; break;
                    case 9: i.Slope = double.NaN; break;
                    case 10: i.FractionLost = 0.0; break;
                    case 11: i.Ex = 0.0; break;
                    }
                } else {
                    double scale = new[] { 1.0, 0.85, 3.5 } [k % 3];
                    i.Ix = 1.2e7 * scale * (0.7 + 0.6 * rng.NextDouble ());
                    i.Iz = i.Ix * (0.95 + 0.1 * rng.NextDouble ());
                    i.Rx = 0.79e6 * scale * (0.5 + rng.NextDouble ());
                    i.Rz = i.Rx * (0.8 + 0.4 * rng.NextDouble ());
                    i.Gx = 14.5e6 * (k % 3 == 1 ? 0.1 : 1.0) * (0.6 + 0.8 * rng.NextDouble ());
                    i.Gz = i.Gx * (0.9 + 0.2 * rng.NextDouble ());
                    double a = 2 * Math.PI * rng.NextDouble (), r = 1.5 * rng.NextDouble ();
                    i.Ex = r * Math.Cos (a);
                    i.Ez = r * Math.Sin (a);
                    double b = 2 * Math.PI * rng.NextDouble (), mm = 4e6 * scale * rng.NextDouble ();
                    i.Mx = mm * Math.Cos (b);
                    i.Mz = mm * Math.Sin (b);
                    i.LawComputed = rng.NextDouble () > 0.1;
                    double alpha = (i.Rx + 0.3 * i.Gx) / i.Ix;
                    i.NuX = alpha * (2 * rng.NextDouble () - 1);
                    i.NuZ = alpha * (2 * rng.NextDouble () - 1);
                    i.DotTarget = 2 * rng.NextDouble () - 0.6;
                    i.DotAxis = 2 * rng.NextDouble () - 0.4;
                    i.FractionLost = rng.NextDouble () > 0.1 ? 1.6 * rng.NextDouble () : 0.0;
                    i.Slope = rng.NextDouble () > 0.05 ? 6.0 * rng.NextDouble () : double.NaN;
                    i.MassKg = 1.18e5 * scale * (0.8 + 0.4 * rng.NextDouble ());
                    i.ThrustN = 9.0e6 * scale * (0.8 + 0.4 * rng.NextDouble ());
                }
                MainThrottle.Calage (i, o);
                sb.Append ("    {\"in\": " + CalageJson (i) + ",\n     \"out\": " + CalageOutJson (o) + "}" +
                    (k + 1 < total ? "," : "") + "\n");
            }
            sb.Append ("  ],\n  \"fraction_lost\": [\n");
            int nf = 40;
            for (int k = 0; k < nf; k++) {
                var v = new double[9];
                for (int j = 0; j < 9; j++)
                    v [j] = 2 * rng.NextDouble () - 1;
                double w = MainThrottle.FractionLost (v [0], v [1], v [2], v [3], v [4], v [5], v [6], v [7], v [8]);
                double cap = MainThrottle.CostCap (5 * rng.NextDouble (), 1e5, 9e6, w);
                sb.Append ("    {\"v\": [" + string.Join (", ", Array.ConvertAll (v, GD)) + "], \"w\": " + GD (w) +
                    ", \"cap\": " + GD (cap) + "}" + (k + 1 < nf ? "," : "") + "\n");
            }
            sb.Append ("  ],\n  \"orders\": [\n");
            var orders = new[] {
                Order (MainThrottle.KindSchedule, 10.0, 10.5, 0.1, 0.8, 0.05, 4),
                Order (MainThrottle.KindPulse, 10.0, 10.5, 0.7, 5, 0.05, 4),
                Order (MainThrottle.KindCrossing, 10.0, 10.5, 1.0, 0.0, 1, 0, 0, -12, 0, 0.05, 4),
                Order (MainThrottle.KindIgnite, 10.0, 10.5, 0.0, 0.8, 100.0, 30.0, -0.5, 0.02, 0.05, 4),
                Order (MainThrottle.KindCalage, 10.0, 10.5, 0, 1, 0, 0, 1, 0, 3.0, 0.79e6, 0.79e6, 1, 0.05, 4),
            };
            var g = new AttitudeGains ();
            for (int q = 0; q < orders.Length; q++) {
                var ord = orders [q];
                sb.Append ("    {\"kind\": " + ord.Kind + ", \"ut_stamp\": " + GD (ord.UtStamp) + ", \"ut_valid_until\": " +
                    GD (ord.UtValidUntil) + ", \"p\": [" + string.Join (", ", Array.ConvertAll (
                    ArraySegmentCopy (ord.P, ord.Count), GD)) + "], \"steps\": [\n");
                double uPrev = 0.0;
                int steps = 30;
                for (int k = 0; k < steps; k++) {
                    var si = Step (10.0 + 0.02 * k);
                    si.CanWrite = k != 7;
                    si.CrossingValue = 12.0 - 0.9 * k;
                    si.Height = 900.0 - 20.0 * k;
                    si.VerticalSpeed = -100.0 + 0.5 * k;
                    si.ThrustNowN = (k > 12 && k < 22 ? 0.0 : uPrev) * 9.0e6;
                    si.FiniteResponse = q == 1;
                    si.AccelerationSpeed = q == 1 ? 5.0 : double.NaN;
                    CopyHope (si.Calage);
                    si.Calage.Mx = -2.0e6 + 0.15e6 * k;
                    si.Calage.LawComputed = k % 9 != 4;
                    si.Calage.DotTarget = k == 25 ? -0.2 : 0.5;
                    int s, m;
                    double u = MainThrottle.Decide (ord, si, g, o, out s, out m);
                    sb.Append ("      {\"ut\": " + GD (si.Ut) + ", \"dt\": " + GD (si.Dt) + ", \"can_write\": " +
                        (si.CanWrite ? "true" : "false") + ", \"s\": " + GD (si.CrossingValue) + ", \"h\": " +
                        GD (si.Height) + ", \"v\": " + GD (si.VerticalSpeed) + ", \"thrust_now\": " + GD (si.ThrustNowN) +
                        ", \"full_thrust\": " + GD (si.FullThrustN) + ", \"finite\": " + (si.FiniteResponse ? "true" : "false") +
                        ", \"accel\": " + GD (si.AccelerationSpeed) + ", \"calage\": " + CalageJson (si.Calage) +
                        ", \"u\": " + GD (u) + ", \"state\": " + s + ", \"motif\": " + m + "}" + (k + 1 < steps ? "," : "") + "\n");
                    if (!double.IsNaN (u))
                        uPrev = u;
                }
                sb.Append ("    ]}" + (q + 1 < orders.Length ? "," : "") + "\n");
            }
            sb.Append ("  ]\n}\n");
            return sb.ToString ();
        }

        static double[] ArraySegmentCopy (double[] a, int n)
        {
            var r = new double[n];
            Array.Copy (a, r, n);
            return r;
        }
    }
}
