// -----------------------------------------------------------------------------------
// option 16384 of AttitudeOptionsV1, the RCS as ModuleRCS fires it (RcsCombined.cs). What these tests pin down:
//   * the pure maths: a module's single-axis command gives the column of AttitudeGeometry.AddRcsColumns (re-coded here
//     from the decompiled rule), the combined rule is homogeneous in |u|, ModuleRCS's dead zones (Update's per component,
//     CalculateThrust's per nozzle) and fullThrust; on the heavy booster's ring (the geometry rebuilt on its coast
//     columns: pitch 2.78 MN.m, roll 0.243 MN.m per unit) a pure roll gives its column, pitch and yaw together their
//     columns' sum (1 %), and a roll beside pitch/yaw above the dead zone almost nothing (the flights' x 1/15) while
//     the pitch/yaw are answered x 1.5 and more;
//   * Decide: the law's command when the RCS answers it, else its roll left out or its pitch/yaw left out, whichever is
//     nearest what the law expects in its own metric (checked against a brute force), never the roll alone when the
//     step does not write pitch/yaw; on the heavy booster a roll and a pitch/yaw above the dead zone never together;
//   * option 16384 is a new bit (supported 32767) and changes NO existing rule;
//   * the CLOSED LOOP of the heavy booster's coast on three axes, with the law of the plugin (AttitudeMath.Step,
//     untouched), its RCS fired by the combined rule one render frame after the fly-by-wire (option 32 on, as flown),
//     a turning pitch/yaw reference and the coast's jitter, and a roll target (a 40 deg step, the pins of 120710, and
// an earlier change's 80 deg ramp), in a calm coast and a busy one (a pitch trim above the dead zone): without the option the
//     pitch/yaw chatter whenever the roll commands, the roll is answered x 1/15 and overshoots, the RCS burns; with it
//     no chatter beyond the coast's own, no roll overshoot beyond 1 deg, the pitch/yaw held as without roll, a fraction
//     of the propellant;
//   * the golden vectors (golden/attitude_v1_golden_rcs.json) the client's mirror replays (the client):
//     sets, torques by both rules, thrusts, decisions.
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static class RcsCombinedTests
    {
        static int checks;
        static int failures;
        // BRIDGE_D267_TRACE=1: a line every 10 steps of every closed loop; =<scenario name>: every step of that one.
        static readonly string traceName = Environment.GetEnvironmentVariable ("BRIDGE_D267_TRACE");
        static readonly bool trace = traceName == "1";

        const int Option = AttitudeWrite.OptionRcsCombined;
        const double Dt = 0.02;

        public static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestColumnsAreAddRcsColumns ();
            TestCombinedRule ();
            TestDeadZonesAndFullThrust ();
            TestHeavyRing ();
            TestDecide ();
            TestBits ();
            TestOptionChangesNoExistingRule ();
            TestClosedLoop ();
            checkCount = checks;
            Console.WriteLine ("rcs combined checks: " + checks + ", failures: " + failures);
            return failures;
        }

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

        // ------------------------------------------------------------------ geometries

        /// <summary>
        /// The heavy booster's RCS, rebuilt on its coast columns (a recorded flight): two rings h -+ 0.35 m
        /// above the CoM, four blocks at R on +-x and +-z, each with two axial nozzles (+-y) and two tangential ones; the
        /// nozzle force P so that the roll column is 8 P R. With R 2.53 m, h 57.9 m and P 12007 N: pitch 2.78 MN.m, roll
        /// 0.243 MN.m per unit (only R/h matters: the torques are homogeneous in R, h, 1/P).
        /// </summary>
        internal static RcsNozzleSet Heavy (double h = 57.9, double r = 2.53, double p = 12007.0, int modules = 8)
        {
            var s = new RcsNozzleSet ();
            double[] rings = { h - 0.35, h + 0.35 };
            int perRing = modules / 2;
            foreach (double y in rings)
                for (int a = 0; a < 4; a++) {
                    double px = a == 0 ? r : (a == 1 ? -r : 0.0);
                    double pz = a == 2 ? r : (a == 3 ? -r : 0.0);
                    double[][] vs = a < 2
                        ? new[] { new double[] { 0, 1, 0 }, new double[] { 0, -1, 0 }, new double[] { 0, 0, 1 }, new double[] { 0, 0, -1 } }
                        : new[] { new double[] { 0, 1, 0 }, new double[] { 0, -1, 0 }, new double[] { 1, 0, 0 }, new double[] { -1, 0, 0 } };
                    int first = s.Add (4, true, true, true, 0.05, false, 0.0);
                    for (int k = 0; k < 4; k++) {
                        s.Lever [3 * (first + k)] = px;
                        s.Lever [3 * (first + k) + 1] = y;
                        s.Lever [3 * (first + k) + 2] = pz;
                        for (int c = 0; c < 3; c++)
                            s.Dir [3 * (first + k) + c] = vs [k] [c];
                        s.Power [first + k] = p;
                    }
                }
            RcsCombined.Prepare (s, new double[3]);
            return s;
        }

        /// <summary>A random vessel: modules of random nozzles (unit directions, levers of a few metres), random flags.</summary>
        internal static RcsNozzleSet RandomSet (Random rng, int modules)
        {
            var s = new RcsNozzleSet ();
            for (int m = 0; m < modules; m++) {
                int count = 1 + rng.Next (6);
                int first = s.Add (count, rng.Next (5) > 0, rng.Next (5) > 0, rng.Next (5) > 0, 0.02 + 0.06 * rng.NextDouble (),
                    rng.Next (6) == 0, 0.5 + 0.5 * rng.NextDouble ());
                for (int k = 0; k < count; k++) {
                    int i = first + k;
                    for (int c = 0; c < 3; c++)
                        s.Lever [3 * i + c] = 12.0 * (rng.NextDouble () - 0.5);
                    double vx = rng.NextDouble () - 0.5, vy = rng.NextDouble () - 0.5, vz = rng.NextDouble () - 0.5;
                    double vn = Math.Sqrt (vx * vx + vy * vy + vz * vz);
                    s.Dir [3 * i] = vx / vn;
                    s.Dir [3 * i + 1] = vy / vn;
                    s.Dir [3 * i + 2] = vz / vn;
                    s.Power [i] = 500.0 + 4000.0 * rng.NextDouble ();
                }
            }
            RcsCombined.Prepare (s, new double[3]);
            return s;
        }

        /// <summary>
        /// AttitudeGeometry.AddRcsColumns re-coded on the set's nozzles (ModuleRCS.FixedUpdate on a unit command of
        /// one axis: rot = axis x unit(ProjectOnPlane(r, axis)), fraction max(v . rot, 0) bounded at 1, no threshold).
        /// </summary>
        static double[] AddRcsColumnsRecoded (RcsNozzleSet s, int m, int a, double sign)
        {
            var t = new double[3];
            double[] axis = { a == 0 ? sign : 0, a == 1 ? sign : 0, a == 2 ? sign : 0 };
            for (int k = 0; k < s.Count [m]; k++) {
                int i = s.Start [m] + k;
                double[] r = { s.Lever [3 * i], s.Lever [3 * i + 1], s.Lever [3 * i + 2] };
                double rn = r [0] * axis [0] + r [1] * axis [1] + r [2] * axis [2];
                double[] perp = { r [0] - rn * axis [0], r [1] - rn * axis [1], r [2] - rn * axis [2] };
                double pn = Math.Sqrt (perp [0] * perp [0] + perp [1] * perp [1] + perp [2] * perp [2]);
                if (!(pn > 1e-5))
                    continue;
                double[] u = { perp [0] / pn, perp [1] / pn, perp [2] / pn };
                double[] rot = { axis [1] * u [2] - axis [2] * u [1], axis [2] * u [0] - axis [0] * u [2], axis [0] * u [1] - axis [1] * u [0] };
                double f = s.Dir [3 * i] * rot [0] + s.Dir [3 * i + 1] * rot [1] + s.Dir [3 * i + 2] * rot [2];
                if (!(f > 0.0))
                    continue;
                if (f > 1.0)
                    f = 1.0;
                double[] F = { -s.Dir [3 * i] * s.Power [i] * f, -s.Dir [3 * i + 1] * s.Power [i] * f, -s.Dir [3 * i + 2] * s.Power [i] * f };
                t [0] += r [1] * F [2] - r [2] * F [1];
                t [1] += r [2] * F [0] - r [0] * F [2];
                t [2] += r [0] * F [1] - r [1] * F [0];
            }
            return t;
        }

        // ------------------------------------------------------------------ pure maths

        static void TestColumnsAreAddRcsColumns ()
        {
            var rng = new Random (2670);
            double worst = 0.0;
            for (int draw = 0; draw < 40; draw++) {
                var s = draw == 0 ? Heavy () : RandomSet (rng, 1 + rng.Next (5));
                for (int m = 0; m < s.Modules; m++)
                    for (int a = 0; a < 3; a++) {
                        bool on = a == 0 ? s.EnablePitch [m] : (a == 1 ? s.EnableRoll [m] : s.EnableYaw [m]);
                        var pos = on ? AddRcsColumnsRecoded (s, m, a, 1.0) : new double[3];
                        var neg = on ? AddRcsColumnsRecoded (s, m, a, -1.0) : new double[3];
                        for (int r = 0; r < 3; r++) {
                            worst = Math.Max (worst, Math.Abs (s.ColPos [9 * m + 3 * a + r] - pos [r]) / Math.Max (1.0, Math.Abs (pos [r])));
                            // ColNeg is per unit of a negative command: the command -1 gave neg.
                            worst = Math.Max (worst, Math.Abs (s.ColNeg [9 * m + 3 * a + r] + neg [r]) / Math.Max (1.0, Math.Abs (neg [r])));
                        }
                    }
            }
            Check (worst < 1e-12, "Prepare: each module's columns are AddRcsColumns' rule (" + worst.ToString ("G3") + ")");
            var h = Heavy ();
            var t = new double[3];
            var c = new double[3];
            RcsCombined.Torque (h, 1.0, 0.0, 0.0, false, t, c);
            Check (Near (t [0], -2.78e6, 0.01), "heavy: pitch column 2.78 MN.m, KSP's sign (" + t [0].ToString ("F0") + ")");
            RcsCombined.Torque (h, 0.0, 1.0, 0.0, false, t, c);
            Check (Near (t [1], -2.43e5, 0.01), "heavy: roll column 0.243 MN.m (" + t [1].ToString ("F0") + ")");
        }

        static void TestCombinedRule ()
        {
            var rng = new Random (2671);
            var a = new double[3];
            var b = new double[3];
            var c = new double[3];
            bool homogeneous = true, singleAxis = true;
            for (int draw = 0; draw < 300; draw++) {
                // Single axis, above every dead zone and nozzle threshold, below the bound of 1: the columns.
                int ax = rng.Next (3);
                double u = (rng.Next (2) == 0 ? 1.0 : -1.0) * (0.3 + 0.7 * rng.NextDouble ());
                // The exact rule differs from the columns only by the per-nozzle threshold and fullThrust: compare
                // on a set without either.
                var plain = RandomSet (new Random (draw), 1 + draw % 3);
                for (int m = 0; m < plain.Modules; m++) {
                    plain.Epsilon [m] = 0.0;
                    plain.FullThrust [m] = false;
                    plain.EnablePitch [m] = plain.EnableRoll [m] = plain.EnableYaw [m] = true;
                }
                RcsCombined.Prepare (plain, c);
                RcsCombined.Torque (plain, ax == 0 ? u : 0, ax == 1 ? u : 0, ax == 2 ? u : 0, true, a, c);
                RcsCombined.Torque (plain, ax == 0 ? u : 0, ax == 1 ? u : 0, ax == 2 ? u : 0, false, b, c);
                for (int r = 0; r < 3; r++)
                    if (!Near (a [r], b [r], 1e-9))
                        singleAxis = false;
                // Homogeneous: half the command, half the torque (fractions below 1 and thresholds 0).
                double p = 0.4 * (rng.NextDouble () - 0.5), ro = 0.4 * (rng.NextDouble () - 0.5), y = 0.4 * (rng.NextDouble () - 0.5);
                RcsCombined.Torque (plain, p, ro, y, true, a, c);
                RcsCombined.Torque (plain, 0.5 * p, 0.5 * ro, 0.5 * y, true, b, c);
                for (int r = 0; r < 3; r++)
                    if (!Near (a [r], 2.0 * b [r], 1e-9))
                        homogeneous = false;
            }
            Check (singleAxis, "a single-axis command (no threshold, fraction under 1): the exact rule gives the columns");
            Check (homogeneous, "the combined rule is homogeneous in |u| below the bound and the thresholds");
            var none = new RcsNozzleSet ();
            RcsCombined.Prepare (none, c);
            Check (RcsCombined.Torque (none, 1, 1, 1, true, a, c) == 0.0 && a [0] == 0.0 && a [1] == 0.0 && a [2] == 0.0,
                "no module: no torque");
        }

        static void TestDeadZonesAndFullThrust ()
        {
            var h = Heavy ();
            var t = new double[3];
            var c = new double[3];
            // Update: a component under EPSILON is zero (the heavy's pitch 0.04 beside a roll: a pure roll).
            RcsCombined.Torque (h, 0.04, 0.5, -0.049, true, t, c);
            var pure = new double[3];
            RcsCombined.Torque (h, 0.0, 0.5, 0.0, true, pure, c);
            Check (t [0] == pure [0] && t [1] == pure [1] && t [2] == pure [2], "Update's dead zone: pitch 0.04 and yaw -0.049 vanish");
            // EPSILON itself is kept by Update (strict square) but CalculateThrust fires nothing at f <= EPSILON.
            RcsCombined.Torque (h, 0.0, 0.05, 0.0, true, t, c);
            Check (t [1] == 0.0, "CalculateThrust: a nozzle at f = EPSILON fires nothing");
            RcsCombined.Torque (h, 0.0, 0.051, 0.0, true, t, c);
            Check (Near (t [1], 0.051 * -2.43e5, 0.01), "just above: the column");
            // fullThrust: from fullThrustMin up the nozzle fires at 1.
            var s = Heavy ();
            for (int m = 0; m < s.Modules; m++) {
                s.FullThrust [m] = true;
                s.FullThrustMin [m] = 0.2;
            }
            RcsCombined.Torque (s, 0.0, 0.25, 0.0, true, t, c);
            Check (Near (t [1], -2.43e5, 0.01), "fullThrust above fullThrustMin: the full roll");
            RcsCombined.Torque (s, 0.0, 0.15, 0.0, true, t, c);
            Check (Near (t [1], 0.15 * -2.43e5, 0.01), "fullThrust below fullThrustMin: proportional");
            // Disabled axis: zero.
            var d = Heavy ();
            for (int m = 0; m < d.Modules; m++)
                d.EnableRoll [m] = false;
            RcsCombined.Prepare (d, c);
            RcsCombined.Torque (d, 0.0, 1.0, 0.0, true, t, c);
            Check (t [0] == 0.0 && t [1] == 0.0 && t [2] == 0.0, "roll disabled: nothing");
        }

        static void TestHeavyRing ()
        {
            var h = Heavy ();
            var t = new double[3];
            var l = new double[3];
            var c = new double[3];
            // Pitch and yaw together: their columns' sum, to 1 % (|r x u| ~ h whatever the axis in the x-z plane).
            double worst = 0.0;
            var rng = new Random (2672);
            for (int i = 0; i < 400; i++) {
                double p = 2.0 * rng.NextDouble () - 1.0, y = 2.0 * rng.NextDouble () - 1.0;
                if (Math.Abs (p) < 0.06 || Math.Abs (y) < 0.06)
                    continue;
                RcsCombined.Torque (h, p, 0.0, y, true, t, c);
                RcsCombined.Torque (h, p, 0.0, y, false, l, c);
                worst = Math.Max (worst, Math.Max (Math.Abs (t [0] - l [0]), Math.Abs (t [2] - l [2])) / 2.78e6);
            }
            Check (worst < 0.01, "heavy: pitch and yaw together are their columns' sum (" + (100 * worst).ToString ("F2") + " % of a column)");
            // The flight's mixed commands: roll 0.3 with pitch 0.2 -- the roll almost gone, the pitch x 1.5 and more.
            RcsCombined.Torque (h, 0.2, 0.3, 0.0, true, t, c);
            RcsCombined.Torque (h, 0.2, 0.3, 0.0, false, l, c);
            Check (Math.Abs (t [1]) < 0.15 * Math.Abs (l [1]), "heavy, (0.2, 0.3, 0): the roll x " + (t [1] / l [1]).ToString ("F3"));
            Check (Math.Abs (t [0]) > 1.4 * Math.Abs (l [0]), "heavy, (0.2, 0.3, 0): the pitch x " + (t [0] / l [0]).ToString ("F2"));
            // The AutoPilot's roll at 1 beside a small pitch (x 4.8 measured on the heavy booster).
            RcsCombined.Torque (h, 0.15, 1.0, 0.0, true, t, c);
            RcsCombined.Torque (h, 0.15, 1.0, 0.0, false, l, c);
            Check (Math.Abs (t [0]) > 3.0 * Math.Abs (l [0]) && Math.Abs (t [1]) < 0.3 * Math.Abs (l [1]),
                "heavy, roll 1 and pitch 0.15: pitch x " + (t [0] / l [0]).ToString ("F2") + ", roll x " + (t [1] / l [1]).ToString ("F2"));
            // A pitch inside the ring's angle (|p| < R/h |roll|) leaves the roll whole and loses the pitch. On the heavy
            // booster of the coast columns (R/h = 0.044) no pitch above the dead zone is that small: roll and pitch/yaw
            // exclude each other; on the ring of the pad (h 37.25 m, R/h = 0.068) a pitch of 0.06 is.
            var pad = Heavy (37.25, 2.53, 9825.0);
            RcsCombined.Torque (pad, 0.06, 1.0, 0.0, true, t, c);
            RcsCombined.Torque (pad, 0.0, 1.0, 0.0, false, l, c);
            Check (t [1] / l [1] > 0.8 && Math.Abs (t [0]) < 0.1 * 0.06 * 1.46e6,
                "pad ring, roll 1 and pitch 0.06 (axis inside the ring): the roll kept (x " + (t [1] / l [1]).ToString ("F2") +
                "), the pitch lost");
            RcsCombined.Torque (h, 0.06, 1.0, 0.0, true, t, c);
            RcsCombined.Torque (h, 0.0, 1.0, 0.0, false, l, c);
            Check (t [1] / l [1] < 0.35 && Math.Abs (t [0]) > 10.0 * 0.06 * 2.78e6,
                "heavy, roll 1 and pitch 0.06 (axis outside the ring): the roll x " + (t [1] / l [1]).ToString ("F2") +
                ", the pitch x " + (t [0] / (-0.06 * 2.78e6)).ToString ("F1"));
            if (trace)
                Console.WriteLine ("heavy: mixed roll x {0:F3}", t [1] / l [1]);
        }

        static double[] Weights (double ix, double iy, double iz)
        {
            var g = new AttitudeGains ();
            return new[] { g.WvPitchYaw / ix, g.WvRoll / iy, g.WvPitchYaw / iz };
        }

        static void TestDecide ()
        {
            var h = Heavy ();
            var w = new RcsCombinedWork ();
            var rest = new double[9];
            var wv = Weights (6.2e7, 9.78e5, 6.2e7);
            int keep = RcsCombined.KeepCommand, noRoll = RcsCombined.RollHeld, rollAlone = RcsCombined.PitchYawHeld;
            // From a served command of 0 (the law's increment is then its whole command).
            var zero = new double[3];
            Check (RcsCombined.Decide (h, zero, 0.0, 0.3, 0.0, rest, wv, true, 0, 0, w) == keep, "a pure roll: kept");
            Check (RcsCombined.Decide (h, zero, 0.3, 0.0, -0.2, rest, wv, true, 0, 0, w) == keep, "pitch/yaw alone: kept");
            Check (RcsCombined.Decide (h, zero, 0.04, 0.8, -0.03, rest, wv, true, 0, 0, w) == keep,
                "pitch/yaw in the dead zone beside a roll: a pure roll for ModuleRCS, kept");
            Check (RcsCombined.Decide (h, zero, 0.6, 0.06, 0.4, rest, wv, true, 0, 0, w) != rollAlone,
                "a small roll beside large pitch/yaw: the pitch/yaw kept");
            Check (RcsCombined.Decide (h, zero, 0.06, 1.0, 0.0, rest, wv, true, 0, 0, w) == rollAlone,
                "a full roll beside a small pitch: the pitch left out (the law's metric)");
            Check (RcsCombined.Decide (h, zero, 0.06, 1.0, 0.0, rest, wv, false, 0, 0, w) != rollAlone,
                "the same when the step does not write pitch/yaw: never the roll alone (the pitch/yaw are not the loop's)");
            Check (RcsCombined.Decide (h, zero, 0.2, 0.3, 0.0, rest, wv, true, 0, 0, w) == rollAlone &&
                RcsCombined.Decide (h, zero, -0.2, -0.3, 0.1, rest, wv, true, 0, 0, w) == rollAlone,
                "the flight's mixed commands (133906: pitch 0.2 beside roll 0.3): the roll alone");
            Check (RcsCombined.Decide (Heavy (37.25, 2.53, 9825.0), zero, 0.03, -0.9, 0.0, rest, wv, true, 0, 0, w) == keep,
                "pad ring, pitch in the dead zone: a pure roll, kept");
            Check (RcsCombined.Decide (h, zero, double.NaN, 0.5, 0.0, rest, wv, true, 0, 0, w) == keep &&
                RcsCombined.Decide (h, zero, 0.1, double.PositiveInfinity, 0.0, rest, wv, true, 0, 0, w) == keep, "not finite: kept");
            // Every decision is the candidate nearest the law's expectation by the combined rule (brute force), the
            // law's command first on a tie, then pitch/yaw alone.
            var rng = new Random (2673);
            var c = new double[3];
            var ex = new double[3];
            var t = new double[3];
            int wrong = 0, cases = 0, kept = 0;
            double worstSpoil = 0.0;
            for (int i = 0; i < 3000; i++) {
                var s = i % 3 == 0 ? h : RandomSet (rng, 1 + rng.Next (4));
                double p = 2.0 * rng.NextDouble () - 1.0, r = 2.0 * rng.NextDouble () - 1.0, y = 2.0 * rng.NextDouble () - 1.0;
                var rs = new double[9];
                if (i % 4 == 0)
                    for (int k = 0; k < 9; k++)
                        rs [k] = 2e4 * (rng.NextDouble () - 0.5);
                bool own = i % 5 != 0;
                int heldR = i % 7 == 0 ? 1 + i % 4 : 0, heldPy = i % 7 == 1 ? 1 + i % 3 : 0;
                var sv = i % 2 == 0 ? zero : new[] { Pick (rng), Pick (rng), Pick (rng) };
                int d = RcsCombined.Decide (s, sv, p, r, y, rs, wv, own, heldR, heldPy, w);
                // The law's expectation: the served command's combined torque plus the increment through the mean
                // columns, plus the rest at the command.
                RcsCombined.Torque (s, sv [0], sv [1], sv [2], true, ex, c);
                for (int k = 0; k < 3; k++) {
                    double mp = 0.0, mr = 0.0, my = 0.0;
                    for (int m = 0; m < s.Modules; m++) {
                        mp += 0.5 * (s.ColPos [9 * m + k] + s.ColNeg [9 * m + k]);
                        mr += 0.5 * (s.ColPos [9 * m + 3 + k] + s.ColNeg [9 * m + 3 + k]);
                        my += 0.5 * (s.ColPos [9 * m + 6 + k] + s.ColNeg [9 * m + 6 + k]);
                    }
                    ex [k] += (p - sv [0]) * mp + (r - sv [1]) * mr + (y - sv [2]) * my;
                    ex [k] += p * rs [k] + r * rs [3 + k] + y * rs [6 + k];
                }
                double best = double.PositiveInfinity;
                var err = new double[3];
                double[][] cand = { new[] { p, r, y }, new[] { p, RcsCombined.HeldValue (s, 1, r), y },
                    new[] { RcsCombined.HeldValue (s, 0, p), r, RcsCombined.HeldValue (s, 2, y) } };
                for (int q = 0; q < 3; q++) {
                    err [q] = double.PositiveInfinity;
                    if ((q == 2 && (!own || (cand [2] [0] == p && cand [2] [2] == y))) || (q == 1 && cand [1] [1] == r))
                        continue;
                    RcsCombined.Torque (s, cand [q] [0], cand [q] [1], cand [q] [2], true, t, c);
                    double e = 0.0;
                    for (int k = 0; k < 3; k++) {
                        double tk = t [k] + cand [q] [0] * rs [k] + cand [q] [1] * rs [3 + k] + cand [q] [2] * rs [6 + k];
                        e += (wv [k] * (tk - ex [k])) * (wv [k] * (tk - ex [k]));
                    }
                    err [q] = e * (q == 1 ? 1.0 + heldR : (q == 2 ? 1.0 + heldPy : 1.0));
                    best = Math.Min (best, err [q]);
                }
                cases++;
                // The chosen candidate is the best one (to round-off), and the law's command whenever there is nothing
                // to choose (no roll, or no pitch/yaw).
                bool trivial = r == 0.0 || (p == 0.0 && y == 0.0);
                if (!(err [d] <= best * (1.0 + 1e-9) + 1e-300) || (trivial && d != keep))
                    wrong++;
                if (s == h && own && heldR == 0 && heldPy == 0 && sv == zero && d == keep && Math.Abs (r) >= 0.05 && Math.Max (Math.Abs (p), Math.Abs (y)) >= 0.05 &&
                        rs [0] == 0.0) {
                    // a roll kept beside pitch/yaw on the heavy ring: how far the pitch/yaw torque is from the expected
                    kept++;
                    RcsCombined.Torque (s, p, r, y, true, t, c);
                    worstSpoil = Math.Max (worstSpoil, Math.Sqrt ((t [0] - ex [0]) * (t [0] - ex [0]) + (t [2] - ex [2]) * (t [2] - ex [2])) /
                        Math.Sqrt (ex [0] * ex [0] + ex [2] * ex [2]));
                }
            }
            Check (wrong == 0, "Decide: the cheapest candidate, a part's waiting counted (" + wrong + " of " + cases + ")");
            // A part that has waited weighs more: the flight's mixed command after 20 steps of roll alone gives pitch/yaw
            // their turn; a full roll beside a small pitch keeps the RCS for the roll a few steps, not for ever.
            Check (RcsCombined.Decide (h, zero, 0.2, 0.3, 0.0, rest, wv, true, 0, 20, w) != rollAlone,
                "the flight's mixed command after 20 steps of roll alone: pitch/yaw served (they do not starve)");
            int run = 0;
            while (run < 1000 && RcsCombined.Decide (h, zero, 0.06, 1.0, 0.0, rest, wv, true, 0, run, w) == rollAlone)
                run++;
            Check (run > 0 && run < 1000, "a full roll beside a small pitch: the roll alone " + run + " steps in a row, then pitch");
            run = 0;
            var wheelRoll = new double[9];
            wheelRoll [4] = -3.0e4;
            while (run < 1000 && RcsCombined.Decide (Heavy (), zero, 0.2, 0.05, 0.1, wheelRoll, wv, true, run, 0, w) == noRoll)
                run++;
            Check (run < 1000, "a roll that also drives a wheel is not held for ever (" + run + " steps)");
            Check (worstSpoil < 0.2, "heavy: a roll kept beside the loop's own pitch/yaw (" + kept + " draws) spoils them by at most " +
                (100.0 * worstSpoil).ToString ("F0") + " % (the flights' mixed commands: x 1.5 to x 4.8)");
            // Wheels only (no RCS module): linear, always kept.
            var none = new RcsNozzleSet ();
            RcsCombined.Prepare (none, new double[3]);
            var wheel = new[] { -3.0e4, 0, 0, 0, -3.0e4, 0, 0, 0, -3.0e4 };
            Check (RcsCombined.Decide (none, zero, 0.5, 0.5, 0.5, wheel, wv, true, 0, 0, w) == keep, "wheels only: always kept");
        }

        // ------------------------------------------------------------------ the option

        static void TestBits ()
        {
            Check (Option == 16384 && (AttitudeWrite.OptionsSupported & 32767) == 32767,
                "option 16384, supported 32767 (mirrored by the client)");
            Check (AttitudeWrite.ValidOptions (16384) && AttitudeWrite.ValidOptions (32767) &&
                !AttitudeWrite.ValidOptions (AttitudeWrite.OptionsSupported + 1) && !AttitudeWrite.ValidOptions (-1),
                "valid options 0-32767");
            for (int o = 0; o < 16384; o += 7)
                Check ((o & Option) == 0, "no older option holds bit 16384 (" + o + ")");
            Check (!AttitudeWrite.AllocationOn (Option), "16384 is not the allocation");
        }

        static void TestOptionChangesNoExistingRule ()
        {
            var rng = new Random (2676);
            for (int options = 0; options < 16384; options += 13) {
                Check (AttitudeWrite.RollBlockingSet (options | Option) == AttitudeWrite.RollBlockingSet (options) &&
                    AttitudeWrite.PitchYawBlockingSet (options | Option) == AttitudeWrite.PitchYawBlockingSet (options) &&
                    AttitudeWrite.AllowedWriteMask (options | Option) == AttitudeWrite.AllowedWriteMask (options),
                    "option 16384: blocking sets and write mask of options " + options);
                for (int w = 0; w < 32; w++)
                    Check (AttitudeWrite.EffectiveWriteMask (w, options | Option) == AttitudeWrite.EffectiveWriteMask (w, options) &&
                        AttitudeWrite.ValidWriteMask (w, options | Option) == AttitudeWrite.ValidWriteMask (w, options),
                        "option 16384: write masks");
            }
            for (int i = 0; i < 5000; i++) {
                long motif = ((long)rng.Next () << 31) ^ rng.Next () ^ ((long)rng.Next (0, 2048) << 32);
                if (i < 64)
                    motif = 1L << i % 42;
                int options = rng.Next (0, 16384);
                for (int mode = 0; mode < 2; mode++)
                    for (int axes = 0; axes < 4; axes++)
                        for (int stage = 1; stage < 3; stage++)
                            Check (AttitudeWrite.MayWrite (mode, axes, stage, true, motif, options | Option) ==
                                AttitudeWrite.MayWrite (mode, axes, stage, true, motif, options) &&
                                AttitudeWrite.MayWritePitchYaw (mode, axes, stage, true, motif, options | Option) ==
                                AttitudeWrite.MayWritePitchYaw (mode, axes, stage, true, motif, options) &&
                                AttitudeWrite.ContactHold (options | Option, mode, axes, stage, true, true, motif) ==
                                AttitudeWrite.ContactHold (options, mode, axes, stage, true, true, motif),
                                "option 16384: the channels' permissions identical");
                int unneutralizable = rng.Next (0, 8);
                Check (AttitudeWrite.FrozenCtrlAxes (options | Option, motif, unneutralizable) ==
                    AttitudeWrite.FrozenCtrlAxes (options, motif, unneutralizable), "option 16384: frozen axes identical");
                int now = rng.Next (0, 8), last = rng.Next (0, 8), answering = rng.Next (0, 8);
                Check (AttitudeWrite.NeutralizeSurfaces (options | Option, AttitudeWrite.ModeArmed, AttitudeWrite.StageEarlyish,
                    now, last, answering) == AttitudeWrite.NeutralizeSurfaces (options, AttitudeWrite.ModeArmed,
                    AttitudeWrite.StageEarlyish, now, last, answering), "option 16384: neutralization identical");
                Check (AttitudeWrite.RollSurfaceGate (options | Option, answering, (i & 1) == 0) ==
                    AttitudeWrite.RollSurfaceGate (options, answering, (i & 1) == 0), "option 16384: roll gate identical");
            }
        }

        // ------------------------------------------------------------------ the closed loop

        internal sealed class Scenario
        {
            internal string Name;
            internal bool Combined;                 // option 16384
            internal double RollStepDeg = 40.0;     // the pins' roll error at the start (120710: 41 deg), 0: none
            internal double RampDeg = 0.0;          // the roof turned at RampRateDeg from 2 s, 4 s ramps
            internal double RampRateDeg = 1.75;
            // N.m: the coast's external pitch/yaw torque (133906, 181655, coast +30-80 s: p10-p90 -30..+34 kN.m, median
            // 0-21): a small trim, a slow swell and a white part (the measured omega_dot's jitter, sigma ~23 kN.m on
            // pitch/yaw, under 0.5 kN.m on roll). Busy: a pitch trim above the dead zone's 139 kN.m.
            internal double PitchTrim = 8.0e3;
            internal double YawTrim = -10.0e3;
            internal double Swell = 15.0e3;
            internal double NoisePitchYaw = 23.0e3;
            internal double NoiseRoll = 300.0;
            internal int Seed = 2679;
            // rad/s: the coast's pitch/yaw reference turns (the retrograde direction bending: the witness 181655 at coast
            // +60 s, body rates -0.005 / +0.01 rad/s, errors 0.4 / 0.8 deg held by the RCS on every other step).
            internal double PitchRate = -0.005, YawRate = 0.01, RateSwing = 0.003;
            internal int SkipEvery = 20;            // two physics steps in one render frame, one in 20
            internal double Seconds = 60.0;
            internal double Ix = 6.2e7, Iy = 9.78e5, Iz = 6.2e7;
        }

        internal sealed class Result
        {
            internal double ChatterPerS, ThrustImpulse, RollOvershootDeg, PeakPitchYawDeg, RmsPitchYawDeg, RollFinalErrDeg;
            internal double RollHeld, PitchYawHeld, PitchYawActive, SettleS = double.NaN, MaxRollCmd;
            internal bool Finite = true;
        }

        static double RollTargetDeg (Scenario s, double t)
        {
            if (s.RampDeg <= 0.0)
                return s.RollStepDeg;
            // a trapezoid of rate: from 2 s, ramps of 4 s to RampRateDeg, down again to reach RampDeg
            double w = s.RampRateDeg, tr = 4.0, t0 = 2.0;
            double cruise = (s.RampDeg - w * tr) / w;
            double x = t - t0;
            if (x <= 0.0)
                return 0.0;
            if (x < tr)
                return 0.5 * w / tr * x * x;
            if (x < tr + cruise)
                return 0.5 * w * tr + w * (x - tr);
            double y = x - tr - cruise;
            if (y < tr)
                return 0.5 * w * tr + w * cruise + w * y - 0.5 * w / tr * y * y;
            return s.RampDeg;
        }

        internal static Result Simulate (Scenario s)
        {
            var res = new Result ();
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            var set = Heavy ();
            var rw = new RcsCombinedWork ();
            const int n = 3;
            inp.Resize (n);
            inp.Dt = Dt;
            var c = new double[3];
            var next = new double[3];
            var before = new double[3];
            var inputRot = new double[3];       // what the last Update stored (body): pitch, roll, yaw after the dead zone
            var fired = new double[3];
            var firedPrevious = new double[3];
            var ctrl = new double[3];           // ctrlState pitch, roll, yaw (the served command)
            var omega = new double[3];
            omega [0] = s.PitchRate;
            omega [2] = s.YawRate + s.RateSwing;
            var omegaPrevious = new double[3];
            var angle = new double[3];          // small angles (pitch, yaw) and the roll angle, rad
            var measured = new double[3];
            var torque = new double[3];
            var rest = new double[9];
            var weight = Weights (s.Ix, s.Iy, s.Iz);
            double[] inertia = { s.Ix, s.Iy, s.Iz };
            int steps = (int)Math.Round (s.Seconds / Dt);
            int chatter = 0, rollHeld = 0, pitchYawHeld = 0, written = 0, active = 0;
            int lastSignP = 0, lastSignY = 0, runRoll = 0, runPitchYaw = 0;
            double maxBeyond = 0.0, sumSq = 0.0;
            double targetFinal = s.RampDeg > 0.0 ? s.RampDeg : s.RollStepDeg;
            var noise = new Random (s.Seed);
            RcsCombined.Prepare (set, c);
            for (int a = 0; a < 3; a++)
                for (int r = 0; r < 3; r++) {
                    double sum = 0.0;
                    for (int m = 0; m < set.Modules; m++)
                        sum += 0.5 * (set.ColPos [9 * m + 3 * a + r] + set.ColNeg [9 * m + 3 * a + r]);
                    inp.B [r * n + a] = sum;
                }
            for (int k = 0; k < steps; k++) {
                double t = k * Dt;
                bool frame = s.SkipEvery <= 0 || k % s.SkipEvery != 1;
                if (frame)
                    RcsInFlight.UpdateCommand (ctrl [0], ctrl [1], ctrl [2], true, true, true, 0.05, inputRot);
                for (int x = 0; x < 3; x++)
                    fired [x] = inputRot [x];
                // ---- Earlyish: the law (the plugin's, untouched).
                double rollTarget = RollTargetDeg (s, t) * Math.PI / 180.0;
                for (int x = 0; x < 3; x++) {
                    inp.Omega [x] = omega [x];
                    inp.OmegaDotMeasured [x] = (omega [x] - omegaPrevious [x]) / Dt;
                    inp.Inertia [x] = inertia [x];
                    inp.Torque [x] = measured [x];
                    inp.TorqueFree [x] = 0.0;
                    inp.OmegaFf [x] = 0.0;
                    inp.OmegaDotFf [x] = 0.0;
                    inp.OmegaMaxAxis [x] = x == 1 ? 0.1 : 0.0;
                }
                double refX = s.PitchRate * t + s.RateSwing / 0.2 * (1.0 - Math.Cos (0.2 * t));
                double refZ = s.YawRate * t + s.RateSwing / 0.13 * Math.Sin (0.13 * t);
                inp.Error [0] = refX - angle [0];
                inp.Error [1] = rollTarget - angle [1];
                inp.Error [2] = refZ - angle [2];
                inp.OmegaMax = 0.2;
                inp.OmegaDotValid = k > 0;
                inp.ReferenceValid = true;
                inp.RollTargetValid = true;
                for (int x = 0; x < 3; x++) {
                    inp.U0 [x] = ctrl [x];
                    inp.UMin [x] = -1.0;
                    inp.UMax [x] = 1.0;
                    inp.UPref [x] = 0.0;
                    inp.Priority [x] = 3.0;
                }
                // Option 32 (as flown): the in-flight torque, the served command's next Update minus the command fired at
                // the step the measured torque came from -- by the per-axis columns without an earlier change, by the combined rule
                // with it (AttitudeV1RcsCombined.RcsCombinedInFlight).
                if (k > 0) {
                    RcsCombined.Torque (set, ctrl [0], ctrl [1], ctrl [2], s.Combined, next, c);
                    before [0] = before [1] = before [2] = 0.0;
                    if (s.Combined) {
                        for (int m = 0; m < set.Modules; m++)
                            RcsCombined.ModuleTorque (set, m, firedPrevious [0], firedPrevious [1], firedPrevious [2], true, before);
                    } else {
                        RcsCombined.Torque (set, firedPrevious [0], firedPrevious [1], firedPrevious [2], false, before, c);
                    }
                    for (int x = 0; x < 3; x++)
                        inp.TorqueFree [x] = next [x] - before [x];
                    inp.UseTorqueFree = true;
                }
                AttitudeMath.Step (g, inp, st, work, o);
                // ---- the fly-by-wire: the law's command, or the one the RCS answers.
                if (o.Computed) {
                    double p = Clamp1 (ctrl [0] + o.DeltaU [0]);
                    double r = Clamp1 (ctrl [1] + o.DeltaU [1]);
                    double y = Clamp1 (ctrl [2] + o.DeltaU [2]);
                    if (s.Combined) {
                        int d = RcsCombined.Decide (set, ctrl, p, r, y, rest, weight, true, runRoll, runPitchYaw, rw);
                        runRoll = d == RcsCombined.RollHeld ? runRoll + 1 : 0;
                        runPitchYaw = d == RcsCombined.PitchYawHeld ? runPitchYaw + 1 : 0;
                        if (d == RcsCombined.RollHeld) {
                            r = RcsCombined.HeldValue (set, 1, r);
                            rollHeld++;
                        } else if (d == RcsCombined.PitchYawHeld) {
                            p = RcsCombined.HeldValue (set, 0, p);
                            y = RcsCombined.HeldValue (set, 2, y);
                            pitchYawHeld++;
                        }
                    }
                    written++;
                    ctrl [0] = p;
                    ctrl [1] = r;
                    ctrl [2] = y;
                    res.MaxRollCmd = Math.Max (res.MaxRollCmd, Math.Abs (r));
                }
                // ---- the parts: the RCS fires what its last Update stored (combined rule, nozzle thresholds).
                double thrust = 0.0;
                torque [0] = torque [1] = torque [2] = 0.0;
                for (int m = 0; m < set.Modules; m++)
                    thrust += RcsCombined.ModuleTorque (set, m, fired [0], fired [1], fired [2], true, torque);
                double sw = Math.Sin (2.0 * Math.PI * 0.2 * t), cw = Math.Cos (2.0 * Math.PI * 0.13 * t);
                double[] dist = { s.PitchTrim + s.Swell * sw + s.NoisePitchYaw * Gauss (noise), s.NoiseRoll * Gauss (noise),
                    s.YawTrim + s.Swell * cw + s.NoisePitchYaw * Gauss (noise) };
                if (Math.Abs (ctrl [0]) >= 0.05 || Math.Abs (ctrl [2]) >= 0.05)
                    active++;
                for (int x = 0; x < 3; x++) {
                    omegaPrevious [x] = omega [x];
                    omega [x] += Dt * (torque [x] + dist [x]) / inertia [x];
                    angle [x] += Dt * omega [x];
                    measured [x] = torque [x];
                    firedPrevious [x] = fired [x];
                }
                if (double.IsNaN (omega [0] + omega [1] + omega [2])) {
                    res.Finite = false;
                    break;
                }
                res.ThrustImpulse += Dt * thrust;
                // chatter: the reversals of the pitch and yaw commands above the dead zone (the flight's +-0.2 every two
                // steps: 25 per second and axis)
                int sp = Math.Abs (ctrl [0]) >= 0.05 ? Math.Sign (ctrl [0]) : 0, sy = Math.Abs (ctrl [2]) >= 0.05 ? Math.Sign (ctrl [2]) : 0;
                if (sp != 0 && lastSignP != 0 && sp != lastSignP)
                    chatter++;
                if (sy != 0 && lastSignY != 0 && sy != lastSignY)
                    chatter++;
                if (sp != 0)
                    lastSignP = sp;
                if (sy != 0)
                    lastSignY = sy;
                double rollDeg = angle [1] * 180.0 / Math.PI;
                double beyond = targetFinal == 0.0 ? 0.0 : (rollDeg - targetFinal) * Math.Sign (targetFinal);
                if (t > 2.0 && beyond > maxBeyond)
                    maxBeyond = beyond;
                res.PeakPitchYawDeg = Math.Max (res.PeakPitchYawDeg, Math.Max (Math.Abs (inp.Error [0]), Math.Abs (inp.Error [2])) * 180.0 / Math.PI);
                sumSq += inp.Error [0] * inp.Error [0] + inp.Error [2] * inp.Error [2];
                if (double.IsNaN (res.SettleS) && Math.Abs (rollDeg - targetFinal) < 1.0 && Math.Abs (omega [1]) < 0.002 &&
                        RollTargetDeg (s, t) == targetFinal)
                    res.SettleS = t;
                if ((trace && k % 10 == 0) || (traceName != null && traceName == s.Name))
                    Console.WriteLine ("{0} t={1:F2} u=({2:F3} {3:F3} {4:F3}) roll={5:F2} tgt={6:F2} w={7:F4} ex={8:F3} ez={9:F3} thrust={10:F0}",
                        s.Name, t, ctrl [0], ctrl [1], ctrl [2], rollDeg, RollTargetDeg (s, t), omega [1], inp.Error [0] * 57.3,
                        inp.Error [2] * 57.3, thrust);
            }
            res.ChatterPerS = chatter / s.Seconds;
            res.RollOvershootDeg = maxBeyond;
            res.RollFinalErrDeg = Math.Abs (angle [1] * 180.0 / Math.PI - targetFinal);
            res.RollHeld = written > 0 ? (double)rollHeld / written : 0.0;
            res.PitchYawHeld = written > 0 ? (double)pitchYawHeld / written : 0.0;
            res.PitchYawActive = (double)active / steps;
            res.RmsPitchYawDeg = Math.Sqrt (sumSq / steps) * 180.0 / Math.PI;
            return res;
        }

        static double Gauss (Random rng)
        {
            double u1 = 1.0 - rng.NextDouble (), u2 = rng.NextDouble ();
            return Math.Sqrt (-2.0 * Math.Log (u1)) * Math.Cos (2.0 * Math.PI * u2);
        }

        static double Clamp1 (double u)
        {
            return u > 1.0 ? 1.0 : (u < -1.0 ? -1.0 : u);
        }

        static void Report (Scenario s, Result r)
        {
            Console.WriteLine ("  {0,-44} chatter {1,5:F1}/s  RCS impulse {2,6:F0} kN.s  roll overshoot {3,5:F2} deg  settle {4,5:F1} s" +
                "  final {5,4:F2} deg  p/y error peak {6,4:F2} rms {7,5:F3} deg  roll held {8,4:P0}  p/y held {9,4:P0}  p/y active {10,4:P0}",
                s.Name, r.ChatterPerS, r.ThrustImpulse / 1000.0, r.RollOvershootDeg, r.SettleS, r.RollFinalErrDeg, r.PeakPitchYawDeg,
                r.RmsPitchYawDeg, r.RollHeld, r.PitchYawHeld, r.PitchYawActive);
        }

        static Scenario Busy (Scenario s, bool busy)
        {
            if (busy) {
                s.PitchTrim = 180.0e3;
                s.Swell = 120.0e3;
            }
            return s;
        }

        static void TestClosedLoop ()
        {
            foreach (bool busy in new[] { false, true }) {
                string coast = busy ? "busy coast" : "coast";
                // The coast without any roll to make: the pitch/yaw's own dead-zone cycle (the witnesses' coast).
                var calm = Busy (new Scenario { Name = coast + ", no roll", RollStepDeg = 0.0 }, busy);
                var rcalm = Simulate (calm);
                Report (calm, rcalm);
                var calmWith = Busy (new Scenario { Name = coast + ", no roll, option", RollStepDeg = 0.0, Combined = true }, busy);
                var rcalmWith = Simulate (calmWith);
                Report (calmWith, rcalmWith);
                Check (rcalm.Finite && rcalmWith.Finite &&
                    Math.Abs (rcalmWith.ThrustImpulse - rcalm.ThrustImpulse) < 0.05 * rcalm.ThrustImpulse,
                    coast + ", no roll: the option changes the RCS impulse by less than 5 % (" +
                    (rcalmWith.ThrustImpulse / rcalm.ThrustImpulse).ToString ("F3") + ")");
                double background = rcalm.ChatterPerS;
                foreach (bool ramp in new[] { false, true }) {
                    string what = coast + (ramp ? ", ramp 80 deg" : ", step 40 deg (120710)");
                    var without = Busy (new Scenario { Name = what + ", no option", RampDeg = ramp ? 80.0 : 0.0 }, busy);
                    var with = Busy (new Scenario { Name = what + ", option", Combined = true, RampDeg = ramp ? 80.0 : 0.0 }, busy);
                    var rn = Simulate (without);
                    var rc = Simulate (with);
                    Report (without, rn);
                    Report (with, rc);
                    double excessNo = rn.ThrustImpulse - rcalm.ThrustImpulse, excessWith = rc.ThrustImpulse - rcalm.ThrustImpulse;
                    Console.WriteLine ("  {0}: RCS impulse beyond the coast without roll {1:F0} kN.s without the option, {2:F0} with it",
                        what, excessNo / 1000.0, excessWith / 1000.0);
                    Check (rn.Finite && rc.Finite, what + ": finite");
                    if (!ramp && !busy)
                        Check (rn.ChatterPerS > 5.0 * background && rn.RollOvershootDeg > 3.0, without.Name +
                            ": the pitch/yaw chatter when the roll commands (" + rn.ChatterPerS.ToString ("F1") + "/s against " +
                            background.ToString ("F1") + " without roll) and the roll overshoots (" +
                            rn.RollOvershootDeg.ToString ("F1") + " deg; 120710: 4.9)");
                    Check (rc.ChatterPerS <= 1.5 * background + 1.0, with.Name + ": no chatter beyond the coast's own (" +
                        rc.ChatterPerS.ToString ("F2") + "/s against " + background.ToString ("F2") + ")");
                    Check (excessWith < (ramp ? 0.5 : 0.1) * excessNo || excessWith < 0.05 * rcalm.ThrustImpulse, with.Name +
                        ": the roll's RCS impulse " + (excessWith / excessNo).ToString ("P0") + " of the flown law's");
                    Check (rc.RollOvershootDeg < 1.0, with.Name + ": roll overshoot under 1 deg (" + rc.RollOvershootDeg.ToString ("F2") + ")");
                    Check (rc.RollFinalErrDeg < 0.5, with.Name + ": the roll reached (" + rc.RollFinalErrDeg.ToString ("F2") + " deg)");
                    Check (rc.PeakPitchYawDeg <= rcalm.PeakPitchYawDeg + 0.1 && rc.RmsPitchYawDeg <= rcalm.RmsPitchYawDeg + 0.02,
                        with.Name + ": pitch/yaw held as in the coast without roll (peak " + rc.PeakPitchYawDeg.ToString ("F2") +
                        " against " + rcalm.PeakPitchYawDeg.ToString ("F2") + " deg, rms " + rc.RmsPitchYawDeg.ToString ("F3") +
                        " against " + rcalm.RmsPitchYawDeg.ToString ("F3") + ")");
                }
            }
        }

        // ------------------------------------------------------------------ golden vectors

        internal static string GoldenJson ()
        {
            var sb = new StringBuilder ();
            sb.Append ("{\n  \"schema\": 1,\n  \"source\": \"krpc_bridge build/tests (RcsCombined.cs, option 16384)\",\n");
            var rng = new Random (2677);
            var sets = new List<RcsNozzleSet> { Heavy (), Heavy (37.25, 2.53, 9825.0) };
            for (int i = 0; i < 4; i++)
                sets.Add (RandomSet (rng, 1 + rng.Next (4)));
            sb.Append ("  \"sets\": [\n");
            for (int k = 0; k < sets.Count; k++) {
                var s = sets [k];
                sb.Append ("    {\"modules\": [");
                for (int m = 0; m < s.Modules; m++) {
                    sb.Append ((m > 0 ? ", " : "") + "{\"enable\": [" + B (s.EnablePitch [m]) + ", " + B (s.EnableRoll [m]) + ", " +
                        B (s.EnableYaw [m]) + "], \"epsilon\": " + D (s.Epsilon [m]) + ", \"full_thrust\": " + B (s.FullThrust [m]) +
                        ", \"full_thrust_min\": " + D (s.FullThrustMin [m]) + ", \"lever\": " + V (s.Lever, 3 * s.Start [m], 3 * s.Count [m]) +
                        ", \"dir\": " + V (s.Dir, 3 * s.Start [m], 3 * s.Count [m]) + ", \"power\": " + V (s.Power, s.Start [m], s.Count [m]) + "}");
                }
                sb.Append ("], \"col_pos\": " + V (s.ColPos, 0, 9 * s.Modules) + ", \"col_neg\": " + V (s.ColNeg, 0, 9 * s.Modules) +
                    ", \"held_limit\": " + V (s.HeldLimit, 0, 3) + ", \"col_mean\": " + V (s.ColMean, 0, 9) + "}" + (k + 1 < sets.Count ? "," : "") + "\n");
            }
            sb.Append ("  ],\n");
            var weight = Weights (6.2e7, 9.78e5, 6.2e7);
            sb.Append ("  \"weight\": " + V (weight, 0, 3) + ",\n");
            sb.Append ("  \"cases\": [\n");
            var t = new double[3];
            var c = new double[3];
            var w = new RcsCombinedWork ();
            int total = 0;
            for (int k = 0; k < sets.Count; k++) {
                var s = sets [k];
                for (int i = 0; i < 50; i++) {
                    double p = Pick (rng), r = Pick (rng), y = Pick (rng);
                    var rest = new double[9];
                    if (i % 5 == 0)
                        for (int q = 0; q < 9; q++)
                            rest [q] = 2e4 * (rng.NextDouble () - 0.5);
                    bool own = i % 7 != 0;
                    int heldR = i % 4 == 1 ? i % 5 : 0, heldPy = i % 4 == 2 ? 1 + i % 3 : 0;
                    double thrust = RcsCombined.Torque (s, p, r, y, true, t, c);
                    var exact = new[] { t [0], t [1], t [2] };
                    RcsCombined.Torque (s, p, r, y, false, t, c);
                    var served = i % 3 == 0 ? new double[3] : new[] { Pick (rng), Pick (rng), Pick (rng) };
                    int decision = RcsCombined.Decide (s, served, p, r, y, rest, weight, own, heldR, heldPy, w);
                    sb.Append ((total > 0 ? ",\n" : "") + "    {\"set\": " + k + ", \"u\": [" + D (p) + ", " + D (r) + ", " + D (y) +
                        "], \"served\": " + V (served, 0, 3) + ", \"rest\": " + V (rest, 0, 9) + ", \"own_pitch_yaw\": " + B (own) + ", \"roll_held_before\": " + heldR + ", \"pitch_yaw_held_before\": " + heldPy + ", \"exact\": " + V (exact, 0, 3) +
                        ", \"thrust\": " + D (thrust) + ", \"linear\": " + V (t, 0, 3) + ", \"decision\": " + decision + "}");
                    total++;
                }
            }
            sb.Append ("\n  ]\n}\n");
            return sb.ToString ();
        }

        static double Pick (Random rng)
        {
            double x = rng.NextDouble ();
            if (x < 0.2)
                return 0.0;
            if (x < 0.35)
                return 0.05 * (rng.NextDouble () - 0.5) * 2.0;
            return 2.0 * rng.NextDouble () - 1.0;
        }

        static string D (double v)
        {
            if (double.IsNaN (v))
                return "\"nan\"";
            if (double.IsPositiveInfinity (v))
                return "\"inf\"";
            if (double.IsNegativeInfinity (v))
                return "\"-inf\"";
            return v.ToString ("R", CultureInfo.InvariantCulture);
        }

        static string B (bool b)
        {
            return b ? "true" : "false";
        }

        static string V (double[] v, int start, int count)
        {
            var sb = new StringBuilder ("[");
            for (int i = 0; i < count; i++)
                sb.Append ((i > 0 ? ", " : "") + D (v [start + i]));
            return sb.Append ("]").ToString ();
        }
    }
}
