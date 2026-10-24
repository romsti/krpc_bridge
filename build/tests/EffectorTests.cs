// Refactor R1 (plan 2.4): the pure rules of the effector registry (EffectorRegistry.cs), and
// the property that makes it born closed: without the registry, the write rules see exactly the
// arming's options. Writes nothing into the golden vectors.

using System;
using System.Collections.Generic;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    internal static class EffectorTests
    {
        static int checks;
        static int failures;

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.WriteLine ("FAIL (effectors): " + what);
            }
        }

        internal static int Run (out int checkCount)
        {
            Statuses ();
            Frozen ();
            Authority ();
            TableLayout ();
            checkCount = checks;
            return failures;
        }

        static void Statuses ()
        {
            Check (EffectorRegistry.ValidMode (0) && EffectorRegistry.ValidMode (1) &&
                !EffectorRegistry.ValidMode (2) && !EffectorRegistry.ValidMode (-1), "modes 0 and 1 only");
            // RCS: exact; precision without lever = exact, scaled by precisionFactor; with lever =
            // approximate (only reduces thrust); fullThrust = unmodeled (raises it).
            Check (EffectorRegistry.RcsStatus (false, false, 0.1, false) == EffectorRegistry.StatusExact, "rcs exact");
            Check (EffectorRegistry.RcsScale (false, false, 0.1) == 1.0, "no precision: scale 1");
            Check (EffectorRegistry.RcsStatus (true, false, 0.1, false) == EffectorRegistry.StatusExact &&
                Math.Abs (EffectorRegistry.RcsScale (true, false, 0.1f) - 0.1f) < 1e-12, "precision: exact, scaled");
            Check (EffectorRegistry.RcsStatus (true, true, 0.1, false) == EffectorRegistry.StatusApproximate &&
                EffectorRegistry.RcsScale (true, true, 0.1) == 1.0, "precision with lever: approximate, unscaled");
            Check (EffectorRegistry.RcsStatus (false, false, 0.1, true) == EffectorRegistry.StatusUnmodeled &&
                EffectorRegistry.RcsStatus (true, false, 0.1, true) == EffectorRegistry.StatusUnmodeled, "fullThrust: unmodeled");
            Check (EffectorRegistry.RcsStatus (true, false, 1.5, false) == EffectorRegistry.StatusUnmodeled &&
                EffectorRegistry.RcsScale (true, false, 1.5) == 1.0, "a precision factor above 1 is not a bound");
            Check (EffectorRegistry.RcsStatus (true, false, double.NaN, false) == EffectorRegistry.StatusUnmodeled,
                "a NaN precision factor is not a model");
            Check (EffectorRegistry.Worse (EffectorRegistry.StatusAbsent, EffectorRegistry.StatusExact) == 0 &&
                EffectorRegistry.Worse (2, 3) == 3, "severity order");
            Check (EffectorRegistry.Axes (true, false, true) == 5, "axes of the enable flags");
        }

        static void Frozen ()
        {
            Check (EffectorRegistry.FrozenAxes (true, 0, 0) == 7, "write path unverified: all three axes");
            Check (EffectorRegistry.FrozenAxes (false, 2, 0) == 2, "fullThrust on the roll only: the roll only");
            Check (EffectorRegistry.FrozenAxes (false, 0, 5) == 5, "an unneutralizable surface: its axes only");
            Check (EffectorRegistry.FrozenAxes (false, 0, 0) == 0, "nothing unmodeled: nothing frozen");
            // Against option 1: a precision-mode RCS froze the three axes there; the registry
            // does not freeze it (scaled, exact).
            Check (AttitudeWrite.FrozenCtrlAxes (AttitudeWrite.OptionFamilyBlocking,
                AttitudeWrite.MotifRcsModelApprox, 0) == 7, "option 1 froze all three on motif 13");
            // Born closed: without the registry the rules see the arming's options, unchanged.
            for (int options = 0; options <= AttitudeWrite.OptionsSupported; options++) {
                int rules = options;   // RuleOptions with EffectorRegistry false
                Check (AttitudeWrite.MayWrite (1, 3, 1, true, AttitudeWrite.MotifSurfaceApprox, rules) ==
                    AttitudeWrite.MayWrite (1, 3, 1, true, AttitudeWrite.MotifSurfaceApprox, options),
                    "rules = options without the registry (" + options + ")");
            }
            // With the registry the model motifs are informational, as with option 1.
            int withRegistry = 0 | AttitudeWrite.OptionFamilyBlocking;
            Check (AttitudeWrite.MayWritePitchYaw (1, 3, 1, true, AttitudeWrite.MotifSurfaceApprox |
                AttitudeWrite.MotifRcsModelApprox, withRegistry), "registry: 13 and 14 do not block pitch/yaw");
            Check (!AttitudeWrite.MayWritePitchYaw (1, 3, 1, true, AttitudeWrite.MotifContact, withRegistry),
                "registry: a real guard still blocks");
        }

        static void Authority ()
        {
            // n = 4 columns: two gimbal columns, two others; B axis-major.
            int n = 4;
            var b = new double[] {
                10, -20, 1, 0,     // body x
                0, 5, 0, 1,        // body y
                -3, 0, 0, 2        // body z
            };
            var uMin = new double[] { -2, -4, 0.5, -1 };
            var uMax = new double[] { 2, 4, 0.5, 1 };
            var a = new double[3];
            EffectorRegistry.AddAuthority (b, n, uMin, uMax, 0, 2, a);
            Check (a [0] == 10 * 2 + 20 * 4 && a [1] == 5 * 4 && a [2] == 3 * 2, "authority = sum |B| half span");
            var frozen = new double[3];
            EffectorRegistry.AddAuthority (b, n, uMin, uMax, 2, 3, frozen);
            Check (frozen [0] == 0 && frozen [1] == 0 && frozen [2] == 0, "a frozen column has no authority");
            var pos = new double[9];
            var neg = new double[9];
            pos [0] = -100; neg [0] = -60;       // pitch column, body x
            pos [4] = -40; neg [4] = -40;        // roll column, body y
            pos [8] = -100; neg [8] = -100;      // yaw column, body z
            var c = new double[3];
            EffectorRegistry.AddCtrlAuthority (pos, neg, 2, c);
            Check (c [0] == 80 && c [1] == 0 && c [2] == 100, "ctrlState authority, the roll frozen");
            var alpha = new double[3];
            EffectorRegistry.Alpha (c, new double[] { 40, 10, 0 }, alpha);
            Check (alpha [0] == 2 && alpha [1] == 0 && double.IsNaN (alpha [2]), "alpha = A / I, NaN without inertia");
        }

        static void TableLayout ()
        {
            var t = new EffectorTable ();
            var off = t.Reply ();
            int expectedOff = EffectorRegistry.HeaderStride + EffectorRegistry.FamilyCount * EffectorRegistry.FamilyStride;
            Check (off.Count == expectedOff && off [2] == 0.0 && off [6] == 0.0 && double.IsNaN (off [8]),
                "off: same header and family rows, NaN, no column");
            t.Begin (42);
            t.EnsureColumns (3);
            t.Column (0, 1, EffectorRegistry.FamilyGimbal, 0, 0, 0, 1, 2, 3, 4);
            t.Column (1, 3, EffectorRegistry.FamilyRcs, 0, 3, 1, 0, 0, 0, 0);
            t.Column (2, 3, EffectorRegistry.FamilyRcs, 1, 2, 2, 5, 6, 7, 1);
            t.Family (EffectorRegistry.FamilyRcs, EffectorRegistry.StatusApproximate, 2);
            t.Family (EffectorRegistry.FamilyRcs, EffectorRegistry.StatusExact, 1);
            t.Enabled = true;
            var on = t.Reply ();
            Check (on.Count == expectedOff + 3 * EffectorRegistry.ColumnStride && on [3] == 42 && on [6] == 3,
                "on: three column rows");
            int rcsRow = EffectorRegistry.HeaderStride + EffectorRegistry.FamilyRcs * EffectorRegistry.FamilyStride;
            Check (on [rcsRow] == EffectorRegistry.FamilyRcs && on [rcsRow + 1] == EffectorRegistry.StatusApproximate &&
                on [rcsRow + 2] == 3 && on [rcsRow + 10] == 2 && on [rcsRow + 11] == 1,
                "family row: worst status, modules, columns, frozen columns");
            int gimbalRow = EffectorRegistry.HeaderStride;
            Check (on [gimbalRow + 1] == EffectorRegistry.StatusAbsent || on [gimbalRow + 1] == -1,
                "a family not declared stays absent");
            t.Begin (43);
            Check (t.Columns == 0 && t.FamilyStatus [EffectorRegistry.FamilyRcs] == EffectorRegistry.StatusAbsent,
                "Begin clears the step");
        }
    }
}
