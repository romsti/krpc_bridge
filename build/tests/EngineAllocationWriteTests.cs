// -----------------------------------------------------------------------------------
// option 8192 of AttitudeOptionsV1, the engine-by-engine allocation WRITES (born closed). What these
// tests pin down (the plugin's write path itself is exercised in game):
//   * option 8192 is a new bit (supported 16383; implies 4096: AllocationOn) and changes NO existing rule;
//   * ModuleRCS's dead zone (a ctrlState component whose square is below EPSILON^2 is zeroed, linear above) and the
//     snap of a command inside it (0 below EPSILON / 2, else +-EPSILON);
//   * SolveDeadZone: no eligible command left strictly inside the zone, the held ones realized exactly, the others
//     taking the rest (the reachable rows still met), the targets given back unchanged; on the heavy booster's random
//     steps the wrench the RCS REALLY gives (dead zone applied) equals what the allocation says it achieves, which a
//     plain Solve does not;
//   * the write rule: the fallback reasons of a solve, the resume after ResumeSteps sound solves (the horizon), a
//     writing allocation going on, every fallback resetting the streak;
//   * the golden vectors (golden/attitude_v1_golden_allocation.json) the client's mirror replays
//     (the client): solves, warm-started sequences, cascades, the dead zone, authorities, the rule.
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static partial class EngineAllocationTests
    {
        const int OptionWrite = AttitudeWrite.OptionEngineAllocationWrite;
        const double Epsilon = 0.05;

        static void TestWriteBits ()
        {
            Check (OptionWrite == 8192 && (AttitudeWrite.OptionsSupported & 16383) == 16383,
                "option 8192, supported 16383 (mirrored by the client)");
            Check (AttitudeWrite.ValidOptions (8192) && AttitudeWrite.ValidOptions (16383) &&
                !AttitudeWrite.ValidOptions (AttitudeWrite.OptionsSupported + 1),
                "valid options 0-16383 (and 16384)");
            Check (AttitudeWrite.AllocationOn (4096) && AttitudeWrite.AllocationOn (8192) && AttitudeWrite.AllocationOn (12288) &&
                !AttitudeWrite.AllocationOn (0) && !AttitudeWrite.AllocationOn (2047), "8192 implies the allocation");
            Check ((AttitudeWrite.AllowedWriteMask (OptionWrite) & ~AttitudeWrite.WriteMaskBase) == 0,
                "option 8192 allows no write-mask bit");
            var rng = new Random (2642);
            for (int i = 0; i < 8192; i += 5) {
                int o = i | OptionWrite;
                Check (AttitudeWrite.RollBlockingSet (o) == AttitudeWrite.RollBlockingSet (i) &&
                    AttitudeWrite.PitchYawBlockingSet (o) == AttitudeWrite.PitchYawBlockingSet (i) &&
                    AttitudeWrite.AllowedWriteMask (o) == AttitudeWrite.AllowedWriteMask (i),
                    "option 8192: blocking sets and write mask of options " + i);
                long motif = ((long)rng.Next () << 31) ^ rng.Next ();
                for (int mode = 0; mode < 2; mode++)
                    for (int axes = 0; axes < 4; axes++)
                        Check (AttitudeWrite.MayWritePitchYaw (mode, axes, Earlyish, true, motif, o) ==
                            AttitudeWrite.MayWritePitchYaw (mode, axes, Earlyish, true, motif, i) &&
                            AttitudeWrite.MayWrite (mode, axes, Earlyish, true, motif, o) ==
                            AttitudeWrite.MayWrite (mode, axes, Earlyish, true, motif, i),
                            "option 8192: the channels' permissions identical");
                Check (AttitudeWrite.FrozenCtrlAxes (o, motif, i & 7) == AttitudeWrite.FrozenCtrlAxes (i, motif, i & 7),
                    "option 8192: frozen axes identical");
            }
        }

        static void TestDeadZoneArithmetic ()
        {
            Check (EngineAllocation.DeadZoned (0.04, Epsilon) == 0.0 && EngineAllocation.DeadZoned (-0.0499, Epsilon) == 0.0,
                "dead zone: a component below EPSILON is zeroed");
            Check (EngineAllocation.DeadZoned (0.05, Epsilon) == 0.05 && EngineAllocation.DeadZoned (-0.3, Epsilon) == -0.3,
                "dead zone: at and above EPSILON the component stays (linear)");
            Check (EngineAllocation.DeadZoned (0.01, double.NaN) == 0.01 && EngineAllocation.DeadZoned (0.01, 0.0) == 0.01,
                "dead zone: none without an EPSILON");
            Check (EngineAllocation.SnapDeadZone (0.02, Epsilon) == 0.0 && EngineAllocation.SnapDeadZone (0.03, Epsilon) == Epsilon &&
                EngineAllocation.SnapDeadZone (-0.03, Epsilon) == -Epsilon && EngineAllocation.SnapDeadZone (0.06, Epsilon) == 0.06 &&
                EngineAllocation.SnapDeadZone (-0.024999, Epsilon) == 0.0, "snap: nearest realized (0 below EPSILON/2, +-EPSILON above)");
            for (int k = -100; k <= 100; k++) {
                double c = k * 0.001;
                double s = EngineAllocation.SnapDeadZone (c, Epsilon);
                Check (EngineAllocation.DeadZoned (s, Epsilon) == s, "snap: the snapped command is realized as such (" + c + ")");
                Check (Math.Abs (s - c) <= 0.5 * Epsilon + 1e-15, "snap: never more than EPSILON/2 away (" + c + ")");
            }
        }

        /// <summary>The heavy booster, RCS columns eligible to the dead zone (its rotation and translation channels).</summary>
        static void HeavyDeadZone (EngineAllocation x, double main)
        {
            Heavy (x, main, 1.0, 1.0, double.NaN);
            for (int j = 0; j < x.Count; j++)
                x.DeadZoneEligible [j] = x.Kind [j] == EngineAllocation.KindRotation || x.Kind [j] == EngineAllocation.KindLinear;
        }

        /// <summary>The wrench the actuators REALLY give at a command set, the RCS through its dead zone.</summary>
        static void RealizedWrench (EngineAllocation x, double[] command, double[] wrench)
        {
            for (int r = 0; r < 6; r++)
                wrench [r] = 0.0;
            for (int j = 0; j < x.Count; j++) {
                double dx;
                if (x.DeadZoneEligible [j])
                    dx = EngineAllocation.DeadZoned (command [j], Epsilon) - EngineAllocation.DeadZoned (x.U0 [j], Epsilon);
                else
                    dx = EngineAllocation.Realized (command [j], x.U0 [j], x.PhiUp [j], x.PhiDown [j]);
                for (int r = 0; r < 6; r++)
                    wrench [r] += x.Col [6 * j + r] * dx;
            }
        }

        static void TestDeadZoneSolve ()
        {
            // A small roll and pitch torque: the regularization hands the RCS a small part, inside the zone.
            var plain = new EngineAllocation ();
            HeavyDeadZone (plain, 0.4);
            Targets (plain, 0.0, 0.0, 0.0, 2.0e4, 1.2e4, -1.0e4, false);
            Check (plain.Solve (true), "dead zone: plain solve");
            int inside = 0;
            for (int j = 0; j < plain.Count; j++)
                if (plain.DeadZoneEligible [j] && plain.Command [j] != 0.0 && Math.Abs (plain.Command [j]) < Epsilon)
                    inside++;
            Check (inside > 0, "dead zone: the plain solve leaves RCS commands inside the zone: " + inside);
            var x = new EngineAllocation ();
            HeavyDeadZone (x, 0.4);
            Targets (x, 0.0, 0.0, 0.0, 2.0e4, 1.2e4, -1.0e4, false);
            var target = (double[])x.Target.Clone ();
            Check (x.SolveDeadZone (Epsilon, true) && x.Converged, "dead zone: solved and converged");
            Check (x.Snapped > 0 && (x.Reasons & EngineAllocation.ReasonDeadZoneSnapped) != 0, "dead zone: snapped " + x.Snapped);
            for (int r = 0; r < 6; r++)
                Check (x.Target [r] == target [r], "dead zone: the targets given back unchanged (" + r + ")");
            for (int j = 0; j < x.Count; j++) {
                if (!x.DeadZoneEligible [j])
                    continue;
                double c = x.Command [j];
                Check (c == 0.0 || Math.Abs (c) >= Epsilon - 1e-15, "dead zone: no command left inside (" + j + "): " + c);
                Check (Math.Abs (x.X [j] - EngineAllocation.Realized (c, x.U0 [j], x.PhiUp [j], x.PhiDown [j])) < 1e-12,
                    "dead zone: X is the command's realization (" + j + ")");
            }
            var sum = new double[6];
            for (int j = 0; j < x.Count; j++)
                for (int r = 0; r < 6; r++)
                    sum [r] += x.Col [6 * j + r] * x.X [j];
            for (int r = 0; r < 6; r++)
                Check (Math.Abs (sum [r] - x.Achieved [r]) <= 1e-9 * (1.0 + x.RowAuthority [r]),
                    "dead zone: Achieved is the sum of the columns times X (" + r + ")");
            foreach (int r in new[] { 1, 3, 4, 5 })
                Check (Math.Abs (x.Achieved [r] - target [r]) <= 1e-5 * x.RowAuthority [r],
                    "dead zone: the reachable rows still met (" + r + "): " + x.Achieved [r] + " vs " + target [r]);
            // Nothing eligible, or no EPSILON: SolveDeadZone is Solve, to the bit.
            var a = new EngineAllocation ();
            var b = new EngineAllocation ();
            Heavy (a, 0.4, 1.0, 1.0, double.NaN);
            Heavy (b, 0.4, 1.0, 1.0, double.NaN);
            Targets (a, 0.0, 0.0, 0.0, 2.0e4, 1.2e4, -1.0e4, false);
            Targets (b, 0.0, 0.0, 0.0, 2.0e4, 1.2e4, -1.0e4, false);
            a.Solve (true);
            b.SolveDeadZone (Epsilon, true);
            bool same = b.Snapped == 0;
            for (int j = 0; j < a.Count; j++)
                same &= a.Command [j] == b.Command [j] && a.X [j] == b.X [j];
            Check (same, "dead zone: without eligible variables, SolveDeadZone is Solve to the bit");
            HeavyDeadZone (b, 0.4);
            Targets (b, 0.0, 0.0, 0.0, 2.0e4, 1.2e4, -1.0e4, false);
            b.SolveDeadZone (double.NaN, true);
            same = b.Snapped == 0;
            for (int j = 0; j < a.Count; j++)
                same &= a.Command [j] == b.Command [j];
            Check (same, "dead zone: without EPSILON, SolveDeadZone is Solve");
        }

        static void TestDeadZoneHeavyLoop ()
        {
            // Random steps of the heavy booster: torques and lateral demands of every size. With the dead zone in the
            // command, the wrench the RCS really gives is what the allocation achieves; with a plain solve, not.
            var rng = new Random (2643);
            var w = new double[6];
            int plainMiss = 0, snapMiss = 0, steps = 400, snappedSteps = 0;
            var x = new EngineAllocation ();
            var p = new EngineAllocation ();
            for (int k = 0; k < steps; k++) {
                double main = 0.1 + 0.8 * rng.NextDouble ();
                double scale = Math.Pow (10.0, -2.0 + 3.0 * rng.NextDouble ());
                double mx = scale * 1e5 * (2 * rng.NextDouble () - 1), my = scale * 3e4 * (2 * rng.NextDouble () - 1);
                double mz = scale * 1e5 * (2 * rng.NextDouble () - 1);
                bool lateral = (k & 1) == 0;
                double fz = lateral ? scale * 0.05 * HeavyMassKg * (2 * rng.NextDouble () - 1) : 0.0;
                HeavyDeadZone (x, main);
                HeavyDeadZone (p, main);
                Targets (x, 0.0, 0.0, fz, mx, my, mz, lateral);
                Targets (p, 0.0, 0.0, fz, mx, my, mz, lateral);
                x.SolveDeadZone (Epsilon, true);
                p.Solve (true);
                if (x.Snapped > 0)
                    snappedSteps++;
                RealizedWrench (x, x.Command, w);
                bool miss = false;
                for (int r = 0; r < 6; r++)
                    miss |= Math.Abs (w [r] - x.Achieved [r]) > 1e-9 * (1.0 + x.RowAuthority [r]);
                if (miss)
                    snapMiss++;
                RealizedWrench (p, p.Command, w);
                miss = false;
                for (int r = 0; r < 6; r++)
                    miss |= Math.Abs (w [r] - p.Achieved [r]) > 1e-4 * (1.0 + p.RowAuthority [r]);
                if (miss)
                    plainMiss++;
            }
            Check (snapMiss == 0, "dead zone loop: the realized wrench is the achieved one on every step: " + snapMiss + " missed");
            Check (snappedSteps > steps / 10 && plainMiss > steps / 10,
                "dead zone loop: the zone matters (snapped on " + snappedSteps + " steps, a plain solve misses on " + plainMiss + ")");
            Console.WriteLine ("  dead zone (heavy, " + steps + " random steps): snapped on " + snappedSteps +
                ", a plain solve's realized wrench misses its claim by > 1e-4 of an authority on " + plainMiss);
        }

        static void TestWriteRule ()
        {
            long none = AllocationWriteRule.SolveFallbacks (EngineAllocation.StateSolved, true, 0L, true, false, false, false);
            Check (none == 0L, "rule: a sound solve may write");
            Check (AllocationWriteRule.SolveFallbacks (EngineAllocation.StateSolved, true,
                EngineAllocation.ReasonLimitedLateral | EngineAllocation.ReasonLimitedRoll | EngineAllocation.ReasonDeadZoneSnapped |
                EngineAllocation.ReasonRcsDeadZone | EngineAllocation.ReasonNoLateralDemand, true, false, false, false) == 0L,
                "rule: a limited lateral force or roll, a snap, no demand: still writes (the layer would be limited too)");
            Check (AllocationWriteRule.SolveFallbacks (EngineAllocation.StateNoLaw, true, 0L, true, false, false, false) ==
                AllocationWriteRule.FallbackNotSolved, "rule: not solved");
            Check (AllocationWriteRule.SolveFallbacks (EngineAllocation.StateSolved, false, 0L, true, false, false, false) ==
                AllocationWriteRule.FallbackNotConverged, "rule: not converged");
            Check (AllocationWriteRule.SolveFallbacks (EngineAllocation.StateSolved, true, EngineAllocation.ReasonNotConverged,
                true, false, false, false) == AllocationWriteRule.FallbackNotConverged, "rule: not converged (reason)");
            Check (AllocationWriteRule.SolveFallbacks (EngineAllocation.StateSolved, true, EngineAllocation.ReasonLimitedAxial,
                true, false, false, false) == AllocationWriteRule.FallbackSaturated, "rule: axial saturated");
            Check (AllocationWriteRule.SolveFallbacks (EngineAllocation.StateSolved, true, EngineAllocation.ReasonLimitedPitchYaw,
                true, false, false, false) == AllocationWriteRule.FallbackSaturated, "rule: pitch/yaw saturated");
            Check (AllocationWriteRule.SolveFallbacks (EngineAllocation.StateSolved, true, 0L, false, false, false, false) ==
                AllocationWriteRule.FallbackStale, "rule: stale");
            Check (AllocationWriteRule.SolveFallbacks (EngineAllocation.StateSolved, true, 0L, true, true, false, false) ==
                AllocationWriteRule.FallbackLease, "rule: lease");
            Check (AllocationWriteRule.SolveFallbacks (EngineAllocation.StateSolved, true, 0L, true, false, true, false) ==
                AllocationWriteRule.FallbackGimbalMissing, "rule: gimbal missing");
            Check (AllocationWriteRule.SolveFallbacks (EngineAllocation.StateSolved, true, 0L, true, false, false, true) ==
                AllocationWriteRule.FallbackException, "rule: exception");
            Check (AllocationWriteRule.ResumeSteps (5.735) == 6 && AllocationWriteRule.ResumeSteps (6.0) == 6 &&
                AllocationWriteRule.ResumeSteps (1.0) == 1 && AllocationWriteRule.ResumeSteps (0.3) == 1 &&
                AllocationWriteRule.ResumeSteps (double.NaN) == 1 && AllocationWriteRule.ResumeSteps (double.PositiveInfinity) == 1,
                "rule: resume after the horizon in whole steps, at least one");
        }

        static void TestWriteRuleSequence ()
        {
            // resume 3: three sound solves before the hand is taken; a writing allocation goes on; a fallback resets.
            long sat = AllocationWriteRule.FallbackSaturated;
            long[] fallbacks = { 0, 0, 0, 0, sat, 0, 0, 0, 0, AllocationWriteRule.FallbackLease, 0 };
            bool[] expected = { false, false, true, true, false, false, false, true, true, false, false };
            int streak = 0;
            bool wrote = false;
            for (int k = 0; k < fallbacks.Length; k++) {
                long published;
                bool w = AllocationWriteRule.Decide (fallbacks [k], ref streak, 3, wrote, out published);
                Check (w == expected [k], "rule sequence: step " + k + " writes " + w);
                Check (w ? published == 0L : (fallbacks [k] != 0L ? published == fallbacks [k]
                    : published == AllocationWriteRule.FallbackResuming), "rule sequence: published reasons at " + k);
                wrote = w;
            }
            // resume 1 (instant actuators): the first sound solve writes.
            streak = 0;
            long pub;
            Check (AllocationWriteRule.Decide (0L, ref streak, 1, false, out pub) && pub == 0L, "rule: resume 1 writes at once");
        }

        // ------------------------------------------------------------------ golden vectors

        static string G (double v)
        {
            if (double.IsNaN (v))
                return "\"nan\"";
            if (double.IsPositiveInfinity (v))
                return "\"inf\"";
            if (double.IsNegativeInfinity (v))
                return "\"-inf\"";
            return v.ToString ("R", CultureInfo.InvariantCulture);
        }

        static string GA (double[] a, int n)
        {
            var sb = new StringBuilder ("[");
            for (int i = 0; i < n; i++) {
                if (i > 0)
                    sb.Append (", ");
                sb.Append (G (a [i]));
            }
            return sb.Append ("]").ToString ();
        }

        static string GI (int[] a, int n)
        {
            var sb = new StringBuilder ("[");
            for (int i = 0; i < n; i++)
                sb.Append (i > 0 ? ", " : "").Append (a [i].ToString (CultureInfo.InvariantCulture));
            return sb.Append ("]").ToString ();
        }

        static string GB (bool[] a, int n)
        {
            var sb = new StringBuilder ("[");
            for (int i = 0; i < n; i++)
                sb.Append (i > 0 ? ", " : "").Append (a [i] ? "true" : "false");
            return sb.Append ("]").ToString ();
        }

        /// <summary>One step of a golden case: the inputs as set, the call, the outputs.</summary>
        static string GoldenStep (EngineAllocation x, string call, double epsilon, double ux, double uz, bool warm, int lastLevel)
        {
            int n = x.Count;
            var sb = new StringBuilder ();
            sb.Append ("{\"call\": \"" + call + "\", \"warm\": " + (warm ? "true" : "false") + ", \"last_level\": " + lastLevel +
                ", \"epsilon\": " + G (epsilon) + ", \"ux\": " + G (ux) + ", \"uz\": " + G (uz) + ",\n       \"n\": " + n +
                ", \"col\": " + GA (x.Col, 6 * n) + ",\n       \"u0\": " + GA (x.U0, n) + ", \"lo\": " + GA (x.Lo, n) +
                ", \"hi\": " + GA (x.Hi, n) + ", \"pref\": " + GA (x.Pref, n) + ", \"priority\": " + GA (x.Priority, n) +
                ", \"phi_up\": " + GA (x.PhiUp, n) + ", \"phi_down\": " + GA (x.PhiDown, n) + ", \"kind\": " + GI (x.Kind, n) +
                ", \"eligible\": " + GB (x.DeadZoneEligible, n) + ", \"target\": " + GA (x.Target, 6) +
                ", \"row_on\": " + GB (x.RowOn, 6));
            double along = double.NaN;
            bool ok;
            if (call == "solve")
                ok = x.Solve (warm, lastLevel);
            else if (call == "dead_zone")
                ok = x.SolveDeadZone (epsilon, warm);
            else {
                along = x.AuthorityAlong (ux, uz);
                ok = true;
            }
            sb.Append (",\n       \"out\": {\"ok\": " + (ok ? "true" : "false") + ", \"along\": " + G (along) +
                ", \"converged\": " + (x.Converged ? "true" : "false") + ", \"joint\": " + (x.JointTaken ? "true" : "false") +
                ", \"stages\": " + x.Stages + ", \"iterations\": " + x.Iterations + ", \"reasons\": " + x.Reasons +
                ", \"snapped\": " + (call == "dead_zone" ? x.Snapped : 0) + ",\n         \"x\": " + GA (x.X, n) +
                ", \"command\": " + GA (x.Command, n) + ",\n         \"achieved\": " + GA (x.Achieved, 6) +
                ", \"authority\": " + GA (x.RowAuthority, 6) + ", \"target_after\": " + GA (x.Target, 6) +
                ", \"row_on_after\": " + GB (x.RowOn, 6) + "}}");
            return sb.ToString ();
        }

        static void RandomProblem (Random rng, EngineAllocation x, int n)
        {
            x.Resize (n);
            for (int j = 0; j < n; j++) {
                for (int r = 0; r < 6; r++)
                    x.Col [6 * j + r] = rng.NextDouble () < 0.25 ? 0.0 : (2 * rng.NextDouble () - 1) * Math.Pow (10.0, 2 + 3 * rng.NextDouble ());
                int kind = 1 + rng.Next (4);
                double lo = kind == EngineAllocation.KindThrottle ? 0.0 : -1.0 - rng.NextDouble ();
                double hi = kind == EngineAllocation.KindThrottle ? 1.0 : 1.0 + rng.NextDouble ();
                x.Lo [j] = lo;
                x.Hi [j] = hi;
                x.U0 [j] = lo + (hi - lo) * rng.NextDouble ();
                x.Pref [j] = kind == EngineAllocation.KindThrottle ? x.U0 [j] : 0.0;
                x.Priority [j] = new[] { 0.5, 1.0, 3.0 } [rng.Next (3)];
                x.PhiUp [j] = rng.NextDouble () < 0.5 ? 1.0 : EngineAllocation.Phi (0.05 + 0.9 * rng.NextDouble (), 4.0);
                x.PhiDown [j] = rng.NextDouble () < 0.5 ? x.PhiUp [j] : EngineAllocation.Phi (0.05 + 0.9 * rng.NextDouble (), 4.0);
                x.Kind [j] = kind;
                x.DeadZoneEligible [j] = kind == EngineAllocation.KindRotation || kind == EngineAllocation.KindLinear;
            }
            for (int r = 0; r < 6; r++) {
                double a = 0.0;
                for (int j = 0; j < n; j++)
                    a += Math.Abs (x.Col [6 * j + r]) * 0.5 * (x.Hi [j] - x.Lo [j]);
                x.Target [r] = (2 * rng.NextDouble () - 1) * a * (rng.NextDouble () < 0.3 ? 1.5 : 0.3);
                x.RowOn [r] = rng.NextDouble () < 0.85;
            }
        }

        /// <summary>
        /// the golden vectors of the allocation for the client's mirror (the client):
        /// each case a sequence of steps on ONE instance (the warm start carried), each step its inputs, its call (solve,
        /// dead_zone, authority) and its outputs; then the write rule's sequences. Deterministic.
        /// </summary>
        internal static string GoldenAllocationJson ()
        {
            var sb = new StringBuilder ();
            sb.Append ("{\n  \"schema\": 1,\n  \"source\": \"krpc_bridge build/tests (EngineAllocation.cs, options 4096 and 8192)\",\n" +
                "  \"cases\": [\n");
            var cases = new List<string> ();
            var steps = new List<string> ();
            // 1. The heavy booster, steady: a torque, then a lateral demand (warm), then the cascade alone.
            {
                var x = new EngineAllocation ();
                steps.Clear ();
                Heavy (x, 0.4, 1.0, 1.0, double.NaN);
                Targets (x, 0.0, 0.0, 0.0, 3.0e6, 0.05e6, -2.0e6, false);
                steps.Add (GoldenStep (x, "solve", double.NaN, double.NaN, double.NaN, true, EngineAllocation.LevelRoll));
                Heavy (x, 0.4, 1.0, 1.0, double.NaN);
                Targets (x, 0.0, 0.0, 0.2 * HeavyMassKg, 1.0e5, 0.0, -5.0e4, true);
                steps.Add (GoldenStep (x, "solve", double.NaN, double.NaN, double.NaN, true, EngineAllocation.LevelRoll));
                Heavy (x, 0.4, 1.0, 1.0, double.NaN);
                Targets (x, 0.0, 0.0, 0.2 * HeavyMassKg, 1.0e5, 0.0, -5.0e4, true);
                steps.Add (GoldenStep (x, "solve", double.NaN, double.NaN, double.NaN, false, EngineAllocation.LevelRoll));
                cases.Add ("    {\"name\": \"heavy_steady\", \"steps\": [\n      " + string.Join (",\n      ", steps) + "]}");
            }
            // 2. The heavy booster at the horizon (gimbals 0.16 a step, engines instant), realized gimbals off neutral.
            {
                var x = new EngineAllocation ();
                steps.Clear ();
                double h = EngineAllocation.Horizon (new[] { 0.16, 1.0, 1.0 }, 3);
                var gu0 = new double[2 * HeavyEngines];
                for (int k = 0; k < gu0.Length; k++)
                    gu0 [k] = 0.4 * Math.Cos (1.3 * k);
                for (int s = 0; s < 3; s++) {
                    Heavy (x, 0.3 + 0.1 * s, 0.16, 1.0, h, gu0);
                    Targets (x, 0.1 * HeavyMassKg, 0.0, -0.05 * HeavyMassKg * s, 2.0e5 * (s - 1), 1.0e4, 3.0e5, true);
                    steps.Add (GoldenStep (x, "solve", double.NaN, double.NaN, double.NaN, true, EngineAllocation.LevelRoll));
                }
                cases.Add ("    {\"name\": \"heavy_horizon\", \"steps\": [\n      " + string.Join (",\n      ", steps) + "]}");
            }
            // 3. Saturations: a torque beyond everything, an unreachable lateral demand, a full main throttle.
            {
                var x = new EngineAllocation ();
                steps.Clear ();
                Heavy (x, 0.4, 1.0, 1.0, double.NaN);
                Targets (x, 0.0, 0.0, 0.0, 1.0e9, 1.0e5, 0.0, false);
                steps.Add (GoldenStep (x, "solve", double.NaN, double.NaN, double.NaN, true, EngineAllocation.LevelRoll));
                Heavy (x, 0.4, 1.0, 1.0, double.NaN);
                Targets (x, 5.0 * HeavyMassKg, 0.0, 0.0, 1.0e5, 0.0, 0.0, true);
                steps.Add (GoldenStep (x, "solve", double.NaN, double.NaN, double.NaN, true, EngineAllocation.LevelRoll));
                Heavy (x, 1.0, 1.0, 1.0, double.NaN);
                Targets (x, 0.0, 0.0, 0.0, 2.0e6, 0.0, 0.0, false);
                steps.Add (GoldenStep (x, "solve", double.NaN, double.NaN, double.NaN, true, EngineAllocation.LevelRoll));
                cases.Add ("    {\"name\": \"heavy_saturations\", \"steps\": [\n      " + string.Join (",\n      ", steps) + "]}");
            }
            // 4. The dead zone: small torques and lateral demands, warm-started.
            {
                var x = new EngineAllocation ();
                steps.Clear ();
                var rng = new Random (2644);
                for (int s = 0; s < 6; s++) {
                    HeavyDeadZone (x, 0.2 + 0.1 * s);
                    double scale = Math.Pow (10.0, -1.5 + 2.0 * rng.NextDouble ());
                    Targets (x, 0.0, 0.0, scale * 0.02 * HeavyMassKg * (2 * rng.NextDouble () - 1),
                        scale * 1e5 * (2 * rng.NextDouble () - 1), scale * 2e4 * (2 * rng.NextDouble () - 1),
                        scale * 1e5 * (2 * rng.NextDouble () - 1), (s & 1) == 0);
                    steps.Add (GoldenStep (x, "dead_zone", Epsilon, double.NaN, double.NaN, true, EngineAllocation.LevelRoll));
                }
                cases.Add ("    {\"name\": \"heavy_dead_zone\", \"steps\": [\n      " + string.Join (",\n      ", steps) + "]}");
            }
            // 5. Authorities along the four body directions and a slanted one, free (a torque wanted) and neutral.
            {
                var x = new EngineAllocation ();
                steps.Clear ();
                double[] uxs = { 1.0, -1.0, 0.0, 0.0, 0.6 }, uzs = { 0.0, 0.0, 1.0, -1.0, -0.8 };
                for (int d = 0; d < 5; d++) {
                    Heavy (x, 0.25 + 0.15 * d, 1.0, 1.0, double.NaN);
                    Targets (x, 0.0, 0.0, 0.0, d == 4 ? 4.0e5 : 0.0, 0.0, 0.0, false);
                    steps.Add (GoldenStep (x, "authority", double.NaN, uxs [d], uzs [d], false, EngineAllocation.LevelLateral));
                }
                cases.Add ("    {\"name\": \"heavy_authority\", \"steps\": [\n      " + string.Join (",\n      ", steps) + "]}");
            }
            // 6. Random problems, each three warm-started steps, solve and dead zone alternately.
            {
                var rng = new Random (2645);
                for (int c = 0; c < 8; c++) {
                    var x = new EngineAllocation ();
                    steps.Clear ();
                    int n = 6 + rng.Next (20);
                    for (int s = 0; s < 3; s++) {
                        RandomProblem (rng, x, n);
                        bool dz = ((c + s) & 1) == 1;
                        steps.Add (GoldenStep (x, dz ? "dead_zone" : "solve", Epsilon, double.NaN, double.NaN, true,
                            EngineAllocation.LevelRoll));
                    }
                    cases.Add ("    {\"name\": \"random_" + c + "\", \"steps\": [\n      " + string.Join (",\n      ", steps) + "]}");
                }
            }
            sb.Append (string.Join (",\n", cases)).Append ("\n  ],\n  \"rule\": [\n");
            // The write rule: sequences of fallbacks (resume steps, the reasons per step) and the decisions.
            var rules = new List<string> ();
            var rr = new Random (2646);
            long[] bits = { 0, 0, 0, AllocationWriteRule.FallbackSaturated, AllocationWriteRule.FallbackStale,
                AllocationWriteRule.FallbackLease, AllocationWriteRule.FallbackNotConverged };
            foreach (int resume in new[] { 1, 3, 6 }) {
                int streak = 0;
                bool wrote = false;
                var fb = new List<string> ();
                var dec = new List<string> ();
                for (int k = 0; k < 30; k++) {
                    long f = bits [rr.Next (bits.Length)];
                    long published;
                    bool w = AllocationWriteRule.Decide (f, ref streak, resume, wrote, out published);
                    fb.Add (f.ToString (CultureInfo.InvariantCulture));
                    dec.Add ("[" + (w ? "true" : "false") + ", " + streak + ", " + published + "]");
                    wrote = w;
                }
                rules.Add ("    {\"resume\": " + resume + ", \"fallbacks\": [" + string.Join (", ", fb) + "], \"decisions\": [" +
                    string.Join (", ", dec) + "]}");
            }
            var hz = new List<string> ();
            foreach (double h in new[] { 0.3, 1.0, 5.735477907117629, 6.0, 12.5 })
                hz.Add ("[" + G (h) + ", " + AllocationWriteRule.ResumeSteps (h) + "]");
            sb.Append (string.Join (",\n", rules)).Append ("\n  ],\n  \"resume_steps\": [" + string.Join (", ", hz) + "]\n}\n");
            return sb.ToString ();
        }
    }
}
