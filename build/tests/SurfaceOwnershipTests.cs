// an armed control surface obeys the loop only on axes the loop OWNS. The pure rules
// (AttitudeWrite.WrittenAxes, LoopOwnedAxes; SurfaceModel.CommandedAxes, EngagementAxes, Owned, the
// sanction bits 15-17) and the bumpless change of definition of the law's M at an engagement or a
// disengagement (Biquad3.Shift, AttitudeLawState.ShiftTorque) against the law run with the new
// definition from the start.
//
// a recorded flight (armed, bridge 30aebeb): the four grid fins of the super hope were
// engaged through the coast while the kRPC AutoPilot wrote pitch/yaw and the loop only the roll (the
// roll gate of option 1 lifted by the engagement). Each fin's column contributes to the three body
// axes (roll 12 % of its norm) and its command answers pitch or yaw and roll: it needs the three
// axes owned, which the coast never gives. Writes nothing into the golden vectors.

using System;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    internal static class SurfaceOwnershipTests
    {
        static int checks;
        static int failures;

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.WriteLine ("FAIL (surface ownership): " + what);
            }
        }

        internal static int Run (out int checkCount)
        {
            SanctionBits ();
            Written ();
            Ownership ();
            Commanded ();
            Engagement ();
            FilterShift ();
            LawShift ();
            checkCount = checks;
            return failures;
        }

        // ------------------------------------------------------------------ the bits

        static void SanctionBits ()
        {
            int[] older = {
                SurfaceModel.SanctionScaleX, SurfaceModel.SanctionScaleY, SurfaceModel.SanctionScaleZ,
                SurfaceModel.SanctionLag, SurfaceModel.SanctionIdentity, SurfaceModel.SanctionNotArmed,
                SurfaceModel.SanctionUnproven, SurfaceModel.SanctionKappaUnproven, SurfaceModel.SanctionKappaOut
            };
            int all = 0;
            foreach (int b in older)
                all |= b;
            Check (SurfaceModel.SanctionNotOwned == 7 << 15 && SurfaceModel.SanctionNotOwnedShift == 15,
                "the not-owned bits are 15-17 (32768, 65536, 131072)");
            Check ((all & SurfaceModel.SanctionNotOwned) == 0, "the not-owned bits are new bits");
            Check ((SurfaceModel.SanctionPerSurface & SurfaceModel.SanctionNotOwned) == SurfaceModel.SanctionNotOwned,
                "not owned is a per-surface condition, never a family block");
            Check ((SurfaceModel.SanctionPerSurface & (SurfaceModel.SanctionLag | SurfaceModel.SanctionNotArmed |
                SurfaceModel.SanctionScaleX)) == 0, "the family blocks are unchanged");
        }

        // ------------------------------------------------------------------ owned axes

        static void Written ()
        {
            double nan = double.NaN;
            Check (AttitudeWrite.WrittenAxes (false, 0.1, 0.2, 0.3) == 0,
                "nothing written when our callback did not run at the previous step");
            Check (AttitudeWrite.WrittenAxes (true, nan, nan, nan) == 0, "nothing written (NaN)");
            Check (AttitudeWrite.WrittenAxes (true, nan, 0.0, nan) == 2, "the roll alone (the coast)");
            Check (AttitudeWrite.WrittenAxes (true, -1.0, 0.0, 1.0) == 7, "the three axes (the descent)");
            Check (AttitudeWrite.WrittenAxes (true, 0.5, nan, nan) == 1 && AttitudeWrite.WrittenAxes (true, nan, nan, 0.5) == 4,
                "pitch alone, yaw alone (a frozen axis of option 1 is not written)");
        }

        static void Ownership ()
        {
            int armed = AttitudeWrite.ModeArmed, shadow = AttitudeWrite.ModeShadow;
            int early = AttitudeWrite.StageEarlyish, pump = AttitudeWrite.StagePumpFallback;
            int both = AttitudeWrite.ArmRoll | AttitudeWrite.ArmPitchYaw;
            // The coast of the flight: the loop writes the roll, the client's write mask has no pitch/yaw.
            Check (AttitudeWrite.LoopOwnedAxes (armed, both, AttitudeWrite.ArmRoll, early, 2) == 2,
                "the coast owns the roll only");
            // The descent: the three written, both masks.
            Check (AttitudeWrite.LoopOwnedAxes (armed, both, both, early, 7) == 7, "the descent owns the three axes");
            // The PDI: the mask opens, nothing written yet -> nothing owned (one written step first).
            Check (AttitudeWrite.LoopOwnedAxes (armed, both, both, early, 2) == 2,
                "the first step of the PDI owns the roll only (pitch/yaw not written yet)");
            // The client closes the pitch/yaw write mask: owned at once, before the law, no lag.
            Check (AttitudeWrite.LoopOwnedAxes (armed, both, AttitudeWrite.ArmRoll, early, 7) == 2,
                "a write mask closed now takes pitch/yaw away at once");
            Check (AttitudeWrite.LoopOwnedAxes (armed, AttitudeWrite.ArmPitchYaw, both, early, 7) == 5,
                "an arm mask without the roll never owns it");
            Check (AttitudeWrite.LoopOwnedAxes (shadow, both, both, early, 7) == 0, "shadow owns nothing");
            Check (AttitudeWrite.LoopOwnedAxes (armed, both, both, pump, 7) == 0, "the pump fallback owns nothing");
            Check (AttitudeWrite.LoopOwnedAxes (armed, both, 0, early, 7) == 0, "write mask 0 owns nothing");
            for (int last = 0; last < 8; last++)
                for (int arm = 0; arm < 4; arm++)
                    for (int write = 0; write < 4; write++) {
                        int o = AttitudeWrite.LoopOwnedAxes (armed, arm, write, early, last);
                        Check ((o & ~last) == 0 && (o & ~AttitudeWrite.ArmedBodyAxes (arm & write)) == 0 &&
                            o == (last & AttitudeWrite.ArmedBodyAxes (arm & write)),
                            "owned = written at the previous step AND permitted now (" + last + ", " + arm + ", " + write + ")");
                    }
        }

        // ------------------------------------------------------------------ the surface's axes

        /// <summary>A super hope grid fin's command (surface_model_v1, a recorded flight): pitch and roll.</summary>
        static SurfaceCommand PitchFin ()
        {
            // gains -35 deg/unit pitch, -31.7 roll, ~3e-13 yaw over a 35 deg range.
            return new SurfaceCommand {
                CPitch = 1.0, KRoll = 31.7 / 35.0, CYaw = 1e-14, Range = 35.0, Authority = 100.0,
                Direction = 1.0, Speed = 40.0
            };
        }

        static void Commanded ()
        {
            Check (SurfaceModel.CommandedAxes (PitchFin ()) == 3, "a pitch fin answers pitch and roll, not yaw");
            var yawFin = new SurfaceCommand { CYaw = -1.0, KRoll = 31.7 / 35.0, Range = 35.0, Authority = 100.0, Direction = 1.0 };
            Check (SurfaceModel.CommandedAxes (yawFin) == 6, "a yaw fin answers roll and yaw");
            var none = new SurfaceCommand { Range = 35.0, Authority = 100.0, Direction = 1.0 };
            Check (SurfaceModel.CommandedAxes (none) == 0, "ignore pitch/yaw/roll answers nothing");
            var tiny = new SurfaceCommand { CPitch = 1.0, KRoll = 0.999e-3, CYaw = 1.001e-3 };
            Check (SurfaceModel.CommandedAxes (tiny) == 5, "the floor is 1e-3 of the coefficients' norm, as the column's");
            var bad = new SurfaceCommand { CPitch = double.NaN };
            Check (SurfaceModel.CommandedAxes (bad) == 7, "a command that is not finite answers every axis");
            // Neither the range, the authority, the direction nor the deploy term changes the axes.
            var c = PitchFin ();
            c.Direction = -1.0;
            c.Authority = 30.0;
            c.DeployTerm = 5.0;
            Check (SurfaceModel.CommandedAxes (c) == 3, "the axes come from the command's geometry only");
        }

        static void Engagement ()
        {
            // The flight's fin 1 at 60 kPa: column (+25, +3, -2) kN.m/deg -- the three body axes.
            var column = new[] { 25.0, 3.0, -2.0 };
            int needed = SurfaceModel.EngagementAxes (column, PitchFin ());
            Check (needed == 7, "the super hope's fin needs the three axes (its column contributes to all)");
            Check (!SurfaceModel.Owned (needed, 2), "the coast (roll owned) engages no fin");
            Check (!SurfaceModel.Owned (needed, 5), "pitch/yaw without the roll engages no fin");
            Check (SurfaceModel.Owned (needed, 7), "the descent (three owned) engages it");
            // A flap with no roll column whose command answers the roll: the AutoPilot's roll would
            // move it -- it needs the roll owned too.
            var flapColumn = new[] { 40.0, 0.0, 0.0 };
            var flap = new SurfaceCommand { CPitch = 1.0, KRoll = 0.5, Range = 20.0, Authority = 100.0, Direction = 1.0 };
            Check (SurfaceModel.ContributedAxes (flapColumn) == 1 && SurfaceModel.EngagementAxes (flapColumn, flap) == 3,
                "the command's axes join the column's");
            Check (!SurfaceModel.Owned (3, 1) && SurfaceModel.Owned (3, 3) && SurfaceModel.Owned (1, 5),
                "owned covers every needed axis");
            // A column that tells nothing still needs every axis (an earlier change's rule).
            Check (SurfaceModel.EngagementAxes (new[] { 0.0, 0.0, 0.0 }, flap) == 7, "a zero column needs every axis");
            // Every combination: owned exactly when needed is a subset of owned.
            for (int n = 0; n < 8; n++)
                for (int o = 0; o < 8; o++)
                    Check (SurfaceModel.Owned (n, o) == ((n & o) == n), "Owned(" + n + ", " + o + ")");
        }

        // ------------------------------------------------------------------ bumpless M

        static void FilterShift ()
        {
            var a = new Biquad3 ();
            var b = new Biquad3 ();
            a.Configure (0.02, 18.0, 0.7);
            b.Configure (0.02, 18.0, 0.7);
            Check (Math.Abs (a.B0 + a.B1 + a.B2 - (1.0 + a.A1 + a.A2)) < 1e-12, "H(z) has a unit DC gain");
            var x = new double[3];
            var xc = new double[3];
            var ya = new double[3];
            var yb = new double[3];
            var c = new[] { 1.5e5, -2.4e6, 7.0 };
            var rng = new Random (180);
            double worst = 0.0;
            // A from the old definition, then moved by c at step 40; B with the new one from the start.
            for (int k = 0; k < 120; k++) {
                for (int i = 0; i < 3; i++) {
                    x [i] = 1e5 * Math.Sin (0.11 * k + i) + 2e4 * (rng.NextDouble () - 0.5);
                    xc [i] = x [i] + c [i];
                }
                if (k == 40)
                    a.Shift (c);
                a.Step (k >= 40 ? xc : x, ya);
                b.Step (xc, yb);
                if (k >= 40)
                    for (int i = 0; i < 3; i++)
                        worst = Math.Max (worst, Math.Abs (ya [i] - yb [i]) / (Math.Abs (yb [i]) + 1.0));
            }
            Check (worst < 1e-12, "a filter moved by c then fed x + c is the filter fed x + c from the start (" +
                worst.ToString ("E2") + ")");
            var unprimed = new Biquad3 ();
            unprimed.Configure (0.02, 18.0, 0.7);
            unprimed.Shift (c);
            Check (!unprimed.Primed, "nothing to move before the first sample");
        }

        /// <summary>One step of the law on one axis (pitch), the fins' torque F in M or not.</summary>
        sealed class Bench
        {
            internal readonly AttitudeGains G = new AttitudeGains ();
            internal readonly AttitudeLawState St = new AttitudeLawState ();
            internal readonly AttitudeWork Work = new AttitudeWork ();
            internal readonly AttitudeOutput O = new AttitudeOutput ();
            internal readonly AttitudeInput Inp = new AttitudeInput ();

            internal Bench ()
            {
                Inp.Resize (4);
                Inp.Dt = 0.02;
            }

            internal void Step (int k, double others, double fins, bool finsInM)
            {
                var i = Inp;
                for (int a = 0; a < 3; a++) {
                    i.Omega [a] = a == 0 ? 0.02 * Math.Sin (0.07 * k) : 0.0;
                    i.OmegaDotMeasured [a] = a == 0 ? 0.0014 * Math.Cos (0.07 * k) : 0.0;
                    i.Inertia [a] = a == 1 ? 2.5e5 : 9.48e6;
                    i.Error [a] = a == 0 ? 0.01 * Math.Cos (0.05 * k) : 0.0;
                    i.OmegaFf [a] = 0.0;
                    i.OmegaDotFf [a] = 0.0;
                    i.TorqueFree [a] = 0.0;
                    i.Torque [a] = a == 0 ? others + (finsInM ? fins : 0.0) : 0.0;
                }
                i.OmegaDotValid = k > 0;
                i.ReferenceValid = true;
                i.OmegaMax = 0.2;
                i.UseTorqueFree = false;
                for (int j = 0; j < 12; j++)
                    i.B [j] = 0.0;
                // One gimbal column, the three ctrlState columns (RCS + wheel), on pitch.
                i.B [0 * 4 + 0] = 3.0e5;
                i.B [0 * 4 + 1] = 7.0e5;
                i.B [1 * 4 + 2] = 1.2e5;
                i.B [2 * 4 + 3] = 7.0e5;
                for (int j = 0; j < 4; j++) {
                    i.U0 [j] = 0.0;
                    i.UMin [j] = j == 0 ? -10.5 : -1.0;
                    i.UMax [j] = j == 0 ? 10.5 : 1.0;
                    i.UPref [j] = 0.0;
                    i.Priority [j] = j == 0 ? 1.0 : 3.0;
                }
                AttitudeMath.Step (G, i, St, Work, O);
            }
        }

        static void LawShift ()
        {
            // The fins' realized torque at an engagement at the PDI of the shadow flights: 0.04-0.25
            // MN.m (surface_model_v1 of 091242-134825); a slowly varying one here, constant before T.
            const int T = 60;
            Func<int, double> fins = k => k < T ? -2.5e5 : -2.5e5 + 3e4 * Math.Sin (0.2 * (k - T));
            Func<int, double> others = k => 4e5 * Math.Sin (0.09 * k);
            var reference = new Bench ();   // the fins in M from the start (the new definition)
            var shifted = new Bench ();     // engaged at T with ShiftTorque
            var plain = new Bench ();       // engaged at T without it (the build before an earlier change)
            double worstShift = 0.0, worstResidual = 0.0, plainJump = 0.0, plainResidual = 0.0;
            var delta = new double[3];
            for (int k = 0; k < 160; k++) {
                bool inM = k >= T;
                if (k == T) {
                    delta [0] = fins (k);
                    shifted.St.ShiftTorque (delta);
                }
                reference.Step (k, others (k), fins (k), true);
                shifted.Step (k, others (k), fins (k), inM);
                plain.Step (k, others (k), fins (k), inM);
                if (k >= T) {
                    worstShift = Math.Max (worstShift, Math.Abs (shifted.O.TorqueAllocTarget [0] - reference.O.TorqueAllocTarget [0]));
                    worstResidual = Math.Max (worstResidual, Math.Abs (shifted.O.Residual [0] - reference.O.Residual [0]));
                    plainJump = Math.Max (plainJump, Math.Abs (plain.O.TorqueAllocTarget [0] - reference.O.TorqueAllocTarget [0]));
                    plainResidual = Math.Max (plainResidual, Math.Abs (plain.O.Residual [0] - reference.O.Residual [0]));
                }
            }
            Console.WriteLine ("  engagement at 0.25 MN.m: allocation target off the new definition by {0:E2} N.m with " +
                "the shift, {1:E3} N.m without; residual {2:E2} / {3:E2} rad/s^2", worstShift, plainJump, worstResidual,
                plainResidual);
            Check (worstShift < 1e-6 * 2.5e5, "with the shift the engagement is the new definition from the start (target)");
            Check (worstResidual < 1e-12, "with the shift the INDI residual takes no step");
            Check (plainJump > 0.5 * 2.5e5, "without it the target jumps by about the fins' torque");
            Check (plainResidual > 0.1 * 2.5e5 / 9.48e6, "without it the residual takes the step too");
            // Disengagement: the same in reverse (the definition loses the fins).
            var ref2 = new Bench ();
            var out2 = new Bench ();
            double worstOut = 0.0;
            for (int k = 0; k < 160; k++) {
                bool inM = k < T;
                if (k == T) {
                    delta [0] = -fins (T - 1);
                    out2.St.ShiftTorque (delta);
                }
                ref2.Step (k, others (k), fins (k), false);
                out2.Step (k, others (k), fins (k), inM);
                if (k >= T)
                    worstOut = Math.Max (worstOut, Math.Abs (out2.O.TorqueAllocTarget [0] - ref2.O.TorqueAllocTarget [0]));
            }
            Check (worstOut < 1e-6 * 2.5e5, "a disengagement with the shift is the old definition from the start (" +
                worstOut.ToString ("E2") + ")");
            // Unprimed law: nothing moves (the first filtered step primes from M).
            var fresh = new AttitudeLawState ();
            fresh.ShiftTorque (delta);
            Check (!fresh.FilterTorque.Primed && !fresh.HavePrevious, "a law not yet filtered is not moved");
        }
    }
}
