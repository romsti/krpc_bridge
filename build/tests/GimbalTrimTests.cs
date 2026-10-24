// -----------------------------------------------------------------------------------
// option 128 of AttitudeOptionsV1, the gimbals without thrust to neutral and the gimbal
// columns at the coming thrust (AttitudeMath.TrimNullColumns, GimbalSpool.cs). What these tests pin down:
//   * option 128 is a new bit (supported 255, 511; the client) and changes NO existing
//     rule (blocking sets, write masks, frozen axes, neutralization, roll gate, reference age) for any
//     older option set;
//   * GimbalSpool: the horizon is the time constant of the stock gimbal Lerp; the closed form of the
//     engine spool equals KSP's step-by-step float Lerp (ModuleEngines.UpdateThrottle) at every integer
//     step, its snap, its two speeds, an engine without response time or with the alternate law; the
//     expected thrust never below the thrust now;
//   * the law: TrimNullColumns changes nothing while every column has an effect (random draws, bit for
//     bit); a dead gimbal column held at its bound by the warm working set stays there without it (the
//     flight defect: its multiplier under KktTolerance) and goes exactly to u_pref with it;
//   * the closed loop on a booster-like plant (the heavy booster's numbers: 4 modules of 1.8 MN.m/deg at
//     full thrust, +-2 deg, RCS 2.8 MN.m, a spool time constant of 0.67 s, gimbals Lerping 0.16 a step):
//     a burn that saturates the gimbals, the thrust cut, a coast, the ignition. Without the option the
//     gimbals are still at their stop at the ignition and the rising thrust brings their torque (0.92
//     MN.m on this plant); with it they are at neutral and the misalignment is trimmed as the thrust
//     comes (0.018 MN.m). Then the AutoPilot's parking: the law takes the ignition with the gimbals at
//     their stop; the spool's free torque brings them back from the first step (0.83 MN.m), their slew
//     does the rest: the full gain needs the law to own the seconds before the ignition.
// The golden vectors of the option (attitude_v1_golden_trim.json) are written by Program.WriteGoldenTrim.
// -----------------------------------------------------------------------------------

using System;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static class GimbalTrimTests
    {
        static int checks;
        static int failures;
        static readonly bool trace = Environment.GetEnvironmentVariable ("BRIDGE_D237_TRACE") == "1";

        const int Option = AttitudeWrite.OptionGimbalTrim;
        const int Armed = AttitudeWrite.ModeArmed;
        const int Both = AttitudeWrite.ArmRoll | AttitudeWrite.ArmPitchYaw;
        const int Earlyish = AttitudeWrite.StageEarlyish;

        public static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestBits ();
            TestOptionChangesNoExistingRule ();
            TestHorizon ();
            TestSpoolAgainstKsp ();
            TestExpectedThrust ();
            TestLawUnchangedWithEffect ();
            TestDeadColumnAtItsBound ();
            TestClosedLoop ();
            checkCount = checks;
            Console.WriteLine ("gimbal trim checks: " + checks + ", failures: " + failures);
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

        // ------------------------------------------------------------------ the option

        static void TestBits ()
        {
            Check (Option == 128 && (AttitudeWrite.OptionsSupported & 255) == 255,
                "option 128, supported 255 and above (mirrored by the client; " +
                "256, TranslationTests)");
            Check (AttitudeWrite.ValidOptions (128) && AttitudeWrite.ValidOptions (255) &&
                !AttitudeWrite.ValidOptions (AttitudeWrite.OptionsSupported + 1) && !AttitudeWrite.ValidOptions (-1),
                "valid options 0-255 (and 256 and above)");
            for (int o = 0; o < 128; o++)
                Check ((o & Option) == 0, "no older option holds bit 128 (" + o + ")");
            Check ((AttitudeWrite.AllowedWriteMask (Option) & ~AttitudeWrite.WriteMaskBase) == 0,
                "option 128 allows no write-mask bit");
        }

        static void TestOptionChangesNoExistingRule ()
        {
            var rng = new Random (2371);
            for (int options = 0; options < 128; options++) {
                int o = options | Option;
                Check (AttitudeWrite.RollBlockingSet (o) == AttitudeWrite.RollBlockingSet (options),
                    "option 128: roll blocking set of options " + options);
                Check (AttitudeWrite.PitchYawBlockingSet (o) == AttitudeWrite.PitchYawBlockingSet (options),
                    "option 128: pitch/yaw blocking set of options " + options);
                Check (AttitudeWrite.AllowedWriteMask (o) == AttitudeWrite.AllowedWriteMask (options),
                    "option 128: allowed write mask of options " + options);
                for (int w = -2; w < 32; w++)
                    Check (AttitudeWrite.ValidWriteMask (w, o) == AttitudeWrite.ValidWriteMask (w, options),
                        "option 128: write mask " + w + " (options " + options + ")");
                for (int w = 0; w < 32; w++) {
                    Check (AttitudeWrite.EffectiveWriteMask (w, o) == AttitudeWrite.EffectiveWriteMask (w, options),
                        "option 128: stored mask " + w);
                    foreach (double age in new[] { 0.1, 0.31, 1.0, double.PositiveInfinity, double.NaN }) {
                        Check (AttitudeWrite.PitchYawReferenceOld (age, w, o) ==
                            AttitudeWrite.PitchYawReferenceOld (age, w, options), "option 128: reference age rule");
                        Check (AttitudeWrite.PitchYawReferenceHeld (age, w, o) ==
                            AttitudeWrite.PitchYawReferenceHeld (age, w, options), "option 128: reference held");
                    }
                    for (int last = 0; last < 8; last++)
                        for (int stage = 1; stage < 3; stage++)
                            Check (AttitudeWrite.SurfaceOwnedAxes (Armed, Both, w, o, stage, last) ==
                                AttitudeWrite.SurfaceOwnedAxes (Armed, Both, w, options, stage, last),
                                "option 128: surfaces' owned axes");
                }
            }
            for (int i = 0; i < 20000; i++) {
                long motif = ((long)rng.Next () << 31) ^ rng.Next () ^ ((long)rng.Next (0, 4096) << 32);
                if (i < 64)
                    motif = 1L << i % 44;
                int options = rng.Next (0, 128);
                int o = options | Option;
                int unneutralizable = rng.Next (0, 8);
                Check (AttitudeWrite.FrozenCtrlAxes (o, motif, unneutralizable) ==
                    AttitudeWrite.FrozenCtrlAxes (options, motif, unneutralizable), "option 128: frozen axes identical");
                int now = rng.Next (0, 8), last = rng.Next (0, 8), answering = rng.Next (0, 8);
                Check (AttitudeWrite.NeutralizeSurfaces (o, Armed, Earlyish, now, last, answering) ==
                    AttitudeWrite.NeutralizeSurfaces (options, Armed, Earlyish, now, last, answering),
                    "option 128: neutralization identical");
                Check (AttitudeWrite.RollSurfaceGate (o, answering, (i & 1) == 0) ==
                    AttitudeWrite.RollSurfaceGate (options, answering, (i & 1) == 0), "option 128: roll gate identical");
                for (int mode = 0; mode < 2; mode++)
                    for (int axes = 0; axes < 4; axes++)
                        for (int stage = 1; stage < 3; stage++) {
                            bool computed = (i & 2) == 0;
                            Check (AttitudeWrite.MayWritePitchYaw (mode, axes, stage, computed, motif, o) ==
                                AttitudeWrite.MayWritePitchYaw (mode, axes, stage, computed, motif, options),
                                "option 128: MayWritePitchYaw identical");
                            Check (AttitudeWrite.MayWrite (mode, axes, stage, computed, motif, o) ==
                                AttitudeWrite.MayWrite (mode, axes, stage, computed, motif, options),
                                "option 128: MayWrite identical");
                            Check (AttitudeWrite.ContactHold (o, mode, axes, stage, computed, true, motif) ==
                                AttitudeWrite.ContactHold (options, mode, axes, stage, computed, true, motif),
                                "option 128: ContactHold identical");
                        }
            }
        }

        // ------------------------------------------------------------------ GimbalSpool

        static void TestHorizon ()
        {
            double f = AttitudeTranslation.RampFactor (true, 8.0, 0.02);
            Check (Math.Abs (f - 0.16) < 1e-15, "the stock gimbal Lerp: 8 x 0.02 = 0.16 a step");
            double h = GimbalSpool.HorizonSteps (f);
            Check (Math.Abs (h - (-1.0 / Math.Log (0.84))) < 1e-12 && h > 5.73 && h < 5.74,
                "horizon: the time constant, 5.735 steps (0.115 s)");
            // After the horizon a Lerp of factor f has covered 1 - 1/e of its way.
            Check (Math.Abs (Math.Pow (1.0 - f, h) - Math.Exp (-1.0)) < 1e-12, "1 - 1/e covered at the horizon");
            Check (GimbalSpool.HorizonSteps (1.0) == 1.0 && GimbalSpool.HorizonSteps (2.0) == 1.0,
                "an instant gimbal acts at the next step");
            Check (GimbalSpool.HorizonSteps (AttitudeTranslation.RampFactor (false, 8.0, 0.02)) == 1.0,
                "no response speed: instant");
            Check (double.IsPositiveInfinity (GimbalSpool.HorizonSteps (0.0)) &&
                double.IsPositiveInfinity (GimbalSpool.HorizonSteps (double.NaN)), "a gimbal that does not move");
        }

        /// <summary>ModuleEngines.UpdateThrottle with useEngineResponseTime, KSP 1.12.5 (decompiled), in float.</summary>
        static float KspSpoolStep (float current, float requested, float acceleration, float deceleration, float dt)
        {
            if (Math.Abs (current - requested) <= 0.0001f)
                return requested;
            float speed = current < requested ? acceleration : deceleration;
            float t = speed * dt;
            if (t < 0f)
                t = 0f;
            if (t > 1f)
                t = 1f;
            return current + (requested - current) * t;      // Mathf.Lerp
        }

        static void TestSpoolAgainstKsp ()
        {
            float dt = 0.02f;
            foreach (float accel in new[] { 0.5f, 1.5f, 3.0f, 10.0f, 49.0f }) {
                foreach (var pair in new[] { new[] { 0f, 1f }, new[] { 0f, 0.4f }, new[] { 0.2f, 0.9f },
                        new[] { 1f, 0f }, new[] { 0.7f, 0.05f } }) {
                    float c = pair [0];
                    float req = pair [1];
                    float decel = accel * 0.7f;
                    float ksp = c;
                    for (int k = 1; k <= 60; k++) {
                        ksp = KspSpoolStep (ksp, req, accel, decel, dt);
                        double model = GimbalSpool.ExpectedThrottle (c, req, true, false, accel, decel, dt, k);
                        // The snap: KSP jumps to requested within 1e-4; the closed form approaches it.
                        Check (Math.Abs (model - ksp) < 2e-4 + 1e-5 * k,
                            "closed form = KSP's spool, accel " + accel + ", " + c + " -> " + req + ", step " + k +
                            " (" + model + " / " + ksp + ")");
                    }
                }
            }
            Check (GimbalSpool.ExpectedThrottle (0.3, 0.8, false, false, 1.0, 1.0, 0.02, 5.0) == 0.8,
                "without the response time: requested at once");
            Check (GimbalSpool.ExpectedThrottle (0.3, 0.8, true, true, 1.0, 1.0, 0.02, 5.0) == 0.8,
                "the alternate law: requested (the upper bound)");
            Check (GimbalSpool.ExpectedThrottle (0.50005, 0.5, true, false, 1.0, 1.0, 0.02, 5.0) == 0.5,
                "the snap");
            Check (GimbalSpool.ExpectedThrottle (0.0, 1.0, true, false, 1.0, 1.0, 0.02, double.PositiveInfinity) == 1.0,
                "an infinite horizon: requested");
            Check (GimbalSpool.ExpectedThrottle (0.0, 1.0, true, false, 0.0, 1.0, 0.02, 5.0) == 0.0,
                "no acceleration: stays");
            Check (GimbalSpool.ExpectedThrottle (0.0, 1.0, true, false, 100.0, 1.0, 0.02, 5.0) == 1.0,
                "a Lerp factor of 1 or more: requested at the next step");
            Check (double.IsNaN (GimbalSpool.ExpectedThrottle (double.NaN, 1.0, true, false, 1.0, 1.0, 0.02, 5.0)),
                "a non-finite current is returned");
            double up = GimbalSpool.ExpectedThrottle (0.0, 1.0, true, false, 1.5, 1.5, 0.02, 5.735);
            Check (Math.Abs (up - (1.0 - Math.Pow (0.97, 5.735))) < 1e-15, "spool up at the gimbal horizon: 16 %");
        }

        static void TestExpectedThrust ()
        {
            Check (GimbalSpool.ExpectedThrust (100.0, 0.25, double.NaN, 0.5) == 200.0,
                "K = finalThrust / currentThrottle while it thrusts");
            Check (GimbalSpool.ExpectedThrust (0.0, 0.0, 4500.0, 0.16) == 4500.0 * 0.16,
                "not thrusting: the full thrust at this atmosphere");
            Check (GimbalSpool.ExpectedThrust (0.002, 0.0005, 4500.0, 0.16) == 4500.0 * 0.16,
                "below the readable throttle: the full thrust at this atmosphere");
            Check (GimbalSpool.ExpectedThrust (100.0, 0.5, double.NaN, 0.1) == 100.0,
                "a spool down keeps the thrust now (b / b_hat <= 1)");
            Check (GimbalSpool.ExpectedThrust (0.0, 0.0, double.NaN, 0.5) == 0.0 &&
                GimbalSpool.ExpectedThrust (0.0, 0.0, -1.0, 0.5) == 0.0 &&
                GimbalSpool.ExpectedThrust (50.0, 0.5, double.NaN, double.NaN) == 50.0, "unreadable: the thrust now");
        }

        // ------------------------------------------------------------------ the law

        static void Fill (AttitudeInput inp, Random rng, double thrustScale)
        {
            inp.Resize (11);
            inp.Dt = 0.02;
            inp.Inertia [0] = 6.0e7; inp.Inertia [1] = 4.0e6; inp.Inertia [2] = 6.0e7;
            for (int i = 0; i < 33; i++)
                inp.B [i] = 0.0;
            for (int g = 0; g < 4; g++) {
                inp.B [0 * 11 + 2 * g] = thrustScale * (1.0e5 + 2.0e4 * rng.NextDouble ());
                inp.B [1 * 11 + 2 * g] = thrustScale * (rng.NextDouble () - 0.5) * 4.0e3;
                inp.B [2 * 11 + 2 * g + 1] = thrustScale * (1.0e5 + 2.0e4 * rng.NextDouble ());
                for (int k = 0; k < 2; k++) {
                    int j = 2 * g + k;
                    inp.UMin [j] = -2;
                    inp.UMax [j] = 2;
                    inp.Priority [j] = 1.0;
                    inp.UPref [j] = 0.0;
                    inp.U0 [j] = -2 + 4 * rng.NextDouble ();
                }
            }
            for (int c = 0; c < 3; c++) {
                int j = 8 + c;
                inp.B [c * 11 + j] = -2.0e4;
                inp.UMin [j] = -1;
                inp.UMax [j] = 1;
                inp.Priority [j] = 3.0;
                inp.UPref [j] = 0.0;
                inp.U0 [j] = rng.NextDouble () * 2 - 1;
            }
            for (int a = 0; a < 3; a++) {
                inp.Omega [a] = 0.05 * (rng.NextDouble () - 0.5);
                inp.OmegaDotMeasured [a] = 0.01 * (rng.NextDouble () - 0.5);
                inp.Torque [a] = 1.0e5 * (rng.NextDouble () - 0.5);
            }
            AttitudeMath.PointingError (0.4 * (rng.NextDouble () - 0.5), 1.0, 0.4 * (rng.NextDouble () - 0.5), inp.Error);
            inp.OmegaDotValid = true;
            inp.ReferenceValid = true;
            inp.OmegaMax = 0.2;
        }

        static void Same (AttitudeOutput a, AttitudeOutput b, int n, string what)
        {
            bool same = a.Computed == b.Computed && a.Iterations == b.Iterations && a.Converged == b.Converged &&
                a.AxisGuardMask == b.AxisGuardMask;
            for (int k = 0; k < 3; k++)
                same = same && a.Nu [k] == b.Nu [k] && a.TorqueAllocated [k] == b.TorqueAllocated [k] &&
                    a.OmegaPch [k] == b.OmegaPch [k] && a.Alpha [k] == b.Alpha [k];
            for (int j = 0; j < n; j++)
                same = same && a.DeltaU [j] == b.DeltaU [j] && a.RegularizationWeight [j] == b.RegularizationWeight [j] &&
                    a.ActiveSet [j] == b.ActiveSet [j];
            Check (same, what);
        }

        static void TestLawUnchangedWithEffect ()
        {
            var rng = new Random (2372);
            var g = new AttitudeGains ();
            for (int draw = 0; draw < 400; draw++) {
                var s0 = new AttitudeLawState ();
                var s1 = new AttitudeLawState ();
                var w0 = new AttitudeWork ();
                var w1 = new AttitudeWork ();
                var o0 = new AttitudeOutput ();
                var o1 = new AttitudeOutput ();
                var i0 = new AttitudeInput ();
                var i1 = new AttitudeInput ();
                double scale = draw % 3 == 0 ? 0.02 : (draw % 3 == 1 ? 1.0 : 10.0);
                int seed = rng.Next ();
                for (int t = 0; t < 12; t++) {
                    Fill (i0, new Random (seed + t), scale);
                    Fill (i1, new Random (seed + t), scale);
                    i1.TrimNullColumns = 8;
                    AttitudeMath.Step (g, i0, s0, w0, o0);
                    AttitudeMath.Step (g, i1, s1, w1, o1);
                    Same (o0, o1, 11, "every column with an effect: TrimNullColumns changes nothing (draw " + draw +
                        ", tick " + t + ")");
                    Check (o1.NullColumnsTrimmed == 0 && o0.NullColumnsTrimmed == 0, "no null column counted");
                }
            }
        }

        static void TestDeadColumnAtItsBound ()
        {
            var g = new AttitudeGains ();
            foreach (int trim in new[] { 0, 8 }) {
                var st = new AttitudeLawState ();
                var work = new AttitudeWork ();
                var o = new AttitudeOutput ();
                var inp = new AttitudeInput ();
                var rng = new Random (2373);
                // Under thrust, a large demand: every gimbal to its bound (the warm working set holds them).
                Fill (inp, rng, 0.05);
                for (int j = 0; j < 8; j++)
                    inp.U0 [j] = 0.0;
                AttitudeMath.PointingError (0.0, 1.0, 0.0, inp.Error);
                inp.TrimNullColumns = trim;
                for (int t = 0; t < 25; t++) {
                    inp.Error [0] = 0.5;
                    inp.Error [2] = -0.5;
                    AttitudeMath.Step (g, inp, st, work, o);
                    for (int j = 0; j < 8; j++)
                        inp.U0 [j] = o.U [j];
                }
                bool bounded = true;
                for (int j = 0; j < 8; j += 2)
                    bounded = bounded && Math.Abs (Math.Abs (o.U [j]) - 2.0) < 1e-12;
                Check (bounded, "the burn drives the pitch gimbals to their bounds (trim " + trim + ")");
                // The thrust is cut: every gimbal column is null.
                for (int j = 0; j < 8; j++)
                    for (int a = 0; a < 3; a++)
                        inp.B [a * 11 + j] = 0.0;
                for (int t = 0; t < 10; t++) {
                    inp.Error [0] = 0.01;
                    inp.Error [2] = -0.01;
                    AttitudeMath.Step (g, inp, st, work, o);
                    for (int j = 0; j < 8; j++)
                        inp.U0 [j] = o.U [j];
                }
                double far = 0.0;
                for (int j = 0; j < 8; j++)
                    far = Math.Max (far, Math.Abs (o.U [j]));
                if (trim == 0) {
                    Check (far == 2.0, "without the option a dead gimbal stays at its bound (the flight defect): " + far);
                    Check (o.NullColumnsTrimmed == 0, "nothing counted without the option");
                    // Its multiplier never passes KktTolerance: the floor's weight squared times its distance.
                    double wu = o.RegularizationWeight [0];
                    Check (wu * wu * 2.0 < AttitudeMath.KktTolerance, "the floor's multiplier, " + (wu * wu * 2.0));
                } else {
                    Check (far == 0.0, "with the option every dead gimbal is exactly at u_pref (neutral): " + far);
                    Check (o.NullColumnsTrimmed == 8 && o.Converged, "eight null columns regularized, converged");
                    Check (o.RegularizationWeight [0] == Math.Sqrt (g.Gamma), "the zero-span weight, sqrt(gamma)");
                    Check (o.RegularizationWeight [8] != Math.Sqrt (g.Gamma), "the ctrlState columns untouched");
                }
            }
            // u_pref is honored where it is not zero, within the box.
            {
                var st = new AttitudeLawState ();
                var o = new AttitudeOutput ();
                var inp = new AttitudeInput ();
                Fill (inp, new Random (2374), 1.0);
                for (int j = 0; j < 8; j++) {
                    for (int a = 0; a < 3; a++)
                        inp.B [a * 11 + j] = 0.0;
                    inp.U0 [j] = 1.5;
                    inp.UPref [j] = j == 0 ? 3.0 : -0.5;
                }
                inp.TrimNullColumns = 8;
                AttitudeMath.Step (g, inp, st, new AttitudeWork (), o);
                Check (o.U [0] == 2.0 && Math.Abs (o.U [1] + 0.5) < 1e-12, "toward u_pref, within the box");
            }
        }

        // ------------------------------------------------------------------ closed loop

        /// <summary>
        /// One axis of a booster (pitch, body x), four gimbal modules on it: the realized engine torque is
        /// c (sum b u + misalignment), c the spool's throttle; the RCS on the ctrlState column; the
        /// gimbals Lerp 0.16 a step toward their command; the engines spool as KSP (1.5 / s). The law
        /// writes the gimbals and the RCS while `owned`; otherwise the gimbals follow `parked`. With
        /// `option`, the law gets what AttitudeV1 gives it under option 128: the columns at the thrust of
        /// the gimbal's horizon (never below now), the spool's torque at the current deflections as free
        /// torque, the null gimbal columns regularized. Returns the largest |engine torque| and |omega| in
        /// the second after the ignition, and the largest |gimbal command| at the ignition.
        /// </summary>
        static void Loop (bool option, bool lawOwnsCoast, out double kick, out double omegaAfter, out double atIgnition)
        {
            const int n = 11;
            const double dt = 0.02, f = 0.16, accel = 1.5, decel = 50.0;
            const double bFull = 1.8e6, misalignment = 0.4e6, rcs = -2.8e6, inertia = 6.35e7;
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            inp.Resize (n);
            inp.Dt = dt;
            inp.Inertia [0] = inertia; inp.Inertia [1] = 4.0e6; inp.Inertia [2] = inertia;
            for (int j = 0; j < n; j++) {
                inp.UMin [j] = j < 8 ? -2.0 : -1.0;
                inp.UMax [j] = j < 8 ? 2.0 : 1.0;
                inp.Priority [j] = j < 8 ? 1.0 : 3.0;
            }
            var u = new double[8];          // realized deflections
            var target = new double[8];     // their command
            double rcsCmd = 0.0, c = 0.0, omega = 0.0, theta = 0.0, omegaPrev = 0.0, torque = 0.0;
            double horizon = GimbalSpool.HorizonSteps (f);
            kick = 0.0;
            omegaAfter = 0.0;
            atIgnition = double.NaN;
            int burnEnd = 60, ignition = 260, end = 320;
            for (int k = 0; k < end; k++) {
                double requested = k < burnEnd || k >= ignition ? 1.0 : 0.0;
                bool owned = k < burnEnd || lawOwnsCoast || k >= ignition;
                if (k == ignition) {
                    // The coast ended on its attitude: from here any engine torque is a disturbance.
                    theta = 0.0;
                    omega = 0.0;
                    omegaPrev = 0.0;
                    atIgnition = 0.0;
                    for (int j = 0; j < 8; j++)
                        atIgnition = Math.Max (atIgnition, Math.Abs (u [j]));
                }
                // ------------------------------------------------ Earlyish: the law
                double cExp = option ? GimbalSpool.ExpectedThrottle (c, requested, true, false, accel, decel, dt, horizon) : c;
                if (cExp < c)
                    cExp = c;
                for (int a = 0; a < 3; a++)
                    for (int j = 0; j < n; j++)
                        inp.B [a * n + j] = 0.0;
                for (int j = 0; j < 8; j++) {
                    inp.B [0 * n + j] = j % 2 == 0 ? bFull * cExp : 0.0;
                    inp.B [2 * n + j] = j % 2 == 1 ? bFull * cExp : 0.0;
                    inp.U0 [j] = u [j];
                    inp.UPref [j] = 0.0;
                }
                for (int a = 0; a < 3; a++) {
                    inp.B [a * n + 8 + a] = rcs;
                    inp.U0 [8 + a] = a == 0 ? rcsCmd : 0.0;
                    inp.UPref [8 + a] = 0.0;
                }
                inp.Omega [0] = omega;
                inp.OmegaDotMeasured [0] = k == 0 ? 0.0 : (omega - omegaPrev) / dt;
                inp.Torque [0] = torque;
                inp.OmegaDotValid = k > 0;
                inp.ReferenceValid = true;
                inp.Error [0] = -theta;
                inp.OmegaMax = 0.2;
                inp.TrimNullColumns = option ? 8 : 0;
                inp.UseTorqueFree = false;
                if (option && cExp != c) {
                    double free = 0.0;
                    for (int j = 0; j < 8; j += 2)
                        free += bFull * (cExp - c) * u [j];
                    inp.TorqueFree [0] = free + misalignment * (cExp - c);
                    inp.TorqueFree [1] = 0.0;
                    inp.TorqueFree [2] = 0.0;
                    inp.UseTorqueFree = true;
                }
                AttitudeMath.Step (g, inp, st, work, o);
                if (owned && o.Computed) {
                    for (int j = 0; j < 8; j++)
                        target [j] = o.U [j];
                    rcsCmd = o.U [8];
                } else {
                    for (int j = 0; j < 8; j++)
                        target [j] = 2.0;         // the AutoPilot's saturated ctrlState, a gimbal at its stop
                    rcsCmd = 0.0;
                }
                // ------------------------------------------------ physics
                c = c + (requested - c) * Math.Min (1.0, (c < requested ? accel : decel) * dt);
                for (int j = 0; j < 8; j++)
                    u [j] = u [j] + (target [j] - u [j]) * f;
                double engine = misalignment * c;
                for (int j = 0; j < 8; j += 2)
                    engine += bFull * c * u [j];
                torque = engine + rcs * rcsCmd;
                // The burn fights a torque beyond its gimbals (the end of a boostback): they end at their stop.
                double disturbance = k < burnEnd ? -1.6e7 * c : 0.0;
                omegaPrev = omega;
                omega += (torque + disturbance) / inertia * dt;
                theta += omega * dt;
                if (k >= ignition && k < ignition + 50) {
                    kick = Math.Max (kick, Math.Abs (engine));
                    omegaAfter = Math.Max (omegaAfter, Math.Abs (omega));
                }
                if (trace && (k % 10 == 0 || (k >= ignition && k < ignition + 20)))
                    Console.WriteLine ("option=" + option + " coast=" + lawOwnsCoast + " k=" + k + " c=" + c.ToString ("F3") +
                        " u0=" + u [0].ToString ("F3") + " engine=" + (engine / 1e6).ToString ("F3") +
                        " omega=" + (omega * 57.3).ToString ("F3") + " theta=" + (theta * 57.3).ToString ("F3"));
            }
        }

        static void TestClosedLoop ()
        {
            double kickOff, kickOn, wOff, wOn, atOff, atOn;
            // The loop owns the coast (the super hope under M5): the burn's working set holds the gimbals.
            Loop (false, true, out kickOff, out wOff, out atOff);
            Loop (true, true, out kickOn, out wOn, out atOn);
            if (trace)
                Console.WriteLine ("loop owns the coast: kick " + kickOff + " / " + kickOn + ", omega " + wOff + " / " +
                    wOn + ", at ignition " + atOff + " / " + atOn);
            // Measured on this plant: 0.92 MN.m and 0.0024 rad/s without, 0.018 MN.m and 0.00011 rad/s with.
            Check (atOff > 2.0 - 1e-9, "without the option the gimbals are at their stop at the ignition: " + atOff);
            Check (kickOff > 0.5e6, "and the spool brings a kick (the gimbals' torque on the rising thrust): " + kickOff);
            Check (atOn < 1e-9, "with the option they are at neutral: " + atOn);
            Check (kickOn < 0.05 * kickOff, "the kick: " + kickOn + " against " + kickOff);
            Check (wOn < 0.1 * wOff, "|omega| after the ignition: " + wOn + " against " + wOff);
            // The AutoPilot owns the coast (the heavy booster today): the gimbals at their stops, the law takes the
            // ignition. The option can only command them back from the first step: the spool's free torque does
            // (0.83 against 0.92 MN.m here), their slew (a Lerp of 0.16 a step) does the rest. The coast trim needs
            // the law to own the coast, or at least the seconds before the ignition.
            Loop (false, false, out kickOff, out wOff, out atOff);
            Loop (true, false, out kickOn, out wOn, out atOn);
            if (trace)
                Console.WriteLine ("AutoPilot owns the coast: kick " + kickOff + " / " + kickOn + ", omega " + wOff + " / " +
                    wOn + ", at ignition " + atOff + " / " + atOn);
            Check (atOff > 2.0 - 1e-9 && atOn > 2.0 - 1e-9, "the AutoPilot leaves them at their stop either way");
            Check (kickOn < 0.95 * kickOff, "taken at the stops: the spool's free torque brings them back first, kick " +
                kickOn + " against " + kickOff);
            Check (wOn <= wOff, "|omega| after the ignition: " + wOn + " against " + wOff);
        }
    }
}
