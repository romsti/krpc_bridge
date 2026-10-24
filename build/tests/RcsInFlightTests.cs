// -----------------------------------------------------------------------------------
// option 32 of AttitudeOptionsV1, the RCS torque in flight (RcsInFlight.cs). What these tests pin
// down:
//   * the pure maths: ModuleRCS.Update's dead zone (a component whose square is below EPSILON^2 is zero,
//     EPSILON itself kept), its enable flags, the signed columns, the in-flight torque C(next) -
//     C(realized), all-or-nothing on a non-finite value;
//   * option 32 is a new bit (supported 63, the client) and changes NO existing rule
//     (blocking sets, MayWrite, MayWritePitchYaw, frozen axes, neutralization, roll gate, contact hold,
//     write masks) for any motif mask;
//   * the CLOSED LOOP on one axis, with the law of the plugin (AttitudeMath.Step) and a stock RCS: the
//     ctrlState written at the fly-by-wire of step k is read by the next render frame's Update and
//     fired from step k + 1 (later when two physics steps share a frame), proportional above the
//     EPSILON dead zone; the wheel lerps 0.6 toward the same step's command. The numbers are those of
//     a recorded flight at 40-60 kPa (yaw inertia 9.55e6 kg.m^2, RCS column -697.6 kN.m
//     per unit, wheel -30 kN.m, the coast's trim -55 kN.m and its +-10 kN.m swell, the AutoPilot's
//     0.2 rad/s limit): without the option the loop reproduces the flight's six-step cycle at full RCS
//     power; with it the RCS lands on the trim -- no saturation, the RCS impulse within a few percent of
//     the trim's, the error held -- one frame in three skipped, a 10 % RCS efficiency error (the
//     increment keeps the INDI's integral action), a demand under the dead zone, the wheel absent (the
//     flight's battery drained by the cycle).
// Writes nothing into the golden vectors.
// -----------------------------------------------------------------------------------

using System;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static class RcsInFlightTests
    {
        static int checks;
        static int failures;
        static readonly bool trace = Environment.GetEnvironmentVariable ("BRIDGE_D188_TRACE") == "1";

        const int Option = AttitudeWrite.OptionRcsInFlight;

        public static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestDeadZone ();
            TestColumns ();
            TestInFlight ();
            TestBits ();
            TestOptionChangesNoExistingRule ();
            TestClosedLoop ();
            checkCount = checks;
            Console.WriteLine ("rcs in flight checks: " + checks + ", failures: " + failures);
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

        // ------------------------------------------------------------------ pure maths

        static void TestDeadZone ()
        {
            const double eps = 0.05;
            Check (RcsInFlight.DeadZone (0.0, eps) == 0.0 && RcsInFlight.DeadZone (0.0499, eps) == 0.0 &&
                RcsInFlight.DeadZone (-0.0499, eps) == 0.0, "under EPSILON: zero");
            // ModuleRCS.Update: inputRot.x * inputRot.x < num (EPSILON * EPSILON): EPSILON itself is kept.
            Check (RcsInFlight.DeadZone (eps, eps) == eps && RcsInFlight.DeadZone (-eps, eps) == -eps,
                "at EPSILON: kept (the test is a strict square)");
            Check (RcsInFlight.DeadZone (0.0501, eps) == 0.0501 && RcsInFlight.DeadZone (-1.0, eps) == -1.0,
                "above: unchanged (proportional)");
            Check (RcsInFlight.DeadZone (double.NaN, eps) == 0.0 && RcsInFlight.DeadZone (double.PositiveInfinity, eps) == 0.0,
                "non-finite: zero");
            Check (RcsInFlight.DeadZone (0.01, 0.0) == 0.01 && RcsInFlight.DeadZone (0.2, 0.3) == 0.0,
                "the module's own EPSILON");
            var rng = new Random (188);
            for (int i = 0; i < 20000; i++) {
                double u = (rng.NextDouble () - 0.5) * 2.4, e = rng.NextDouble () * 0.2;
                float uf = (float)u, ef = (float)e;
                // The stock test in float, as Update computes it (the bridge passes the float fields as double).
                bool zero = uf * uf < ef * ef;
                Check ((RcsInFlight.DeadZone (uf, ef) == 0.0) == (zero || uf == 0f), "dead zone of " + u + " at " + e);
            }
            var c = new double[3];
            RcsInFlight.UpdateCommand (0.5, -0.02, -0.7, true, true, true, 0.05, c);
            Check (c [0] == 0.5 && c [1] == 0.0 && c [2] == -0.7, "Update: pitch x, roll y (dead), yaw z");
            RcsInFlight.UpdateCommand (0.5, 0.3, -0.7, false, true, false, 0.05, c);
            Check (c [0] == 0.0 && c [1] == 0.3 && c [2] == 0.0, "Update: the enable flags zero their axes first");
        }

        static void TestColumns ()
        {
            var bPos = new double[9];
            var bNeg = new double[9];
            for (int k = 0; k < 9; k++) {
                bPos [k] = 1000.0 * (k + 1);
                bNeg [k] = -500.0 * (k + 2);
            }
            var t = new double[3];
            RcsInFlight.AddColumnsTorque (bPos, bNeg, new[] { 0.5, 0.0, -0.25 }, t);
            Check (t [0] == 0.5 * bPos [0] - 0.25 * bNeg [6] && t [1] == 0.5 * bPos [1] - 0.25 * bNeg [7] &&
                t [2] == 0.5 * bPos [2] - 0.25 * bNeg [8], "signed columns: bPos for u > 0, u x bNeg for u < 0");
            RcsInFlight.AddColumnsTorque (bPos, bNeg, new[] { 0.0, 0.0, 0.0 }, t);
            Check (t [0] == 0.5 * bPos [0] - 0.25 * bNeg [6], "a zero command adds nothing");
        }

        static void TestInFlight ()
        {
            var bPos = new double[9];
            var bNeg = new double[9];
            bPos [0] = -697622.7;
            bNeg [0] = -697622.2;
            bPos [4] = -84348.7;
            bNeg [4] = -84365.3;
            bPos [8] = -697620.1;
            bNeg [8] = -697624.8;
            var scratch = new double[3];
            var t = new double[3];
            Check (RcsInFlight.AddInFlight (bPos, bNeg, new[] { 0.3, -0.1, 0.0 }, new[] { 0.3, -0.1, 0.0 }, scratch, t) &&
                t [0] == 0.0 && t [1] == 0.0 && t [2] == 0.0, "served = fired: nothing in flight");
            // The flight's step 7146 (ut 300704.53): served -1 on pitch and yaw, fired -1 / -1 at the step before.
            RcsInFlight.AddInFlight (bPos, bNeg, new[] { -1.0, 0.0, -1.0 }, new[] { 0.0, 0.0, 0.0 }, scratch, t);
            Check (Math.Abs (t [0] - 697622.2) < 1e-6 && Math.Abs (t [2] - 697624.8) < 1e-6 && t [1] == 0.0,
                "served -1, fired 0: the whole -1 in flight (+697.6 kN.m on x and z, KSP's sign)");
            var t2 = new double[] { 1.0, 2.0, 3.0 };
            RcsInFlight.AddInFlight (bPos, bNeg, new[] { 1.0, 0.0, 0.0 }, new[] { -1.0, 0.0, 0.0 }, scratch, t2);
            Check (Math.Abs (t2 [0] - (1.0 + bPos [0] + bNeg [0])) < 1e-6 && t2 [1] == 2.0 && t2 [2] == 3.0,
                "a reversal: the two columns' torques, ADDED to what was there");
            var t3 = new double[] { 1.0, 2.0, 3.0 };
            Check (!RcsInFlight.AddInFlight (bPos, bNeg, new[] { double.NaN, 0.0, 0.0 }, new[] { 0.0, 0.0, 0.0 }, scratch, t3) &&
                t3 [0] == 1.0 && t3 [1] == 2.0 && t3 [2] == 3.0, "a non-finite command: false, nothing added");
            var bad = (double[])bPos.Clone ();
            bad [3] = double.PositiveInfinity;
            Check (!RcsInFlight.AddInFlight (bad, bNeg, new[] { 0.5, 0.0, 0.0 }, new[] { 0.0, 0.0, 0.0 }, scratch, t3) &&
                t3 [0] == 1.0, "a non-finite column: false, nothing added");
        }

        // ------------------------------------------------------------------ the option

        static void TestBits ()
        {
            Check (Option == 32 && (AttitudeWrite.OptionsSupported & 63) == 63,
                "option 32, supported 63 and above (mirrored by the client; " +
                "64, BoostbackWriteTests)");
            Check (AttitudeWrite.ValidOptions (32) && AttitudeWrite.ValidOptions (63) && !AttitudeWrite.ValidOptions (AttitudeWrite.OptionsSupported + 1) &&
                !AttitudeWrite.ValidOptions (-1), "valid options 0-63 (and 64 and above)");
            for (int o = 0; o < 32; o++)
                Check ((o & Option) == 0, "no older option holds bit 32 (" + o + ")");
        }

        static void TestOptionChangesNoExistingRule ()
        {
            var rng = new Random (1880);
            for (int options = 0; options < 32; options++) {
                Check (AttitudeWrite.RollBlockingSet (options | Option) == AttitudeWrite.RollBlockingSet (options),
                    "option 32: roll blocking set of options " + options);
                Check (AttitudeWrite.PitchYawBlockingSet (options | Option) == AttitudeWrite.PitchYawBlockingSet (options),
                    "option 32: pitch/yaw blocking set of options " + options);
                for (int w = -2; w < 64; w++)
                    Check (AttitudeWrite.ValidWriteMask (w, options | Option) == AttitudeWrite.ValidWriteMask (w, options),
                        "option 32: write mask " + w + " (options " + options + ")");
                for (int w = 0; w < 16; w++) {
                    Check (AttitudeWrite.EffectiveWriteMask (w, options | Option) == AttitudeWrite.EffectiveWriteMask (w, options),
                        "option 32: stored mask " + w);
                    foreach (double age in new[] { 0.1, 0.31, 1.0, double.PositiveInfinity })
                        Check (AttitudeWrite.PitchYawReferenceOld (age, w, options | Option) ==
                            AttitudeWrite.PitchYawReferenceOld (age, w, options), "option 32: reference age rule");
                    for (int last = 0; last < 8; last++)
                        Check (AttitudeWrite.SurfaceOwnedAxes (AttitudeWrite.ModeArmed, 3, w, options | Option,
                            AttitudeWrite.StageEarlyish, last) == AttitudeWrite.SurfaceOwnedAxes (AttitudeWrite.ModeArmed, 3, w,
                            options, AttitudeWrite.StageEarlyish, last), "option 32: surfaces' owned axes");
                }
            }
            for (int i = 0; i < 20000; i++) {
                long motif = ((long)rng.Next () << 31) ^ rng.Next () ^ ((long)rng.Next (0, 2048) << 32);
                if (i < 64)
                    motif = 1L << i % 42;
                int options = rng.Next (0, 32);
                for (int mode = 0; mode < 2; mode++)
                    for (int axes = 0; axes < 4; axes++)
                        for (int stage = 1; stage < 3; stage++) {
                            Check (AttitudeWrite.MayWrite (mode, axes, stage, true, motif, options | Option) ==
                                AttitudeWrite.MayWrite (mode, axes, stage, true, motif, options), "option 32: MayWrite identical");
                            Check (AttitudeWrite.MayWritePitchYaw (mode, axes, stage, true, motif, options | Option) ==
                                AttitudeWrite.MayWritePitchYaw (mode, axes, stage, true, motif, options),
                                "option 32: MayWritePitchYaw identical");
                            Check (AttitudeWrite.ContactHold (options | Option, mode, axes, stage, true, true, motif) ==
                                AttitudeWrite.ContactHold (options, mode, axes, stage, true, true, motif),
                                "option 32: ContactHold identical");
                        }
                int unneutralizable = rng.Next (0, 8);
                Check (AttitudeWrite.FrozenCtrlAxes (options | Option, motif, unneutralizable) ==
                    AttitudeWrite.FrozenCtrlAxes (options, motif, unneutralizable), "option 32: frozen axes identical");
                int now = rng.Next (0, 8), last2 = rng.Next (0, 8), answering = rng.Next (0, 8);
                Check (AttitudeWrite.NeutralizeSurfaces (options | Option, AttitudeWrite.ModeArmed, AttitudeWrite.StageEarlyish,
                    now, last2, answering) == AttitudeWrite.NeutralizeSurfaces (options, AttitudeWrite.ModeArmed,
                    AttitudeWrite.StageEarlyish, now, last2, answering), "option 32: neutralization identical");
                Check (AttitudeWrite.RollSurfaceGate (options | Option, answering, (i & 1) == 0) ==
                    AttitudeWrite.RollSurfaceGate (options, answering, (i & 1) == 0), "option 32: roll gate identical");
            }
        }

        // ------------------------------------------------------------------ the closed loop

        internal sealed class Scenario
        {
            internal string Name;
            internal bool InFlight;              // option 32
            internal double Inertia = 9.55e6;    // kg.m^2, yaw at 40-60 kPa (300704.5)
            internal double RcsColumn = -697.6e3;    // N.m per unit, the law's model (KSP's sign)
            internal double RcsEfficiency = 1.0;     // what the RCS really gives / the model
            internal double Epsilon = 0.05;          // ModuleRCS.EPSILON
            internal double WheelColumn = -30.0e3;   // N.m per unit (0: no wheel, battery drained)
            internal double WheelLerp = 0.6;         // torqueResponseSpeed 30 x dt
            internal double Trim = -55.0e3;          // N.m, the coast's aero trim (yaw, 40-60 kPa)
            internal double Swell = 10.0e3;          // N.m, its slow swell (0.2 Hz)
            // Two physics steps in one render frame every N steps (0: never). The flight: 4-7 % of the steps
            // (the RCS fired the command one step older than at one step per frame): one in 20.
            internal int SkipEvery = 20;
            internal double OmegaMax = 0.2;          // rad/s, the AutoPilot's limit of the coast
            // The flight's yaw error when the loop took the coast (300635.8): 0.060 rad. Its transient threw the
            // marginal cycle onto the +-1 bounds within a second, where the bounds hold it (the clipped
            // -1, -1, 0, +1, +1, 0 is a fixed cycle of u_k = u_{k-1} - u_{k-2} + c).
            internal double Error0 = 0.060;          // rad
            internal double Seconds = 16.0;
        }

        internal sealed class Result
        {
            internal double LateSaturated, LateFiring, LateRcsAbsP50, LateReversalsPerS, LatePeriodSteps;
            internal double LateMeanTorque, LateMeanDisturbance, LateRcsImpulse, LateDisturbanceImpulse;
            internal double LatePeakErrorDeg, LatePeakOmega, LatePeakU;
            internal bool Finite = true;
        }

        const double Dt = 0.02;

        internal static Result Simulate (Scenario s)
        {
            var res = new Result ();
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            const int n = 3, a = 2;          // the three ctrlState columns, the yaw axis (body z)
            inp.Resize (n);
            inp.Dt = Dt;
            var bPos = new double[9];
            var bNeg = new double[9];
            bPos [3 * a + a] = s.RcsColumn;
            bNeg [3 * a + a] = s.RcsColumn;
            var scratch = new double[3];
            var next = new double[3];
            var fired = new double[3];
            var firedPrevious = new double[3];
            var inputRot = new double[3];     // what the last Update stored (body)
            double ctrl = 0.0;                // ctrlState yaw: the served command
            double wheel = 0.0;               // the wheel's realized torque
            double omega = 0.0, omegaPrevious = 0.0, angle = -s.Error0;
            double measured = 0.0;            // the realized torque the next Earlyish reads
            int steps = (int)Math.Round (s.Seconds / Dt);
            int lateFrom = steps / 2;           // the last 8 s: after the takeover's transient
            int late = 0, saturated = 0, firing = 0, reversals = 0;
            int lastSign = 0;
            var rcsAbs = new System.Collections.Generic.List<double> ();
            double sumTorque = 0.0, sumDist = 0.0;
            for (int k = 0; k < steps; k++) {
                double t = k * Dt;
                // ---- a render frame ended before this physics step: ModuleRCS.Update reads ctrlState.
                bool frame = s.SkipEvery <= 0 || k % s.SkipEvery != 1;
                if (frame)
                    RcsInFlight.UpdateCommand (0.0, 0.0, ctrl, true, true, true, s.Epsilon, inputRot);
                // ---- Earlyish: the law, from the state the last step left.
                for (int x = 0; x < 3; x++) {
                    inp.Omega [x] = x == a ? omega : 0.0;
                    inp.OmegaDotMeasured [x] = x == a ? (omega - omegaPrevious) / Dt : 0.0;
                    inp.Inertia [x] = s.Inertia;
                    inp.Torque [x] = x == a ? measured : 0.0;
                    inp.TorqueFree [x] = 0.0;
                    inp.Error [x] = 0.0;
                    inp.OmegaFf [x] = 0.0;
                    inp.OmegaDotFf [x] = 0.0;
                }
                inp.OmegaDotValid = k > 0;
                inp.ReferenceValid = true;
                inp.Error [a] = -angle;
                inp.OmegaMax = s.OmegaMax;
                for (int j = 0; j < 3 * n; j++)
                    inp.B [j] = 0.0;
                for (int x = 0; x < 3; x++) {
                    inp.U0 [x] = x == a ? ctrl : 0.0;
                    inp.UMin [x] = x == a ? -1.0 : 0.0;
                    inp.UMax [x] = x == a ? 1.0 : 0.0;
                    inp.UPref [x] = 0.0;
                    inp.Priority [x] = 3.0;
                }
                inp.B [a * n + a] = s.RcsColumn + s.WheelColumn;
                inp.UseTorqueFree = false;
                // What this step's FixedUpdate will fire (inputRot now), kept for the next step's Earlyish.
                for (int x = 0; x < 3; x++)
                    fired [x] = inputRot [x];
                if (s.InFlight && k > 0) {
                    // AttitudeV1RcsInFlight: the served command's next Update minus the command fired at the
                    // step the measured torque was realized from.
                    RcsInFlight.UpdateCommand (0.0, 0.0, ctrl, true, true, true, s.Epsilon, next);
                    RcsInFlight.AddInFlight (bPos, bNeg, next, firedPrevious, scratch, inp.TorqueFree);
                    inp.UseTorqueFree = true;
                }
                AttitudeMath.Step (g, inp, st, work, o);
                // ---- the fly-by-wire: the law's command into ctrlState.
                if (o.Computed)
                    ctrl = Math.Max (-1.0, Math.Min (1.0, ctrl + o.DeltaU [a]));
                // ---- the parts: the RCS fires what its last Update stored; the wheel lerps to this command.
                double rcs = s.RcsEfficiency * s.RcsColumn * fired [a];
                wheel += s.WheelLerp * (s.WheelColumn * ctrl - wheel);
                double dist = s.Trim + s.Swell * Math.Sin (2.0 * Math.PI * 0.2 * t);
                double total = rcs + wheel + dist;
                omegaPrevious = omega;
                omega += Dt * total / s.Inertia;
                angle += Dt * omega;
                measured = rcs + wheel;
                for (int x = 0; x < 3; x++)
                    firedPrevious [x] = fired [x];
                if (double.IsNaN (omega) || double.IsInfinity (omega)) {
                    res.Finite = false;
                    break;
                }
                if (k >= lateFrom) {
                    late++;
                    if (Math.Abs (ctrl) >= 0.99)
                        saturated++;
                    if (Math.Abs (rcs) > 1.0)
                        firing++;
                    rcsAbs.Add (Math.Abs (rcs));
                    int sign = rcs > 1.0 ? 1 : (rcs < -1.0 ? -1 : 0);
                    if (sign != 0 && lastSign != 0 && sign != lastSign)
                        reversals++;
                    if (sign != 0)
                        lastSign = sign;
                    sumTorque += rcs + wheel;
                    sumDist += dist;
                    res.LateRcsImpulse += Dt * Math.Abs (rcs);
                    res.LateDisturbanceImpulse += Dt * Math.Abs (dist);
                    res.LatePeakErrorDeg = Math.Max (res.LatePeakErrorDeg, Math.Abs (angle) * 180.0 / Math.PI);
                    res.LatePeakOmega = Math.Max (res.LatePeakOmega, Math.Abs (omega));
                    res.LatePeakU = Math.Max (res.LatePeakU, Math.Abs (ctrl));
                }
                if (trace)
                    Console.WriteLine ("{0} k={1} frame={2} u={3:F4} fired={4:F4} rcs={5:F0} wheel={6:F0} w={7:G4} e={8:G4} free={9:F0}",
                        s.Name, k, frame ? 1 : 0, ctrl, fired [a], rcs, wheel, omega, angle, inp.TorqueFree [a]);
            }
            rcsAbs.Sort ();
            res.LateSaturated = late > 0 ? (double)saturated / late : 0.0;
            res.LateFiring = late > 0 ? (double)firing / late : 0.0;
            res.LateRcsAbsP50 = rcsAbs.Count > 0 ? rcsAbs [rcsAbs.Count / 2] : 0.0;
            res.LateReversalsPerS = late > 0 ? reversals / (late * Dt) : 0.0;
            res.LatePeriodSteps = reversals > 1 ? 2.0 * late / reversals : 0.0;
            res.LateMeanTorque = late > 0 ? sumTorque / late : 0.0;
            res.LateMeanDisturbance = late > 0 ? sumDist / late : 0.0;
            return res;
        }

        static void Report (Scenario s, Result r)
        {
            Console.WriteLine ("  {0,-34} saturated {1,5:P0}  firing {2,5:P0}  |rcs| p50 {3,7:F0} N.m  reversals {4,5:F1}/s" +
                " (period {5,4:F1} steps)  mean torque {6,7:F0} vs trim {7,7:F0}  rcs impulse / trim's {8,5:F2}  peak err {9,6:F3} deg",
                s.Name, r.LateSaturated, r.LateFiring, r.LateRcsAbsP50, r.LateReversalsPerS, r.LatePeriodSteps,
                r.LateMeanTorque, -r.LateMeanDisturbance,
                r.LateDisturbanceImpulse > 0 ? r.LateRcsImpulse / r.LateDisturbanceImpulse : 0.0, r.LatePeakErrorDeg);
        }

        static void TestClosedLoop ()
        {
            // ---- the flight: no option, one physics step per frame.
            var flown = new Scenario { Name = "flown (no option)" };
            var rf = Simulate (flown);
            Report (flown, rf);
            Check (rf.Finite && rf.LateSaturated > 0.25 && rf.LateRcsAbsP50 > 0.75 * Math.Abs (flown.RcsColumn),
                "no option: the flight's clatter -- served at +-1 a third of the steps or more, the RCS near full power");
            Check (rf.LatePeriodSteps >= 5.0 && rf.LatePeriodSteps <= 7.0,
                "no option: the period of six steps (z^2 - z + 1), " + rf.LatePeriodSteps.ToString ("F2"));
            Check (rf.LateRcsImpulse > 5.0 * rf.LateDisturbanceImpulse,
                "no option: the RCS impulse is many times the trim's (" + (rf.LateRcsImpulse / rf.LateDisturbanceImpulse).ToString ("F1") + ")");
            // The same without the wheel (the flight's battery, drained by the cycle, from 300696).
            var flownDry = new Scenario { Name = "flown, no wheel", WheelColumn = 0.0 };
            var rfd = Simulate (flownDry);
            Report (flownDry, rfd);
            Check (rfd.LateSaturated > 0.25 && rfd.LatePeriodSteps >= 5.0 && rfd.LatePeriodSteps <= 7.0,
                "no option, no wheel: the same cycle (the wheel is not its cause)");

            // ---- option 32.
            Scenario[] fixedCases = {
                new Scenario { Name = "option 32", InFlight = true },
                new Scenario { Name = "option 32, no wheel", InFlight = true, WheelColumn = 0.0 },
                new Scenario { Name = "option 32, 1 frame in 3 skipped", InFlight = true, SkipEvery = 3 },
                new Scenario { Name = "option 32, 1 frame in 2 skipped", InFlight = true, SkipEvery = 2 },
                new Scenario { Name = "option 32, RCS at 90 %", InFlight = true, RcsEfficiency = 0.9 },
                new Scenario { Name = "option 32, RCS at 110 %", InFlight = true, RcsEfficiency = 1.1 },
                new Scenario { Name = "option 32, trim 60 kPa", InFlight = true, Trim = -66.0e3 },
            };
            foreach (var s in fixedCases) {
                var r = Simulate (s);
                Report (s, r);
                Check (r.Finite && r.LateSaturated == 0.0 && r.LatePeakU < 0.2, s.Name + ": never saturated, small commands");
                Check (r.LateReversalsPerS < 0.5, s.Name + ": no clatter (RCS reversals " + r.LateReversalsPerS.ToString ("F2") + "/s)");
                Check (Math.Abs (r.LateMeanTorque + r.LateMeanDisturbance) < 0.03 * Math.Abs (r.LateMeanDisturbance),
                    s.Name + ": the mean torque meets the trim (integral action kept)");
                Check (r.LateRcsImpulse < 1.15 * r.LateDisturbanceImpulse, s.Name + ": the RCS impulse is the trim's, not more (" +
                    (r.LateRcsImpulse / r.LateDisturbanceImpulse).ToString ("F2") + ")");
                Check (r.LatePeakErrorDeg < 0.05, s.Name + ": the error held (" + r.LatePeakErrorDeg.ToString ("F4") + " deg)");
            }
            // ---- a trim under the dead zone (20 kN.m < 0.05 x 697.6): the RCS pulses at its smallest useful
            // command and the mean meets the trim; nothing near full power.
            var small = new Scenario { Name = "option 32, trim under the dead zone", InFlight = true, Trim = -20.0e3, Swell = 2.0e3 };
            var rs = Simulate (small);
            Report (small, rs);
            Check (rs.Finite && rs.LateSaturated == 0.0 && rs.LatePeakU < 0.12, "under the dead zone: no saturation, small commands");
            Check (Math.Abs (rs.LateMeanTorque + rs.LateMeanDisturbance) < 0.05 * Math.Abs (rs.LateMeanDisturbance),
                "under the dead zone: the mean meets the trim");
            Check (rs.LateRcsImpulse < 1.15 * rs.LateDisturbanceImpulse, "under the dead zone: the RCS impulse is the trim's");
            Check (rs.LatePeakErrorDeg < 0.05, "under the dead zone: the error held");
            var smallFlown = new Scenario { Name = "flown, trim under the dead zone", Trim = -20.0e3, Swell = 2.0e3 };
            var rsf = Simulate (smallFlown);
            Report (smallFlown, rsf);
            Check (rsf.LateSaturated > 0.25 && rsf.LatePeriodSteps >= 5.0 && rsf.LatePeriodSteps <= 7.0,
                "no option, trim under the dead zone: the cycle anyway");
            // Without the flight's frame skips (one physics step per frame, always) the bare cycle is marginal:
            // it keeps the amplitude the transient left -- the skips (one step more of delay) push it to the
            // bounds, as in the flight.
            var regular = new Scenario { Name = "flown, one step per frame", SkipEvery = 0 };
            var rr = Simulate (regular);
            Report (regular, rr);
            Check (rr.LatePeriodSteps >= 5.0 && rr.LatePeriodSteps <= 7.0 && rr.LateReversalsPerS > 10.0,
                "no option, one step per frame: the six-step cycle is there (marginal)");
            var regularFixed = new Scenario { Name = "option 32, one step per frame", SkipEvery = 0, InFlight = true };
            var rrf = Simulate (regularFixed);
            Report (regularFixed, rrf);
            Check (rrf.LateSaturated == 0.0 && rrf.LateReversalsPerS < 0.5 && rrf.LateRcsImpulse < 1.15 * rrf.LateDisturbanceImpulse,
                "option 32, one step per frame: no cycle");
        }
    }
}
