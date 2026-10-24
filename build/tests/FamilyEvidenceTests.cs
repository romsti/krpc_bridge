// -----------------------------------------------------------------------------------
// Refactor lot 1: tests of the pure rules behind AttitudeOptionsV1. What they
// pin down:
//   * options 0 reproduce the existing rules EXACTLY (blocking sets, write rules, no frozen
//     axis, no neutralization, no roll gate; the share means and factor of bit 32);
//   * option 1 (per-family blocking): the model motifs 13, 14 and 20 no longer block either
//     channel, every other blocking motif still does, the new bits 32-36 never block; 13 and
//     20 freeze all three ctrlState axes, a surface that cannot be neutralized freezes its
//     own; the surfaces are neutralized only when ctrlState pitch/yaw is written (now or at
//     the last fly-by-wire), armed, at Earlyish; a roll write is gated while a surface
//     answers the roll un-neutralized;
//   * option 4: motif 15 informational;
//   * option 2 (evidence aging): AccumulateShareAged after regular steps is
//     AccumulateShare bit for bit; the aged factor 1 - (1 - f) e^(-A/tau); and the PROPERTY,
//     on a fake game clock, closed loop with the client protocol that made (the client
//     asks for the translation only while the published authority is > 0): with the model
//     above 0, no published authority stays at 0 for more than N = 2 physics steps (it is 0
//     only on a step whose fresh sample says 0; the step after the client stops asking has
//     no sample), and none stays under 5 % of the model for more than 3 tau of game time
//     without a sample. The same loop WITHOUT the option stays at 0 for good: an earlier change.
// Writes nothing into the golden vectors.
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static class FamilyEvidenceTests
    {
        static int checks;
        static int failures;

        const double Dt = 0.02;
        const double Tau = 1.0;            // TranslationParams.ShareTauSeconds
        const double ShareMinDeg = 0.05;   // TranslationParams.ShareMinDegrees

        public static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestOptionValues ();
            TestOptionsZeroIsIdentity ();
            TestFamilyBlockingSets ();
            TestServeNotConverged ();
            TestFrozenAxes ();
            TestNeutralizeAndRollGate ();
            TestAgedAccumulationMatchesLegacy ();
            TestAgeShareAndAgedFactor ();
            TestEvidenceLayout ();
            TestEvidencePropertyCircularClient ();
            TestEvidencePropertyAnyHistory ();
            checkCount = checks;
            Console.WriteLine ("family/evidence checks: " + checks + ", failures: " + failures);
            return failures;
        }

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.Error.WriteLine ("FAIL (lot1): " + what);
            }
        }

        // ------------------------------------------------------------------ options

        static void TestOptionValues ()
        {
            Check (AttitudeWrite.OptionFamilyBlocking == 1 && AttitudeWrite.OptionEvidenceAging == 2 &&
                AttitudeWrite.OptionServeNotConverged == 4 && (AttitudeWrite.OptionsSupported & 511) == 511,
                "option bits 1, 2, 4 (the client; 8, ContactHoldTests; 16, CoastWriteTests; " +
                "32, RcsInFlightTests; 64, BoostbackWriteTests; 128, GimbalTrimTests; 256, " +
                "TranslationTests; 512 and above, MainThrottleTests)");
            for (int o = 0; o <= AttitudeWrite.OptionsSupported; o++)
                Check (AttitudeWrite.ValidOptions (o), "valid option " + o);
            for (int o = AttitudeWrite.OptionsSupported + 1; o < 2 * (AttitudeWrite.OptionsSupported + 1); o++)
                Check (!AttitudeWrite.ValidOptions (o), "unknown option bits refused: " + o);
            Check (!AttitudeWrite.ValidOptions (-1), "negative options refused");
            // The new motif bits are above every existing one and exact in a double.
            long all = AttitudeWrite.MotifCtrlFrozenPitch | AttitudeWrite.MotifCtrlFrozenRoll |
                AttitudeWrite.MotifCtrlFrozenYaw | AttitudeWrite.MotifSurfacesNeutralized |
                AttitudeWrite.MotifRollSurfaceGate;
            Check ((all & 0xFFFFFFFFL) == 0, "new motif bits above bit 31");
            Check ((long)(double)(all | 0xFFFFFFFFL) == (all | 0xFFFFFFFFL), "every motif bit exact in a double");
        }

        static void TestOptionsZeroIsIdentity ()
        {
            Check (AttitudeWrite.RollBlockingSet (0) == AttitudeWrite.BlockingMotifs,
                "options 0: roll blocking set unchanged");
            Check (AttitudeWrite.PitchYawBlockingSet (0) == AttitudeWrite.BlockingMotifsPitchYaw,
                "options 0: pitch/yaw blocking set unchanged");
            Check (AttitudeWrite.RollBlockingSet (AttitudeWrite.OptionEvidenceAging) == AttitudeWrite.BlockingMotifs &&
                AttitudeWrite.PitchYawBlockingSet (AttitudeWrite.OptionEvidenceAging) == AttitudeWrite.BlockingMotifsPitchYaw,
                "evidence aging alone changes no blocking set");
            var rng = new Random (124);
            for (int i = 0; i < 20000; i++) {
                long motif = ((long)rng.Next () << 31) ^ rng.Next () ^ ((long)rng.Next (0, 32) << 32);
                if (i < 64)
                    motif = 1L << i % 37;
                for (int mode = 0; mode < 2; mode++)
                    for (int axes = 0; axes < 4; axes++)
                        for (int stage = 1; stage < 3; stage++) {
                            Check (AttitudeWrite.MayWrite (mode, axes, stage, true, motif, 0) ==
                                AttitudeWrite.MayWrite (mode, axes, stage, true, motif),
                                "options 0: MayWrite identical");
                            Check (AttitudeWrite.MayWritePitchYaw (mode, axes, stage, true, motif, 0) ==
                                AttitudeWrite.MayWritePitchYaw (mode, axes, stage, true, motif),
                                "options 0: MayWritePitchYaw identical");
                        }
                Check (AttitudeWrite.FrozenCtrlAxes (0, motif, rng.Next (0, 8)) == 0,
                    "options 0: nothing frozen");
            }
            Check (!AttitudeWrite.NeutralizeSurfaces (0, 1, 1, 5, 7, 7) &&
                !AttitudeWrite.NeutralizeSurfaces (AttitudeWrite.OptionEvidenceAging | AttitudeWrite.OptionServeNotConverged, 1, 1, 5, 7, 7),
                "without option 1: never a neutralization");
            Check (!AttitudeWrite.RollSurfaceGate (0, 7, false) && !AttitudeWrite.RollSurfaceGate (6, 7, false),
                "without option 1: never a roll gate");
        }

        static void TestFamilyBlockingSets ()
        {
            int f = AttitudeWrite.OptionFamilyBlocking;
            long model = AttitudeWrite.MotifRcsModelApprox | AttitudeWrite.MotifSurfaceApprox |
                AttitudeWrite.MotifFlyByWireOrder;
            Check (AttitudeWrite.FamilyModelMotifs == model, "family model motifs = 13, 14, 20");
            Check ((AttitudeWrite.RollBlockingSet (f) & model) == 0 &&
                (AttitudeWrite.PitchYawBlockingSet (f) & model) == 0,
                "option 1: 13, 14 and 20 block neither channel");
            Check (AttitudeWrite.RollBlockingSet (f) == (AttitudeWrite.BlockingMotifs & ~model) &&
                AttitudeWrite.PitchYawBlockingSet (f) == (AttitudeWrite.BlockingMotifsPitchYaw & ~model),
                "option 1: every other blocking motif still blocks");
            for (int bit = 0; bit < 37; bit++) {
                long m = 1L << bit;
                bool rollBlocks = (AttitudeWrite.BlockingMotifs & m) != 0 && (model & m) == 0;
                bool pyBlocks = (AttitudeWrite.BlockingMotifsPitchYaw & m) != 0 && (model & m) == 0;
                Check (AttitudeWrite.MayWrite (1, 1, 1, true, m, f) == !rollBlocks,
                    "option 1, roll, motif bit " + bit);
                Check (AttitudeWrite.MayWritePitchYaw (1, 2, 1, true, m, f) == !pyBlocks,
                    "option 1, pitch/yaw, motif bit " + bit);
            }
            Check ((AttitudeWrite.RollBlockingSet (7) & AttitudeWrite.MotifNotConverged) == 0 &&
                (AttitudeWrite.RollBlockingSet (1) & AttitudeWrite.MotifNotConverged) != 0,
                "15 stays blocking with option 1 alone");
            // The family still needs everything else: shadow, the fallback stage, a law not computed.
            Check (!AttitudeWrite.MayWritePitchYaw (0, 2, 1, true, 0L, f) &&
                !AttitudeWrite.MayWritePitchYaw (1, 2, 2, true, 0L, f) &&
                !AttitudeWrite.MayWritePitchYaw (1, 2, 1, false, 0L, f) &&
                !AttitudeWrite.MayWrite (1, 2, 1, true, 0L, f),
                "option 1: shadow, fallback stage, law not computed, roll not armed: no write");
        }

        static void TestServeNotConverged ()
        {
            int s = AttitudeWrite.OptionServeNotConverged;
            long m = AttitudeWrite.MotifNotConverged;
            Check (AttitudeWrite.MayWrite (1, 1, 1, true, m, s) && AttitudeWrite.MayWritePitchYaw (1, 2, 1, true, m, s),
                "option 4: a non-converged step writes");
            Check (!AttitudeWrite.MayWrite (1, 1, 1, true, m, 0) && !AttitudeWrite.MayWritePitchYaw (1, 2, 1, true, m, 0),
                "without option 4: a non-converged step falls back, as before");
            Check (AttitudeWrite.RollBlockingSet (s) == (AttitudeWrite.BlockingMotifs & ~m) &&
                AttitudeWrite.PitchYawBlockingSet (s) == (AttitudeWrite.BlockingMotifsPitchYaw & ~m),
                "option 4 removes 15 only");
        }

        static void TestFrozenAxes ()
        {
            int f = AttitudeWrite.OptionFamilyBlocking;
            Check (AttitudeWrite.FrozenCtrlAxes (f, 0L, 0) == 0, "clean step: nothing frozen");
            Check (AttitudeWrite.FrozenCtrlAxes (f, AttitudeWrite.MotifRcsModelApprox, 0) == 7,
                "13: the three ctrlState axes frozen");
            Check (AttitudeWrite.FrozenCtrlAxes (f, AttitudeWrite.MotifFlyByWireOrder, 0) == 7,
                "20: the three ctrlState axes frozen");
            Check (AttitudeWrite.FrozenCtrlAxes (f, AttitudeWrite.MotifSurfaceApprox, 0) == 0,
                "14 alone freezes nothing (the surfaces leave B and are neutralized)");
            Check (AttitudeWrite.FrozenCtrlAxes (f, 0L, 5) == 5 && AttitudeWrite.FrozenCtrlAxes (f, 0L, 2) == 2,
                "an un-neutralizable surface freezes its own axes");
            Check (AttitudeWrite.FrozenMotifs (0) == 0 && AttitudeWrite.FrozenMotifs (1) == AttitudeWrite.MotifCtrlFrozenPitch &&
                AttitudeWrite.FrozenMotifs (2) == AttitudeWrite.MotifCtrlFrozenRoll &&
                AttitudeWrite.FrozenMotifs (4) == AttitudeWrite.MotifCtrlFrozenYaw &&
                AttitudeWrite.FrozenMotifs (7) == (AttitudeWrite.MotifCtrlFrozenPitch |
                    AttitudeWrite.MotifCtrlFrozenRoll | AttitudeWrite.MotifCtrlFrozenYaw),
                "frozen axes to motif bits 32-34");
            // The frozen bits never block: a frozen step still writes its gimbals.
            long frozen = AttitudeWrite.FrozenMotifs (7) | AttitudeWrite.MotifSurfacesNeutralized |
                AttitudeWrite.MotifRollSurfaceGate | AttitudeWrite.MotifRcsModelApprox |
                AttitudeWrite.MotifFlyByWireOrder | AttitudeWrite.MotifSurfaceApprox;
            Check (AttitudeWrite.MayWritePitchYaw (1, 3, 1, true, frozen, f),
                "13 + 14 + 20 + 32-36: the gimbals stay allocatable and written");
        }

        static void TestNeutralizeAndRollGate ()
        {
            int f = AttitudeWrite.OptionFamilyBlocking;
            Check (AttitudeWrite.NeutralizeSurfaces (f, 1, 1, 5, 0, 5) && AttitudeWrite.NeutralizeSurfaces (f, 1, 1, 1, 0, 2),
                "ctrlState pitch/yaw written now: neutralize (whatever axis the surfaces answer)");
            Check (AttitudeWrite.NeutralizeSurfaces (f, 1, 1, 0, 4, 5), "pitch/yaw written at the last fly-by-wire: neutralize");
            Check (AttitudeWrite.NeutralizeSurfaces (f, 1, 1, 0, 2, 7),
                "a roll they answer written at the last fly-by-wire: neutralize (the chain the pitch/yaw started)");
            Check (!AttitudeWrite.NeutralizeSurfaces (f, 1, 1, 0, 2, 5),
                "a roll they do not answer: no neutralization (their pitch/yaw stays the AutoPilot's)");
            Check (!AttitudeWrite.NeutralizeSurfaces (f, 1, 1, 0, 0, 7), "nothing written: the surfaces stay the AutoPilot's");
            Check (!AttitudeWrite.NeutralizeSurfaces (f, 0, 1, 5, 7, 7), "shadow: never");
            Check (!AttitudeWrite.NeutralizeSurfaces (f, 1, 2, 5, 7, 7), "fallback stage: never");
            Check (!AttitudeWrite.NeutralizeSurfaces (f, 1, 1, 5, 7, 0), "no surface answers: nothing to hold");
            Check (AttitudeWrite.RollSurfaceGate (f, 2, false) && AttitudeWrite.RollSurfaceGate (f, 7, false),
                "a surface answers the roll, not neutralized: gate");
            Check (!AttitudeWrite.RollSurfaceGate (f, 2, true) && !AttitudeWrite.RollSurfaceGate (f, 5, false),
                "neutralized, or no surface on the roll: no gate");
            // The roll alone never starts a neutralization: a step that writes no pitch/yaw, after
            // a step that wrote nothing, gates the roll whenever a surface answers it.
            bool n0 = AttitudeWrite.NeutralizeSurfaces (f, 1, 1, 0, 0, 7);
            Check (!n0 && AttitudeWrite.RollSurfaceGate (f, 7, n0), "roll alone: gated, never self-started");
        }

        // ------------------------------------------------------------------ 

        static void TestAgedAccumulationMatchesLegacy ()
        {
            var rng = new Random (125);
            for (int trial = 0; trial < 50; trial++) {
                double numL = 0, denL = 0, numA = 0, denA = 0;
                double ut = 197026.43 + rng.NextDouble () * 1e4, shareUt = double.NaN;
                bool same = true;
                for (int k = 0; k < 400; k++) {
                    var c = new[] { rng.NextDouble () * 4 - 2, rng.NextDouble () * 4 - 2 };
                    double s = rng.NextDouble () * 3 - 1.5;
                    var real = new[] { s * c [0] + rng.NextDouble () * 0.1, s * c [1] };
                    if (rng.Next (0, 10) == 0)
                        c = new double[2];       // a step without a common: no sample in either
                    double elapsed = double.IsNaN (shareUt) ? 0.0 : ut - shareUt;
                    AttitudeTranslation.AccumulateShare (real, c, 2, Dt / Tau, ref numL, ref denL);
                    bool sampled = AttitudeTranslation.AccumulateShareAged (real, c, 2, Dt / Tau, elapsed,
                        Dt, Tau, ref numA, ref denA);
                    if (!sampled) {
                        // A regular step without a sample: the aged means forget one step, the
                        // legacy ones do not; resynchronize to compare the sampled steps only.
                        numA = numL;
                        denA = denL;
                    }
                    same &= numA == numL && denA == denL;
                    shareUt = ut;
                    ut += Dt;
                }
                Check (same, "regular steps: AccumulateShareAged == AccumulateShare bit for bit (trial " + trial + ")");
            }
            double n1 = 0.3, d1 = 0.5;
            var cc = new[] { 1.0, 1.0 };
            var rr = new[] { 0.5, 0.5 };
            AttitudeTranslation.AccumulateShareAged (rr, cc, 2, 0.02, 2.0 + Dt, Dt, Tau, ref n1, ref d1);
            double kGap = Math.Exp (-2.0 / Tau);
            Check (Near (n1, 0.3 * kGap + 0.02 * (1.0 - 0.3 * kGap), 1e-14) &&
                Near (d1, 0.5 * kGap + 0.02 * (2.0 - 0.5 * kGap), 1e-14),
                "a sample after a 2 s gap: the means forget the gap, then the sample");
            double n2 = 0.3, d2 = 0.5;
            Check (!AttitudeTranslation.AccumulateShareAged (rr, new double[2], 2, 0.02, 1.0, Dt, Tau, ref n2, ref d2) &&
                Near (n2, 0.3 * Math.Exp (-1.0), 1e-14) && Near (d2, 0.5 * Math.Exp (-1.0), 1e-14),
                "no sample: the whole elapsed time is forgotten");
        }

        static void TestAgeShareAndAgedFactor ()
        {
            double n = 0.4, d = 0.8;
            AttitudeTranslation.AgeShare (ref n, ref d, 0.0, Tau);
            Check (n == 0.4 && d == 0.8, "age 0: unchanged");
            AttitudeTranslation.AgeShare (ref n, ref d, -1.0, Tau);
            AttitudeTranslation.AgeShare (ref n, ref d, double.NaN, Tau);
            AttitudeTranslation.AgeShare (ref n, ref d, 1.0, 0.0);
            Check (n == 0.4 && d == 0.8, "negative, NaN elapsed or tau 0: unchanged");
            AttitudeTranslation.AgeShare (ref n, ref d, Tau * Math.Log (2.0), Tau);
            Check (Near (n, 0.2, 1e-12) && Near (d, 0.4, 1e-12) && Near (n / d, 0.5, 1e-12),
                "tau ln 2: the evidence halves, the share does not change");
            Check (AttitudeTranslation.AgedFactor (0.3, 0.0, Tau) == 0.3, "age 0: the measured factor itself");
            Check (AttitudeTranslation.AgedFactor (0.0, double.PositiveInfinity, Tau) == 1.0 &&
                AttitudeTranslation.AgedFactor (0.0, double.NaN, Tau) == 1.0, "never sampled: the model (1)");
            Check (AttitudeTranslation.AgedFactor (double.NaN, 1.0, Tau) == 1.0, "measured NaN: 1");
            Check (Near (AttitudeTranslation.AgedFactor (0.0, Tau * Math.Log (2.0), Tau), 0.5, 1e-12),
                "0 measured: half the model after tau ln 2 (0.69 s)");
            Check (AttitudeTranslation.AgedFactor (0.0, 3.0 * Tau, Tau) >= 0.95,
                "0 measured: 95 % of the model after 3 tau (3 s)");
            Check (AttitudeTranslation.AgedFactor (0.0, Dt, Tau) > 0.0,
                "0 measured: above 0 one step later");
            double prev = -1.0;
            bool monotone = true, bounded = true;
            for (int i = 0; i <= 600; i++) {
                double a = i * 0.01;
                for (int m = 0; m <= 10; m++) {
                    double f = AttitudeTranslation.AgedFactor (m * 0.1, a, Tau);
                    bounded &= f >= m * 0.1 - 1e-15 && f <= 1.0;
                }
                double f0 = AttitudeTranslation.AgedFactor (0.2, a, Tau);
                monotone &= f0 >= prev;
                prev = f0;
            }
            Check (monotone, "the aged factor never decreases with the age");
            Check (bounded, "the aged factor stays in [measured, 1]");
            Check (AttitudeTranslation.AgedFactor (1.7, 0.0, Tau) == 1.0 && AttitudeTranslation.AgedFactor (-0.4, 0.0, Tau) == 0.0,
                "the measured factor is clamped to [0, 1]");
        }

        static void TestEvidenceLayout ()
        {
            var names = AttitudeTranslation.EvidenceFields;
            Check (names.Length == 11, "evidence block schema 1: 11 fields");
            var seen = new HashSet<string> ();
            bool unique = true;
            foreach (var n in names)
                unique &= seen.Add (n);
            Check (unique, "evidence field names unique");
            Check (names [0] == "model_authority_ms2" && names [7] == "servable_authority_ms2",
                "model and servable authorities named");
            Check (AttitudeTranslation.ReturnLengthHonest == 17 + 8 + 7,
                "the block starts after the longest legacy layout (17 + 8 + 7)");
            Check (AttitudeTranslation.EvidenceBlockProtocol == 1 && AttitudeTranslation.EvidenceBlockSchema == 1,
                "evidence block protocol 1, schema 1");
        }

        /// <summary>
        /// One run of the protocol on a fake game clock. Each physics step k: the client's
        /// request (decided on the return of step k-1); the DLL's sample of the common committed
        /// at step k-1 (the adversary decides which share the gimbals held); the means; the
        /// published authority = model x factor; the commit of step k if the client asked.
        /// Returns the longest run of steps with the published authority at 0 (model > 0).
        /// </summary>
        static int SimulateProtocol (bool aging, bool circular, int seed, double sMin, double sMax,
            int steps, out bool zeroOnlyOnFreshZeroSample, out bool recovered95, out int firstStuckStep)
        {
            var rng = new Random (seed);
            double num = 0.0, den = 0.0, shareUt = double.NaN, lastSampleUt = double.NaN;
            double denMin = 2 * ShareMinDeg * ShareMinDeg;
            double ut = 1000.0 + rng.NextDouble () * 1e5;
            var commonPrev = new double[2];
            bool committedPrev = false;
            double publishedPrev = 1.0;
            int run = 0, longest = 0;
            zeroOnlyOnFreshZeroSample = true;
            recovered95 = true;
            firstStuckStep = -1;
            for (int k = 0; k < steps; k++) {
                double model = 0.1 + 0.9 * rng.NextDouble ();
                bool ask = circular ? publishedPrev > 0.0 : true;
                bool candidate = committedPrev;
                bool sampled = false;
                double s = sMin + (sMax - sMin) * rng.NextDouble ();
                var real = new[] { s * commonPrev [0], s * commonPrev [1] };
                if (aging) {
                    double elapsed = double.IsNaN (shareUt) ? 0.0 : ut - shareUt;
                    if (candidate)
                        sampled = AttitudeTranslation.AccumulateShareAged (real, commonPrev, 2, Dt / Tau,
                            elapsed, Dt, Tau, ref num, ref den);
                    else
                        AttitudeTranslation.AgeShare (ref num, ref den, elapsed, Tau);
                    shareUt = ut;
                    if (sampled)
                        lastSampleUt = ut;
                } else if (candidate) {
                    sampled = AttitudeTranslation.AccumulateShare (real, commonPrev, 2, Dt / Tau,
                        ref num, ref den);
                }
                double raw = AttitudeTranslation.MeasuredFactor (num, den, denMin);
                double age = double.IsNaN (lastSampleUt) ? double.PositiveInfinity : ut - lastSampleUt;
                double factor = aging ? AttitudeTranslation.AgedFactor (raw, age, Tau) : raw;
                double published = model * factor;
                if (published == 0.0) {
                    run++;
                    if (aging && !(sampled && raw == 0.0))
                        zeroOnlyOnFreshZeroSample = false;
                    if (run == 50 && firstStuckStep < 0)
                        firstStuckStep = k - 49;
                } else {
                    run = 0;
                }
                if (run > longest)
                    longest = run;
                if (aging && age >= 3.0 * Tau && published < 0.95 * model)
                    recovered95 = false;
                // The commit of this step: a common deflection of 0.5-3 deg per column.
                committedPrev = ask;
                if (ask) {
                    commonPrev [0] = (0.5 + 2.5 * rng.NextDouble ()) * (rng.Next (2) == 0 ? -1 : 1);
                    commonPrev [1] = (0.5 + 2.5 * rng.NextDouble ()) * (rng.Next (2) == 0 ? -1 : 1);
                }
                publishedPrev = published;
                ut += Dt;
            }
            return longest;
        }

        static void TestEvidencePropertyCircularClient ()
        {
            // The adversary of the gimbals never hold the common (share <= 0: the attitude
            // law takes them), the client asks only while the published authority is > 0.
            int worstAged = 0, bestLegacy = int.MaxValue;
            bool p1 = true, p3 = true;
            for (int seed = 0; seed < 40; seed++) {
                bool z, r;
                int stuck;
                int aged = SimulateProtocol (true, true, 1000 + seed, -1.2, 0.0, 3000, out z, out r, out stuck);
                worstAged = Math.Max (worstAged, aged);
                p1 &= z;
                p3 &= r;
                int legacy = SimulateProtocol (false, true, 1000 + seed, -1.2, 0.0, 3000, out z, out r, out stuck);
                bestLegacy = Math.Min (bestLegacy, legacy);
            }
            // N = 2 steps (0.04 s of game time): the sample of the last commit, then the step the
            // client stopped asking, which has no sample and publishes >= (1 - e^(-dt/tau)) x model.
            Check (worstAged <= 2, "aged, circular client: never more than 2 steps (0.04 s) at 0 (worst " +
                worstAged + ")");
            Check (p1, "aged: 0 only on a step whose fresh sample says 0");
            Check (p3, "aged: >= 95 % of the model after 3 tau without a sample");
            // Without the option the same loop locks at 0: the defect, reproduced.
            Check (bestLegacy >= 500, "without the option the circular client stays at 0 for good: " +
                bestLegacy + " steps at least");
        }

        static void TestEvidencePropertyAnyHistory ()
        {
            // Any share (the gimbals hold more, less or the opposite of the common), the client
            // asking or not at random, with game-time gaps: the published authority is 0 only on a
            // fresh sample that says 0, and back above 95 % of the model 3 tau after the last sample.
            var rng = new Random (4242);
            double denMin = 2 * ShareMinDeg * ShareMinDeg;
            bool p1 = true, p3 = true, p0 = true;
            for (int trial = 0; trial < 60; trial++) {
                double num = 0.0, den = 0.0, shareUt = double.NaN, lastSampleUt = double.NaN;
                double ut = 5000.0 + rng.NextDouble () * 1e5;
                var commonPrev = new double[2];
                bool committedPrev = false;
                for (int k = 0; k < 2000; k++) {
                    double model = 0.05 + rng.NextDouble ();
                    bool ask = rng.Next (0, 4) != 0 && (k / 300) % 3 != 2;    // long silent stretches
                    bool candidate = committedPrev;
                    double s = rng.NextDouble () * 3.0 - 1.5;
                    var real = new[] { s * commonPrev [0], s * commonPrev [1] };
                    double elapsed = double.IsNaN (shareUt) ? 0.0 : ut - shareUt;
                    bool sampled = candidate && AttitudeTranslation.AccumulateShareAged (real, commonPrev, 2,
                        Dt / Tau, elapsed, Dt, Tau, ref num, ref den);
                    if (!candidate)
                        AttitudeTranslation.AgeShare (ref num, ref den, elapsed, Tau);
                    shareUt = ut;
                    if (sampled)
                        lastSampleUt = ut;
                    double raw = AttitudeTranslation.MeasuredFactor (num, den, denMin);
                    double age = double.IsNaN (lastSampleUt) ? double.PositiveInfinity : ut - lastSampleUt;
                    double published = model * AttitudeTranslation.AgedFactor (raw, age, Tau);
                    if (published == 0.0 && !(sampled && raw == 0.0))
                        p1 = false;
                    if (age >= 3.0 * Tau && published < 0.95 * model)
                        p3 = false;
                    if (!(published >= 0.0) || published > model)
                        p0 = false;
                    committedPrev = ask;
                    if (ask) {
                        commonPrev [0] = rng.NextDouble () * 6.0 - 3.0;
                        commonPrev [1] = rng.NextDouble () * 6.0 - 3.0;
                    }
                    // Game-time gaps (missed attitude ticks): 0.1-4 s, now and then.
                    ut += rng.Next (0, 100) == 0 ? 0.1 + 3.9 * rng.NextDouble () : Dt;
                }
            }
            Check (p1, "any history: 0 only on a fresh sample that says 0");
            Check (p3, "any history: >= 95 % of the model 3 tau after the last sample");
            Check (p0, "any history: 0 <= published <= model");
        }

        static bool Near (double a, double b, double tol)
        {
            return Math.Abs (a - b) <= tol * Math.Max (1.0, Math.Max (Math.Abs (a), Math.Abs (b)));
        }
    }
}
