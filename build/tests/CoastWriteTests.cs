// -----------------------------------------------------------------------------------
// tests of the pure rules behind option 16 of AttitudeOptionsV1, the coast write bits. What
// they pin down:
//   * option 16 is a new bit of the options (supported 31, the client), its
//     write-mask bits 4 and 8, its motifs 40 and 41 above every existing motif bit, informational and
//     exact in a double;
//   * option 16 changes NO existing rule: blocking sets, MayWrite, MayWritePitchYaw, frozen axes,
//     neutralization, roll gate and the contact hold are those of the same options without it, for any
//     motif mask;
//   * born closed: without the option a write mask above 3 is refused as before, the stored mask is read
//     as its two low bits, the surfaces' owned axes are an earlier change's and the 0.3 s cap of the pitch/yaw
//     reference is M4's, for any mask, age and stored bits;
//   * with the option: bit 4 owns no axis for the surfaces' engagement whatever was written, bit 8 keeps
//     a reference past 0.3 s (any finite age) and never an absent one; the other bits are refused.
// Writes nothing into the golden vectors.
// -----------------------------------------------------------------------------------

using System;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static class CoastWriteTests
    {
        static int checks;
        static int failures;

        const int Armed = AttitudeWrite.ModeArmed;
        const int Shadow = AttitudeWrite.ModeShadow;
        const int Both = AttitudeWrite.ArmRoll | AttitudeWrite.ArmPitchYaw;
        const int Earlyish = AttitudeWrite.StageEarlyish;
        const int Coast = AttitudeWrite.OptionCoastWrite;
        const int Out = AttitudeWrite.WriteSurfacesOut;
        const int Hold = AttitudeWrite.WriteHoldReference;

        public static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestBits ();
            TestOptionChangesNoExistingRule ();
            TestBornClosed ();
            TestWriteMasks ();
            TestSurfacesOut ();
            TestHoldReference ();
            checkCount = checks;
            Console.WriteLine ("coast write checks: " + checks + ", failures: " + failures);
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

        static void TestBits ()
        {
            Check (AttitudeWrite.OptionCoastWrite == 16 && (AttitudeWrite.OptionsSupported & 31) == 31,
                "option 16, supported 31 and above (mirrored by the client; " +
                "32, RcsInFlightTests)");
            Check (Out == 4 && Hold == 8 && AttitudeWrite.WriteMaskBase == 3 && AttitudeWrite.WriteMaskCoast == 15,
                "write-mask bits 4 and 8 (the client's constants)");
            Check ((Out & (AttitudeWrite.ArmRoll | AttitudeWrite.ArmPitchYaw)) == 0 &&
                (Hold & (AttitudeWrite.ArmRoll | AttitudeWrite.ArmPitchYaw)) == 0,
                "the new write bits are not arm bits (ArmedBodyAxes of axes & mask never sees them)");
            for (int axes = 0; axes < 4; axes++)
                for (int w = 0; w < 16; w++)
                    Check (AttitudeWrite.ArmedBodyAxes (axes & w) == AttitudeWrite.ArmedBodyAxes (axes & w & 3),
                        "arm axes of axes " + axes + " & mask " + w + " unchanged by bits 4/8");
            long held = AttitudeWrite.MotifPitchYawReferenceHeld, outM = AttitudeWrite.MotifSurfacesHeldOut;
            Check (held == 1L << 40 && outM == 1L << 41, "motifs 40 and 41");
            long blocking = AttitudeWrite.BlockingMotifs | AttitudeWrite.BlockingMotifsPitchYaw |
                AttitudeWrite.ContactHoldTolerated;
            Check (((held | outM) & blocking) == 0, "motifs 40 and 41 block nothing and are no contact consequence");
            for (int o = 0; o < 32; o++) {
                Check (((held | outM) & AttitudeWrite.RollBlockingSet (o)) == 0 &&
                    ((held | outM) & AttitudeWrite.PitchYawBlockingSet (o)) == 0, "informational under options " + o);
            }
            long known = 0xFFFFFFFFL | AttitudeWrite.MotifCtrlFrozenPitch | AttitudeWrite.MotifCtrlFrozenRoll |
                AttitudeWrite.MotifCtrlFrozenYaw | AttitudeWrite.MotifSurfacesNeutralized |
                AttitudeWrite.MotifRollSurfaceGate | (1L << 37) | (1L << 38) | AttitudeWrite.MotifPitchYawContactHold;
            Check (((held | outM) & known) == 0, "motifs 40 and 41 above every existing bit (0-39)");
            Check ((long)(double)(known | held | outM) == (known | held | outM), "every motif bit exact in a double");
        }

        static void TestOptionChangesNoExistingRule ()
        {
            var rng = new Random (183);
            for (int options = 0; options < 16; options++) {
                Check (AttitudeWrite.RollBlockingSet (options | Coast) == AttitudeWrite.RollBlockingSet (options),
                    "option 16: roll blocking set of options " + options);
                Check (AttitudeWrite.PitchYawBlockingSet (options | Coast) == AttitudeWrite.PitchYawBlockingSet (options),
                    "option 16: pitch/yaw blocking set of options " + options);
            }
            for (int i = 0; i < 20000; i++) {
                long motif = ((long)rng.Next () << 31) ^ rng.Next () ^ ((long)rng.Next (0, 1024) << 32);
                if (i < 64)
                    motif = 1L << i % 42;
                int options = rng.Next (0, 16);
                for (int mode = 0; mode < 2; mode++)
                    for (int axes = 0; axes < 4; axes++)
                        for (int stage = 1; stage < 3; stage++) {
                            Check (AttitudeWrite.MayWrite (mode, axes, stage, true, motif, options | Coast) ==
                                AttitudeWrite.MayWrite (mode, axes, stage, true, motif, options),
                                "option 16: MayWrite identical");
                            Check (AttitudeWrite.MayWritePitchYaw (mode, axes, stage, true, motif, options | Coast) ==
                                AttitudeWrite.MayWritePitchYaw (mode, axes, stage, true, motif, options),
                                "option 16: MayWritePitchYaw identical");
                            Check (AttitudeWrite.ContactHold (options | Coast, mode, axes, stage, true, true, motif) ==
                                AttitudeWrite.ContactHold (options, mode, axes, stage, true, true, motif),
                                "option 16: ContactHold identical");
                        }
                int unneutralizable = rng.Next (0, 8);
                Check (AttitudeWrite.FrozenCtrlAxes (options | Coast, motif, unneutralizable) ==
                    AttitudeWrite.FrozenCtrlAxes (options, motif, unneutralizable), "option 16: frozen axes identical");
                int now = rng.Next (0, 8), last = rng.Next (0, 8), answering = rng.Next (0, 8);
                Check (AttitudeWrite.NeutralizeSurfaces (options | Coast, Armed, Earlyish, now, last, answering) ==
                    AttitudeWrite.NeutralizeSurfaces (options, Armed, Earlyish, now, last, answering),
                    "option 16: neutralization identical");
                Check (AttitudeWrite.RollSurfaceGate (options | Coast, answering, (i & 1) == 0) ==
                    AttitudeWrite.RollSurfaceGate (options, answering, (i & 1) == 0), "option 16: roll gate identical");
            }
        }

        static void TestBornClosed ()
        {
            var rng = new Random (1830);
            for (int options = 0; options < 16; options++) {
                for (int w = -2; w < 64; w++)
                    Check (AttitudeWrite.ValidWriteMask (w, options) == (w >= 0 && w <= 3),
                        "no option 16: write mask " + w + " accepted only in 0-3 (options " + options + ")");
                for (int w = 0; w < 16; w++) {
                    Check (AttitudeWrite.EffectiveWriteMask (w, options) == (w & 3),
                        "no option 16: a stored mask " + w + " reads as its low bits");
                    for (int mode = 0; mode < 2; mode++)
                        for (int axes = 0; axes < 4; axes++)
                            for (int stage = 1; stage < 3; stage++)
                                for (int last = 0; last < 8; last++)
                                    Check (AttitudeWrite.SurfaceOwnedAxes (mode, axes, w, options, stage, last) ==
                                        AttitudeWrite.LoopOwnedAxes (mode, axes, w, stage, last),
                                        "no option 16: the surfaces' owned axes are an earlier change's");
                }
            }
            double[] ages = { 0.0, 0.1, 0.299, 0.3, 0.30001, 0.5, 1.0, 9.0, double.PositiveInfinity, double.NaN };
            for (int options = 0; options < 16; options++)
                for (int w = 0; w < 16; w++)
                    foreach (double age in ages) {
                        Check (AttitudeWrite.PitchYawReferenceOld (age, w, options) ==
                            !(age <= AttitudeWrite.PitchYawMaxReferenceAgeS),
                            "no option 16: the 0.3 s cap of M4 (age " + age + ", mask " + w + ")");
                        Check (!AttitudeWrite.PitchYawReferenceHeld (age, w, options), "no option 16: never held");
                    }
            for (int i = 0; i < 2000; i++) {
                double age = rng.NextDouble () * 3.0;
                int w = rng.Next (0, 16), options = rng.Next (0, 16);
                Check (AttitudeWrite.PitchYawReferenceOld (age, w, options) == (age > AttitudeWrite.PitchYawMaxReferenceAgeS),
                    "no option 16, random age " + age);
            }
        }

        static void TestWriteMasks ()
        {
            for (int options = 0; options < 16; options++) {
                int o = options | Coast;
                for (int w = -2; w < 64; w++)
                    Check (AttitudeWrite.ValidWriteMask (w, o) == (w >= 0 && w <= 15),
                        "option 16: write mask " + w + " accepted only in 0-15");
                for (int w = 0; w < 16; w++)
                    Check (AttitudeWrite.EffectiveWriteMask (w, o) == w, "option 16: the stored mask as written");
            }
        }

        static void TestSurfacesOut ()
        {
            for (int axes = 0; axes < 4; axes++)
                for (int w = 0; w < 16; w++)
                    for (int last = 0; last < 8; last++)
                        for (int stage = 1; stage < 3; stage++)
                            for (int mode = 0; mode < 2; mode++) {
                                int owned = AttitudeWrite.SurfaceOwnedAxes (mode, axes, w, Coast | 1, stage, last);
                                int d180 = AttitudeWrite.LoopOwnedAxes (mode, axes, w, stage, last);
                                Check (owned == ((w & Out) != 0 ? 0 : d180), "option 16: bit 4 owns nothing, else (mode " +
                                    mode + ", axes " + axes + ", mask " + w + ", last " + last + ", stage " + stage + ")");
                            }
            // The coast case: the loop wrote the three axes, armed on both, mask 1|2|4|8: none owned; the
            // descent's mask 1|2: the three.
            Check (AttitudeWrite.SurfaceOwnedAxes (Armed, Both, 15, Coast, Earlyish, 7) == 0, "coast: none owned");
            Check (AttitudeWrite.SurfaceOwnedAxes (Armed, Both, 3, Coast, Earlyish, 7) == 7, "descent: the three axes");
            Check (AttitudeWrite.SurfaceOwnedAxes (Armed, Both, 11, Coast, Earlyish, 7) == 7,
                "coast with the fins (the client's second flag: 1|2|8): the three axes");
            Check (AttitudeWrite.SurfaceOwnedAxes (Shadow, Both, 3, Coast, Earlyish, 7) == 0, "shadow owns nothing");
        }

        static void TestHoldReference ()
        {
            double cap = AttitudeWrite.PitchYawMaxReferenceAgeS;
            double[] fresh = { 0.0, 0.02, 0.1, cap };
            double[] old = { cap + 1e-9, 0.31, 0.5, 0.99, 1.0, 1.9, 10.0 };
            for (int w = 0; w < 16; w++) {
                int o = Coast | 1;
                bool hold = (w & Hold) != 0;
                foreach (double age in fresh) {
                    Check (!AttitudeWrite.PitchYawReferenceOld (age, w, o), "fresh reference never old (" + age + ")");
                    Check (!AttitudeWrite.PitchYawReferenceHeld (age, w, o), "fresh reference not 'held' (" + age + ")");
                }
                foreach (double age in old) {
                    Check (AttitudeWrite.PitchYawReferenceOld (age, w, o) == !hold,
                        "past 0.3 s: old unless bit 8 (mask " + w + ", age " + age + ")");
                    Check (AttitudeWrite.PitchYawReferenceHeld (age, w, o) == hold,
                        "past 0.3 s: held exactly with bit 8 (mask " + w + ", age " + age + ")");
                }
                Check (AttitudeWrite.PitchYawReferenceOld (double.PositiveInfinity, w, o),
                    "no reference is never held (mask " + w + ")");
                Check (AttitudeWrite.PitchYawReferenceOld (double.NaN, w, o), "a NaN age is old (mask " + w + ")");
                Check (!AttitudeWrite.PitchYawReferenceHeld (double.PositiveInfinity, w, o) &&
                    !AttitudeWrite.PitchYawReferenceHeld (double.NaN, w, o), "no reference, NaN: never held");
            }
        }
    }
}
