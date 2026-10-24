// -----------------------------------------------------------------------------------
// tests of the pure rule behind option 8 of AttitudeOptionsV1, the contact hold. What they
// pin down:
//   * option 8 is a new bit of the options (supported 15, the client), its
//     motif 39 above every existing motif bit and exact in a double;
//   * option 8 changes NO existing rule: blocking sets, MayWrite, MayWritePitchYaw, frozen axes,
//     neutralization and roll gate are those of the same options without it, for any motif mask;
//   * ContactHold is false without the option, in shadow, without the pitch/yaw arm, outside
//     Earlyish, without a MEASURED contact, without a lease to continue, without the contact motif;
//   * with all of them, it holds through exactly the motifs of ContactHoldTolerated and ends on
//     any other blocking motif of the channel (the client's write-off and a stale reference first);
//     a motif that does not block pitch/yaw (roll-only, informational) never ends it.
// Writes nothing into the golden vectors.
// -----------------------------------------------------------------------------------

using System;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static class ContactHoldTests
    {
        static int checks;
        static int failures;

        const int Armed = AttitudeWrite.ModeArmed;
        const int Both = AttitudeWrite.ArmRoll | AttitudeWrite.ArmPitchYaw;
        const int Earlyish = AttitudeWrite.StageEarlyish;
        const int Hold = AttitudeWrite.OptionContactHold;

        public static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestBits ();
            TestOptionChangesNoExistingRule ();
            TestConditions ();
            TestToleratedAndEndingMotifs ();
            checkCount = checks;
            Console.WriteLine ("contact hold checks: " + checks + ", failures: " + failures);
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
            Check (AttitudeWrite.OptionContactHold == 8 && (AttitudeWrite.OptionsSupported & 15) == 15,
                "option 8, supported with 1-4 (mirrored by the client; " +
                "16, 32, 64, 128, 256, 512 and above)");
            for (int o = 0; o <= AttitudeWrite.OptionsSupported; o++)
                Check (AttitudeWrite.ValidOptions (o), "valid option " + o);
            for (int o = AttitudeWrite.OptionsSupported + 1; o < 2 * (AttitudeWrite.OptionsSupported + 1); o++)
                Check (!AttitudeWrite.ValidOptions (o), "unknown option bits refused: " + o);
            long m = AttitudeWrite.MotifPitchYawContactHold;
            Check (m == 1L << 39, "motif 39");
            Check ((m & (AttitudeWrite.BlockingMotifs | AttitudeWrite.BlockingMotifsPitchYaw)) == 0,
                "motif 39 blocks nothing (informational)");
            long known = 0xFFFFFFFFL | AttitudeWrite.MotifCtrlFrozenPitch | AttitudeWrite.MotifCtrlFrozenRoll |
                AttitudeWrite.MotifCtrlFrozenYaw | AttitudeWrite.MotifSurfacesNeutralized |
                AttitudeWrite.MotifRollSurfaceGate | (1L << 37) | (1L << 38);
            Check ((m & known) == 0, "motif 39 above every existing bit (0-38: M2-M4, lot 1, SurfaceModelV1)");
            Check ((long)(double)(known | m) == (known | m), "every motif bit exact in a double");
            Check ((AttitudeWrite.ContactHoldTolerated & AttitudeWrite.MotifContact) != 0,
                "the contact itself is tolerated");
            Check ((AttitudeWrite.ContactHoldTolerated & (AttitudeWrite.MotifPitchYawWriteOff |
                AttitudeWrite.MotifPitchYawReferenceOld | AttitudeWrite.MotifLeaseExpired |
                AttitudeWrite.MotifUnloaded | AttitudeWrite.MotifTopologyChanged |
                AttitudeWrite.MotifGimbalAuthority | AttitudeWrite.MotifReferenceStale |
                AttitudeWrite.MotifNoReference | AttitudeWrite.MotifException)) == 0,
                "the client's write-off, a stale or old reference, the lease, the vessel and the authority end it");
        }

        static void TestOptionChangesNoExistingRule ()
        {
            var rng = new Random (181);
            for (int options = 0; options < 8; options++) {
                Check (AttitudeWrite.RollBlockingSet (options | Hold) == AttitudeWrite.RollBlockingSet (options),
                    "option 8: roll blocking set of options " + options);
                Check (AttitudeWrite.PitchYawBlockingSet (options | Hold) == AttitudeWrite.PitchYawBlockingSet (options),
                    "option 8: pitch/yaw blocking set of options " + options);
            }
            for (int i = 0; i < 20000; i++) {
                long motif = ((long)rng.Next () << 31) ^ rng.Next () ^ ((long)rng.Next (0, 256) << 32);
                if (i < 64)
                    motif = 1L << i % 40;
                int options = rng.Next (0, 8);
                for (int mode = 0; mode < 2; mode++)
                    for (int axes = 0; axes < 4; axes++)
                        for (int stage = 1; stage < 3; stage++) {
                            Check (AttitudeWrite.MayWrite (mode, axes, stage, true, motif, options | Hold) ==
                                AttitudeWrite.MayWrite (mode, axes, stage, true, motif, options),
                                "option 8: MayWrite identical");
                            Check (AttitudeWrite.MayWritePitchYaw (mode, axes, stage, true, motif, options | Hold) ==
                                AttitudeWrite.MayWritePitchYaw (mode, axes, stage, true, motif, options),
                                "option 8: MayWritePitchYaw identical");
                        }
                int unneutralizable = rng.Next (0, 8);
                Check (AttitudeWrite.FrozenCtrlAxes (options | Hold, motif, unneutralizable) ==
                    AttitudeWrite.FrozenCtrlAxes (options, motif, unneutralizable), "option 8: frozen axes identical");
                int now = rng.Next (0, 8), last = rng.Next (0, 8), answering = rng.Next (0, 8);
                Check (AttitudeWrite.NeutralizeSurfaces (options | Hold, Armed, Earlyish, now, last, answering) ==
                    AttitudeWrite.NeutralizeSurfaces (options, Armed, Earlyish, now, last, answering),
                    "option 8: neutralization identical");
                Check (AttitudeWrite.RollSurfaceGate (options | Hold, answering, (i & 1) == 0) ==
                    AttitudeWrite.RollSurfaceGate (options, answering, (i & 1) == 0), "option 8: roll gate identical");
            }
        }

        static void TestConditions ()
        {
            long contact = AttitudeWrite.MotifContact;
            Check (AttitudeWrite.ContactHold (Hold, Armed, Both, Earlyish, true, true, contact),
                "the reference case holds");
            Check (AttitudeWrite.ContactHold (Hold | 1 | 4, Armed, AttitudeWrite.ArmPitchYaw, Earlyish, true, true,
                contact), "with the lot 1 options, pitch/yaw armed alone");
            for (int options = 0; options < 8; options++)
                Check (!AttitudeWrite.ContactHold (options, Armed, Both, Earlyish, true, true, contact),
                    "no option 8, no hold: " + options);
            Check (!AttitudeWrite.ContactHold (Hold, AttitudeWrite.ModeShadow, Both, Earlyish, true, true, contact),
                "shadow");
            Check (!AttitudeWrite.ContactHold (Hold, Armed, AttitudeWrite.ArmRoll, Earlyish, true, true, contact),
                "roll armed alone");
            Check (!AttitudeWrite.ContactHold (Hold, Armed, Both, AttitudeWrite.StagePumpFallback, true, true, contact),
                "the pump's fallback stage");
            Check (!AttitudeWrite.ContactHold (Hold, Armed, Both, Earlyish, false, true, contact),
                "the impossible demand alone is no measured contact");
            Check (!AttitudeWrite.ContactHold (Hold, Armed, Both, Earlyish, true, false, contact),
                "no lease to continue (released before, never written)");
            Check (!AttitudeWrite.ContactHold (Hold, Armed, Both, Earlyish, true, true, 0L),
                "no contact motif");
        }

        static void TestToleratedAndEndingMotifs ()
        {
            long contact = AttitudeWrite.MotifContact;
            for (int bit = 0; bit < 40; bit++) {
                long m = 1L << bit;
                bool blocks = (m & AttitudeWrite.BlockingMotifsPitchYaw) != 0;
                bool tolerated = (m & AttitudeWrite.ContactHoldTolerated) != 0;
                bool holds = AttitudeWrite.ContactHold (Hold, Armed, Both, Earlyish, true, true, contact | m);
                Check (holds == (!blocks || tolerated), "motif " + bit + (holds ? " keeps" : " ends") + " the hold");
            }
            // The contact's own consequences, together, keep it.
            long consequences = contact | AttitudeWrite.MotifAxisResidualGuard | AttitudeWrite.MotifAxisOmegaGuard |
                AttitudeWrite.MotifAxisSaturationGuard | AttitudeWrite.MotifPitchYawGuardHold |
                AttitudeWrite.MotifNotConverged | AttitudeWrite.MotifPitchYawFallback;
            Check (AttitudeWrite.ContactHold (Hold, Armed, Both, Earlyish, true, true, consequences),
                "guards, guard hold, not converged at the contact: held");
            Check (!AttitudeWrite.ContactHold (Hold, Armed, Both, Earlyish, true, true,
                consequences | AttitudeWrite.MotifPitchYawWriteOff), "the client's write-off ends it");
            Check (!AttitudeWrite.ContactHold (Hold, Armed, Both, Earlyish, true, true,
                consequences | AttitudeWrite.MotifPitchYawReferenceOld), "a reference older than 0.3 s ends it");
            Check (AttitudeWrite.ContactHold (Hold, Armed, Both, Earlyish, true, true,
                contact | AttitudeWrite.MotifClientWriteOff | AttitudeWrite.MotifGuardHold |
                AttitudeWrite.MotifRollChannelUnmodeled), "roll-only motifs never end it");
        }
    }
}
