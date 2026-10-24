// -----------------------------------------------------------------------------------
// tests of the pure rules of AttitudeWrite.cs for the
// NON-ACTIVE vessel of a double flight (AttitudeArmV2 options). What they pin down:
//   * option 0 (AttitudeArmV1) reproduces the V1 rules EXACTLY, on the active and the
//     non-active vessel alike: active-vessel authority path, lateral force on the active
//     vessel only, owner check at every pitch/yaw arming;
//   * option 1 changes only the NON-ACTIVE vessel: own authority, owner check skipped when
//     it is loaded and not active at arming; the active vessel keeps its path;
//   * option 3 adds the lateral force on the non-active vessel; 2 alone is refused;
//   * the options need the armed pitch/yaw channel.
// -----------------------------------------------------------------------------------

using System;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static class InactiveVesselTests
    {
        static int checks;
        static int failures;

        public static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestOptionValues ();
            TestOptionsNeedPitchYaw ();
            TestAuthorityPath ();
            TestTranslationAllowed ();
            TestArmOwnerCheck ();
            TestMotifUnchanged ();
            checkCount = checks;
            Console.WriteLine ("inactive-vessel checks: " + checks + ", failures: " + failures);
            return failures;
        }

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.Error.WriteLine ("FAIL (inactive): " + what);
            }
        }

        static void TestOptionValues ()
        {
            Check (AttitudeWrite.InactivePitchYaw == 1 && AttitudeWrite.InactiveTranslation == 2,
                "option bits: 1 pitch/yaw, 2 lateral force (the client)");
            Check (AttitudeWrite.ValidInactiveOptions (0), "0 valid (V1 arming)");
            Check (AttitudeWrite.ValidInactiveOptions (1), "1 valid (pitch/yaw)");
            Check (AttitudeWrite.ValidInactiveOptions (3), "3 valid (pitch/yaw + lateral force)");
            Check (!AttitudeWrite.ValidInactiveOptions (2), "2 refused (lateral force alone)");
            for (int o = 4; o < 64; o++)
                Check (!AttitudeWrite.ValidInactiveOptions (o), "unknown bits refused: " + o);
            Check (!AttitudeWrite.ValidInactiveOptions (-1), "negative refused");
        }

        static void TestOptionsNeedPitchYaw ()
        {
            for (int mode = 0; mode <= 1; mode++)
                for (int axes = 0; axes <= 3; axes++)
                    Check (AttitudeWrite.InactiveOptionsFit (0, mode, axes),
                        "option 0 fits every arming (mode " + mode + ", axes " + axes + ")");
            foreach (int o in new[] { 1, 3 }) {
                Check (AttitudeWrite.InactiveOptionsFit (o, AttitudeWrite.ModeArmed, 2),
                    "option " + o + " with mode 1 axes 2");
                Check (AttitudeWrite.InactiveOptionsFit (o, AttitudeWrite.ModeArmed, 3),
                    "option " + o + " with mode 1 axes 3");
                Check (!AttitudeWrite.InactiveOptionsFit (o, AttitudeWrite.ModeArmed, 1),
                    "option " + o + " refused on the roll alone");
                Check (!AttitudeWrite.InactiveOptionsFit (o, AttitudeWrite.ModeShadow, 3),
                    "option " + o + " refused in shadow");
            }
        }

        static void TestAuthorityPath ()
        {
            // Option 0: never the per-vessel authority (the active path refuses a
            // non-active vessel, as in V1: motif 28 on the other booster).
            Check (!AttitudeWrite.UsesInactiveAuthority (true, 0), "option 0, active: active path");
            Check (!AttitudeWrite.UsesInactiveAuthority (false, 0),
                "option 0, non-active: active path (refused, as before)");
            foreach (int o in new[] { 1, 3 }) {
                Check (!AttitudeWrite.UsesInactiveAuthority (true, o),
                    "option " + o + ", active vessel: its unchanged path");
                Check (AttitudeWrite.UsesInactiveAuthority (false, o),
                    "option " + o + ", non-active vessel: its own authority");
            }
        }

        static void TestTranslationAllowed ()
        {
            foreach (int o in new[] { 0, 1, 3 })
                Check (AttitudeWrite.TranslationAllowed (true, o),
                    "lateral force always allowed on the active vessel (option " + o + ")");
            Check (!AttitudeWrite.TranslationAllowed (false, 0),
                "option 0: not on the non-active vessel (reason 15, as before)");
            Check (!AttitudeWrite.TranslationAllowed (false, 1),
                "option 1: pitch/yaw only, no lateral force on the non-active vessel");
            Check (AttitudeWrite.TranslationAllowed (false, 3),
                "option 3: lateral force on the non-active vessel");
        }

        static void TestArmOwnerCheck ()
        {
            int m1 = AttitudeWrite.ModeArmed, m0 = AttitudeWrite.ModeShadow;
            // V1 rule: the check applies to every armed pitch/yaw arming, and to nothing else.
            for (int mode = 0; mode <= 1; mode++)
                for (int axes = 0; axes <= 3; axes++)
                    foreach (bool inactive in new[] { false, true }) {
                        bool v1 = mode == m1 && (axes & AttitudeWrite.ArmPitchYaw) != 0;
                        Check (AttitudeWrite.ArmNeedsOwnerCheck (mode, axes, 0, inactive) == v1,
                            "option 0 = the V1 rule (mode " + mode + ", axes " + axes +
                            ", inactive " + inactive + ")");
                    }
            foreach (int o in new[] { 1, 3 }) {
                Check (AttitudeWrite.ArmNeedsOwnerCheck (m1, 3, o, false),
                    "option " + o + ", active (or unloaded) vessel: checked as before");
                Check (!AttitudeWrite.ArmNeedsOwnerCheck (m1, 3, o, true),
                    "option " + o + ", loaded non-active vessel: not checked");
                Check (!AttitudeWrite.ArmNeedsOwnerCheck (m1, 1, o, false),
                    "roll alone: never checked");
                Check (!AttitudeWrite.ArmNeedsOwnerCheck (m0, 3, o, false),
                    "shadow: never checked");
            }
        }

        static void TestMotifUnchanged ()
        {
            // The per-vessel authority is still an authority: its refusal is motif 28, which
            // keeps blocking the pitch/yaw write and only it.
            Check ((AttitudeWrite.BlockingMotifsPitchYaw & AttitudeWrite.MotifGimbalAuthority) != 0,
                "motif 28 blocks pitch/yaw");
            Check ((AttitudeWrite.BlockingMotifs & AttitudeWrite.MotifGimbalAuthority) == 0,
                "motif 28 does not block the roll (the roll wrote on the non-active booster)");
        }
    }
}
