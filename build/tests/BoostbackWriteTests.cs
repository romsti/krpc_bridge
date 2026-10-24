// -----------------------------------------------------------------------------------
// tests of the pure rules behind option 64 of AttitudeOptionsV1, the boostback write bit. What
// they pin down:
//   * option 64 is a new bit of the options (supported 127, the client), its
//     write-mask bit 16, its motifs 42 and 43 above every existing motif bit, exact in a double, in no
//     blocking set (they block pitch/yaw through LeadMotifs, not through the sets);
//   * option 64 changes NO existing rule: blocking sets, MayWrite, MayWritePitchYaw, the contact hold,
//     frozen axes, neutralization, the roll gate, the reference age and the surfaces' owned axes are those
//     of the same options without it, for any motif mask and any write mask without bit 16;
//   * born closed: without the option a write mask carrying bit 16 is refused as before and a stored one
//     reads without it; without the bit in effect LeadMotifs is 0 and the latch clears;
//   * GimbalsLead: the law's alpha rule (|B| x the larger bound) on body x AND z, gimbals against the three
//     ctrlState columns, the super hope's columns at full thrust and through the throttle taper (the
//     crossover exact to the column), one axis short, a NaN, no thrust, too few columns;
//   * the latch and the channel's sequence over a boostback (thrust arrives, full thrust, taper, a thrust
//     that comes back, the bit dropped and raised again), as AttitudeV1 composes them.
// Writes nothing into the golden vectors.
// -----------------------------------------------------------------------------------

using System;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static class BoostbackWriteTests
    {
        static int checks;
        static int failures;

        const int Armed = AttitudeWrite.ModeArmed;
        const int Both = AttitudeWrite.ArmRoll | AttitudeWrite.ArmPitchYaw;
        const int Earlyish = AttitudeWrite.StageEarlyish;
        const int Option = AttitudeWrite.OptionBoostbackWrite;
        const int Coast = AttitudeWrite.OptionCoastWrite;
        const int Lead = AttitudeWrite.WriteGimbalsLead;

        public static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestBits ();
            TestOptionChangesNoExistingRule ();
            TestBornClosed ();
            TestWriteMasks ();
            TestGimbalsLead ();
            TestLatch ();
            TestBoostbackSequence ();
            checkCount = checks;
            Console.WriteLine ("boostback write checks: " + checks + ", failures: " + failures);
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
            Check (Option == 64 && (AttitudeWrite.OptionsSupported & 127) == 127,
                "option 64, supported 127 and above (mirrored by the client; " +
                "128, GimbalTrimTests)");
            Check (AttitudeWrite.ValidOptions (64) && AttitudeWrite.ValidOptions (127) &&
                !AttitudeWrite.ValidOptions (AttitudeWrite.OptionsSupported + 1),
                "valid options 0-127 (and 128 and above)");
            for (int o = 0; o < 64; o++)
                Check ((o & Option) == 0, "no older option holds bit 64 (" + o + ")");
            Check (Lead == 16 && (Lead & AttitudeWrite.WriteMaskCoast) == 0,
                "write-mask bit 16 (the client's constants), none of the coast's");
            for (int axes = 0; axes < 4; axes++)
                for (int w = 0; w < 32; w++)
                    Check (AttitudeWrite.ArmedBodyAxes (axes & w) == AttitudeWrite.ArmedBodyAxes (axes & w & 3),
                        "arm axes of axes " + axes + " & mask " + w + " unchanged by bit 16");
            long notLeading = AttitudeWrite.MotifGimbalsNotLeading, lost = AttitudeWrite.MotifGimbalsLeadLost;
            Check (notLeading == 1L << 42 && lost == 1L << 43, "motifs 42 and 43");
            long blocking = AttitudeWrite.BlockingMotifs | AttitudeWrite.BlockingMotifsPitchYaw |
                AttitudeWrite.ContactHoldTolerated;
            Check (((notLeading | lost) & blocking) == 0, "motifs 42 and 43 in no blocking set");
            long older = 0xFFFFFFFFFFL | AttitudeWrite.MotifPitchYawReferenceHeld | AttitudeWrite.MotifSurfacesHeldOut;
            Check (((notLeading | lost) & older) == 0, "motifs 42 and 43 above every existing bit (0-41)");
            Check ((long)(double)(older | notLeading | lost) == (older | notLeading | lost), "exact in a double");
        }

        static void TestOptionChangesNoExistingRule ()
        {
            var rng = new Random (1890);
            for (int options = 0; options < 64; options++) {
                int o = options | Option;
                Check (AttitudeWrite.RollBlockingSet (o) == AttitudeWrite.RollBlockingSet (options),
                    "option 64: roll blocking set of options " + options);
                Check (AttitudeWrite.PitchYawBlockingSet (o) == AttitudeWrite.PitchYawBlockingSet (options),
                    "option 64: pitch/yaw blocking set of options " + options);
                for (int w = -2; w < 16; w++)
                    Check (AttitudeWrite.ValidWriteMask (w, o) == AttitudeWrite.ValidWriteMask (w, options),
                        "option 64: write mask " + w + " without bit 16 (options " + options + ")");
                for (int w = 0; w < 16; w++) {
                    Check (AttitudeWrite.EffectiveWriteMask (w, o) == AttitudeWrite.EffectiveWriteMask (w, options),
                        "option 64: stored mask " + w);
                    foreach (double age in new[] { 0.1, 0.31, 1.0, double.PositiveInfinity, double.NaN }) {
                        Check (AttitudeWrite.PitchYawReferenceOld (age, w, o) ==
                            AttitudeWrite.PitchYawReferenceOld (age, w, options), "option 64: reference age rule");
                        Check (AttitudeWrite.PitchYawReferenceHeld (age, w, o) ==
                            AttitudeWrite.PitchYawReferenceHeld (age, w, options), "option 64: reference held");
                    }
                    for (int last = 0; last < 8; last++)
                        for (int stage = 1; stage < 3; stage++)
                            Check (AttitudeWrite.SurfaceOwnedAxes (Armed, Both, w, o, stage, last) ==
                                AttitudeWrite.SurfaceOwnedAxes (Armed, Both, w, options, stage, last),
                                "option 64: surfaces' owned axes");
                }
            }
            for (int i = 0; i < 20000; i++) {
                long motif = ((long)rng.Next () << 31) ^ rng.Next () ^ ((long)rng.Next (0, 4096) << 32);
                if (i < 64)
                    motif = 1L << i % 44;
                int options = rng.Next (0, 64);
                int o = options | Option;
                int unneutralizable = rng.Next (0, 8);
                Check (AttitudeWrite.FrozenCtrlAxes (o, motif, unneutralizable) ==
                    AttitudeWrite.FrozenCtrlAxes (options, motif, unneutralizable), "option 64: frozen axes identical");
                int now = rng.Next (0, 8), last = rng.Next (0, 8), answering = rng.Next (0, 8);
                Check (AttitudeWrite.NeutralizeSurfaces (o, Armed, Earlyish, now, last, answering) ==
                    AttitudeWrite.NeutralizeSurfaces (options, Armed, Earlyish, now, last, answering),
                    "option 64: neutralization identical");
                Check (AttitudeWrite.RollSurfaceGate (o, answering, (i & 1) == 0) ==
                    AttitudeWrite.RollSurfaceGate (options, answering, (i & 1) == 0), "option 64: roll gate identical");
                for (int mode = 0; mode < 2; mode++)
                    for (int axes = 0; axes < 4; axes++)
                        for (int stage = 1; stage < 3; stage++) {
                            Check (AttitudeWrite.MayWrite (mode, axes, stage, true, motif, o) ==
                                AttitudeWrite.MayWrite (mode, axes, stage, true, motif, options), "option 64: MayWrite identical");
                            Check (AttitudeWrite.MayWritePitchYaw (mode, axes, stage, true, motif, o) ==
                                AttitudeWrite.MayWritePitchYaw (mode, axes, stage, true, motif, options),
                                "option 64: MayWritePitchYaw identical");
                            Check (AttitudeWrite.ContactHold (o, mode, axes, stage, true, true, motif) ==
                                AttitudeWrite.ContactHold (options, mode, axes, stage, true, true, motif),
                                "option 64: ContactHold identical");
                        }
            }
        }

        static void TestBornClosed ()
        {
            for (int options = 0; options < 64; options++) {
                for (int w = 16; w < 32; w++) {
                    Check (!AttitudeWrite.ValidWriteMask (w, options),
                        "no option 64: a write mask with bit 16 refused (" + w + ", options " + options + ")");
                    Check ((AttitudeWrite.EffectiveWriteMask (w, options) & Lead) == 0,
                        "no option 64: a stored mask " + w + " reads without bit 16");
                    Check (AttitudeWrite.EffectiveWriteMask (w, options) == AttitudeWrite.EffectiveWriteMask (w & 15, options),
                        "no option 64: the rest of a stored mask " + w + " as before");
                }
            }
            for (int i = 0; i < 16; i++) {
                bool latched = (i & 1) != 0, wrote = (i & 2) != 0, lead = (i & 4) != 0;
                Check (!AttitudeWrite.LeadLatch (latched, false, wrote, lead), "no bit in effect: the latch clears");
                Check (AttitudeWrite.LeadMotifs (false, lead, latched) == 0, "no bit in effect: no motif");
            }
        }

        static void TestWriteMasks ()
        {
            for (int options = 0; options < 64; options++) {
                int o = options | Option;
                int allowed = ((options & Coast) != 0 ? 15 : 3) | 16;
                for (int w = -2; w < 128; w++)
                    Check (AttitudeWrite.ValidWriteMask (w, o) == (w >= 0 && (w & ~allowed) == 0),
                        "option 64: write mask " + w + " (options " + options + ")");
                for (int w = 0; w < 32; w++)
                    Check (AttitudeWrite.EffectiveWriteMask (w, o) == (w & allowed), "option 64: the stored mask " + w);
            }
            // The boostback's mask, with options 16 and 64: roll, pitch/yaw, surfaces out, hold, gimbals lead.
            Check (AttitudeWrite.ValidWriteMask (31, Coast | Option) && AttitudeWrite.EffectiveWriteMask (31, Coast | Option) == 31,
                "the boostback's mask 1|2|4|8|16");
            Check (!AttitudeWrite.ValidWriteMask (31, Coast) && !AttitudeWrite.ValidWriteMask (19, Coast),
                "without option 64 the boostback's mask is refused");
        }

        // The super hope at the boostback (a recorded flight, attitude_v1 frame of ut 300585.0):
        // 9 gimbals, each two columns of ~1.26e5 N.m per degree, +-10.5 deg; ctrlState pitch/yaw columns -7.93e5
        // N.m (RCS + wheel), roll -1.22e5. The gimbals' columns scale with the thrust.
        static void SuperHope (double thrustFraction, out double[] b, out double[] uMin, out double[] uMax, out int n,
            out int nG)
        {
            nG = 18;
            n = nG + 3;
            b = new double[3 * n];
            uMin = new double[n];
            uMax = new double[n];
            for (int g = 0; g < 9; g++) {
                double az = 2.0 * Math.PI * g / 9.0;
                int jx = 2 * g, jy = 2 * g + 1;
                // a local frame turned by the gimbal's azimuth, the same 1.257e5 N.m/deg in the body plane
                b [0 * n + jx] = thrustFraction * 1.257e5 * Math.Cos (az);
                b [2 * n + jx] = thrustFraction * 1.257e5 * Math.Sin (az);
                b [0 * n + jy] = -thrustFraction * 1.257e5 * Math.Sin (az);
                b [2 * n + jy] = thrustFraction * 1.257e5 * Math.Cos (az);
                b [1 * n + jx] = thrustFraction * 3.0e3;
                b [1 * n + jy] = -thrustFraction * 1.1e4;
                uMin [jx] = uMin [jy] = -10.5;
                uMax [jx] = uMax [jy] = 10.5;
            }
            b [0 * n + nG] = -7.93e5;
            b [1 * n + nG + 1] = -1.22e5;
            b [2 * n + nG + 2] = -7.93e5;
            for (int j = nG; j < n; j++) {
                uMin [j] = -1.0;
                uMax [j] = 1.0;
            }
        }

        static double GimbalAuthority (double[] b, double[] uMin, double[] uMax, int n, int nG, int a)
        {
            double s = 0.0;
            for (int j = 0; j < nG; j++)
                s += Math.Abs (b [a * n + j]) * Math.Max (Math.Abs (uMin [j]), Math.Abs (uMax [j]));
            return s;
        }

        static void TestGimbalsLead ()
        {
            double[] b, lo, hi;
            int n, nG;
            SuperHope (1.0, out b, out lo, out hi, out n, out nG);
            Check (AttitudeWrite.GimbalsLead (b, lo, hi, n, nG), "full thrust: the gimbals lead (~15 MN.m against 0.79)");
            // The crossover: the gimbals' authority on the weaker of x and z equals the channel's.
            double ax = GimbalAuthority (b, lo, hi, n, nG, 0), az = GimbalAuthority (b, lo, hi, n, nG, 2);
            double crossover = 7.93e5 / Math.Min (ax, az);
            Check (crossover > 0.04 && crossover < 0.07, "the super hope's crossover at 4-7 % of full thrust (" + crossover + ")");
            SuperHope (crossover * (1.0 + 1e-9), out b, out lo, out hi, out n, out nG);
            Check (AttitudeWrite.GimbalsLead (b, lo, hi, n, nG), "just above the crossover: lead");
            SuperHope (crossover * (1.0 - 1e-9), out b, out lo, out hi, out n, out nG);
            Check (!AttitudeWrite.GimbalsLead (b, lo, hi, n, nG), "just below the crossover: not lead");
            SuperHope (0.0, out b, out lo, out hi, out n, out nG);
            Check (!AttitudeWrite.GimbalsLead (b, lo, hi, n, nG), "no thrust: the gimbals do not lead");
            // No ctrlState authority at all (frozen columns): the gimbals lead while they have any.
            SuperHope (1e-6, out b, out lo, out hi, out n, out nG);
            for (int a = 0; a < 3; a++)
                for (int j = nG; j < n; j++)
                    b [a * n + j] = 0.0;
            Check (AttitudeWrite.GimbalsLead (b, lo, hi, n, nG), "frozen ctrlState columns: any gimbal authority leads");
            for (int j = 0; j < nG; j++)
                for (int a = 0; a < 3; a++)
                    b [a * n + j] = 0.0;
            Check (!AttitudeWrite.GimbalsLead (b, lo, hi, n, nG), "nothing at all: no lead (zero is not leading)");
            // One axis short: x leads, z does not.
            SuperHope (1.0, out b, out lo, out hi, out n, out nG);
            for (int j = 0; j < nG; j++)
                b [2 * n + j] *= 0.01;
            b [2 * n + nG + 2] = -7.93e5;
            Check (!AttitudeWrite.GimbalsLead (b, lo, hi, n, nG), "yaw short: no lead (both axes needed)");
            // The roll does not count.
            SuperHope (1.0, out b, out lo, out hi, out n, out nG);
            b [1 * n + nG + 1] = -1e12;
            Check (AttitudeWrite.GimbalsLead (b, lo, hi, n, nG), "the roll channel is not compared");
            // A NaN anywhere it reads: no lead.
            SuperHope (1.0, out b, out lo, out hi, out n, out nG);
            b [0 * n + 3] = double.NaN;
            Check (!AttitudeWrite.GimbalsLead (b, lo, hi, n, nG), "a NaN in a gimbal column: no lead");
            SuperHope (1.0, out b, out lo, out hi, out n, out nG);
            b [2 * n + nG + 2] = double.NaN;
            Check (!AttitudeWrite.GimbalsLead (b, lo, hi, n, nG), "a NaN in the channel: no lead");
            SuperHope (1.0, out b, out lo, out hi, out n, out nG);
            hi [5] = double.NaN;
            lo [5] = double.NaN;
            Check (!AttitudeWrite.GimbalsLead (b, lo, hi, n, nG), "a NaN bound: no lead");
            // Too few columns, none of gimbals.
            Check (!AttitudeWrite.GimbalsLead (b, lo, hi, nG + 2, nG), "fewer than the three ctrlState columns: no lead");
            Check (!AttitudeWrite.GimbalsLead (b, lo, hi, n, 0), "no gimbal column: no lead");
            // The span is the larger bound, as the law's alpha: an asymmetric box counts its far side.
            double[] b1 = new double[3 * 5], lo1 = { -1.0, -1.0, -1.0, -1.0, -1.0 }, hi1 = { 1.0, 1.0, 1.0, 1.0, 1.0 };
            b1 [0 * 5 + 0] = 1.0;
            b1 [2 * 5 + 1] = 1.0;
            b1 [0 * 5 + 2] = 1.5;
            b1 [2 * 5 + 4] = 1.5;
            Check (!AttitudeWrite.GimbalsLead (b1, lo1, hi1, 5, 2), "unit spans: 1 against 1.5, no lead");
            lo1 [0] = -2.0;
            hi1 [1] = 2.0;
            Check (AttitudeWrite.GimbalsLead (b1, lo1, hi1, 5, 2), "far side 2: 2 against 1.5, lead");
            // Translation motif columns after the three ctrlState ones are not read.
            double[] b2 = new double[3 * 7], lo2 = new double[7], hi2 = new double[7];
            for (int j = 0; j < 7; j++) {
                lo2 [j] = -1.0;
                hi2 [j] = 1.0;
            }
            b2 [0 * 7 + 0] = 2.0;
            b2 [2 * 7 + 1] = 2.0;
            b2 [0 * 7 + 2] = 1.0;
            b2 [2 * 7 + 4] = 1.0;
            b2 [0 * 7 + 5] = 1e9;
            b2 [2 * 7 + 6] = 1e9;
            Check (AttitudeWrite.GimbalsLead (b2, lo2, hi2, 7, 2), "columns past the ctrlState three are not compared");
        }

        static void TestLatch ()
        {
            Check (!AttitudeWrite.LeadLatch (false, true, false, false), "not leading before any write: no latch");
            Check (!AttitudeWrite.LeadLatch (false, true, false, true), "leading: no latch");
            Check (!AttitudeWrite.LeadLatch (false, true, true, true), "wrote and still leading: no latch");
            Check (AttitudeWrite.LeadLatch (false, true, true, false), "wrote, no longer leading: latched");
            Check (AttitudeWrite.LeadLatch (true, true, false, true), "latched stays latched while the bit stays");
            Check (AttitudeWrite.LeadLatch (true, true, true, true), "latched stays latched, leading again or not");
            Check (!AttitudeWrite.LeadLatch (true, false, true, false), "the bit dropped: unlatched");
            Check (AttitudeWrite.LeadMotifs (true, true, false) == 0, "leading, unlatched: written");
            Check (AttitudeWrite.LeadMotifs (true, false, false) == AttitudeWrite.MotifGimbalsNotLeading, "not leading: 42");
            Check (AttitudeWrite.LeadMotifs (true, true, true) == AttitudeWrite.MotifGimbalsLeadLost, "latched, leading: 43");
            Check (AttitudeWrite.LeadMotifs (true, false, true) ==
                (AttitudeWrite.MotifGimbalsNotLeading | AttitudeWrite.MotifGimbalsLeadLost), "latched, not leading: 42 and 43");
        }

        // The channel's decision over a boostback, composed as AttitudeV1 composes it: the latch first (with
        // "wrote at the previous step"), then MayWritePitchYaw and no lead motif.
        static void TestBoostbackSequence ()
        {
            // thrust fraction per step: 25 steps before the thrust, 400 at full, a taper to 1 %, a thrust that
            // comes back to full for 50 steps, 30 more steps without bit 16, 50 with it again.
            var thrust = new System.Collections.Generic.List<double> ();
            var bit = new System.Collections.Generic.List<bool> ();
            for (int i = 0; i < 25; i++) { thrust.Add (0.0); bit.Add (true); }
            for (int i = 0; i < 400; i++) { thrust.Add (1.0); bit.Add (true); }
            for (int i = 0; i < 600; i++) { thrust.Add (1.0 - 0.99 * i / 599.0); bit.Add (true); }
            for (int i = 0; i < 50; i++) { thrust.Add (1.0); bit.Add (true); }
            for (int i = 0; i < 30; i++) { thrust.Add (1.0); bit.Add (false); }
            for (int i = 0; i < 50; i++) { thrust.Add (1.0); bit.Add (true); }
            bool latched = false, wroteLast = false;
            int firstWrite = -1, lastWrite = -1, writesAfterLatch = 0, writesWithoutBit = 0, writesAfterRaise = 0;
            double thrustAtLoss = double.NaN;
            for (int k = 0; k < thrust.Count; k++) {
                double[] b, lo, hi;
                int n, nG;
                SuperHope (thrust [k], out b, out lo, out hi, out n, out nG);
                int mask = 3 | 4 | 8 | (bit [k] ? Lead : 0);
                bool inEffect = (AttitudeWrite.EffectiveWriteMask (mask, Coast | Option) & Lead) != 0;
                long lm = 0;
                if (inEffect || latched) {
                    bool lead = inEffect && AttitudeWrite.GimbalsLead (b, lo, hi, n, nG);
                    bool was = latched;
                    latched = AttitudeWrite.LeadLatch (latched, inEffect, wroteLast, lead);
                    if (latched && !was)
                        thrustAtLoss = thrust [k];
                    lm = AttitudeWrite.LeadMotifs (inEffect, lead, latched);
                }
                bool write = AttitudeWrite.MayWritePitchYaw (Armed, Both, Earlyish, true, 0, Coast | Option) && lm == 0;
                if (write) {
                    if (firstWrite < 0)
                        firstWrite = k;
                    if (k < 1025)
                        lastWrite = k;
                    if (k >= 1025 && k < 1075)
                        writesAfterLatch++;
                    if (k >= 1075 && k < 1105)
                        writesWithoutBit++;
                    if (k >= 1105)
                        writesAfterRaise++;
                }
                wroteLast = write;
            }
            Check (firstWrite == 25, "the channel waits for the thrust (first write at the first thrust step: " + firstWrite + ")");
            Check (lastWrite > 425 && lastWrite < 1024 && thrustAtLoss > 0.04 && thrustAtLoss < 0.07,
                "the taper: the channel stops at the crossover (" + lastWrite + ", thrust " + thrustAtLoss + ")");
            Check (writesAfterLatch == 0, "latched: a thrust that comes back under bit 16 is not written (" + writesAfterLatch + ")");
            Check (writesWithoutBit == 30, "the bit dropped: the channel writes as before (" + writesWithoutBit + ")");
            Check (writesAfterRaise == 50, "the bit raised again on a leading step: written (" + writesAfterRaise + ")");
        }
    }
}
