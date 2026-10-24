// -----------------------------------------------------------------------------------
// ContactV1 (conn.telemetry.contact_v1, refactor lot 3): pure tests of the latch. See
// ContactTests.csproj.
//
// Usage:
//     dotnet run --project build/tests/contact/ContactTests.csproj -c Release
//
// The latch is fed what ContactWatch reads from KSP at each physics step: per wheel its
// WheelState.grounded, whether it is a deployed leg, its part persistentId; per vessel the
// parts in ground contact, the colliding parts, Vessel.Landed, the LANDED situation and the
// kinematics. Ticks are master ticks.
// -----------------------------------------------------------------------------------

using System;

namespace KRPC.Bridge.Telemetry
{
    public static class Program
    {
        static int failures;
        static int checks;

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                if (failures <= 40)
                    Console.WriteLine ("FAIL: " + what);
            }
        }

        public static int Main (string[] args)
        {
            TestTouchdownOnTwoLegs ();
            TestBounceStartsAnEpisode ();
            TestWatchStartedOnThePad ();
            TestBodyContactAndCollision ();
            TestUndeployedLegAndSameTick ();
            TestRemapFollowsTheWheel ();
            TestNoAllocationPerStep ();
            Console.WriteLine ("contact latch: " + checks + " checks, " + failures + " failures");
            return failures == 0 ? 0 : 1;
        }

        /// <summary>A latch with n deployed legs (part persistentIds 100, 101, ...).</summary>
        static ContactLatch Legs (int n)
        {
            var l = new ContactLatch ();
            l.WheelCount = n;
            for (int i = 0; i < n; i++) {
                l.WheelPartPid [i] = (uint)(100 + i);
                l.WheelOrdinal [i] = 0;
                l.IsLeg [i] = true;
                l.Deployed [i] = true;
            }
            return l;
        }

        static void Set (ContactLatch l, params bool[] grounded)
        {
            for (int i = 0; i < grounded.Length; i++) {
                l.Grounded [i] = grounded [i];
                l.CompressionRate [i] = grounded [i] ? 0.5 + i : double.NaN;
            }
        }

        static void Step (ContactLatch l, long tick, int groundParts = 0, int groundWheelParts = 0,
            uint firstGround = 0, int colliding = 0, bool landed = false, bool situation = false)
        {
            l.Step (tick, 1000.0 + 0.02 * tick, groundParts, groundWheelParts, firstGround, colliding, landed, situation,
                -1.5, 0.2, 0.03);
        }

        static void TestTouchdownOnTwoLegs ()
        {
            var l = Legs (2);
            Set (l, false, false);
            Step (l, 50);
            Check (l.Stepped && l.WatchStartTick == 50 && !l.StartedInContact && l.AirborneSinceTick == 50 &&
                l.Transitions == 0, "first step: the start, no transition");
            Step (l, 51);
            Check (l.FirstContactTick == -1 && l.Episodes == 0, "airborne: nothing latched");
            Set (l, true, false);
            Step (l, 100);
            Check ((l.Transitions & ContactLatch.EventFirstContact) != 0 && (l.Transitions & ContactLatch.EventWheel) != 0,
                "leg 0 touches: first contact and a wheel");
            Check (l.FirstContactTick == 100 && l.FirstEpisodeTick == 100 && l.Episodes == 1, "first contact tick");
            Check (l.FirstContactSource == ContactLatch.SourceWheel && l.FirstContactPartPid == 100, "source wheel, part 100");
            Check (l.FirstContactVerticalSpeed == -1.5 && l.FirstContactTilt == 0.03, "kinematics at contact");
            Check (l.FirstGroundedTick [0] == 100 && l.FirstGroundedTick [1] == -1 && l.FirstGroundedRate [0] == 0.5,
                "leg 0 latched, leg 1 not");
            Check (l.NewlyGrounded [0] && !l.NewlyGrounded [1], "newly grounded flags");
            Check (l.AllLegsTick == -1, "not all legs yet");
            // KSP's landing detection sets the leg part's GroundContact the step after.
            Step (l, 101, 0, 1, 100);
            Check (l.Transitions == 0 && l.FirstContactTick == 100, "the leg part's flag adds nothing");
            Set (l, true, true);
            Step (l, 102, 0, 1, 100);
            Check ((l.Transitions & ContactLatch.EventAllLegs) != 0 && l.AllLegsTick == 102 && l.FirstGroundedTick [1] == 102,
                "leg 1: all legs");
            Step (l, 104, 0, 2, 100, 0, true);
            Check ((l.Transitions & ContactLatch.EventLanded) != 0 && l.LandedTick == 104, "Vessel.Landed rises");
            Step (l, 106, 0, 2, 100, 0, true, true);
            Check ((l.Transitions & ContactLatch.EventSituationLanded) != 0 && l.SituationLandedTick == 106,
                "LANDED situation rises");
            Step (l, 107, 0, 2, 100, 0, true, true);
            Check (l.Transitions == 0, "steady: no transition");
            Check (l.WheelTransitions [0] == 1 && l.LastChangeTick [0] == 100 && l.WheelTransitions [1] == 1 &&
                l.LastChangeTick [1] == 102, "wheel edges counted");
        }

        static void TestBounceStartsAnEpisode ()
        {
            var l = Legs (2);
            Set (l, false, false);
            Step (l, 10);
            Set (l, true, true);
            Step (l, 20);
            Check (l.AllLegsTick == 20, "both at once: all legs at the first contact");
            Set (l, false, false);
            Step (l, 30);
            Check ((l.Transitions & ContactLatch.EventContactLost) != 0 && l.LostTick == 30 && l.AirborneSinceTick == 30,
                "bounce: contact lost");
            Check (l.FirstGroundedTick [0] == 20, "a wheel keeps its first ground until the next episode");
            Set (l, false, true);
            Step (l, 35);
            Check (l.Episodes == 2 && l.FirstContactTick == 35 && l.FirstEpisodeTick == 20, "second episode");
            Check (l.FirstContactPartPid == 101, "second episode on leg 1");
            Check (l.FirstGroundedTick [0] == -1 && l.FirstGroundedTick [1] == 35 && l.AllLegsTick == -1,
                "the episode restarts the wheels and all legs");
            Set (l, true, true);
            Step (l, 36);
            Check (l.AllLegsTick == 36 && l.FirstGroundedTick [0] == 36, "leg 0 back: all legs");
        }

        static void TestWatchStartedOnThePad ()
        {
            var l = Legs (4);
            Set (l, true, true, true, true);
            Step (l, 5, 1, 4, 7, 0, true, true);
            Check (l.StartedInContact && l.ContactNow && l.AirborneSinceTick == -1 && l.Transitions == 0,
                "on the pad: started in contact, no transition");
            Step (l, 6, 1, 4, 7, 0, true, true);
            Check (l.Episodes == 0 && l.FirstContactTick == -1 && l.FirstGroundedTick [0] == -1 && l.AllLegsTick == -1,
                "no episode on the pad: nothing latched");
            Check (l.LandedTick == -1 && l.SituationLandedTick == -1, "Landed since the watch started has no rising edge");
            Set (l, false, false, false, false);
            Step (l, 200);
            Check ((l.Transitions & ContactLatch.EventContactLost) != 0 && l.AirborneSinceTick == 200, "liftoff");
            Set (l, false, true, false, false);
            Step (l, 900);
            Check (l.Episodes == 1 && l.FirstContactTick == 900 && l.FirstEpisodeTick == 900, "touchdown after the flight");
        }

        static void TestBodyContactAndCollision ()
        {
            var l = Legs (2);
            Set (l, false, false);
            Step (l, 1);
            Step (l, 2, 0, 0, 0, 3);
            Check ((l.Transitions & ContactLatch.EventCollision) != 0 && l.CollisionTick == 2 && !l.ContactNow &&
                l.Episodes == 0, "a collision alone is not a contact");
            Step (l, 3, 0, 0, 0, 3);
            Check (l.Transitions == 0, "a collision held: no new edge");
            Step (l, 4, 1, 0, 555);
            Check (l.FirstContactTick == 4 && l.FirstContactSource == ContactLatch.SourcePart &&
                l.FirstContactPartPid == 555, "the body touches first: source part, its pid");
            Set (l, true, false);
            Step (l, 5, 1, 1, 555);
            Check (l.FirstGroundedTick [0] == 5 && l.FirstContactTick == 4, "a leg after the body: same episode");
        }

        static void TestUndeployedLegAndSameTick ()
        {
            var l = Legs (3);
            l.Deployed [2] = false;
            Set (l, false, false, false);
            Step (l, 1);
            Set (l, true, true, false);
            Step (l, 2);
            Check (l.AllLegsTick == 2, "a retracted leg does not count for all legs");
            int before = l.Transitions;
            Step (l, 2);
            Check (l.Transitions == 0 && l.Episodes == 1, "a second call for the same tick does nothing");
            Check (before != 0, "the first call of that tick had transitions");
            var none = new ContactLatch ();
            none.WheelCount = 0;
            none.Step (1, 0, 0, 0, 0, 0, false, false, 0, 0, 0);
            none.Step (2, 0, 1, 0, 9, 0, false, false, 0, 0, 0);
            Check (none.FirstContactTick == 2 && none.AllLegsTick == -1, "no legs: contact by a part, never all legs");
        }

        static void TestRemapFollowsTheWheel ()
        {
            var l = Legs (3);
            Set (l, false, false, false);
            Step (l, 1);
            Set (l, false, true, false);
            Step (l, 2);
            Check (l.FirstGroundedTick [1] == 2, "leg 101 latched at index 1");
            // Leg 100 is lost (a decoupled part): the scan finds 101, 102 and a new wheel 300.
            l.SnapshotWheels ();
            l.WheelCount = 3;
            l.WheelPartPid [0] = 101;
            l.WheelPartPid [1] = 102;
            l.WheelPartPid [2] = 300;
            l.RemapFromSnapshot ();
            Check (l.FirstGroundedTick [0] == 2 && l.WheelTransitions [0] == 1, "leg 101 keeps its latch at index 0");
            Check (l.FirstGroundedTick [1] == -1, "leg 102 kept its empty latch");
            Check (l.FirstGroundedTick [2] == -1 && l.WheelTransitions [2] == 0 && !l.WheelSeen [2], "a new wheel starts empty");
            Set (l, true, false, true);
            Step (l, 3);
            Check (l.WheelTransitions [2] == 0, "a new wheel's first state is not an edge");
            Check (l.FirstGroundedTick [2] == 3, "but it latches its first ground in the episode");
        }

        static void TestNoAllocationPerStep ()
        {
            var l = Legs (4);
            Set (l, false, false, false, false);
            for (int t = 0; t < 100; t++)
                Step (l, t);
            long before = GC.GetAllocatedBytesForCurrentThread ();
            for (int t = 100; t < 20100; t++) {
                bool g = (t / 37) % 3 == 0;
                l.Grounded [0] = g;
                l.Grounded [1] = g;
                l.Grounded [2] = !g;
                l.Grounded [3] = g;
                Step (l, t, g ? 1 : 0, 0, 0, (t / 11) % 2, g, g);
            }
            long allocated = GC.GetAllocatedBytesForCurrentThread () - before;
            Check (allocated == 0, "20000 steps allocate nothing (" + allocated + " bytes)");
        }
    }
}
