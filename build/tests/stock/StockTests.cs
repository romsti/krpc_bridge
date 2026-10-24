// -----------------------------------------------------------------------------------
// conn.stock.stage_in_frame_v2 / staging_locks_v1: pure tests of the staging locks, and
// of the part module reads (ModuleStatesTests.cs). See StockTests.csproj.
//
// Usage:
//     dotnet run --project build/tests/stock/StockTests.csproj -c Release
//
// The lock stacks are what InputLockManager.lockStack holds in the game: id -> mask. STAGING is
// ControlTypes.STAGING (128, the mask kRPC's Alt+L lock "manualStageLock" sets); the focus lock
// carries every control type, 0xFFFFFFFFFFFFFFF, the mask the ascents of 16/10 and 20/10/2026
// logged in their refusals.
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;

namespace KRPC.Bridge.Stock
{
    public static class Program
    {
        const ulong Staging = 128UL;
        const ulong All = 0xFFFFFFFFFFFFFFFUL;

        static int failures;
        static int checks;

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.WriteLine ("FAIL: " + what);
            }
        }

        static Dictionary<string, ulong> Stack (params object[] idMask)
        {
            var d = new Dictionary<string, ulong> ();
            for (int i = 0; i + 1 < idMask.Length; i += 2)
                d [(string)idMask [i]] = (ulong)idMask [i + 1];
            return d;
        }

        static string Join (List<string> ids)
        {
            return string.Join (", ", ids.ToArray ());
        }

        public static int Main (string[] args)
        {
            TestTheFocusLockAloneDoesNotStopAProgram ();
            TestTheStageLockStillStops ();
            TestALockWithoutStagingIsNotAHolder ();
            TestSeveralLocks ();
            TestEmptyAndNullStacks ();
            TestTheConstantIsKspsLockId ();
            ModuleStatesTests.Run (Check);
            Console.WriteLine ("stock: " + checks + " checks, " + failures + " failures");
            return failures == 0 ? 0 : 1;
        }

        /// <summary>The refusals of 16/10 and 20/10: the focus lock alone. V1 saw it (Holders with
        /// nothing ignored), V2 does not (ProgramHolders).</summary>
        static void TestTheFocusLockAloneDoesNotStopAProgram ()
        {
            var s = Stack ("flightDriver_ApplicationFocus", All);
            Check (Join (StagingLocks.Holders (s, Staging, null))
                   == "flightDriver_ApplicationFocus=0xFFFFFFFFFFFFFFF",
                   "V1's message names the focus lock as the 20/10 refusals did: "
                   + Join (StagingLocks.Holders (s, Staging, null)));
            Check (StagingLocks.ProgramHolders (s, Staging).Count == 0,
                   "the focus lock alone stops no staging asked by a program");
        }

        /// <summary>The player's Alt+L lock, kRPC's own refusal: V2 refuses too, alone or beside
        /// the focus lock.</summary>
        static void TestTheStageLockStillStops ()
        {
            var alone = Stack ("manualStageLock", Staging);
            Check (Join (StagingLocks.ProgramHolders (alone, Staging)) == "manualStageLock=0x80",
                   "the Alt+L lock stops it: " + Join (StagingLocks.ProgramHolders (alone, Staging)));
            var both = Stack ("flightDriver_ApplicationFocus", All, "manualStageLock", Staging);
            Check (Join (StagingLocks.ProgramHolders (both, Staging)) == "manualStageLock=0x80",
                   "beside the focus lock, only the Alt+L lock is named: "
                   + Join (StagingLocks.ProgramHolders (both, Staging)));
            var paused = Stack ("gamePause", All & ~(1UL << 40));
            Check (StagingLocks.ProgramHolders (paused, Staging).Count == 1,
                   "the pause lock (every type but UI and PAUSE) still stops it");
            var hidden = Stack ("lockStage", Staging);
            Check (StagingLocks.ProgramHolders (hidden, Staging).Count == 1,
                   "the hidden stage stack (StageManager \"lockStage\") still stops it");
        }

        static void TestALockWithoutStagingIsNotAHolder ()
        {
            var s = Stack ("someWindow", 1UL | 2UL | 4UL);
            Check (StagingLocks.Holders (s, Staging, null).Count == 0,
                   "a lock without the staging bit holds nothing");
            Check (StagingLocks.ProgramHolders (s, Staging).Count == 0,
                   "nor for a program");
        }

        static void TestSeveralLocks ()
        {
            var s = Stack ("a", Staging, "flightDriver_ApplicationFocus", All, "b", 1UL, "c", Staging | 1UL);
            Check (Join (StagingLocks.Holders (s, Staging, null))
                   == "a=0x80, flightDriver_ApplicationFocus=0xFFFFFFFFFFFFFFF, c=0x81",
                   "every holder, in the stack's order: " + Join (StagingLocks.Holders (s, Staging, null)));
            Check (Join (StagingLocks.ProgramHolders (s, Staging)) == "a=0x80, c=0x81",
                   "all but the focus lock: " + Join (StagingLocks.ProgramHolders (s, Staging)));
        }

        static void TestEmptyAndNullStacks ()
        {
            Check (StagingLocks.Holders (Stack (), Staging, null).Count == 0, "an empty stack: no holder");
            Check (StagingLocks.Holders (null, Staging, null).Count == 0, "a null stack: no holder, no throw");
            Check (StagingLocks.ProgramHolders (null, Staging).Count == 0, "nor for a program");
        }

        /// <summary>The id KSP 1.12.5 uses (FlightDriver.OnApplicationFocus, decompiled: the
        /// string is in Assembly-CSharp's user strings).</summary>
        static void TestTheConstantIsKspsLockId ()
        {
            Check (StagingLocks.ApplicationFocus == "flightDriver_ApplicationFocus",
                   "the focus lock id is KSP's");
        }
    }
}
