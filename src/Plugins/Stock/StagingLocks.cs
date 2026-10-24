using System;
using System.Collections.Generic;

namespace KRPC.Bridge.Stock
{
    /// <summary>
    /// Which input locks hold the staging, and which of them stop a staging asked by a program
    /// (StockService.StageInFrameV2). Pure: System only, so build/tests/stock tests it exactly as
    /// the plugin compiles it; StockService hands it InputLockManager.lockStack.
    ///
    /// WHY. KSP's FlightDriver sets the lock "flightDriver_ApplicationFocus" with EVERY control
    /// type (InputLockManager.SetControlLock(string), mask 0xFFFFFFFFFFFFFFF) whenever the game
    /// window loses the focus, and removes it when the window gets it back
    /// (FlightDriver.OnApplicationFocus, KSP 1.12.5 decompiled). It keeps the keyboard away from a
    /// game the player is not looking at; it says nothing against a staging asked by a program.
    /// StageManager.ActivateNextStage checks no lock (decompiled), and kRPC's
    /// Control.ActivateNextStage refuses only on the stage lock of Alt+L ("manualStageLock").
    /// StageInFrameV1 refused on any lock whose mask includes STAGING, so on the focus lock too:
    /// the client's ascents, launched from a cockpit window beside KSP, were refused on the pad
    /// (20/10/2026 16h01 and 16h02, 16/10 10h49: "Staging is locked by:
    /// flightDriver_ApplicationFocus=0xFFFFFFFFFFFFFFF", 30 refusals in 6.1 s, no lift-off) and
    /// left a pair of spent side boosters attached at 10 km (20/10 16h05).
    /// </summary>
    public static class StagingLocks
    {
        /// <summary>The lock KSP sets while its window does not have the focus.</summary>
        public const string ApplicationFocus = "flightDriver_ApplicationFocus";

        /// <summary>
        /// The locks whose mask includes the staging bits, as "id=0xMASK", in the order of the
        /// lock stack given, without the ids of ignored (null: none).
        /// </summary>
        public static List<string> Holders (IEnumerable<KeyValuePair<string, ulong>> locks, ulong staging,
                                            ICollection<string> ignored)
        {
            var ids = new List<string> ();
            if (locks == null)
                return ids;
            foreach (var entry in locks) {
                if ((entry.Value & staging) == 0)
                    continue;
                if (ignored != null && entry.Key != null && ignored.Contains (entry.Key))
                    continue;
                ids.Add (entry.Key + "=0x" + entry.Value.ToString ("X"));
            }
            return ids;
        }

        /// <summary>
        /// The locks that stop a staging asked by a program: every lock holding the staging except
        /// the focus lock (ApplicationFocus).
        /// </summary>
        public static List<string> ProgramHolders (IEnumerable<KeyValuePair<string, ulong>> locks, ulong staging)
        {
            return Holders (locks, staging, new[] { ApplicationFocus });
        }
    }
}
