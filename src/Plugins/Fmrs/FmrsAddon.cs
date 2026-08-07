using System;
using System.Collections;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using UnityEngine;

namespace KRPC.Bridge.Fmrs
{
    /// <summary>
    /// Registration. Hands Core the resolver and gets out of the way.
    ///
    /// Startup.Instantly, once=true. Core's own addon is also Instantly, and KSP has
    /// already ordered the two assemblies by KSPAssemblyDependency, so this has
    /// registered before Core resolves.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Instantly, true)]
    public sealed class FmrsAddon : MonoBehaviour
    {
        void Awake ()
        {
            ModRegistry.Register ("FMRS", FmrsApi.Resolve);
        }
    }

    /// <summary>
    /// Watches FMRS's dropped-stage table and publishes a bridge event the tick a stage
    /// appears or leaves.
    ///
    /// WHY THIS EXISTS. Separation is an instant, and an instant is the one thing a kRPC
    /// stream cannot report: a stream samples a value once per update and tells you what
    /// something IS. A script that wants to act on separation therefore has to poll
    /// dropped_vessels in a Python loop, which costs a round trip per iteration and
    /// resolves the moment to however long the loop sleeps.
    ///
    /// Diffing the table here - one FindObjectOfType, two reflective field reads and a
    /// pass over a table holding a handful of entries, every five physics ticks - turns
    /// separation into something a script can block on:
    /// conn.bridge.on_event("fmrs.dropped").
    ///
    /// Flight only. FMRS only fills the table in flight, and there is no reason to run
    /// this in the space centre.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Flight, false)]
    public sealed class FmrsWatcher : MonoBehaviour
    {
        /// <summary>Ticks between diffs. A stage stays in the table, so sub-tick precision buys nothing.</summary>
        const int PollTicks = 5;

        readonly HashSet<string> known = new HashSet<string> ();
        int tick;
        bool wasSwitched;
        bool primed;
        static bool complained;

        void FixedUpdate ()
        {
            if (!FmrsApi.Resolved)
                return;
            if (++tick < PollTicks)
                return;
            tick = 0;

            try {
                Diff ();
            } catch (Exception e) {
                // Normally this is the scene tearing down, or FMRS between states, and
                // it must not surface as an error mid-flight. But scripts are told to
                // block on fmrs.dropped instead of polling, so silent PERMANENT failure
                // is the wrong default: log the first one, then go quiet.
                if (!complained) {
                    complained = true;
                    BridgeLog.Warn ("FMRS", "the dropped-stage watcher threw and will stay "
                                    + "quiet from here; fmrs.dropped events may be missing. "
                                    + e.Message);
                }
            }
        }

        void Diff ()
        {
            var live = ModRegistry.FindLiveAssignable (FmrsApi.CoreType);
            if (live == null)
                return;

            var table = ModRegistry.Field (FmrsApi.DroppedField, live) as IDictionary;
            var names = ModRegistry.Field (FmrsApi.DroppedNamesField, live) as IDictionary;
            if (table == null)
                return;

            var present = new HashSet<string> ();
            foreach (DictionaryEntry entry in table) {
                if (!(entry.Key is Guid))
                    continue;
                var guid = (Guid) entry.Key;
                var id = guid.ToString ();
                present.Add (id);
                if (known.Contains (id))
                    continue;
                known.Add (id);

                // The first pass after a scene load re-observes stages that were already
                // tracked before the reload. Those are not new separations, and reporting
                // them would make a script re-handle a booster it has already flown.
                if (!primed)
                    continue;

                var name = names != null && names.Contains (guid) ? names [guid] as string : null;
                var save = entry.Value as string;
                // No tab in the detail: CoreService packs event rows tab-separated and
                // neutralises any tab it finds inside a field, which would silently
                // mangle this one.
                EventBus.Record ("fmrs.dropped", id,
                                 (name ?? "?") + " | save=" + (save ?? string.Empty));
            }

            foreach (var id in new List<string> (known)) {
                if (present.Contains (id))
                    continue;
                known.Remove (id);
                if (primed)
                    EventBus.Record ("fmrs.forgotten", id, string.Empty);
            }

            var switched = false;
            var raw = ModRegistry.Field (FmrsApi.SwitchedField, live);
            if (raw is bool)
                switched = (bool) raw;
            if (primed && switched != wasSwitched)
                EventBus.Record (switched ? "fmrs.on_dropped" : "fmrs.on_main", string.Empty,
                                 switched ? "flying a dropped stage" : "flying the main mission");
            wasSwitched = switched;

            primed = true;
        }
    }
}
