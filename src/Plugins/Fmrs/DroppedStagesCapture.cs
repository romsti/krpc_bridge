using System;
using System.Collections;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using UnityEngine;

namespace KRPC.Bridge.Fmrs
{
    /// <summary>
    /// DroppedStagesV1, the game half: GameEvents in, FMRS's table read, the ledger fed.
    ///
    /// COST. The hooks are static and live for the session (subscribed once, from
    /// <see cref="FmrsAddon"/>); each one is a handful of field reads on an event that
    /// fires a few times per flight. The only periodic work is
    /// <see cref="DroppedStagesWatch"/>, and it is a disabled MonoBehaviour - Unity does
    /// not call its FixedUpdate at all - except during the few seconds after a
    /// separation, while FMRS has not registered it yet.
    /// </summary>
    internal static class DroppedStagesCapture
    {
        internal static readonly DroppedStageLedger Ledger = new DroppedStageLedger ();

        /// <summary>A save was written while a separation was pending: diff on the next tick.</summary>
        internal static bool DiffRequested;

        static bool subscribed;
        static bool complained;

        internal static void Subscribe ()
        {
            if (subscribed)
                return;
            subscribed = true;

            // Each hook on its own: a GameEvents member that moved must cost that one
            // capture, not the others. The table needs the staging and the separation; the
            // rest only sharpens it.
            // KSP's EventData.Add dereferences the delegate's Target: a delegate to a STATIC
            // method throws there (deployed 03/10: every hook failed, the table never captured).
            // Lambdas carry the compiler's closure singleton as Target.
            bool staging = Hook ("onStageActivate", () => GameEvents.onStageActivate.Add (s => OnStageActivate (s)));
            bool decouple = Hook ("onPartDeCoupleNewVesselComplete",
                                  () => GameEvents.onPartDeCoupleNewVesselComplete.Add ((a, b) => OnDecouple (a, b)));
            Hook ("onVesselsUndocking", () => GameEvents.onVesselsUndocking.Add ((a, b) => OnUndock (a, b)));
            Hook ("onGameStateSaved", () => GameEvents.onGameStateSaved.Add (g => OnGameStateSaved (g)));
            Hook ("onLaunch", () => GameEvents.onLaunch.Add (r => OnLaunch (r)));
            Ledger.Hooked = staging && decouple;
        }

        static bool Hook (string name, Action subscribe)
        {
            try {
                subscribe ();
                return true;
            } catch (Exception e) {
                BridgeLog.Warn ("FMRS", "dropped-stage table could not hook " + name + ": " + e.Message);
                return false;
            }
        }

        // ------------------------------------------------------------------ handlers
        // They run inside KSP's own event dispatch: never let one throw.

        static void OnStageActivate (int stage)
        {
            Guard (() => {
                var active = FlightGlobals.ActiveVessel;
                // Fired by StageManager.ActivateStage BEFORE the parts fire and before the
                // vessel's currentStage moves: it still holds the stage before.
                Ledger.NoteStaging (UniversalTime (), stage, active != null ? active.currentStage : -1);
            });
        }

        static void OnDecouple (Vessel original, Vessel created)
        {
            Guard (() => {
                Baseline ();
                Ledger.NoteSeparation (UniversalTime (), "decouple", Side (original), Side (created));
                Wake ();
            });
        }

        static void OnUndock (Vessel oldVessel, Vessel newVessel)
        {
            Guard (() => {
                Baseline ();
                Ledger.NoteSeparation (UniversalTime (), "undock", Side (oldVessel), Side (newVessel));
                Wake ();
            });
        }

        /// <summary>
        /// FMRS's table before this frame's separations, read once per frame: a staging
        /// that sheds ten parts fires ten separations in one frame, and FMRS registers
        /// nothing before Timer_Stage_Delay has passed.
        /// </summary>
        static void Baseline ()
        {
            var frame = UnityEngine.Time.frameCount;
            if (frame == baselineFrame)
                return;
            baselineFrame = frame;
            Ledger.NoteBaseline (TrackedIdsOrNull ());
        }

        static int baselineFrame = -1;

        static void OnGameStateSaved (Game game)
        {
            // Every save of the game fires this; outside a pending separation it is one
            // integer comparison.
            if (!Ledger.HasPending)
                return;
            Guard (() => {
                if (Ledger.NoteSave (UniversalTime ())) {
                    // Not now: FMRS registers, saves, THEN may swap the main vessel, all in
                    // one Update. The next physics tick sees the settled table.
                    DiffRequested = true;
                    Wake ();
                }
            });
        }

        static void OnLaunch (EventReport report)
        {
            Guard (() => {
                var active = FlightGlobals.ActiveVessel;
                Ledger.NoteLaunch (UniversalTime (), active != null ? active.vesselName : string.Empty,
                                   TrackedIdsOrNull ());
            });
        }

        static void Guard (Action body)
        {
            try {
                body ();
            } catch (Exception e) {
                if (!complained) {
                    complained = true;
                    BridgeLog.Warn ("FMRS", "dropped-stage capture threw and will stay quiet from "
                                    + "here; DroppedStagesV1 may lack event facts. " + e.Message);
                }
            }
        }

        static VesselSide Side (Vessel v)
        {
            if (v == null)
                return new VesselSide ();
            return VesselSide.Of (v.id.ToString (), v.persistentId, v.vesselName);
        }

        static void Wake ()
        {
            var watch = DroppedStagesWatch.Current;
            if (watch != null)
                watch.enabled = true;
        }

        internal static double UniversalTime ()
        {
            try {
                return Planetarium.GetUniversalTime ();
            } catch (Exception) {
                return 0.0;
            }
        }

        // ------------------------------------------------------------------ watch

        internal static bool WatchWanted ()
        {
            return DiffRequested || Ledger.WatchWanted (UniversalTime ());
        }

        /// <summary>One tick of the watch: diff when asked or due, then close expired windows.</summary>
        internal static void Step (bool periodic)
        {
            var ut = UniversalTime ();
            if (DiffRequested || periodic) {
                DiffRequested = false;
                Reconcile (ut);
            }
            Ledger.Expire (ut);
        }

        /// <summary>
        /// Bind the entries FMRS registered since the last call. Publishes one
        /// fmrs.stage_registered bus event per new binding.
        /// </summary>
        internal static void Reconcile (double ut)
        {
            if (!FmrsApi.Resolved)
                return;
            var live = ModRegistry.FindLiveAssignable (FmrsApi.CoreType);
            if (live == null)
                return;
            var index = new Dictionary<Guid, uint> ();
            foreach (var f in Ledger.Observe (ut, ReadEntries (live, false), guid => PersistentIdOf (guid, index)))
                EventBus.Record ("fmrs.stage_registered", f.Guid, DroppedStagesFormat.EventDetail (f));
        }

        // ------------------------------------------------------------------ FMRS reads

        /// <summary>
        /// FMRS's table as entries. With <paramref name="full"/>, also each vessel's FMRS
        /// state, its live persistentId and the UT FMRS recorded for its save.
        /// </summary>
        internal static List<FmrsEntry> ReadEntries (object live, bool full)
        {
            var entries = new List<FmrsEntry> ();
            var table = ModRegistry.Field (FmrsApi.DroppedField, live) as IDictionary;
            if (table == null)
                return entries;
            var names = ModRegistry.Field (FmrsApi.DroppedNamesField, live) as IDictionary;
            Dictionary<Guid, uint> index = full ? new Dictionary<Guid, uint> () : null;
            Dictionary<string, double> saveTimes = full ? new Dictionary<string, double> () : null;
            object saveFileCategory = null;
            if (full && FmrsApi.GetSaveValue != null && FmrsApi.SaveCatType != null) {
                try {
                    saveFileCategory = Enum.Parse (FmrsApi.SaveCatType, "SAVEFILE");
                } catch (Exception) {
                    saveFileCategory = null;
                }
            }

            foreach (DictionaryEntry entry in table) {
                if (!(entry.Key is Guid))
                    continue;
                var guid = (Guid) entry.Key;
                var e = new FmrsEntry {
                    Guid = guid.ToString (),
                    Save = entry.Value as string ?? string.Empty,
                    Name = names != null && names.Contains (guid) ? names [guid] as string ?? string.Empty
                           : string.Empty
                };
                if (full) {
                    e.State = StateOf (live, guid);
                    e.LivePersistentId = PersistentIdOf (e.Guid, index);
                    e.FmrsSaveUt = SaveTimeOf (live, e.Save, saveFileCategory, saveTimes);
                }
                entries.Add (e);
            }
            return entries;
        }

        static string StateOf (object live, Guid guid)
        {
            if (FmrsApi.GetVesselState == null)
                return string.Empty;
            try {
                var raw = FmrsApi.GetVesselState.Invoke (live, new object[] { guid });
                return raw == null ? string.Empty : raw.ToString ();
            } catch (Exception) {
                return string.Empty;
            }
        }

        static double SaveTimeOf (object live, string save, object category, Dictionary<string, double> cache)
        {
            if (category == null || string.IsNullOrEmpty (save))
                return double.NaN;
            double known;
            if (cache.TryGetValue (save, out known))
                return known;
            var ut = double.NaN;
            try {
                var text = FmrsApi.GetSaveValue.Invoke (live, new[] { category, (object) save }) as string;
                double parsed;
                // FMRS answers the literal "False" for a key it does not hold.
                if (!string.IsNullOrEmpty (text) && text != "False"
                    && double.TryParse (text, System.Globalization.NumberStyles.Float,
                                        System.Globalization.CultureInfo.InvariantCulture, out parsed))
                    ut = parsed;
            } catch (Exception) {
                ut = double.NaN;
            }
            cache [save] = ut;
            return ut;
        }

        /// <summary>persistentId of a vessel KSP has right now, else 0. The index is filled once per call site.</summary>
        internal static uint PersistentIdOf (string guid, Dictionary<Guid, uint> index)
        {
            if (index.Count == 0) {
                var all = FlightGlobals.Vessels;
                if (all != null)
                    foreach (var v in all)
                        if (v != null)
                            index [v.id] = v.persistentId;
                if (index.Count == 0)
                    return 0;
            }
            Guid parsed;
            uint pid;
            return Guid.TryParse (guid, out parsed) && index.TryGetValue (parsed, out pid) ? pid : 0u;
        }

        static HashSet<string> TrackedIdsOrNull ()
        {
            try {
                if (!FmrsApi.Resolved)
                    return null;
                var live = ModRegistry.FindLiveAssignable (FmrsApi.CoreType);
                if (live == null)
                    return null;
                var found = new HashSet<string> ();
                foreach (var e in ReadEntries (live, false))
                    found.Add (e.Guid);
                return found;
            } catch (Exception) {
                return null;
            }
        }
    }

    /// <summary>
    /// The bounded watch behind DroppedStagesV1. Disabled - so never called by Unity - except
    /// from a separation until FMRS has had its chance to register it (at most
    /// <see cref="DroppedStageLedger.WatchWindow"/> seconds of game time). While enabled it
    /// diffs FMRS's table the tick after each save write, and every fifth tick as a backstop.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Flight, false)]
    public sealed class DroppedStagesWatch : MonoBehaviour
    {
        const int PollTicks = 5;

        internal static DroppedStagesWatch Current;

        int tick;
        static bool complained;

        void Awake ()
        {
            Current = this;
            enabled = DroppedStagesCapture.WatchWanted ();
        }

        void OnDestroy ()
        {
            if (ReferenceEquals (Current, this))
                Current = null;
        }

        void FixedUpdate ()
        {
            try {
                DroppedStagesCapture.Step (++tick % PollTicks == 0);
                if (!DroppedStagesCapture.WatchWanted ())
                    enabled = false;
            } catch (Exception e) {
                enabled = false;
                if (!complained) {
                    complained = true;
                    BridgeLog.Warn ("FMRS", "the dropped-stage watch threw and stopped; "
                                    + "DroppedStagesV1 may lack event facts. " + e.Message);
                }
            }
        }
    }
}
