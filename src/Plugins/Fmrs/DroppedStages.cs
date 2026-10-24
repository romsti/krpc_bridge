using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace KRPC.Bridge.Fmrs
{
    // ---------------------------------------------------------------------------------
    // DroppedStagesV1: the pure half. System only - no Unity, no KSP, no kRPC - so the
    // table, the pairing rules and the wire format are tested outside the game, compiled
    // from this very file (build/tests/fmrs/FmrsTests.csproj).
    //
    // WHAT THE TABLE IS FOR. FMRS keeps four loose facts per dropped vessel: its Guid, the
    // save it belongs to, a name and a state. It does not keep WHICH staging dropped it,
    // WHO it came off, or WHEN. A campaign that has to tie "the core booster, stage 4" to
    // an FMRS entry used to infer it on the client side (the number at the end of the save
    // name, the current stage when the save appeared, the pilot's staging log, the order
    // of separations). The bridge sees the separation itself, so it records those facts
    // at the moment they happen and joins them to FMRS's own table when asked.
    //
    // HOW FMRS REGISTERS A STAGE (FMRSContinued 1.2.9.6, decompiled, read for
    // understanding only):
    //   * onStageSeparation / onVesselCreate arm a timer;
    //   * Timer_Stage_Delay (0.2 s of game time; +1 s for a separation without staging)
    //     later, in its Update, it names the save "FMRS_save_" + ActiveVessel.currentStage
    //     (the stage AFTER the staging), or "FMRS_save_separated_N";
    //   * it adds every new commandable vessel to Vessels_dropped (Guid -> save) and
    //     Vessels_dropped_names, writes the save (GamePersistence.SaveGame, which fires
    //     GameEvents.onGameStateSaved), and, when the active vessel is not the recorded
    //     main vessel any more, swaps them (the old main vessel becomes a dropped entry,
    //     added AFTER the save event);
    //   * then it records the save's UT (save_cat.SAVEFILE).
    //
    // So the bridge records, as they fire:
    //   * each staging: GameEvents.onStageActivate(stage), stage = the activated stage,
    //     with the active vessel's currentStage still at the stage before;
    //   * each separation: onPartDeCoupleNewVesselComplete(original, created) and
    //     onVesselsUndocking(old, new), both sides with Guid, persistentId and name;
    //   * each save write while a separation is pending (onGameStateSaved);
    // and, while a separation is pending only, diffs FMRS's table to bind each new entry
    // to the separation that produced it. Nothing runs per tick when nothing happens.
    // ---------------------------------------------------------------------------------

    /// <summary>One vessel of a separation, as it was at the event.</summary>
    internal sealed class VesselSide
    {
        internal string Guid = string.Empty;
        internal uint PersistentId;
        internal string Name = string.Empty;

        internal bool Known { get { return !string.IsNullOrEmpty (Guid); } }

        internal static VesselSide Of (string guid, uint persistentId, string name)
        {
            return new VesselSide {
                Guid = guid ?? string.Empty, PersistentId = persistentId, Name = name ?? string.Empty
            };
        }
    }

    /// <summary>A staging, as GameEvents.onStageActivate reported it.</summary>
    internal sealed class StagingEvent
    {
        internal double Ut;
        /// <summary>The activated stage: the number FMRS puts in its save name.</summary>
        internal int StageAfter;
        /// <summary>The active vessel's currentStage when the event fired, or -1.</summary>
        internal int StageBefore;
    }

    /// <summary>A separation waiting for FMRS to register one of its two vessels.</summary>
    internal sealed class SeparationRecord
    {
        internal double Ut;
        internal string Kind = "decouple";
        internal VesselSide Original;
        internal VesselSide Created;
        internal int Stage = -1;
        internal int StageBefore = -1;
        internal string Craft = string.Empty;
    }

    /// <summary>What the bridge saw of one FMRS-registered vessel. Kept by Guid.</summary>
    internal sealed class DroppedStageFacts
    {
        internal string Guid = string.Empty;
        internal uint PersistentId;
        internal string Save = string.Empty;
        internal string Name = string.Empty;
        internal int Stage = -1;
        internal int StageBefore = -1;
        internal double SeparationUt = double.NaN;
        internal double SaveUt = double.NaN;
        internal uint ParentPersistentId;
        internal string ParentGuid = string.Empty;
        internal string ParentName = string.Empty;
        internal string Craft = string.Empty;
        internal string Kind = string.Empty;
    }

    /// <summary>One entry of FMRS's own table, read at query time.</summary>
    internal sealed class FmrsEntry
    {
        internal string Guid = string.Empty;
        internal string Save = string.Empty;
        internal string Name = string.Empty;
        internal string State = string.Empty;
        /// <summary>persistentId of the vessel if KSP has it right now, else 0.</summary>
        internal uint LivePersistentId;
        /// <summary>UT FMRS recorded for the save (SAVEFILE), or NaN.</summary>
        internal double FmrsSaveUt = double.NaN;
    }

    /// <summary>One row of DroppedStagesV1, before formatting.</summary>
    internal sealed class DroppedStageRow
    {
        internal string Guid = string.Empty;
        internal uint PersistentId;
        internal string Save = string.Empty;
        internal int Stage = -1;
        internal int StageBefore = -1;
        internal string StageSource = string.Empty;
        internal double SeparationUt = double.NaN;
        internal double SaveUt = double.NaN;
        internal uint ParentPersistentId;
        internal string ParentGuid = string.Empty;
        internal string ParentName = string.Empty;
        internal string Name = string.Empty;
        internal string Craft = string.Empty;
        internal string State = string.Empty;
        internal string Capture = string.Empty;
        internal string Kind = string.Empty;
    }

    /// <summary>
    /// The separations, stagings and registrations of the session. Main thread only:
    /// GameEvents, FixedUpdate and kRPC procedure bodies all run there, so there is no
    /// lock.
    /// </summary>
    internal sealed class DroppedStageLedger
    {
        /// <summary>A separation this close after a staging (game seconds) belongs to it.</summary>
        internal const double StagingWindow = 0.5;

        /// <summary>
        /// Game seconds FMRS gets to register a separation. It does so 0.2 s after a
        /// staging and 1.2 s after a separation without one; ten is a wide margin and still
        /// a bounded watch.
        /// </summary>
        internal const double WatchWindow = 10.0;

        /// <summary>Slack for "the same instant" across two events of one frame.</summary>
        internal const double Epsilon = 1e-3;

        internal const int MaxStagings = 32;
        internal const int MaxPending = 64;
        internal const int MaxFacts = 256;

        readonly List<StagingEvent> stagings = new List<StagingEvent> ();
        readonly List<SeparationRecord> pending = new List<SeparationRecord> ();
        readonly Dictionary<string, DroppedStageFacts> facts = new Dictionary<string, DroppedStageFacts> ();
        readonly List<string> factOrder = new List<string> ();
        readonly HashSet<string> seenUnmatched = new HashSet<string> ();
        double lastSaveUt = double.NaN;

        /// <summary>True once the GameEvents hooks are in. Published in the header.</summary>
        internal bool Hooked { get; set; }

        /// <summary>Name of the vessel at the last launch, or empty.</summary>
        internal string Craft { get; private set; } = string.Empty;

        /// <summary>UT of the last launch the bridge saw, or NaN.</summary>
        internal double LaunchUt { get; private set; } = double.NaN;

        /// <summary>Stagings seen since the last launch.</summary>
        internal int StagingsSinceLaunch { get; private set; }

        internal int PendingCount { get { return pending.Count; } }
        internal bool HasPending { get { return pending.Count > 0; } }
        internal int FactCount { get { return facts.Count; } }

        // ------------------------------------------------------------------ events

        /// <summary>
        /// A new mission: forget what it cannot use. Facts are kept only for the vessels
        /// FMRS still tracks (<paramref name="stillTracked"/>; null keeps them all).
        ///
        /// GameEvents.onLaunch fires from StageManager.ActivateStage AFTER the parts of the
        /// lift-off staging have fired: that staging and its separations, of this very
        /// instant, belong to the new mission and are kept.
        /// </summary>
        internal void NoteLaunch (double ut, string craft, ICollection<string> stillTracked)
        {
            Craft = craft ?? string.Empty;
            LaunchUt = ut;
            stagings.RemoveAll (s => Math.Abs (s.Ut - ut) > Epsilon);
            pending.RemoveAll (r => Math.Abs (r.Ut - ut) > Epsilon);
            foreach (var r in pending)
                r.Craft = Craft;
            StagingsSinceLaunch = stagings.Count;
            lastSaveUt = double.NaN;
            // The guard set only ever holds Guids FMRS had before a separation; it is a
            // filter, never an answer, so it is bounded rather than reset.
            if (seenUnmatched.Count > 4 * MaxFacts)
                seenUnmatched.Clear ();
            if (stillTracked == null)
                return;
            foreach (var guid in new List<string> (factOrder)) {
                if (!stillTracked.Contains (guid)) {
                    facts.Remove (guid);
                    factOrder.Remove (guid);
                }
            }
        }

        /// <summary>
        /// FMRS's table as it stands at a separation. Those vessels were registered
        /// before it, so none of them may be bound to it - in particular not the main
        /// vessel, the original side of every separation, if a previous flight left it in
        /// FMRS's table. Call it before <see cref="NoteSeparation"/>.
        /// </summary>
        internal void NoteBaseline (IEnumerable<string> trackedNow)
        {
            if (trackedNow == null)
                return;
            foreach (var guid in trackedNow)
                if (!string.IsNullOrEmpty (guid) && !facts.ContainsKey (guid))
                    seenUnmatched.Add (guid);
        }

        internal void NoteStaging (double ut, int stageAfter, int stageBefore)
        {
            stagings.Add (new StagingEvent { Ut = ut, StageAfter = stageAfter, StageBefore = stageBefore });
            if (stagings.Count > MaxStagings)
                stagings.RemoveAt (0);
            StagingsSinceLaunch++;
        }

        /// <summary>
        /// A separation. It belongs to the latest staging at most
        /// <see cref="StagingWindow"/> before it (the same frame, in stock KSP: decouplers
        /// fire inside StageManager.ActivateStage, right after onStageActivate).
        /// </summary>
        internal SeparationRecord NoteSeparation (double ut, string kind, VesselSide original, VesselSide created)
        {
            var record = new SeparationRecord {
                Ut = ut, Kind = string.IsNullOrEmpty (kind) ? "decouple" : kind,
                Original = original ?? new VesselSide (), Created = created ?? new VesselSide (),
                Craft = Craft
            };
            var staging = StagingFor (ut);
            if (staging != null) {
                record.Stage = staging.StageAfter;
                record.StageBefore = staging.StageBefore;
            }
            pending.Add (record);
            if (pending.Count > MaxPending)
                pending.RemoveAt (0);
            return record;
        }

        /// <summary>
        /// A save was written. Only remembered while a separation is pending: that is the
        /// FMRS save of the separation, written in the frame FMRS registered it.
        /// Returns whether a reconciliation is worth running.
        /// </summary>
        internal bool NoteSave (double ut)
        {
            if (pending.Count == 0)
                return false;
            lastSaveUt = ut;
            return true;
        }

        internal StagingEvent StagingFor (double ut)
        {
            for (int i = stagings.Count - 1; i >= 0; i--) {
                var s = stagings [i];
                if (s.Ut <= ut + Epsilon && ut - s.Ut <= StagingWindow)
                    return s;
            }
            return null;
        }

        // ------------------------------------------------------------------ watch

        static bool InWindow (SeparationRecord r, double ut)
        {
            // A clock that went backwards (revert, an FMRS jump to an older save) closes
            // the window: that separation will never be registered in this timeline.
            return ut >= r.Ut - Epsilon && ut - r.Ut <= WatchWindow;
        }

        /// <summary>True while a separation may still be registered by FMRS.</summary>
        internal bool WatchWanted (double ut)
        {
            foreach (var r in pending)
                if (InWindow (r, ut))
                    return true;
            return false;
        }

        /// <summary>Drop the separations whose window is over. Returns how many.</summary>
        internal int Expire (double ut)
        {
            return pending.RemoveAll (r => !InWindow (r, ut));
        }

        // ------------------------------------------------------------------ pairing

        /// <summary>
        /// The pending separation that produced <paramref name="guid"/>: preferably one
        /// that CREATED it (a booster coming off), otherwise one it was the original side
        /// of (FMRS's "main vessel changed": the old main vessel, which kept the root part,
        /// becomes the dropped entry). The latest one wins.
        /// </summary>
        internal SeparationRecord RecordFor (string guid, double ut)
        {
            SeparationRecord asOriginal = null;
            for (int i = pending.Count - 1; i >= 0; i--) {
                var r = pending [i];
                if (!InWindow (r, ut))
                    continue;
                if (r.Created != null && r.Created.Guid == guid)
                    return r;
                if (asOriginal == null && r.Original != null && r.Original.Guid == guid)
                    asOriginal = r;
            }
            return asOriginal;
        }

        /// <summary>
        /// Bind every entry FMRS has registered since the last call to the separation
        /// that produced it. Entries with no pending separation are left to FMRS alone
        /// (capture "fmrs"): they predate the watch, or the bridge never saw them made.
        /// Returns the facts created by this call, in table order.
        /// </summary>
        internal List<DroppedStageFacts> Observe (double ut, IList<FmrsEntry> entries,
                                                  Func<string, uint> livePersistentId)
        {
            var created = new List<DroppedStageFacts> ();
            if (entries == null)
                return created;
            foreach (var e in entries) {
                if (e == null || string.IsNullOrEmpty (e.Guid))
                    continue;
                if (facts.ContainsKey (e.Guid) || seenUnmatched.Contains (e.Guid))
                    continue;
                var record = RecordFor (e.Guid, ut);
                if (record == null) {
                    seenUnmatched.Add (e.Guid);
                    continue;
                }
                var self = record.Created != null && record.Created.Guid == e.Guid
                    ? record.Created : record.Original;
                var parent = ReferenceEquals (self, record.Created) ? record.Original : record.Created;
                uint pid = livePersistentId != null ? livePersistentId (e.Guid) : 0u;
                var f = new DroppedStageFacts {
                    Guid = e.Guid,
                    PersistentId = pid != 0 ? pid : self.PersistentId,
                    Save = e.Save ?? string.Empty,
                    Name = !string.IsNullOrEmpty (e.Name) ? e.Name : self.Name,
                    Stage = record.Stage,
                    StageBefore = record.StageBefore,
                    SeparationUt = record.Ut,
                    SaveUt = !double.IsNaN (lastSaveUt) && lastSaveUt >= record.Ut - Epsilon
                             && lastSaveUt <= ut + Epsilon ? lastSaveUt : ut,
                    ParentPersistentId = parent != null ? parent.PersistentId : 0u,
                    ParentGuid = parent != null ? parent.Guid : string.Empty,
                    ParentName = parent != null ? parent.Name : string.Empty,
                    Craft = record.Craft ?? string.Empty,
                    Kind = record.Kind
                };
                facts [e.Guid] = f;
                factOrder.Add (e.Guid);
                if (factOrder.Count > MaxFacts) {
                    facts.Remove (factOrder [0]);
                    factOrder.RemoveAt (0);
                }
                created.Add (f);
            }
            return created;
        }

        internal DroppedStageFacts Facts (string guid)
        {
            DroppedStageFacts f;
            return guid != null && facts.TryGetValue (guid, out f) ? f : null;
        }

        // ------------------------------------------------------------------ rows

        /// <summary>
        /// The stage number at the end of an FMRS save name ("FMRS_save_4" -> 4), or -1.
        /// "FMRS_save_separated_N" and anything else give -1: their number is not a stage.
        /// </summary>
        internal static int StageFromSaveName (string save)
        {
            const string prefix = "FMRS_save_";
            if (string.IsNullOrEmpty (save) || !save.StartsWith (prefix, StringComparison.Ordinal))
                return -1;
            var tail = save.Substring (prefix.Length);
            if (tail.Length == 0 || tail.Length > 6)
                return -1;
            foreach (var c in tail)
                if (c < '0' || c > '9')
                    return -1;
            return int.Parse (tail, NumberStyles.None, CultureInfo.InvariantCulture);
        }

        /// <summary>One row: FMRS's entry, enriched by what the bridge saw of it.</summary>
        internal DroppedStageRow RowFor (FmrsEntry e)
        {
            var f = Facts (e.Guid);
            var row = new DroppedStageRow {
                Guid = e.Guid ?? string.Empty,
                Save = e.Save ?? string.Empty,
                Name = !string.IsNullOrEmpty (e.Name) ? e.Name : (f != null ? f.Name : string.Empty),
                State = e.State ?? string.Empty,
                PersistentId = f != null && f.PersistentId != 0 ? f.PersistentId : e.LivePersistentId,
                SaveUt = f != null && !double.IsNaN (f.SaveUt) ? f.SaveUt : e.FmrsSaveUt,
                Capture = f != null ? "event" : "fmrs"
            };
            if (f != null) {
                row.SeparationUt = f.SeparationUt;
                row.ParentPersistentId = f.ParentPersistentId;
                row.ParentGuid = f.ParentGuid;
                row.ParentName = f.ParentName;
                row.Craft = f.Craft;
                row.Kind = f.Kind;
                if (f.Stage >= 0) {
                    row.Stage = f.Stage;
                    row.StageBefore = f.StageBefore;
                    row.StageSource = "staging";
                }
                // A separation the bridge saw WITHOUT a staging has no stage: FMRS's
                // number in that case is the current stage, not a released one.
            } else {
                var n = StageFromSaveName (e.Save);
                if (n >= 0) {
                    row.Stage = n;
                    row.StageSource = "save_name";
                }
            }
            return row;
        }

        /// <summary>Every row, ordered by separation, then save write, then Guid.</summary>
        internal List<DroppedStageRow> Rows (IList<FmrsEntry> entries)
        {
            var rows = new List<DroppedStageRow> ();
            if (entries == null)
                return rows;
            foreach (var e in entries)
                if (e != null && !string.IsNullOrEmpty (e.Guid))
                    rows.Add (RowFor (e));
            rows.Sort (CompareRows);
            return rows;
        }

        static int CompareRows (DroppedStageRow a, DroppedStageRow b)
        {
            int c = CompareTimes (a.SeparationUt, b.SeparationUt);
            if (c != 0)
                return c;
            c = CompareTimes (a.SaveUt, b.SaveUt);
            if (c != 0)
                return c;
            return string.CompareOrdinal (a.Guid, b.Guid);
        }

        /// <summary>Known times first, in order; unknown (NaN) last.</summary>
        static int CompareTimes (double a, double b)
        {
            bool na = double.IsNaN (a), nb = double.IsNaN (b);
            if (na && nb)
                return 0;
            if (na)
                return 1;
            if (nb)
                return -1;
            return a.CompareTo (b);
        }
    }

    /// <summary>The wire format of DroppedStagesV1 and ReadyV1.</summary>
    internal static class DroppedStagesFormat
    {
        /// <summary>Wire protocol of DroppedStagesV1 and ReadyV1. Bumped on any breaking change.</summary>
        internal const int Protocol = 1;

        /// <summary>Column set. Columns are only ever appended; a client maps them by name.</summary>
        internal const int Schema = 1;

        internal static readonly string[] Columns = {
            "guid", "persistent_id", "save", "stage", "stage_before", "stage_source",
            "separation_ut", "save_ut", "parent_persistent_id", "parent_guid", "parent_name",
            "name", "craft", "state", "capture", "kind"
        };

        /// <summary>
        /// Header row, column row, then one row per entry. Every row is tab-separated;
        /// the header is key=value pairs. Unknown values are empty strings, never zeros.
        /// </summary>
        internal static List<string> Serialize (IList<KeyValuePair<string, string>> header,
                                                IList<DroppedStageRow> rows)
        {
            var n = rows != null ? rows.Count : 0;
            var lines = new List<string> (n + 2);
            var head = new StringBuilder ();
            head.Append ("protocol=").Append (Protocol.ToString (CultureInfo.InvariantCulture));
            head.Append ("\tschema=").Append (Schema.ToString (CultureInfo.InvariantCulture));
            head.Append ("\tn=").Append (n.ToString (CultureInfo.InvariantCulture));
            if (header != null) {
                foreach (var kv in header) {
                    if (string.IsNullOrEmpty (kv.Key) || kv.Key == "protocol" || kv.Key == "schema"
                        || kv.Key == "n")
                        continue;
                    head.Append ('\t').Append (Clean (kv.Key).Replace ('=', '_'))
                        .Append ('=').Append (Clean (kv.Value));
                }
            }
            lines.Add (head.ToString ());
            lines.Add (string.Join ("\t", Columns));
            if (rows == null)
                return lines;
            foreach (var r in rows) {
                lines.Add (string.Join ("\t", new[] {
                    Clean (r.Guid), Pid (r.PersistentId), Clean (r.Save), Stage (r.Stage),
                    Stage (r.StageBefore), Clean (r.StageSource), Time (r.SeparationUt),
                    Time (r.SaveUt), Pid (r.ParentPersistentId), Clean (r.ParentGuid),
                    Clean (r.ParentName), Clean (r.Name), Clean (r.Craft), Clean (r.State),
                    Clean (r.Capture), Clean (r.Kind)
                }));
            }
            return lines;
        }

        /// <summary>
        /// Why FMRS is not ready, or empty when it is. Ready means: the bridge resolved
        /// FMRS, an FMRS object lives in this scene, the scene is one where FMRS runs and
        /// kRPC answers (flight, space centre, tracking station), and no jump is starting.
        /// </summary>
        internal static string NotReadyReason (bool resolved, bool active, bool sceneOk, bool jumpInProgress)
        {
            if (!resolved)
                return "FMRS did not resolve (not installed, or a member was renamed)";
            if (!sceneOk)
                return "this scene has no FMRS (it runs in flight, the space centre and the tracking station)";
            if (!active)
                return "no FMRS object in this scene yet (scene loading or a jump in progress)";
            if (jumpInProgress)
                return "an FMRS jump is starting";
            return string.Empty;
        }

        internal static string Time (double ut)
        {
            return double.IsNaN (ut) || double.IsInfinity (ut)
                ? string.Empty : ut.ToString ("R", CultureInfo.InvariantCulture);
        }

        internal static string Stage (int stage)
        {
            return stage < 0 ? string.Empty : stage.ToString (CultureInfo.InvariantCulture);
        }

        internal static string Pid (uint pid)
        {
            return pid == 0 ? string.Empty : pid.ToString (CultureInfo.InvariantCulture);
        }

        internal static string Flag (bool value)
        {
            return value ? "1" : "0";
        }

        /// <summary>Fields are tab-separated and rows are lines: tabs and newlines must go.</summary>
        internal static string Clean (string text)
        {
            if (string.IsNullOrEmpty (text))
                return string.Empty;
            return text.Replace ('\t', ' ').Replace ('\n', ' ').Replace ('\r', ' ');
        }

        /// <summary>The detail of an fmrs.stage_registered bus event. No tab: the bus packs on tabs.</summary>
        internal static string EventDetail (DroppedStageFacts f)
        {
            return "stage=" + Stage (f.Stage)
                + " | stage_before=" + Stage (f.StageBefore)
                + " | save=" + Clean (f.Save)
                + " | pid=" + Pid (f.PersistentId)
                + " | parent_pid=" + Pid (f.ParentPersistentId)
                + " | sep_ut=" + Time (f.SeparationUt)
                + " | save_ut=" + Time (f.SaveUt)
                + " | name=" + Clean (f.Name);
        }
    }
}
