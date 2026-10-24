// -----------------------------------------------------------------------------------
// FMRS dropped-stage table (DroppedStagesV1): pure tests. See FmrsTests.csproj.
//
// Usage:
//     dotnet run --project build/tests/fmrs/FmrsTests.csproj -c Release
//
// Each scenario plays the GameEvents in the order KSP fires them (onStageActivate, then the
// decouplers' onPartDeCoupleNewVesselComplete in the same frame, then FMRS's registration
// Timer_Stage_Delay later with its save write), then reads the table back.
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Globalization;

namespace KRPC.Bridge.Fmrs
{
    public static class Program
    {
        static int failures;
        static int checks;

        public static int Main (string[] args)
        {
            TestStageFromSaveName ();
            TestJoolHope16h45 ();
            TestJoolHopeOpaqueSaveName ();
            TestDresLanderSideBoostersThenCore ();
            TestMainVesselChanged ();
            TestSeparationWithoutStaging ();
            TestStagingWindow ();
            TestRowsFmrsAlone ();
            TestBaselineKeepsOldEntriesOut ();
            TestWatchWindowAndRevert ();
            TestSaveOutsidePendingIsIgnored ();
            TestLaunchPurgesUntrackedFacts ();
            TestLiftOffStagingSurvivesOnLaunch ();
            TestRowOrder ();
            TestSerialize ();
            TestNotReadyReason ();
            TestEventDetail ();

            Console.WriteLine ();
            Console.WriteLine ("checks: " + checks + ", failures: " + failures);
            return failures == 0 ? 0 : 1;
        }

        // ------------------------------------------------------------------ helpers

        static void Check (bool ok, string what)
        {
            checks++;
            if (ok)
                return;
            failures++;
            Console.WriteLine ("FAIL: " + what);
        }

        static void Equal<T> (T expected, T actual, string what)
        {
            Check (EqualityComparer<T>.Default.Equals (expected, actual),
                   what + " (expected " + expected + ", got " + actual + ")");
        }

        static void Near (double expected, double actual, string what)
        {
            Check (Math.Abs (expected - actual) < 1e-9, what + " (expected " + expected + ", got " + actual + ")");
        }

        static FmrsEntry Entry (string guid, string save, string name = "", string state = "FLY",
                                uint livePid = 0, double fmrsSaveUt = double.NaN)
        {
            return new FmrsEntry {
                Guid = guid, Save = save, Name = name, State = state, LivePersistentId = livePid,
                FmrsSaveUt = fmrsSaveUt
            };
        }

        static Func<string, uint> Pids (Dictionary<string, uint> map)
        {
            return guid => {
                uint pid;
                return map.TryGetValue (guid, out pid) ? pid : 0u;
            };
        }

        const string MainId = "caad3eef-797f-4a4f-8932-e1e0fede32a7";
        const string Booster = "c0f7d80c-c2c6-4fb4-ae7e-6fc41229a1a3";

        // ------------------------------------------------------------------ tests

        static void TestStageFromSaveName ()
        {
            Equal (4, DroppedStageLedger.StageFromSaveName ("FMRS_save_4"), "FMRS_save_4");
            Equal (12, DroppedStageLedger.StageFromSaveName ("FMRS_save_12"), "FMRS_save_12");
            Equal (-1, DroppedStageLedger.StageFromSaveName ("FMRS_save_separated_0"), "separated is not a stage");
            Equal (-1, DroppedStageLedger.StageFromSaveName (Booster), "a guid is not a stage");
            Equal (-1, DroppedStageLedger.StageFromSaveName ("FMRS_save_"), "empty number");
            Equal (-1, DroppedStageLedger.StageFromSaveName (""), "empty");
            Equal (-1, DroppedStageLedger.StageFromSaveName (null), "null");
            Equal (-1, DroppedStageLedger.StageFromSaveName ("FMRS_save_-1"), "negative");
        }

        /// <summary>
        /// The first campaign flight, 03/10 16h45 (vols_20261003_1645, asc_arme_20261003_164636):
        /// the pilot staged at ut 196984.27 with currentStage 5, activating stage 4; the
        /// super hope booster (behind the istg-4 decoupler) came off; FMRS wrote FMRS_save_4
        /// 0.2 s later. A second staging at 197122.79 (4 -> 3) dropped nothing commandable.
        /// </summary>
        static void TestJoolHope16h45 ()
        {
            var l = new DroppedStageLedger ();
            l.NoteLaunch (196968.645, "Jool Hope (Orbiter) test-9", null);
            const double sep = 196984.26549543845;
            l.NoteBaseline (new string [0]);
            l.NoteStaging (sep, 4, 5);
            var rec = l.NoteSeparation (sep, "decouple",
                VesselSide.Of (MainId, 1001, "Jool Hope (Orbiter) test-9"),
                VesselSide.Of (Booster, 2002, "Jool Hope (Orbiter) test-9 Probe"));
            Equal (4, rec.Stage, "16h45: the separation belongs to the staging");
            Check (l.WatchWanted (sep + 0.1), "16h45: watch open after the separation");

            // FMRS registers and saves 0.2 s later, in one Update.
            Check (l.NoteSave (sep + 0.2), "16h45: the save write is remembered while pending");
            var made = l.Observe (sep + 0.22, new List<FmrsEntry> {
                Entry (Booster, "FMRS_save_4", "Jool Hope (Orbiter) test-9 Probe")
            }, Pids (new Dictionary<string, uint> { { Booster, 2002 } }));
            Equal (1, made.Count, "16h45: one booster bound");
            var f = made [0];
            Equal (4, f.Stage, "16h45: stage = the activated stage (FMRS name number)");
            Equal (5, f.StageBefore, "16h45: stage_before = the pilot's log number");
            Near (sep, f.SeparationUt, "16h45: separation ut");
            Near (sep + 0.2, f.SaveUt, "16h45: save ut = the save event");
            Equal (1001u, f.ParentPersistentId, "16h45: parent = the main vessel");
            Equal (MainId, f.ParentGuid, "16h45: parent guid");
            Equal ("Jool Hope (Orbiter) test-9", f.Craft, "16h45: craft = launch name");
            Equal ("decouple", f.Kind, "16h45: kind");

            // Second staging: a fairing or a spent stage without probe, never registered.
            const double sep2 = 197122.78549536588;
            l.NoteBaseline (new[] { Booster });
            l.NoteStaging (sep2, 3, 4);
            l.NoteSeparation (sep2, "decouple", VesselSide.Of (MainId, 1001, "J"),
                              VesselSide.Of ("dddddddd-0000-0000-0000-000000000001", 3003, "J Debris"));
            l.NoteSave (sep2 + 0.2);
            var none = l.Observe (sep2 + 0.3, new List<FmrsEntry> { Entry (Booster, "FMRS_save_4") }, null);
            Equal (0, none.Count, "16h45: the second staging registers nothing");
            // Records are not consumed by a binding (the other side may still be registered:
            // FMRS's main-vessel swap); they leave by their window only.
            Equal (1, l.Expire (sep2 + 5.0), "16h45: only the booster's old record expires at +5 s");
            Equal (1, l.Expire (sep2 + 10.5), "16h45: the debris separation expires");
            Check (!l.WatchWanted (sep2 + 10.5), "16h45: watch closed");
            Equal (2, l.StagingsSinceLaunch, "16h45: two stagings since launch");

            var rows = l.Rows (new List<FmrsEntry> {
                Entry (Booster, "FMRS_save_4", "Jool Hope (Orbiter) test-9 Probe", "FLY", 2002, sep + 0.2)
            });
            Equal (1, rows.Count, "16h45: one row");
            var r = rows [0];
            Equal (4, r.Stage, "16h45 row: stage");
            Equal ("staging", r.StageSource, "16h45 row: stage source");
            Equal ("event", r.Capture, "16h45 row: capture");
            Equal (2002u, r.PersistentId, "16h45 row: pid");
            Equal ("FMRS_save_4", r.Save, "16h45 row: save");
            Equal ("FLY", r.State, "16h45 row: FMRS state");
        }

        /// <summary>
        /// The same flight with an opaque save name: the stage no longer depends on the
        /// name at all.
        /// </summary>
        static void TestJoolHopeOpaqueSaveName ()
        {
            var l = new DroppedStageLedger ();
            l.NoteLaunch (100.0, "Jool Hope (Orbiter) test-9", null);
            l.NoteStaging (150.0, 4, 5);
            l.NoteSeparation (150.0, "decouple", VesselSide.Of (MainId, 1, "J"), VesselSide.Of (Booster, 2, "J Probe"));
            var made = l.Observe (150.25, new List<FmrsEntry> { Entry (Booster, Booster, "J Probe") }, null);
            Equal (1, made.Count, "opaque: bound");
            Near (150.25, made [0].SaveUt, "opaque: no save event, save ut = the diff");
            var row = l.Rows (new List<FmrsEntry> { Entry (Booster, Booster, "J Probe") }) [0];
            Equal (4, row.Stage, "opaque: stage from the staging, not the name");
            Equal ("staging", row.StageSource, "opaque: source");
            Equal (2u, row.PersistentId, "opaque: pid from the separation when KSP has none live");
        }

        /// <summary>
        /// dres lander: staging 7 -> 6 drops the two side boosters (one save), staging 6 -> 5
        /// drops the core (another save), staging 5 -> 4 drops nothing commandable.
        /// </summary>
        static void TestDresLanderSideBoostersThenCore ()
        {
            const string L1 = "11111111-0000-0000-0000-000000000001";
            const string L2 = "11111111-0000-0000-0000-000000000002";
            const string C = "22222222-0000-0000-0000-000000000001";
            var l = new DroppedStageLedger ();
            l.NoteLaunch (1000.0, "dres lander-12", null);

            l.NoteStaging (1060.0, 6, 7);
            l.NoteSeparation (1060.0, "decouple", VesselSide.Of (MainId, 10, "dres lander-12"),
                              VesselSide.Of (L1, 11, "dres lander-12 Probe"));
            l.NoteSeparation (1060.0, "decouple", VesselSide.Of (MainId, 10, "dres lander-12"),
                              VesselSide.Of (L2, 12, "dres lander-12 Probe"));
            l.NoteSave (1060.2);
            var first = l.Observe (1060.22, new List<FmrsEntry> {
                Entry (L2, "FMRS_save_6", "dres lander-12 Probe"),
                Entry (L1, "FMRS_save_6", "dres lander-12 Probe")
            }, null);
            Equal (2, first.Count, "dres: both side boosters bound");
            foreach (var f in first) {
                Equal (6, f.Stage, "dres: side booster stage 6");
                Equal (7, f.StageBefore, "dres: side booster stage before 7");
                Equal (10u, f.ParentPersistentId, "dres: side booster parent");
            }

            l.NoteStaging (1150.0, 5, 6);
            l.NoteSeparation (1150.0, "decouple", VesselSide.Of (MainId, 10, "dres lander-12"),
                              VesselSide.Of (C, 13, "dres lander-12 Probe"));
            l.NoteSave (1150.2);
            var table = new List<FmrsEntry> {
                Entry (L2, "FMRS_save_6", "dres lander-12 Probe"),
                Entry (L1, "FMRS_save_6", "dres lander-12 Probe"),
                Entry (C, "FMRS_save_5", "dres lander-12 Probe")
            };
            var second = l.Observe (1150.22, table, null);
            Equal (1, second.Count, "dres: only the core is new");
            Equal (C, second [0].Guid, "dres: the core");
            Equal (5, second [0].Stage, "dres: core stage 5");
            Near (1150.2, second [0].SaveUt, "dres: core save ut");

            var rows = l.Rows (table);
            Equal (3, rows.Count, "dres: three rows");
            Equal (L1, rows [0].Guid, "dres: rows by separation then guid (L1)");
            Equal (L2, rows [1].Guid, "dres: rows by separation then guid (L2)");
            Equal (C, rows [2].Guid, "dres: the core last");
            Equal (6, rows [0].Stage, "dres: row stage 6");
            Equal (5, rows [2].Stage, "dres: row stage 5");
        }

        /// <summary>
        /// FMRS's "main vessel changed": the dropped side kept the root part, so it keeps
        /// the original id and the vessel that carries on is the NEW one. FMRS then
        /// registers the old main id as the dropped entry (after its save write).
        /// </summary>
        static void TestMainVesselChanged ()
        {
            const string NewActive = "33333333-0000-0000-0000-000000000001";
            var l = new DroppedStageLedger ();
            l.NoteLaunch (0.0, "craft", null);
            l.NoteStaging (50.0, 3, 4);
            l.NoteSeparation (50.0, "decouple", VesselSide.Of (MainId, 100, "craft"),
                              VesselSide.Of (NewActive, 200, "craft Probe"));
            l.NoteSave (50.2);
            var made = l.Observe (50.22, new List<FmrsEntry> { Entry (MainId, "FMRS_save_3", "craft") }, null);
            Equal (1, made.Count, "main changed: old main id bound");
            Equal (3, made [0].Stage, "main changed: stage");
            Equal (100u, made [0].PersistentId, "main changed: its own pid (original side)");
            Equal (200u, made [0].ParentPersistentId, "main changed: parent = the vessel that carries on");
            Equal (NewActive, made [0].ParentGuid, "main changed: parent guid");
        }

        static void TestSeparationWithoutStaging ()
        {
            var l = new DroppedStageLedger ();
            l.NoteLaunch (0.0, "craft", null);
            l.NoteSeparation (80.0, "decouple", VesselSide.Of (MainId, 1, "craft"), VesselSide.Of (Booster, 2, "x"));
            l.NoteSave (81.2);
            var made = l.Observe (81.25, new List<FmrsEntry> { Entry (Booster, "FMRS_save_separated_0") }, null);
            Equal (1, made.Count, "no staging: still bound");
            Equal (-1, made [0].Stage, "no staging: no stage");
            var row = l.Rows (new List<FmrsEntry> { Entry (Booster, "FMRS_save_separated_0") }) [0];
            Equal (-1, row.Stage, "no staging: row has no stage");
            Equal ("", row.StageSource, "no staging: no stage source");
            Equal ("event", row.Capture, "no staging: captured");
            // Even if FMRS named it FMRS_save_N (a decoupler fired by an action group raises
            // onStageSeparation too), the bridge does not turn that number into a stage.
            var named = l.Rows (new List<FmrsEntry> { Entry (Booster, "FMRS_save_7") }) [0];
            Equal (-1, named.Stage, "no staging: FMRS's number is the current stage, not a released one");
        }

        static void TestStagingWindow ()
        {
            var l = new DroppedStageLedger ();
            l.NoteStaging (10.0, 4, 5);
            Equal (4, l.NoteSeparation (10.0, "decouple", VesselSide.Of ("a", 1, ""), VesselSide.Of ("b", 2, "")).Stage,
                   "window: same frame");
            Equal (4, l.NoteSeparation (10.45, "decouple", VesselSide.Of ("a", 1, ""), VesselSide.Of ("c", 3, "")).Stage,
                   "window: 0.45 s later");
            Equal (-1, l.NoteSeparation (10.6, "decouple", VesselSide.Of ("a", 1, ""), VesselSide.Of ("d", 4, "")).Stage,
                   "window: 0.6 s later is another separation");
            Equal (-1, l.NoteSeparation (9.9, "decouple", VesselSide.Of ("a", 1, ""), VesselSide.Of ("e", 5, "")).Stage,
                   "window: before the staging");
            l.NoteStaging (20.0, 3, 4);
            Equal (3, l.NoteSeparation (20.0, "decouple", VesselSide.Of ("a", 1, ""), VesselSide.Of ("f", 6, "")).Stage,
                   "window: the latest staging");
        }

        /// <summary>FMRS persists its table; after a KSP restart the bridge has no facts.</summary>
        static void TestRowsFmrsAlone ()
        {
            var l = new DroppedStageLedger ();
            var rows = l.Rows (new List<FmrsEntry> {
                Entry ("aaaaaaaa-0000-0000-0000-000000000001", "FMRS_save_6", "tylo Probe", "RECOVERED", 77, 197023.85),
                Entry ("aaaaaaaa-0000-0000-0000-000000000002", Booster, "other", "FLY")
            });
            Equal (2, rows.Count, "fmrs alone: two rows");
            Equal ("fmrs", rows [0].Capture, "fmrs alone: capture");
            Equal (6, rows [0].Stage, "fmrs alone: stage from the save name");
            Equal ("save_name", rows [0].StageSource, "fmrs alone: source save_name");
            Near (197023.85, rows [0].SaveUt, "fmrs alone: FMRS's own save time");
            Equal (77u, rows [0].PersistentId, "fmrs alone: live pid");
            Equal ("RECOVERED", rows [0].State, "fmrs alone: state");
            Equal (-1, rows [1].Stage, "fmrs alone: opaque name gives no stage");
            Equal ("", rows [1].StageSource, "fmrs alone: no source");
            Check (double.IsNaN (rows [1].SaveUt), "fmrs alone: unknown save time stays unknown");
        }

        /// <summary>
        /// An entry FMRS held before the separation is never bound to it - here the main
        /// vessel itself, left in FMRS's table by an earlier flight, which is the original
        /// side of every separation.
        /// </summary>
        static void TestBaselineKeepsOldEntriesOut ()
        {
            var l = new DroppedStageLedger ();
            l.NoteLaunch (0.0, "craft", null);
            l.NoteBaseline (new[] { MainId });
            l.NoteStaging (40.0, 4, 5);
            l.NoteSeparation (40.0, "decouple", VesselSide.Of (MainId, 1, "craft"), VesselSide.Of (Booster, 2, "b"));
            l.NoteSave (40.2);
            var made = l.Observe (40.22, new List<FmrsEntry> {
                Entry (MainId, "FMRS_save_9"), Entry (Booster, "FMRS_save_4")
            }, null);
            Equal (1, made.Count, "baseline: only the new entry");
            Equal (Booster, made [0].Guid, "baseline: the booster");
            Check (l.Facts (MainId) == null, "baseline: no facts for the old entry");
        }

        static void TestWatchWindowAndRevert ()
        {
            var l = new DroppedStageLedger ();
            l.NoteSeparation (500.0, "decouple", VesselSide.Of ("a", 1, ""), VesselSide.Of ("b", 2, ""));
            Check (l.WatchWanted (500.0), "window: open at the separation");
            Check (l.WatchWanted (509.9), "window: open before 10 s");
            Check (!l.WatchWanted (510.1), "window: closed after 10 s");
            // A revert or a jump to an older save: the clock goes back.
            Check (!l.WatchWanted (420.0), "window: closed when the clock goes back");
            Equal (0, l.Observe (420.0, new List<FmrsEntry> { Entry ("b", "FMRS_save_1") }, null).Count,
                   "window: nothing bound in another timeline");
            Equal (1, l.Expire (420.0), "window: expired by the revert");
            Check (!l.HasPending, "window: nothing pending");
        }

        static void TestSaveOutsidePendingIsIgnored ()
        {
            var l = new DroppedStageLedger ();
            Check (!l.NoteSave (10.0), "save: no pending separation, nothing to do");
            l.NoteStaging (20.0, 2, 3);
            l.NoteSeparation (20.0, "decouple", VesselSide.Of ("a", 1, ""), VesselSide.Of ("b", 2, ""));
            var made = l.Observe (20.25, new List<FmrsEntry> { Entry ("b", "FMRS_save_2") }, null);
            Near (20.25, made [0].SaveUt, "save: the early save is not taken for this one");
        }

        static void TestLaunchPurgesUntrackedFacts ()
        {
            var l = new DroppedStageLedger ();
            l.NoteLaunch (0.0, "first", null);
            l.NoteStaging (10.0, 4, 5);
            l.NoteSeparation (10.0, "decouple", VesselSide.Of ("m", 1, ""), VesselSide.Of ("b1", 2, ""));
            l.NoteSeparation (10.0, "decouple", VesselSide.Of ("m", 1, ""), VesselSide.Of ("b2", 3, ""));
            l.Observe (10.3, new List<FmrsEntry> { Entry ("b1", "FMRS_save_4"), Entry ("b2", "FMRS_save_4") }, null);
            Equal (2, l.FactCount, "launch: two facts");
            l.NoteLaunch (100.0, "second", new HashSet<string> { "b2" });
            Equal (1, l.FactCount, "launch: the untracked one is forgotten");
            Check (l.Facts ("b2") != null, "launch: the tracked one is kept");
            Equal ("second", l.Craft, "launch: craft");
            Equal (0, l.StagingsSinceLaunch, "launch: staging count reset");
            Check (!l.HasPending, "launch: nothing pending");
        }

        /// <summary>
        /// KSP fires onStageActivate, then the decouplers, then onLaunch, in one call: a
        /// booster dropped by the lift-off staging must survive the launch bookkeeping.
        /// </summary>
        static void TestLiftOffStagingSurvivesOnLaunch ()
        {
            var l = new DroppedStageLedger ();
            l.NoteLaunch (10.0, "previous", null);
            l.NoteStaging (10.0, 9, 10);
            l.NoteStaging (300.0, 7, 8);
            l.NoteSeparation (300.0, "decouple", VesselSide.Of ("m", 1, "pad name"), VesselSide.Of ("b", 2, "b"));
            l.NoteLaunch (300.0, "new craft", null);
            Equal (1, l.StagingsSinceLaunch, "lift-off: the lift-off staging counts");
            Check (l.HasPending, "lift-off: its separation is still pending");
            var made = l.Observe (300.2, new List<FmrsEntry> { Entry ("b", "FMRS_save_7") }, null);
            Equal (1, made.Count, "lift-off: bound");
            Equal (7, made [0].Stage, "lift-off: stage");
            Equal ("new craft", made [0].Craft, "lift-off: the new mission's craft");
        }

        static void TestRowOrder ()
        {
            var l = new DroppedStageLedger ();
            l.NoteStaging (30.0, 2, 3);
            l.NoteSeparation (30.0, "decouple", VesselSide.Of ("m", 1, ""), VesselSide.Of ("z-late", 2, ""));
            l.NoteStaging (25.0, 3, 4);
            l.NoteSeparation (25.0, "decouple", VesselSide.Of ("m", 1, ""), VesselSide.Of ("y-early", 3, ""));
            l.Observe (30.3, new List<FmrsEntry> { Entry ("z-late", "FMRS_save_2"), Entry ("y-early", "FMRS_save_3") }, null);
            var rows = l.Rows (new List<FmrsEntry> {
                Entry ("x-unknown", "FMRS_save_9"), Entry ("z-late", "FMRS_save_2"), Entry ("y-early", "FMRS_save_3")
            });
            Equal ("y-early", rows [0].Guid, "order: earliest separation first");
            Equal ("z-late", rows [1].Guid, "order: then the later one");
            Equal ("x-unknown", rows [2].Guid, "order: unknown separation last");
        }

        static void TestSerialize ()
        {
            var rows = new List<DroppedStageRow> {
                new DroppedStageRow {
                    Guid = Booster, PersistentId = 2002, Save = "FMRS_save_4", Stage = 4, StageBefore = 5,
                    StageSource = "staging", SeparationUt = 196984.26549543845, SaveUt = 196984.46549543845,
                    ParentPersistentId = 1001, ParentGuid = MainId, ParentName = "Jool\tHope",
                    Name = "Jool Hope (Orbiter) test-9 Probe", Craft = "Jool Hope (Orbiter) test-9",
                    State = "FLY", Capture = "event", Kind = "decouple"
                },
                new DroppedStageRow { Guid = "g2", Save = "x", Capture = "fmrs" }
            };
            var header = new List<KeyValuePair<string, string>> {
                new KeyValuePair<string, string> ("ut", "197000"),
                new KeyValuePair<string, string> ("craft", "a\tb\nc"),
                new KeyValuePair<string, string> ("n", "999"),
                new KeyValuePair<string, string> ("we=ird", "v")
            };
            var lines = DroppedStagesFormat.Serialize (header, rows);
            Equal (4, lines.Count, "format: header + columns + 2 rows");
            Check (lines [0].StartsWith ("protocol=1\tschema=1\tn=2\t", StringComparison.Ordinal),
                   "format: header starts with protocol, schema, n: " + lines [0]);
            Check (lines [0].Contains ("\tcraft=a b c"), "format: header value cleaned");
            Check (!lines [0].Contains ("n=999"), "format: n is never overridden");
            Check (lines [0].Contains ("\twe_ird=v"), "format: '=' in a key replaced");
            Equal (string.Join ("\t", DroppedStagesFormat.Columns), lines [1], "format: column row");
            Equal (16, DroppedStagesFormat.Columns.Length, "format: schema 1 has 16 columns");
            var cells = lines [2].Split ('\t');
            Equal (16, cells.Length, "format: one cell per column");
            Equal (Booster, cells [0], "format: guid");
            Equal ("2002", cells [1], "format: pid");
            Equal ("4", cells [3], "format: stage");
            Equal ("5", cells [4], "format: stage_before");
            Equal (196984.26549543845, double.Parse (cells [6], CultureInfo.InvariantCulture), "format: ut round trip");
            Equal ("Jool Hope", cells [10], "format: tab in a name cleaned");
            var empty = lines [3].Split ('\t');
            Equal (16, empty.Length, "format: unknown cells kept");
            Equal ("", empty [1], "format: unknown pid is empty, not 0");
            Equal ("", empty [3], "format: unknown stage is empty, not -1");
            Equal ("", empty [6], "format: unknown time is empty, not NaN");

            var none = DroppedStagesFormat.Serialize (null, new List<DroppedStageRow> ());
            Equal (2, none.Count, "format: empty table is header + columns");
            Equal ("protocol=1\tschema=1\tn=0", none [0], "format: empty header");
        }

        static void TestNotReadyReason ()
        {
            Equal ("", DroppedStagesFormat.NotReadyReason (true, true, true, false), "ready");
            Check (DroppedStagesFormat.NotReadyReason (false, true, true, false).Length > 0, "not resolved");
            Check (DroppedStagesFormat.NotReadyReason (true, false, true, false).Length > 0, "no live object");
            Check (DroppedStagesFormat.NotReadyReason (true, true, false, false).Length > 0, "wrong scene");
            Check (DroppedStagesFormat.NotReadyReason (true, true, true, true).Length > 0, "jump starting");
        }

        static void TestEventDetail ()
        {
            var detail = DroppedStagesFormat.EventDetail (new DroppedStageFacts {
                Stage = 4, StageBefore = 5, Save = "FMRS_save_4", PersistentId = 2002, ParentPersistentId = 1001,
                SeparationUt = 1.5, SaveUt = 1.7, Name = "a\tb"
            });
            Check (detail.IndexOf ('\t') < 0, "event: no tab in the detail");
            Check (detail.StartsWith ("stage=4 | stage_before=5 | save=FMRS_save_4", StringComparison.Ordinal),
                   "event: detail " + detail);
        }
    }
}
