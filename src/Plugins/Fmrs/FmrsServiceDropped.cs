using System;
using System.Collections.Generic;
using System.Globalization;
using KRPC.Bridge.Core;
using KRPC.Service.Attributes;

namespace KRPC.Bridge.Fmrs
{
    public static partial class FmrsService
    {
        // ==================================================================
        // The dropped-stage table (DroppedStagesV1) and readiness (ReadyV1)
        // ==================================================================

        /// <summary>
        /// The dropped-stage table: one row per vessel FMRS has registered, with what the
        /// bridge saw of its separation. Read-only.
        ///
        /// FORMAT (protocol 1, schema 1). A list of tab-separated lines:
        /// <list type="bullet">
        /// <item><description>line 0, the header: key=value pairs, always starting with
        /// protocol, schema and n (the number of rows), then ut, scene, main_guid, main_pid,
        /// launched_at, has_launched, switched, craft, launch_ut, stagings, pending, tracker;</description></item>
        /// <item><description>line 1, the column names;</description></item>
        /// <item><description>lines 2 to n+1, the rows.</description></item>
        /// </list>
        /// Columns are only ever appended: map them by name. An unknown value is an empty
        /// string, never a zero.
        ///
        /// THE COLUMNS.
        /// <list type="bullet">
        /// <item><description>guid: FMRS's id, what <see cref="JumpToVessel"/> takes.</description></item>
        /// <item><description>persistent_id: KSP's persistentId of the vessel.</description></item>
        /// <item><description>save: the FMRS save the vessel belongs to.</description></item>
        /// <item><description>stage: the stage ACTIVATED by the staging that released it, that is
        /// the vessel's currentStage just AFTER the staging - the number FMRS puts in
        /// "FMRS_save_N" and the istg of the decoupler in the .craft.</description></item>
        /// <item><description>stage_before: the active vessel's currentStage just BEFORE that
        /// staging (usually stage + 1).</description></item>
        /// <item><description>stage_source: "staging" (the bridge saw the staging and the
        /// separation), "save_name" (the bridge did not see it; read from an "FMRS_save_N"
        /// name), or empty (no stage: a separation outside any staging).</description></item>
        /// <item><description>separation_ut: UT of the separation, as the bridge saw it.</description></item>
        /// <item><description>save_ut: UT at which FMRS registered the vessel and wrote its save
        /// (the bridge's observation, else the time FMRS recorded for the save). A row exists
        /// only once FMRS has registered the vessel, and FMRS writes the save in the same call.</description></item>
        /// <item><description>parent_persistent_id, parent_guid, parent_name: the other vessel of
        /// the separation - the one that carried on.</description></item>
        /// <item><description>name: the vessel's name as FMRS recorded it.</description></item>
        /// <item><description>craft: the name of the vessel at the launch of the mission the
        /// separation belongs to.</description></item>
        /// <item><description>state: FMRS's state now - FLY (registered), LANDED, DESTROYED,
        /// RECOVERED, or NONE.</description></item>
        /// <item><description>capture: "event" when the bridge saw the separation and the
        /// registration, "fmrs" when the row comes from FMRS's table alone (FMRS persists
        /// its table across sessions; a KSP restart loses the bridge's facts).</description></item>
        /// <item><description>kind: "decouple" or "undock", for an "event" row.</description></item>
        /// </list>
        ///
        /// Rows are ordered by separation, then save write, then guid. FMRS's own table
        /// decides which vessels appear: a revert empties it, and the rows go with it.
        ///
        /// Raises like the other members when FMRS is not usable in this scene, so an empty
        /// table always means "nothing registered". <see cref="ReadyV1"/> never raises.
        /// </summary>
        [KRPCProcedure]
        public static IList<string> DroppedStagesV1 ()
        {
            FmrsApi.Require ();
            var live = FmrsApi.Live;
            var ut = DroppedStagesCapture.UniversalTime ();
            var ledger = DroppedStagesCapture.Ledger;

            // While a separation is pending, bind what FMRS has registered right now, so the
            // answer is never a watch period behind FMRS.
            if (ledger.HasPending)
                DroppedStagesCapture.Reconcile (ut);

            var rows = ledger.Rows (DroppedStagesCapture.ReadEntries (live, true));

            var mainGuid = MainGuidOf (live);
            var header = new List<KeyValuePair<string, string>> {
                Pair ("ut", DroppedStagesFormat.Time (ut)),
                Pair ("scene", HighLogic.LoadedScene.ToString ()),
                Pair ("main_guid", mainGuid),
                Pair ("main_pid", DroppedStagesFormat.Pid (MainPersistentId (mainGuid))),
                Pair ("launched_at", DroppedStagesFormat.Time (LaunchedAtOf (live))),
                Pair ("has_launched", DroppedStagesFormat.Flag (FlagOf (FmrsApi.HasLaunchedField, live))),
                Pair ("switched", DroppedStagesFormat.Flag (FlagOf (FmrsApi.SwitchedField, live))),
                Pair ("craft", ledger.Craft),
                Pair ("launch_ut", DroppedStagesFormat.Time (ledger.LaunchUt)),
                Pair ("stagings", ledger.StagingsSinceLaunch.ToString (CultureInfo.InvariantCulture)),
                Pair ("pending", ledger.PendingCount.ToString (CultureInfo.InvariantCulture)),
                Pair ("tracker", DroppedStagesFormat.Flag (ledger.Hooked))
            };
            return DroppedStagesFormat.Serialize (header, rows);
        }

        /// <summary>
        /// Whether FMRS can be used right now, and why not. Never raises: this is the probe
        /// to call before anything else, after a scene change or a jump.
        ///
        /// Keys (protocol 1, schema 1): protocol, schema, ready ("1" or "0"), reason (empty
        /// when ready), resolved, active, scene, scene_ok, jump_in_progress, armed, enabled,
        /// has_launched, switched_to_dropped, main_known, main_guid, main_pid, launched_at,
        /// dropped (number of registered vessels), pending (separations FMRS has not
        /// registered yet), tracker (the bridge's separation capture is hooked).
        ///
        /// ready = FMRS resolved, an FMRS object in this scene, a scene where FMRS runs
        /// (flight, space centre, tracking station), and no jump starting. main_known says
        /// separately whether FMRS has recorded a main mission vessel (an armed launch).
        /// Unknown values are empty strings.
        /// </summary>
        [KRPCProcedure]
        public static IDictionary<string, string> ReadyV1 ()
        {
            var r = new Dictionary<string, string> ();
            r ["protocol"] = DroppedStagesFormat.Protocol.ToString (CultureInfo.InvariantCulture);
            r ["schema"] = DroppedStagesFormat.Schema.ToString (CultureInfo.InvariantCulture);

            bool resolved = FmrsApi.Resolved;
            object live = null;
            try {
                if (resolved)
                    live = ModRegistry.FindLiveAssignable (FmrsApi.CoreType);
            } catch (Exception) {
                live = null;
            }
            var scene = HighLogic.LoadedScene;
            bool sceneOk = scene == GameScenes.FLIGHT || scene == GameScenes.SPACECENTER
                           || scene == GameScenes.TRACKSTATION;
            bool jump = resolved && JumpInProgress;
            var reason = DroppedStagesFormat.NotReadyReason (resolved, live != null, sceneOk, jump);

            r ["ready"] = DroppedStagesFormat.Flag (reason.Length == 0);
            r ["reason"] = reason;
            r ["resolved"] = DroppedStagesFormat.Flag (resolved);
            r ["active"] = DroppedStagesFormat.Flag (live != null);
            r ["scene"] = scene.ToString ();
            r ["scene_ok"] = DroppedStagesFormat.Flag (sceneOk);
            r ["jump_in_progress"] = DroppedStagesFormat.Flag (jump);

            var mainGuid = live != null ? MainGuidOf (live) : string.Empty;
            r ["armed"] = live != null ? DroppedStagesFormat.Flag (FlagOf (FmrsApi.ArmedField, live)) : string.Empty;
            r ["enabled"] = live != null ? DroppedStagesFormat.Flag (FlagOf (FmrsApi.EnabledField, live)) : string.Empty;
            r ["has_launched"] = live != null ? DroppedStagesFormat.Flag (FlagOf (FmrsApi.HasLaunchedField, live)) : string.Empty;
            r ["switched_to_dropped"] = live != null ? DroppedStagesFormat.Flag (FlagOf (FmrsApi.SwitchedField, live)) : string.Empty;
            r ["main_known"] = live != null ? DroppedStagesFormat.Flag (mainGuid.Length > 0) : string.Empty;
            r ["main_guid"] = mainGuid;
            r ["main_pid"] = DroppedStagesFormat.Pid (MainPersistentId (mainGuid));
            r ["launched_at"] = live != null ? DroppedStagesFormat.Time (LaunchedAtOf (live)) : string.Empty;
            r ["dropped"] = live != null ? CountDropped (live) : string.Empty;
            r ["pending"] = DroppedStagesCapture.Ledger.PendingCount.ToString (CultureInfo.InvariantCulture);
            r ["tracker"] = DroppedStagesFormat.Flag (DroppedStagesCapture.Ledger.Hooked);
            return r;
        }

        static KeyValuePair<string, string> Pair (string key, string value)
        {
            return new KeyValuePair<string, string> (key, value ?? string.Empty);
        }

        static bool FlagOf (System.Reflection.FieldInfo field, object live)
        {
            if (field == null || live == null)
                return false;
            try {
                var raw = ModRegistry.Field (field, live);
                return raw is bool && (bool) raw;
            } catch (Exception) {
                return false;
            }
        }

        static string MainGuidOf (object live)
        {
            try {
                var raw = FmrsApi.MainVesselField != null ? ModRegistry.Field (FmrsApi.MainVesselField, live) : null;
                return raw is Guid && (Guid) raw != Guid.Empty ? ((Guid) raw).ToString () : string.Empty;
            } catch (Exception) {
                return string.Empty;
            }
        }

        static double LaunchedAtOf (object live)
        {
            try {
                var raw = FmrsApi.LaunchedAtField != null ? ModRegistry.Field (FmrsApi.LaunchedAtField, live) : null;
                return raw is double ? (double) raw : double.NaN;
            } catch (Exception) {
                return double.NaN;
            }
        }

        static uint MainPersistentId (string mainGuid)
        {
            if (string.IsNullOrEmpty (mainGuid))
                return 0;
            try {
                return DroppedStagesCapture.PersistentIdOf (mainGuid, new Dictionary<Guid, uint> ());
            } catch (Exception) {
                return 0;
            }
        }

        static string CountDropped (object live)
        {
            try {
                var table = ModRegistry.Field (FmrsApi.DroppedField, live) as System.Collections.IDictionary;
                return table != null ? table.Count.ToString (CultureInfo.InvariantCulture) : string.Empty;
            } catch (Exception) {
                return string.Empty;
            }
        }
    }
}
