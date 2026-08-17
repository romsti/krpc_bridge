using System;
using System.Collections;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using KRPC.Service;
using KRPC.Service.Attributes;
using UnityEngine;

namespace KRPC.Bridge.Fmrs
{
    /// <summary>
    /// Remote control of FMRS, the Flight Manager for Reusable Stages.
    ///
    /// FMRS records a save file at every staging event and lets you fly a dropped stage
    /// afterwards, then return to the main mission where you left it. It exposes no
    /// KSPEvent, no action group and no key binding, so there is nothing for stock kRPC
    /// to reach; this service calls its public methods by reflection.
    ///
    /// THE THREE THINGS TO KNOW BEFORE SCRIPTING IT.
    ///
    /// One: check <see cref="Available"/> first. Every other member throws when it is
    /// false, and <see cref="Diagnostics"/> then says exactly which member did not
    /// resolve.
    ///
    /// Two: a jump reloads the whole flight scene, which takes roughly 5 to 20 seconds.
    /// <see cref="JumpToVessel"/> blocks for all of it and every kRPC handle you held
    /// beforehand - vessels, parts, modules - is dead afterwards. Remove your streams
    /// first, then rebuild everything from a fresh space_center.active_vessel.
    ///
    /// Three: <see cref="SwitchedToDropped"/> is the guard rail. FMRS hooks
    /// onGameSceneLoadRequested, and if a scene change is requested while that flag is
    /// true it force-loads the main mission a few dozen frames later - dragging the game
    /// back into flight in the middle of whatever your script was doing. Always
    /// <see cref="JumpToMain"/> before reverting, recovering or launching anything else.
    ///
    /// Verified against FMRS Continued 1.2.9.6.
    /// </summary>
    [KRPCService (Name = "FMRS", GameScene = GameScene.All)]
    public static class FmrsService
    {
        /// <summary>How long a jump waits, in seconds of real time, before giving up.</summary>
        const float JumpTimeoutSeconds = 90f;

        /// <summary>Physics ticks to let the new scene settle once the target vessel is unpacked.</summary>
        const int SettleTicks = 25;

        /// <summary>Scratch index for <see cref="DroppedPersistentIds"/>. Reused, never returned.</summary>
        static readonly Dictionary<Guid, Vessel> vesselIndex = new Dictionary<Guid, Vessel> ();

        // ==================================================================
        // Is it there
        // ==================================================================

        /// <summary>
        /// Whether FMRS is installed and its jump API resolved. Check this before
        /// anything else; every other member throws when it is <c>false</c>.
        /// </summary>
        [KRPCProperty]
        public static bool Available {
            get { return FmrsApi.Resolved; }
        }

        /// <summary>
        /// What type resolution found, member by member. Read this when
        /// <see cref="Available"/> is false, or when one member throws while others
        /// work: it names the exact lookup that came back empty, which is the same
        /// information an FMRS rename would change.
        ///
        /// Cheap - the string is built once at load - so it is safe to stream, though
        /// there is no reason to.
        /// </summary>
        [KRPCProperty]
        public static string Diagnostics {
            get { return FmrsApi.Report; }
        }

        /// <summary>
        /// Whether an FMRS object exists in the current scene. FMRS runs in flight, the
        /// space centre, the tracking station and the main menu, so this is true almost
        /// everywhere - but briefly false while a jump reloads the scene.
        /// </summary>
        [KRPCProperty]
        public static bool Active {
            get {
                return FmrsApi.Resolved
                       && ModRegistry.FindLiveAssignable (FmrsApi.CoreType) != null;
            }
        }

        /// <summary>
        /// Round-trip check that the service is loaded and callable. Returns "pong".
        ///
        /// Worth doing before wiring anything else: a single malformed kRPC signature in
        /// any loaded assembly disables the entire kRPC server, not just its own service,
        /// so "the whole connection is dead" and "this plugin is broken" look identical
        /// from Python until you check.
        /// </summary>
        [KRPCProcedure]
        public static string Ping ()
        {
            return "pong";
        }

        // ==================================================================
        // Flight state
        // ==================================================================

        /// <summary>
        /// Whether FMRS will capture this flight's dropped stages.
        ///
        /// There is no "arm" method in FMRS. This is a raw public field, read exactly
        /// once, at the top of its launch routine: if it is false FMRS closes itself and
        /// returns. So it must be true BEFORE GameEvents.onLaunch fires, which for a
        /// scripted campaign means between the craft arriving on the pad in PRELAUNCH and
        /// the first staging.
        ///
        /// It is sticky - FMRS persists it - so it usually survives from one flight to
        /// the next. Read it and correct it rather than assuming. Writing it also writes
        /// FMRS's save file, so the value survives a restart.
        ///
        /// Not to be confused with <see cref="Enabled"/>, which FMRS sets by itself.
        /// </summary>
        [KRPCProperty]
        public static bool Armed {
            get { return FmrsApi.ReadFlag (FmrsApi.ArmedField, "Armed"); }
            set {
                FmrsApi.RequireMember (FmrsApi.ArmedField, "Armed");
                var live = FmrsApi.Live;
                FmrsApi.ArmedField.SetValue (live, value);
                if (FmrsApi.WriteSaveValues != null)
                    FmrsApi.WriteSaveValues.Invoke (live, null);
            }
        }

        /// <summary>
        /// Whether the FMRS plugin is running for this flight.
        ///
        /// FMRS sets this itself from its autoactiveAtLaunch setting and clears it when
        /// the main vessel is recovered. Distinct from <see cref="Armed"/>: armed is
        /// "capture the stages of the flight about to start", enabled is "the plugin is
        /// live right now". A flight can be enabled but not armed, in which case nothing
        /// is recorded.
        ///
        /// Writable, but prefer <see cref="Armed"/> - this one is FMRS's own bookkeeping
        /// and writing it does not persist unless you set <see cref="Armed"/> too.
        /// </summary>
        [KRPCProperty]
        public static bool Enabled {
            get { return FmrsApi.ReadFlag (FmrsApi.EnabledField, "Enabled"); }
            set {
                FmrsApi.RequireMember (FmrsApi.EnabledField, "Enabled");
                FmrsApi.EnabledField.SetValue (FmrsApi.Live, value);
            }
        }

        /// <summary>
        /// Whether FMRS thinks we are currently flying a dropped stage rather than the
        /// main mission.
        ///
        /// THE guard rail for an automated campaign. FMRS hooks
        /// onGameSceneLoadRequested: if a scene change is requested while this is true it
        /// sets its "kick to main" flag, and its space-centre addon then counts 35 fixed
        /// frames and force-loads the main save - dragging the game back into flight in
        /// the middle of whatever the script was doing. So: always
        /// <see cref="JumpToMain"/> before reverting, recovering, or launching the next
        /// craft, and assert on this before doing so.
        /// </summary>
        [KRPCProperty]
        public static bool SwitchedToDropped {
            get { return FmrsApi.ReadFlag (FmrsApi.SwitchedField, "SwitchedToDropped"); }
        }

        /// <summary>Whether FMRS has seen this flight launch, which closes its arming window.</summary>
        [KRPCProperty]
        public static bool HasLaunched {
            get { return FmrsApi.ReadFlag (FmrsApi.HasLaunchedField, "HasLaunched"); }
        }

        /// <summary>
        /// Whether FMRS has queued a forced return to the main mission.
        ///
        /// FMRS sets this when you recover a dropped stage, and when you leave the flight
        /// scene while <see cref="SwitchedToDropped"/> is true. Its space-centre and
        /// tracking-station addons consume it after a short countdown by loading the main
        /// save and dropping you back into flight.
        ///
        /// A script that finds this true in the space centre is about to be interrupted.
        /// It can either let it happen or clear it, but it should not ignore it.
        /// </summary>
        [KRPCProperty]
        public static bool KickToMain {
            get { return FmrsApi.ReadFlag (FmrsApi.KickToMainField, "KickToMain"); }
            set {
                FmrsApi.RequireMember (FmrsApi.KickToMainField, "KickToMain");
                var live = FmrsApi.Live;
                FmrsApi.KickToMainField.SetValue (live, value);
                if (FmrsApi.WriteSaveValues != null)
                    FmrsApi.WriteSaveValues.Invoke (live, null);
            }
        }

        /// <summary>
        /// Universal time at which FMRS recorded the launch of the current mission, or 0
        /// if it has not armed a flight.
        ///
        /// Subtract it from space_center.ut for the mission clock FMRS's own window
        /// shows. Note this is the MAIN mission's launch, not the current vessel's: after
        /// a jump the dropped stage carries its own met while this stays put, which is
        /// what makes the two comparable.
        /// </summary>
        [KRPCProperty]
        public static double LaunchedAt {
            get {
                FmrsApi.RequireMember (FmrsApi.LaunchedAtField, "LaunchedAt");
                var raw = ModRegistry.Field (FmrsApi.LaunchedAtField, FmrsApi.Live);
                return raw is double ? (double) raw : 0.0;
            }
        }

        /// <summary>
        /// Whether an FMRS jump is currently starting.
        ///
        /// It goes true the moment a jump is requested and false about a second later,
        /// when FMRS hands over to the scene loader - so a false reading does NOT mean
        /// the jump is finished. FMRS's own window greys out its buttons while this is
        /// true, and this service refuses to start a second jump on the same basis.
        /// </summary>
        [KRPCProperty]
        public static bool JumpInProgress {
            get {
                try {
                    return (bool) FmrsApi.JumpInProgressProp.GetValue (FmrsApi.SaveUtil, null);
                } catch (Exception) {
                    // FMRS_SAVE_Util not initialised yet, or the getter threw - reflection
                    // wraps that in TargetInvocationException. Either way, no jump is in
                    // progress, and a guard that throws is worse than a guard that says no.
                    return false;
                }
            }
        }

        // ==================================================================
        // Settings
        // ==================================================================

        /// <summary>
        /// Whether FMRS recovers a dropped stage by itself once it has landed.
        ///
        /// This is a STATIC field, which is what makes it settable in flight: FMRS copies
        /// its difficulty settings into these statics from its space-centre addon only,
        /// so changing the setting mid-flight would do nothing while writing the static
        /// works immediately. The flip side is that it does not persist - set it each
        /// session.
        ///
        /// WHEN IT ACTUALLY FIRES, because the obvious guess is wrong. Recovery is NOT
        /// triggered by a stage touching down - FMRS has no landing handler and never
        /// simulates an unloaded stage. It fires only at the four moments you LEAVE a
        /// dropped stage: <see cref="JumpToVessel"/> with saveLanded true,
        /// <see cref="JumpToMain"/>, the stock Recover Vessel button, and a scene change
        /// while <see cref="SwitchedToDropped"/> is true. A booster can sit landed
        /// indefinitely with this on and nothing happens. It is also Kerbin-only.
        ///
        /// SO IT DOES NOT BLOCK A REPLAY. Re-jumping to the stage you are already flying
        /// passes saveLanded false, which never reaches FMRS's recovery path at all - the
        /// replay loop is recovery-free by construction. And even a RECOVERED stage stays
        /// jumpable: <see cref="JumpToVessel"/> does not read the state, and the
        /// separation save is never deleted. What recovery costs you is FMRS's own GUI
        /// button for that stage and any second payout, not the flight.
        ///
        /// Turn it off when you want to leave a landed booster and come back to it later
        /// without banking it, or to keep a campaign's accounting under script control.
        /// </summary>
        [KRPCProperty]
        public static bool AutoRecover {
            get { return FmrsApi.ReadStaticFlag (FmrsApi.AutoRecoverField, "AutoRecover"); }
            set {
                FmrsApi.RequireMember (FmrsApi.AutoRecoverField, "AutoRecover");
                FmrsApi.AutoRecoverField.SetValue (null, value);
            }
        }

        /// <summary>
        /// Whether FMRS tracks a dropped stage that has parachutes but no probe core.
        ///
        /// Session-scoped static, like <see cref="AutoRecover"/>. Turn it on for a
        /// booster that is meant to come down under canopy: without it FMRS ignores any
        /// stage it considers uncontrollable, and nothing appears in
        /// <see cref="DroppedVessels"/>.
        /// </summary>
        [KRPCProperty]
        public static bool TrackParachutes {
            get { return FmrsApi.ReadStaticFlag (FmrsApi.ParachutesField, "TrackParachutes"); }
            set {
                FmrsApi.RequireMember (FmrsApi.ParachutesField, "TrackParachutes");
                FmrsApi.ParachutesField.SetValue (null, value);
            }
        }

        /// <summary>
        /// Whether FMRS lets you fly a stage it considers uncontrollable.
        ///
        /// The other half of the "why is my booster not in the list" question, and the
        /// one to check first for a stage with neither a probe core nor a parachute.
        /// Session-scoped static.
        /// </summary>
        [KRPCProperty]
        public static bool ControlUncontrollable {
            get { return FmrsApi.ReadStaticFlag (FmrsApi.UncontrollableField, "ControlUncontrollable"); }
            set {
                FmrsApi.RequireMember (FmrsApi.UncontrollableField, "ControlUncontrollable");
                FmrsApi.UncontrollableField.SetValue (null, value);
            }
        }

        /// <summary>
        /// Whether FMRS cuts the engines of a stage it stops simulating.
        /// Session-scoped static.
        /// </summary>
        [KRPCProperty]
        public static bool AutoCutOff {
            get { return FmrsApi.ReadStaticFlag (FmrsApi.AutoCutOffField, "AutoCutOff"); }
            set {
                FmrsApi.RequireMember (FmrsApi.AutoCutOffField, "AutoCutOff");
                FmrsApi.AutoCutOffField.SetValue (null, value);
            }
        }

        /// <summary>
        /// Whether FMRS posts its screen messages. Turn it off for a recording: FMRS is
        /// chatty at every separation and every recovery. Session-scoped static.
        /// </summary>
        [KRPCProperty]
        public static bool ScreenMessages {
            get { return FmrsApi.ReadStaticFlag (FmrsApi.MessagesField, "ScreenMessages"); }
            set {
                FmrsApi.RequireMember (FmrsApi.MessagesField, "ScreenMessages");
                FmrsApi.MessagesField.SetValue (null, value);
            }
        }

        /// <summary>
        /// Whether FMRS's own window is hidden.
        ///
        /// For a scripted flight being recorded: the script does not need the window, and
        /// it sits over the view. This hides only FMRS, unlike the HUD toggle in the
        /// Bridge service which hides everything.
        /// </summary>
        [KRPCProperty]
        public static bool WindowHidden {
            get { return FmrsApi.ReadStaticFlag (FmrsApi.HideUiField, "WindowHidden"); }
            set {
                FmrsApi.RequireMember (FmrsApi.HideUiField, "WindowHidden");
                FmrsApi.HideUiField.SetValue (null, value);
            }
        }

        // ==================================================================
        // The tracked stages
        // ==================================================================

        /// <summary>
        /// The dropped stages FMRS is tracking, as a map from vessel id to vessel name.
        /// The keys are what <see cref="JumpToVessel"/> expects.
        ///
        /// A stage appears here a fraction of a second after separation, once FMRS has
        /// written its save file. Rather than polling this in Python, block on the
        /// bridge event: conn.bridge.on_event("fmrs.dropped").
        ///
        /// A staging event that drops several stages at once - two side boosters, say -
        /// puts them ALL here at the same moment. See <see cref="DroppedSaves"/> for how
        /// to tell which came off together.
        /// </summary>
        [KRPCProperty]
        public static IDictionary<string, string> DroppedVessels {
            get {
                var result = new Dictionary<string, string> ();
                var live = FmrsApi.Live;
                var ids = ModRegistry.Field (FmrsApi.DroppedField, live) as IDictionary;
                var names = ModRegistry.Field (FmrsApi.DroppedNamesField, live) as IDictionary;
                if (ids == null)
                    return result;
                foreach (DictionaryEntry entry in ids) {
                    if (!(entry.Key is Guid))
                        continue;
                    var guid = (Guid) entry.Key;
                    var name = names != null && names.Contains (guid) ? names [guid] as string : null;
                    result [guid.ToString ()] = name ?? guid.ToString ();
                }
                return result;
            }
        }

        /// <summary>
        /// Which FMRS save file each dropped stage belongs to - id to save name.
        ///
        /// THIS IS THE BATCH KEY. FMRS writes one save per STAGING EVENT, not one per
        /// vessel: the file is named after the stage number, or "separated_N" for a
        /// detachment. Two stages that came off in the same event therefore carry the
        /// SAME string here, and stages from different events carry different ones.
        ///
        /// Why that matters: several ids being present in <see cref="DroppedVessels"/> at
        /// once does NOT mean they were dropped together. A launcher that sheds side
        /// boosters and then a core has both sets listed simultaneously. Grouping on when
        /// an id first appeared is a guess about poll timing; grouping on this value is
        /// what FMRS actually recorded.
        /// </summary>
        [KRPCProperty]
        public static IDictionary<string, string> DroppedSaves {
            get {
                var result = new Dictionary<string, string> ();
                foreach (DictionaryEntry entry in FmrsApi.DroppedTable) {
                    if (!(entry.Key is Guid))
                        continue;
                    result [((Guid) entry.Key).ToString ()] = entry.Value as string ?? string.Empty;
                }
                return result;
            }
        }

        /// <summary>
        /// Universal time at which each FMRS save file was written - save name to UT, as
        /// a decimal string.
        ///
        /// The timestamps behind <see cref="DroppedSaves"/>: since one save is one
        /// staging event, this is the moment of separation, recorded by FMRS rather than
        /// inferred from when a poll happened to notice. Join the two maps on the save
        /// name to get a separation time per stage.
        ///
        /// Returned as strings because kRPC dictionary values must all be one type and
        /// the rest of this service speaks strings; parse with float() in Python.
        /// </summary>
        [KRPCProperty]
        public static IDictionary<string, string> SeparationTimes {
            get {
                FmrsApi.RequireMember (FmrsApi.GetSaveValue, "SeparationTimes");
                var result = new Dictionary<string, string> ();
                var live = FmrsApi.Live;
                var saveFileCategory = Enum.Parse (FmrsApi.SaveCatType, "SAVEFILE");

                foreach (DictionaryEntry entry in FmrsApi.DroppedTable) {
                    var save = entry.Value as string;
                    if (string.IsNullOrEmpty (save) || result.ContainsKey (save))
                        continue;
                    var raw = FmrsApi.GetSaveValue.Invoke (live, new[] { saveFileCategory, (object) save });
                    var text = raw as string;
                    // FMRS returns the literal "False" for a key it does not hold, which
                    // would parse as neither a time nor an error. Drop it instead: an
                    // absent entry means "ask again", a zero would look like an answer.
                    if (string.IsNullOrEmpty (text) || text == "False")
                        continue;
                    result [save] = text;
                }
                return result;
            }
        }

        /// <summary>
        /// KSP's persistent id for each dropped stage - FMRS id to persistent id, as a
        /// decimal string. Survives scene reloads, and lines up with KSP.log and the .sfs.
        ///
        /// NOT a way to find the matching kRPC Vessel. kRPC's Vessel exposes no identifier
        /// at all - no Id, no Uid, no PersistentId - so there is nothing on that side to
        /// compare this against. To reach a dropped stage, pass its FMRS id to
        /// JumpToVessel: the Guid is the handle. Two uses remain, and both are real:
        /// presence, and correlation with the game's own records.
        ///
        /// Matching on the NAME is not an alternative either: two side boosters built from
        /// the same craft file carry the same name, the same part count and the same mass.
        /// To tell two loaded twins apart on the kRPC side, compare something physical -
        /// their positions differ by kilometres.
        ///
        /// A stage KSP no longer has at all is simply absent from the result rather than
        /// reported as zero - absent means destroyed or recovered, NOT out of physics
        /// range, so retrying will not help. A zero would look like an answer.
        /// </summary>
        [KRPCProperty]
        public static IDictionary<string, string> DroppedPersistentIds {
            get {
                var result = new Dictionary<string, string> ();
                var ids = FmrsApi.DroppedTable;
                if (ids.Count == 0)
                    return result;

                // One pass over FlightGlobals rather than a lookup per stage: the vessel
                // list is short, and this cannot half-fail partway through.
                //
                // The index is reused rather than reallocated. This is a property, a kRPC
                // client can stream any property, and a streamed one runs every physics
                // tick forever - so a fresh dictionary here would be steady GC pressure
                // inside FixedUpdate, which is a stutter. Safe because RPC bodies run on
                // the main thread, so there is no second caller to race with.
                var byGuid = vesselIndex;
                byGuid.Clear ();
                var all = FlightGlobals.Vessels;
                if (all != null) {
                    foreach (var vessel in all) {
                        if (vessel != null)
                            byGuid [vessel.id] = vessel;
                    }
                }

                foreach (DictionaryEntry entry in ids) {
                    if (!(entry.Key is Guid))
                        continue;
                    var guid = (Guid) entry.Key;
                    Vessel found;
                    if (byGuid.TryGetValue (guid, out found) && found != null)
                        result [guid.ToString ()] = found.persistentId.ToString ();
                }
                return result;
            }
        }

        /// <summary>
        /// Which dropped stage each kerbal is aboard - kerbal name to vessel id.
        ///
        /// FMRS tracks this so it can settle the roster when a stage is recovered or
        /// destroyed. For a script it answers the question that decides whether a booster
        /// may be written off: is anyone on it.
        /// </summary>
        [KRPCProperty]
        public static IDictionary<string, string> KerbalsAboard {
            get {
                FmrsApi.RequireMember (FmrsApi.KerbalDroppedField, "KerbalsAboard");
                var result = new Dictionary<string, string> ();
                var table = ModRegistry.Field (FmrsApi.KerbalDroppedField, FmrsApi.Live) as IDictionary;
                if (table == null)
                    return result;
                foreach (DictionaryEntry entry in table) {
                    var name = entry.Key as string;
                    if (name == null || !(entry.Value is Guid))
                        continue;
                    result [name] = ((Guid) entry.Value).ToString ();
                }
                return result;
            }
        }

        /// <summary>
        /// The id of the main mission vessel, as recorded by FMRS at launch.
        /// Empty string if FMRS has not armed a flight yet.
        ///
        /// Note the empty STRING. FMRS stores Guid.Empty rather than null when it has no
        /// main vessel, and "00000000-0000-0000-0000-000000000000" is truthy in Python -
        /// so returning it raw would make `if conn.fmrs.main_vessel_id:` pass on an
        /// unarmed flight. It is flattened here instead.
        /// </summary>
        [KRPCProperty]
        public static string MainVesselId {
            get {
                var raw = ModRegistry.Field (FmrsApi.MainVesselField, FmrsApi.Live);
                if (!(raw is Guid) || (Guid) raw == Guid.Empty)
                    return string.Empty;
                return ((Guid) raw).ToString ();
            }
        }

        /// <summary>
        /// What FMRS thinks became of a dropped stage: "NONE", "FLY", "LANDED",
        /// "DESTROYED" or "RECOVERED".
        ///
        /// A string rather than an enum on purpose: FMRS's vesselstate is a nested type,
        /// and mapping it to a KRPCEnum would pin this service to its exact integer
        /// values. The names are what a script branches on anyway.
        ///
        /// RECOVERED does NOT mean the stage is unreachable - it can still be jumped to,
        /// because <see cref="JumpToVessel"/> never reads this and the separation save
        /// survives. It means FMRS has banked it and will not pay for it twice, and that
        /// FMRS's own window no longer offers a jump button for it. See
        /// <see cref="AutoRecover"/>. <see cref="DeleteDropped"/> is the only thing that
        /// genuinely makes a stage unjumpable.
        /// </summary>
        /// <param name="vesselId">Vessel id, as given by <see cref="DroppedVessels"/>.</param>
        [KRPCProcedure]
        public static string VesselState (string vesselId)
        {
            FmrsApi.RequireMember (FmrsApi.GetVesselState, "VesselState");
            var target = FmrsApi.ParseId (vesselId);
            var raw = FmrsApi.GetVesselState.Invoke (FmrsApi.Live, new object[] { target });
            return raw == null ? "NONE" : raw.ToString ();
        }

        /// <summary>
        /// Overwrite what FMRS thinks became of a dropped stage. Returns false if FMRS is
        /// not tracking that id - it will not add one.
        ///
        /// Bookkeeping repair. Note what this does NOT fix: a RECOVERED stage was already
        /// jumpable, so this is not what makes a replay possible. What it restores is
        /// FMRS's own window, which hides the jump button for any stage in LANDED or
        /// RECOVERED state - useful if you drive the mod by hand as well as by script.
        ///
        /// It also un-banks a stage for accounting purposes: FMRS refuses to recover the
        /// same stage twice, and this is the only way to undo that. Nothing validates the
        /// transition, so it will happily describe a state the stage is not in - use it
        /// to correct bookkeeping, not to lie.
        /// </summary>
        /// <param name="vesselId">Vessel id, as given by <see cref="DroppedVessels"/>.</param>
        /// <param name="state">One of "NONE", "FLY", "LANDED", "DESTROYED", "RECOVERED". Case-insensitive.</param>
        [KRPCProcedure]
        public static bool SetVesselState (string vesselId, string state)
        {
            FmrsApi.RequireMember (FmrsApi.SetVesselState, "SetVesselState");
            var target = FmrsApi.ParseId (vesselId);

            object parsed = null;
            foreach (var name in Enum.GetNames (FmrsApi.VesselStateType)) {
                if (string.Equals (name, state, StringComparison.OrdinalIgnoreCase)) {
                    parsed = Enum.Parse (FmrsApi.VesselStateType, name);
                    break;
                }
            }
            if (parsed == null)
                throw new ArgumentException (
                    "unknown vessel state '" + state + "' - this FMRS offers: "
                    + string.Join (", ", Enum.GetNames (FmrsApi.VesselStateType)));

            var raw = FmrsApi.SetVesselState.Invoke (FmrsApi.Live, new object[] { target, parsed });
            return raw is bool && (bool) raw;
        }

        // ==================================================================
        // Jumping
        // ==================================================================

        /// <summary>
        /// Jump to a dropped stage. Reloads the flight scene and blocks until the target
        /// vessel is active and unpacked - typically 5 to 20 seconds depending on part
        /// count.
        ///
        /// Also valid on the stage you are ALREADY flying: that is FMRS's "return to
        /// separation" button, and it reloads the save frozen at the moment of
        /// separation. A landing can therefore be replayed from bit-identical initial
        /// conditions without re-flying the ascent.
        ///
        /// ALL kRPC object handles obtained before the call - vessels, parts, modules -
        /// and all streams must be considered dead afterwards. Remove the streams first,
        /// then re-read space_center.active_vessel and rebuild everything from it.
        /// </summary>
        /// <param name="vesselId">Vessel id, as given by <see cref="DroppedVessels"/>.</param>
        /// <param name="saveLanded">
        /// Save the current state into the main-mission save before jumping. Keep this
        /// true when leaving the main mission: with false, FMRS skips that save and
        /// jumping back later resumes from an older state. When re-jumping to the stage
        /// you are already flying there is no main mission to preserve, so false is the
        /// right value - it is what FMRS's own "return to separation" button passes.
        /// </param>
        [KRPCProcedure]
        public static void JumpToVessel (string vesselId, bool saveLanded = true)
        {
            FmrsApi.Require ();
            var target = FmrsApi.ParseId (vesselId);

            // FMRS checks nothing: an unknown id makes its load fail silently and the
            // method returns having done nothing at all. Fail loudly here instead.
            if (!FmrsApi.TrackedIds ().Contains (target.ToString ()))
                throw new ArgumentException (
                    "FMRS is not tracking a dropped stage with id " + vesselId);
            if (JumpInProgress)
                throw new InvalidOperationException ("an FMRS jump is already in progress");

            var live = FmrsApi.Live;
            var before = FlightGlobals.ActiveVessel;
            FmrsApi.JumpToVesselGuid.Invoke (live, new object[] { target, saveLanded });
            YieldUntilArrival (target, before);
        }

        /// <summary>
        /// Jump back to the main mission vessel, resuming it where it was left.
        /// Same blocking behaviour and same handle invalidation as
        /// <see cref="JumpToVessel"/>.
        ///
        /// Does nothing, quickly, when you are already on the main mission. That is worth
        /// stating because the cost of getting it wrong is not an exception: FMRS's own
        /// entry point opens with an unconditional early return when it is not flying a
        /// dropped stage, so nothing reloads, the wait for a scene change never ends, and
        /// the call used to sit there for the full 90-second timeout before throwing.
        /// Calling this defensively is exactly what the guard-rail advice recommends, so
        /// it has to be cheap.
        /// </summary>
        [KRPCProcedure]
        public static void JumpToMain ()
        {
            FmrsApi.Require ();
            if (JumpInProgress)
                throw new InvalidOperationException ("an FMRS jump is already in progress");
            if (!SwitchedToDropped)
                return;

            var live = FmrsApi.Live;
            var raw = ModRegistry.Field (FmrsApi.MainVesselField, live);
            if (!(raw is Guid) || (Guid) raw == Guid.Empty)
                throw new InvalidOperationException (
                    "FMRS has no main vessel recorded - was the flight armed at launch?");
            var target = (Guid) raw;

            var before = FlightGlobals.ActiveVessel;
            // The string argument is ignored by FMRS; it always loads the main save.
            // "Main" is what its own button passes, so that is what we pass.
            FmrsApi.JumpToVesselMain.Invoke (live, new object[] { "Main" });
            YieldUntilArrival (target, before);
        }

        /// <summary>
        /// Revert the whole flight to the moment FMRS started recording, and block until
        /// the scene is back.
        ///
        /// This is FMRS's own "Revert To Launch" button. It loads the save FMRS wrote
        /// before launch, so it works even after a jump, when KSP's own revert is gone -
        /// which is the point. The tracked stages of the reverted flight are discarded
        /// with it.
        ///
        /// Same handle invalidation as <see cref="JumpToVessel"/>. Throws if FMRS never
        /// armed this flight, since there is then no pre-launch save to go back to.
        /// </summary>
        [KRPCProcedure]
        public static void RevertToLaunch ()
        {
            FmrsApi.Require ();
            FmrsApi.RequireMember (FmrsApi.JumpToVesselSave, "RevertToLaunch");
            if (JumpInProgress)
                throw new InvalidOperationException ("an FMRS jump is already in progress");
            if (!HasLaunched)
                throw new InvalidOperationException (
                    "FMRS has not recorded a launch for this flight - nothing to revert to");

            var live = FmrsApi.Live;
            var raw = ModRegistry.Field (FmrsApi.MainVesselField, live);
            if (!(raw is Guid) || (Guid) raw == Guid.Empty)
                throw new InvalidOperationException ("FMRS has no main vessel recorded");
            var target = (Guid) raw;

            var before = FlightGlobals.ActiveVessel;
            FmrsApi.JumpToVesselSave.Invoke (live, new object[] { target, "before_launch" });
            YieldUntilArrival (target, before);
        }

        /// <summary>
        /// Hand kRPC a continuation that waits for the scene reload to complete.
        ///
        /// FMRS defers the actual scene load through a one-second coroutine, so the call
        /// that started it returns long before anything happens. A yielded RPC is not
        /// re-checked against the game scene, so the continuation survives the reload -
        /// which is exactly what is needed and also why it must bound itself in real
        /// time.
        /// </summary>
        static void YieldUntilArrival (Guid target, object before)
        {
            var deadline = Time.realtimeSinceStartup + JumpTimeoutSeconds;
            throw new YieldException<Action> (() => WaitForArrival (target, before, 0, deadline));
        }

        /// <summary>
        /// Yield a physics tick at a time until the target vessel is active and unpacked,
        /// then a few more to let the scene settle.
        ///
        /// TWO conditions, and the second one is what makes a replay work.
        ///
        /// Waiting on the target's id is what makes a jump to a DIFFERENT vessel
        /// reliable: FlightGlobals.ActiveVessel still points at the OLD vessel for about
        /// a second after the call, so any "is the scene ready" test would pass
        /// immediately.
        ///
        /// But on a re-jump to the stage we are ALREADY flying, the id matches on the
        /// very first tick and the vessel is unpacked - we just landed it - so that test
        /// alone returns before FMRS has even started loading, and the caller would then
        /// drive a scene that is about to be destroyed under it. The fix is to wait on
        /// object IDENTITY: the reload destroys the Vessel and builds a new one carrying
        /// the same Guid, so ReferenceEquals is false only once the new scene is really
        /// there. ReferenceEquals rather than != on purpose - Unity's operator== reports
        /// a destroyed object as equal to null and would hide the transition.
        ///
        /// Physics ticks do not run during the loading screen, so a tick counter cannot
        /// expire while the game is loading. Hence the separate real-time deadline.
        /// </summary>
        static void WaitForArrival (Guid target, object before, int settled, float deadline)
        {
            if (Time.realtimeSinceStartup > deadline)
                throw new InvalidOperationException (
                    "FMRS jump timed out after " + JumpTimeoutSeconds + " s - the target vessel "
                    + target + " never became active");

            var active = FlightGlobals.ActiveVessel;
            if (active == null || active.packed || active.id != target
                || ReferenceEquals (active, before))
                throw new YieldException<Action> (() => WaitForArrival (target, before, 0, deadline));

            if (settled < SettleTicks)
                throw new YieldException<Action> (() => WaitForArrival (target, before, settled + 1, deadline));
        }

        // ==================================================================
        // Recovery and cleanup
        // ==================================================================

        /// <summary>
        /// Recover the dropped stage currently being flown, banking its funds and science.
        ///
        /// This is FMRS's own recovery path, the one behind recovering a landed booster
        /// from the window, and it does the whole settlement: refunds the parts, credits
        /// any science aboard, completes contracts, and records it all in the ledger
        /// <see cref="RecoveryReport"/> reads.
        ///
        /// IT DOES NOT RETURN YOU TO THE MAIN MISSION. You stay on the dropped stage.
        /// FMRS's own window recovers and kicks back in one gesture, but the kick comes
        /// from a different handler, and this is only the settlement half. Call
        /// <see cref="JumpToMain"/> yourself, or set <see cref="KickToMain"/>.
        ///
        /// IT RECOVERS EVERY LOADED STAGE OF THE CURRENT SUB-SAVE, not only the one you are
        /// flying. Two boosters dropped in the same staging event and still loaded are
        /// settled together.
        ///
        /// Throws when <see cref="SwitchedToDropped"/> is false. Costs real time: FMRS
        /// writes a save and reads two back, synchronously, on the main thread - hundreds
        /// of milliseconds on a large career save, and the game is frozen for all of it.
        /// </summary>
        /// <param name="force">
        /// Recover even when <see cref="AutoRecover"/> is off - it is ORed with that
        /// setting, which is exactly what the stock Recover Vessel button passes.
        ///
        /// It does NOT override the landed test: FMRS only banks a protovessel that is
        /// landed or splashed, and only on Kerbin, whatever this says. An airborne stage
        /// is silently left in FLY state.
        /// </param>
        [KRPCProcedure]
        public static void RecoverCurrent (bool force = true)
        {
            FmrsApi.RequireMember (FmrsApi.SaveLandedVessel, "RecoverCurrent");
            if (!SwitchedToDropped)
                throw new InvalidOperationException (
                    "not flying a dropped stage - FMRS only recovers the stage you are on");
            FmrsApi.SaveLandedVessel.Invoke (FmrsApi.Live, new object[] { true, force });
        }

        /// <summary>
        /// Stop tracking one dropped stage, making it unjumpable. This is the red X in
        /// FMRS's window.
        ///
        /// The .sfs file is NOT deleted - FMRS never deletes one, there is no File.Delete
        /// anywhere in the mod. What goes is the index entry, and losing it is what makes
        /// the stage unreachable: the save-name lookup then returns the string "False",
        /// LoadGame fails on it, and the jump quietly does nothing. The distinction matters
        /// if you ever go looking on disk for a stage you discarded.
        ///
        /// Not undoable through this API either way. Use it to keep a long campaign's list
        /// short, or to discard a booster that was written off.
        /// </summary>
        /// <param name="vesselId">Vessel id, as given by <see cref="DroppedVessels"/>.</param>
        [KRPCProcedure]
        public static void DeleteDropped (string vesselId)
        {
            FmrsApi.RequireMember (FmrsApi.DeleteDropped, "DeleteDropped");
            var target = FmrsApi.ParseId (vesselId);
            if (!FmrsApi.TrackedIds ().Contains (target.ToString ()))
                throw new ArgumentException (
                    "FMRS is not tracking a dropped stage with id " + vesselId);
            FmrsApi.DeleteDropped.Invoke (FmrsApi.Live, new object[] { target });
        }

        /// <summary>
        /// Stop tracking every dropped stage, leaving FMRS otherwise running.
        ///
        /// As with <see cref="DeleteDropped"/>, the .sfs files stay on disk; it is the
        /// index that is emptied, and that is what makes the stages unreachable.
        ///
        /// Lighter than <see cref="Reset"/>, which also closes the plugin for the flight.
        /// Use this between test articles in a campaign that keeps flying.
        /// </summary>
        [KRPCProcedure]
        public static void DeleteAllDropped ()
        {
            FmrsApi.RequireMember (FmrsApi.DeleteAllDropped, "DeleteAllDropped");
            FmrsApi.DeleteAllDropped.Invoke (FmrsApi.Live, null);
        }

        /// <summary>
        /// Reset FMRS: forget the tracked stages and close the plugin for this flight.
        ///
        /// For a campaign that was interrupted. FMRS's bookkeeping lives under GameData
        /// and is shared by EVERY savegame of the install, so a killed run leaves state
        /// the next KSP session inherits. This is FMRS's own close routine, the same one
        /// behind its "Plugin will be reset!" confirmation.
        ///
        /// CALL IT FROM FLIGHT. Outside the flight scene it throws part of the way through,
        /// having already disabled FMRS and discarded the tracked stages but not finished
        /// closing - FMRS's close routine touches a toolbar object that only the flight
        /// addon ever creates. Which is unfortunate, because the space centre is where you
        /// would naturally reach for it.
        /// </summary>
        [KRPCProcedure]
        public static void Reset ()
        {
            FmrsApi.RequireMember (FmrsApi.CloseFmrs, "Reset");
            FmrsApi.CloseFmrs.Invoke (FmrsApi.Live, null);
        }

        /// <summary>
        /// What FMRS's recoveries have credited so far, as "category\tkey\tvalue" rows.
        ///
        /// FMRS keeps a ledger on disk of everything a recovery settled, so that the main
        /// mission - which is resumed from a save written before the recovery happened -
        /// can have it applied when you return. This exposes that ledger, which is the
        /// only place the outcome of a booster recovery is stated as a number rather than
        /// inferred by diffing the funds counter.
        ///
        /// The categories FMRS writes, and what the key and value mean in each:
        /// <list type="bullet">
        /// <item><description>fund / "add" / funds refunded, as a float</description></item>
        /// <item><description>science / subject id / data amount recovered</description></item>
        /// <item><description>science_sent / subject id / science credited</description></item>
        /// <item><description>contract / "complete" / contract id</description></item>
        /// <item><description>kerbal / "kill" / reputation lost</description></item>
        /// <item><description>building / "destroyed" / building name</description></item>
        /// <item><description>message / a heading / the text FMRS would have shown</description></item>
        /// <item><description>warning / "FMRS Info:" / a scene-change warning</description></item>
        /// </list>
        ///
        /// THE LEDGER IS CUMULATIVE FOR THE WHOLE MISSION, not per recovery. It is never
        /// cleared on settlement - FMRS's own Clear() call at that point is commented out -
        /// only at prelaunch. So recover three boosters and the third report contains all
        /// three, and what you want per booster is the difference between two reads.
        ///
        /// A consequence worth knowing, and it is FMRS's bug rather than ours: the ledger
        /// is re-applied about two seconds into every flight-scene start where you are not
        /// on a dropped stage. Return to the main mission twice in one flight and
        /// everything already banked is credited a second time.
        ///
        /// DO NOT PUT A STREAM ON THIS. With the default argument it reads a file from
        /// disk, and a kRPC stream re-runs its call every physics tick, forever.
        /// </summary>
        /// <param name="reread">
        /// Re-read the ledger from disk first. Needed when the recovery happened in a
        /// different scene from the one you are asking in, which is the usual case.
        /// </param>
        [KRPCProcedure]
        public static IList<string> RecoveryReport (bool reread = true)
        {
            FmrsApi.RequireMember (FmrsApi.RecoverValuesField, "RecoveryReport");
            FmrsApi.RequireMember (FmrsApi.RecoverCatField, "RecoveryReport");

            var live = FmrsApi.Live;
            if (reread && FmrsApi.ReadRecoverFile != null) {
                try {
                    FmrsApi.ReadRecoverFile.Invoke (live, null);
                } catch (Exception e) {
                    throw new InvalidOperationException (
                        "FMRS could not read its recovery ledger: " + e.Message);
                }
            }

            var rows = new List<string> ();
            var entries = ModRegistry.Field (FmrsApi.RecoverValuesField, live) as IEnumerable;
            if (entries == null)
                return rows;

            foreach (var entry in entries) {
                if (entry == null)
                    continue;
                var category = FmrsApi.RecoverCatField.GetValue (entry) as string;
                var key = FmrsApi.RecoverKeyField != null
                    ? FmrsApi.RecoverKeyField.GetValue (entry) as string : null;
                var value = FmrsApi.RecoverValueField != null
                    ? FmrsApi.RecoverValueField.GetValue (entry) as string : null;
                rows.Add (Clean (category) + "\t" + Clean (key) + "\t" + Clean (value));
            }
            return rows;
        }

        /// <summary>
        /// Total funds FMRS's ledger says have been refunded by recovery, or 0 if it says
        /// nothing.
        ///
        /// The one number most campaigns want out of <see cref="RecoveryReport"/>,
        /// summed here so a script does not have to parse rows to get it.
        /// </summary>
        /// <param name="reread">Re-read the ledger from disk first. See <see cref="RecoveryReport"/>.</param>
        [KRPCProcedure]
        public static double RecoveredFunds (bool reread = true)
        {
            var total = 0.0;
            foreach (var row in RecoveryReport (reread)) {
                var parts = row.Split ('\t');
                if (parts.Length < 3 || parts [0] != "fund")
                    continue;
                double amount;
                if (double.TryParse (parts [2], System.Globalization.NumberStyles.Float,
                                     System.Globalization.CultureInfo.InvariantCulture, out amount))
                    total += amount;
            }
            return total;
        }

        /// <summary>Rows are tab separated, so a tab or newline inside a field has to go.</summary>
        static string Clean (string text)
        {
            if (string.IsNullOrEmpty (text))
                return string.Empty;
            return text.Replace ('\t', ' ').Replace ('\n', ' ').Replace ('\r', ' ');
        }
    }
}
