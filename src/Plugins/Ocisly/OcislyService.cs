using System;
using System.Collections;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using KRPC.Service;
using KRPC.Service.Attributes;

namespace KRPC.Bridge.Ocisly
{
    /// <summary>
    /// Remote control of OCISLY (OfCourseIStillLoveYou), the mod that streams Hullcam VDS
    /// cameras out of the game.
    ///
    /// WHAT IT IS FOR. OCISLY subscribes to no scene GameEvent and uses no
    /// DontDestroyOnLoad, so a flight scene reload - an FMRS jump, a quickload, a revert -
    /// silently drops every camera it was tracking and nothing brings them back. This
    /// service re-opens them and turns streaming back on, automatically by default, so it
    /// works even when the jump was made by hand from FMRS's own window with no script
    /// connected.
    ///
    /// WHAT IT CANNOT FIX. The camera identifier OCISLY puts on the wire is derived from
    /// Unity's GetInstanceID, so it is different after every reload no matter what
    /// happens in-game. What this service can do is make the camera NAME stable and
    /// unique - see <see cref="NameCameras"/> - so that whatever consumes the stream has
    /// something reliable to key on.
    /// </summary>
    [KRPCService (Name = "OCISLY", GameScene = GameScene.All)]
    public static class OcislyService
    {
        /// <summary>Physics ticks between opening the cameras and enabling their streams.</summary>
        const int RepaintTicks = 25;

        /// <summary>
        /// Whether OCISLY is installed and its API resolved. Every other member throws
        /// when this is <c>false</c>.
        /// </summary>
        [KRPCProperty]
        public static bool Available {
            get { return OcislyApi.Resolved; }
        }

        /// <summary>
        /// What type resolution found, member by member. Read this when
        /// <see cref="Available"/> is false: it says exactly which member is missing,
        /// without digging through KSP.log. This is the mod whose symbol names are the
        /// least stable of the three, so this is the line worth quoting in a bug report.
        /// </summary>
        [KRPCProperty]
        public static string Diagnostics {
            get { return OcislyApi.Report; }
        }

        /// <summary>Round-trip check that the service is loaded and callable. Returns "pong".</summary>
        [KRPCProcedure]
        public static string Ping ()
        {
            return "pong";
        }

        /// <summary>
        /// Whether OCISLY's in-game window is enabled. Cameras can only be opened while it
        /// is - the mod gates its own open-camera call on it - which is also why the
        /// bridge re-arms the cameras BEFORE hiding the HUD, never after.
        /// </summary>
        [KRPCProperty]
        public static bool WindowOpen {
            get {
                OcislyApi.Require ();
                try {
                    return (bool) OcislyApi.GuiEnabledField.GetValue (null);
                } catch (Exception) {
                    return false;
                }
            }
            set {
                OcislyApi.Require ();
                OcislyApi.GuiEnabledField.SetValue (null, value);
            }
        }

        /// <summary>
        /// Every Hullcam VDS camera on every loaded vessel, as "&lt;vessel&gt;.&lt;camera&gt;".
        ///
        /// OCISLY scans all loaded vessels, not just the active one, so a dropped booster
        /// still within physics range shows up here. These are the strings
        /// <see cref="Rearm"/> filters against.
        /// </summary>
        [KRPCProperty]
        public static IList<string> Hullcams {
            get {
                var result = new List<string> ();
                foreach (var cam in OcislyApi.AllHullcams ()) {
                    var key = OcislyApi.KeyOf (cam);
                    if (key != null)
                        result.Add (key);
                }
                return result;
            }
        }

        /// <summary>
        /// The cameras OCISLY currently tracks, as a map from its internal camera id to
        /// the camera name.
        ///
        /// The ids are Unity instance ids and change on every scene reload, which is
        /// precisely why anything downstream should key on the name instead. A camera
        /// that has been opened but not yet painted reports "(not painted yet)" - OCISLY
        /// only fills the name in during a GUI repaint.
        /// </summary>
        [KRPCProperty]
        public static IDictionary<string, string> Cameras {
            get {
                var result = new Dictionary<string, string> ();
                foreach (DictionaryEntry entry in OcislyApi.TrackedCameras) {
                    var name = OcislyApi.NameOf (entry.Value);
                    result [Convert.ToString (entry.Key)] = name ?? "(not painted yet)";
                }
                return result;
            }
        }

        /// <summary>
        /// The names of the cameras currently streaming. Read this before an FMRS jump
        /// and feed it back to <see cref="Rearm"/> afterwards to restore the same set -
        /// though with <see cref="AutoRearm"/> left on, the bridge already does exactly
        /// that by itself.
        /// </summary>
        [KRPCProperty]
        public static IList<string> Streaming {
            get {
                var result = new List<string> ();
                foreach (DictionaryEntry entry in OcislyApi.TrackedCameras) {
                    if (!OcislyApi.IsStreaming (entry.Value))
                        continue;
                    var name = OcislyApi.NameOf (entry.Value);
                    if (name != null)
                        result.Add (name);
                }
                return result;
            }
        }

        // ==================================================================
        // Automatic restore
        // ==================================================================

        /// <summary>
        /// Whether the bridge re-opens the cameras by itself after every flight scene
        /// reload. On by default, so it works with no script involved at all.
        /// </summary>
        [KRPCProperty]
        public static bool AutoRearm {
            get { return AutoRestoreAddon.AutoRearm; }
            set { AutoRestoreAddon.AutoRearm = value; }
        }

        /// <summary>
        /// The camera names the bridge will restore on the next scene load, as captured
        /// when the last flight scene was torn down. Empty if nothing was on air.
        ///
        /// That capture happens on onGameSceneLoadRequested, while the OLD scene is still
        /// intact - the only moment the information exists at all.
        /// </summary>
        [KRPCProperty]
        public static IList<string> Remembered {
            get { return new List<string> (AutoRestoreAddon.Snapshot); }
        }

        /// <summary>What the last automatic scene restore actually did: cameras, then HUD.</summary>
        [KRPCProperty]
        public static string LastRestore {
            get { return AutoRestoreAddon.LastRestore; }
        }

        /// <summary>
        /// Seconds to let a reloaded scene settle before touching anything. Default 2.
        ///
        /// This dominates how long the HUD stays visible after a jump. Lower it for a
        /// tighter cinematic cut; too low and the parts are not all loaded yet, so a
        /// camera can be missed.
        /// </summary>
        [KRPCProperty]
        public static float RestoreDelay {
            get { return AutoRestoreAddon.SettleDelay; }
            set { AutoRestoreAddon.SettleDelay = Sane (value, "restore_delay"); }
        }

        /// <summary>
        /// Hide the HUD automatically once each flight scene has settled, as if F2 had
        /// been pressed. Off by default. Set it once and it applies to every reload,
        /// which is the point: an FMRS jump always brings the HUD back.
        ///
        /// It lives on this service rather than on Bridge because the ORDER is the whole
        /// difficulty. OCISLY refuses to open a camera while its UI is hidden, and it
        /// only fills in a camera's name during a repaint - so the cameras must be
        /// re-opened, and given a repaint, before the HUD goes away. Hiding first
        /// silently produces cameras that never stream.
        /// </summary>
        [KRPCProperty]
        public static bool HideUiOnSceneLoad {
            get { return AutoRestoreAddon.HideUiOnSceneLoad; }
            set { AutoRestoreAddon.HideUiOnSceneLoad = value; }
        }

        /// <summary>Seconds between the cameras coming back and the HUD being hidden.</summary>
        [KRPCProperty]
        public static float HideUiDelay {
            get { return AutoRestoreAddon.HideUiDelay; }
            set { AutoRestoreAddon.HideUiDelay = Sane (value, "hide_ui_delay"); }
        }

        /// <summary>
        /// Reject a delay Unity cannot wait for.
        ///
        /// WaitForSeconds(NaN) never completes, so a client writing NaN would silently
        /// kill the restore coroutine for every scene from then on - no error, no log
        /// line, the cameras just stop coming back. Refusing it is much kinder than
        /// diagnosing it.
        /// </summary>
        static float Sane (float seconds, string name)
        {
            if (float.IsNaN (seconds) || float.IsInfinity (seconds) || seconds < 0f)
                throw new ArgumentException (
                    name + " must be a finite number of seconds, not " + seconds);
            return seconds > 60f ? 60f : seconds;
        }

        /// <summary>
        /// Whether the bridge renames duplicate cameras on every scene load. On by
        /// default: without it, two identical Hullcam parts are indistinguishable on the
        /// wire and their feeds can swap between flights. See <see cref="NameCameras"/>.
        /// </summary>
        [KRPCProperty]
        public static bool DisambiguateNames {
            get { return AutoRestoreAddon.DisambiguateNames; }
            set { AutoRestoreAddon.DisambiguateNames = value; }
        }

        // ==================================================================
        // Manual control
        // ==================================================================

        /// <summary>
        /// Give every Hullcam a name that is unique and stable, and return how many were
        /// renamed.
        ///
        /// A Hullcam's cameraName comes from the part config, not the instance, so two
        /// cameras of the same part type on one vessel report an identical name and
        /// anything downstream has to invent a tie-break from frame arrival order - which
        /// can swap between flights, exchanging two feeds mid-broadcast. This appends an
        /// ordinal derived from part.flightID, which is persistent and survives an FMRS
        /// jump, plus the flightID itself after an "@" so a returning camera is
        /// recognisable with certainty.
        ///
        /// Idempotent, and it never touches a save file - cameraName is not persisted.
        /// Run it before opening the cameras so the very first frame already carries the
        /// final name.
        /// </summary>
        [KRPCProcedure]
        public static int NameCameras ()
        {
            return CameraNames.Apply ();
        }

        /// <summary>
        /// Re-open the matching Hullcam cameras in OCISLY and turn their streams back on.
        ///
        /// Blocks for about half a second: OCISLY only fills in a camera's name during a
        /// GUI repaint, and enabling a stream before that pushes a null name into
        /// protobuf, which throws inside the mod. Call it after a scene reload has
        /// settled.
        ///
        /// Every extra camera costs three full scene renders per cycle, so filter down to
        /// the ones actually on air rather than arming everything.
        ///
        /// BRINGS THE HUD BACK UP FOR THE DURATION, and puts it back afterwards. OCISLY
        /// refuses to open a camera while the game UI is hidden - its own toggle rides on
        /// the GameEvents that F2 fires - and it refuses silently, so without this the
        /// call reports success and arms nothing. That case is not exotic: with
        /// <see cref="HideUiOnSceneLoad"/> on, which is the point of a recorded flight,
        /// the scene ends up in exactly that state and the obvious next move is to arm the
        /// booster that just separated. The cost is that the HUD flashes for about half a
        /// second, which will be on the recording.
        /// </summary>
        /// <param name="filter">
        /// Comma-separated substrings matched case-insensitively against
        /// "&lt;vessel&gt;.&lt;camera&gt;". Empty or "*" arms every camera found.
        ///
        /// Filter on the "@flightID" token, which is what <see cref="Remembered"/> gives
        /// you. NEITHER half of "&lt;vessel&gt;.&lt;camera&gt;" survives a jump: the vessel half
        /// obviously changes, and the camera half changes too, because the ordinal in
        /// "Aerocam DN 2" is only appended while several cameras share a name on the same
        /// vessel - separate the stack and it becomes "Aerocam DN". Only the token holds,
        /// because part.flightID is persistent.
        /// </param>
        [KRPCProcedure]
        public static void Rearm (string filter = "")
        {
            // Show first, open second: see the note above. Hud.Show returns false when it
            // was already up, which is exactly the "do not touch it" signal we want.
            var weRaisedIt = Hud.Show ();
            OpenMatching (filter);
            throw new YieldException<Action> (() => EnableStreamsYielding (filter, 0, weRaisedIt));
        }

        /// <summary>Open every matching camera OCISLY is not already tracking. Returns how many were opened.</summary>
        internal static int OpenMatching (string filter)
        {
            OcislyApi.Require ();

            var gui = OcislyApi.GuiFetchField.GetValue (null);
            if (gui == null)
                throw new InvalidOperationException (
                    "OCISLY's GUI object does not exist in this scene - is this a flight scene?");

            // OpenCameraInstance is gated on TWO things inside the mod: this flag, which we
            // can set, and a private instance field that follows the game's own onHideUI /
            // onShowUI events, which we cannot. Callers are expected to have raised the HUD
            // already - Rearm does. Warn rather than throw, because the automatic restore
            // path calls this while the HUD is up and must not be made to fail here.
            OcislyApi.GuiEnabledField.SetValue (null, true);
            if (!Hud.Visible)
                BridgeLog.Error ("OCISLY", "rearm: the game UI is hidden, and OCISLY refuses "
                                 + "to open a camera in that state - expect 0 opened");

            var opened = 0;
            var ignored = 0;
            foreach (var cam in OcislyApi.AllHullcams ()) {
                var key = OcislyApi.KeyOf (cam);
                if (!ModRegistry.MatchesFilter (key, filter))
                    continue;
                var unityObject = cam as UnityEngine.Object;
                var id = unityObject != null ? unityObject.GetInstanceID () : 0;
                if (unityObject != null && OcislyApi.TrackedCameras.Contains (id))
                    continue;
                try {
                    OcislyApi.OpenCameraInstance.Invoke (gui, new object[] { cam });
                } catch (Exception e) {
                    BridgeLog.Error ("OCISLY", "OpenCameraInstance failed for " + key + ": " + e.Message);
                    continue;
                }
                // Confirm instead of assuming. The mod returns quietly when its own UI
                // toggle is down, so "no exception" is not "it worked" - counting the call
                // rather than the result is what let a silent failure report success.
                if (unityObject == null || OcislyApi.TrackedCameras.Contains (id))
                    opened++;
                else
                    ignored++;
            }
            BridgeLog.Info ("OCISLY", "rearm: opened " + opened + " camera(s)"
                            + (ignored > 0 ? ", " + ignored + " IGNORED by the mod" : "")
                            + " for filter '" + (string.IsNullOrEmpty (filter) ? "*" : filter) + "'");
            return opened;
        }

        /// <summary>Turn streaming on for every matching tracked camera whose name is painted.</summary>
        internal static int EnableMatching (string filter)
        {
            OcislyApi.Require ();
            var enabled = 0;
            var skipped = 0;
            foreach (DictionaryEntry entry in OcislyApi.TrackedCameras) {
                var name = OcislyApi.NameOf (entry.Value);
                if (name == null) {
                    // Not painted yet: enabling now would push a null name into protobuf.
                    skipped++;
                    continue;
                }
                if (!ModRegistry.MatchesFilter (name, filter))
                    continue;
                if (OcislyApi.IsStreaming (entry.Value))
                    continue;
                try {
                    OcislyApi.SetStreaming (entry.Value, true);
                    enabled++;
                } catch (Exception e) {
                    BridgeLog.Error ("OCISLY", "could not enable streaming on " + name + ": " + e.Message);
                }
            }
            BridgeLog.Info ("OCISLY", "rearm: streaming enabled on " + enabled + " camera(s), "
                            + skipped + " skipped (name not painted yet)");
            return enabled;
        }

        /// <summary>
        /// Identifiers for the cameras currently on air, reduced to the ONLY part of the
        /// name that survives a separation.
        ///
        /// Neither half of "&lt;vessel&gt;.&lt;camera&gt;" is stable across an FMRS jump, and the
        /// camera half is the one that fools you. The vessel half obviously changes - the
        /// camera ends up on the dropped booster. But the camera half changes too,
        /// because the ordinal in "Aerocam DN 2" is only added when several cameras share
        /// a name ON THE SAME VESSEL. Separate the stack and each camera is suddenly alone
        /// on its own vessel, so the next naming pass drops the ordinal and "Aerocam DN 2"
        /// becomes "Aerocam DN". A filter remembered before the jump then matches nothing,
        /// and the restore silently arms zero cameras.
        ///
        /// What does survive is the "@flightID" identity token, because part.flightID is
        /// persistent and rides through the save-and-load. That is what this returns, and
        /// it is the whole reason the token exists.
        ///
        /// Falls back to the camera half when there is no token - which happens only when
        /// Hullcam's cameraName field did not resolve, so nothing could be renamed anyway.
        /// </summary>
        internal static List<string> StreamingCameraNames ()
        {
            var result = new List<string> ();
            if (!OcislyApi.Resolved)
                return result;
            try {
                foreach (DictionaryEntry entry in OcislyApi.TrackedCameras) {
                    if (!OcislyApi.IsStreaming (entry.Value))
                        continue;
                    var name = OcislyApi.NameOf (entry.Value);
                    if (string.IsNullOrEmpty (name))
                        continue;

                    var at = name.LastIndexOf (CameraNames.IdentitySeparator, StringComparison.Ordinal);
                    string token;
                    if (at >= 0 && at + 1 < name.Length) {
                        // "@1898164639" - stable, and unique per part.
                        token = name.Substring (at);
                    } else {
                        var dot = name.LastIndexOf ('.');
                        token = dot >= 0 && dot + 1 < name.Length ? name.Substring (dot + 1) : name;
                    }
                    if (!result.Contains (token))
                        result.Add (token);
                }
            } catch (Exception) {
                // The scene is tearing down. Whatever we already collected is what we keep.
            }
            return result;
        }

        static void EnableStreamsYielding (string filter, int tick, bool restoreHidden)
        {
            if (tick < RepaintTicks)
                throw new YieldException<Action> (() => EnableStreamsYielding (filter, tick + 1, restoreHidden));
            try {
                EnableMatching (filter);
            } finally {
                // In a finally, because leaving the HUD up after a throw would silently
                // undo the caller's hide_ui and put the toolbar back in the recording.
                if (restoreHidden)
                    Hud.Hide ();
            }
        }
    }
}
