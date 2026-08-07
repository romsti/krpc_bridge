using System;
using System.Collections;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using UnityEngine;

namespace KRPC.Bridge.Ocisly
{
    /// <summary>Registration. Hands Core the resolver and gets out of the way.</summary>
    [KSPAddon (KSPAddon.Startup.Instantly, true)]
    public sealed class OcislyAddon : MonoBehaviour
    {
        void Awake ()
        {
            ModRegistry.Register ("OCISLY", OcislyApi.Resolve);
        }
    }

    /// <summary>
    /// Puts the flight scene back the way it was after a reload: camera streams on air,
    /// and optionally the HUD hidden again.
    ///
    /// WHY THIS IS IN THE GAME AND NOT IN PYTHON. It has to work when the FMRS jump was
    /// made by hand from the mod's own window with no script connected at all. A script
    /// can only restore what it was there to observe.
    ///
    /// ORDER MATTERS, and it is the reason this is one coroutine rather than three
    /// independent features. OCISLY gates its open-camera call on its own UI being
    /// visible, and it only fills in a camera's name during a GUI repaint - so the
    /// cameras must be renamed, re-opened, and given a repaint, BEFORE the HUD is hidden.
    /// Hiding first silently produces cameras that never stream, with no error anywhere.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Flight, false)]
    public sealed class AutoRestoreAddon : MonoBehaviour
    {
        /// <summary>Re-open and re-enable the cameras that were on air before the reload.</summary>
        internal static bool AutoRearm = true;

        /// <summary>
        /// Rename duplicate Hullcams so each has a unique, stable name. On by default:
        /// two cameras of the same part type report an identical cameraName, and any
        /// tie-break invented downstream can swap their feeds between flights.
        /// </summary>
        internal static bool DisambiguateNames = true;

        /// <summary>Hide the HUD once the scene has settled, as if F2 had been pressed.</summary>
        internal static bool HideUiOnSceneLoad;

        /// <summary>Seconds to wait after the cameras are back before hiding the HUD.</summary>
        internal static float HideUiDelay = 1.0f;

        /// <summary>
        /// Seconds to let the reloaded scene settle before touching anything. This is
        /// what dominates how long the HUD stays visible after a jump.
        /// </summary>
        internal static float SettleDelay = 2.0f;

        /// <summary>Seconds given to OCISLY's repaint to paint the camera names.</summary>
        internal static float RepaintDelay = 0.4f;

        /// <summary>
        /// Identity tokens of the cameras last seen on air. Static, so it survives the
        /// reload that destroys this addon.
        ///
        /// Written from two places: the FixedUpdate poll, which keeps it current, and the
        /// scene-change handler, which catches anything the poll's half-second cadence
        /// might have missed. Never cleared - a moment with nothing streaming must not
        /// erase a good snapshot.
        /// </summary>
        internal static List<string> Snapshot = new List<string> ();

        /// <summary>What the last restore actually did, for diagnosis.</summary>
        internal static string LastRestore = "nothing yet";

        bool restored;

        /// <summary>Physics ticks between snapshot refreshes. Half a second is ample.</summary>
        const int SnapshotTicks = 30;

        int snapshotTick;

        void Start ()
        {
            GameEvents.onGameSceneLoadRequested.Add (OnSceneLoadRequested);
            StartCoroutine (Restore ());
        }

        /// <summary>
        /// Keep the snapshot current while cameras are on air, instead of relying on
        /// catching the single instant the scene is torn down.
        ///
        /// The event handler below is the precise mechanism and runs while the old scene
        /// is still whole - but it only works if onGameSceneLoadRequested actually fires
        /// for the transition in question, and if OCISLY's table still holds the
        /// streaming flags at that exact moment. Neither is guaranteed for an FMRS jump,
        /// which reloads flight from inside flight through its own coroutine. Miss that
        /// instant and the restore has nothing to work with, silently.
        ///
        /// Polling removes the dependency on timing altogether: whatever was last seen
        /// streaming is already recorded, so the teardown does not have to be observed at
        /// all. It costs a walk over a table holding a handful of entries, twice a
        /// second, and it only ever writes when something IS streaming - so a moment with
        /// nothing on air never erases a good snapshot.
        /// </summary>
        void FixedUpdate ()
        {
            if (!OcislyApi.Resolved)
                return;
            if (++snapshotTick < SnapshotTicks)
                return;
            snapshotTick = 0;

            var live = OcislyService.StreamingCameraNames ();
            if (live.Count == 0)
                return;

            // Only log when the set actually changes, or this fills KSP.log.
            if (live.Count != Snapshot.Count || !AllPresent (live, Snapshot)) {
                BridgeLog.Info ("OCISLY", "cameras on air: " + string.Join (", ", live.ToArray ()));
            }
            Snapshot = live;
        }

        static bool AllPresent (List<string> a, List<string> b)
        {
            foreach (var item in a)
                if (!b.Contains (item))
                    return false;
            return true;
        }

        void OnDestroy ()
        {
            GameEvents.onGameSceneLoadRequested.Remove (OnSceneLoadRequested);
        }

        /// <summary>
        /// Fired while we are still in the OLD scene, so OCISLY's camera table is intact.
        /// This is the only moment the information exists.
        ///
        /// It logs on EVERY path, including the empty one. "nothing was on air" in the
        /// restore report otherwise cannot be told apart from "this handler never ran",
        /// and those two have completely different causes: the first means no camera was
        /// streaming when the scene went, the second means the scene change did not raise
        /// onGameSceneLoadRequested at all - or raised it after OCISLY had already dropped
        /// its table. Without this line the difference costs a debugging session.
        /// </summary>
        void OnSceneLoadRequested (GameScenes scene)
        {
            if (!OcislyApi.Resolved) {
                BridgeLog.Info ("OCISLY", "scene change to " + scene
                                + ": OCISLY unresolved, nothing to remember");
                return;
            }

            var live = OcislyService.StreamingCameraNames ();
            if (live.Count == 0) {
                // Deliberately NOT clearing Snapshot: a scene change that happens to
                // catch a moment with nothing on air should not erase what the previous
                // one captured.
                BridgeLog.Info ("OCISLY", "scene change to " + scene
                                + ": no camera streaming at teardown, keeping the previous "
                                + "snapshot (" + Snapshot.Count + " entry/entries)");
                return;
            }

            Snapshot = live;
            BridgeLog.Info ("OCISLY", "scene change to " + scene
                            + ": remembering cameras on air: " + string.Join (", ", live.ToArray ()));
        }

        IEnumerator Restore ()
        {
            if (restored)
                yield break;
            restored = true;

            // Wait for a scene that can actually be worked with. The guard is a frame
            // count rather than a timeout because Time is unreliable across a load.
            var guard = 0;
            while ((!FlightGlobals.ready || FlightGlobals.ActiveVessel == null) && guard < 3000) {
                guard++;
                yield return null;
            }
            var startedAt = Time.realtimeSinceStartup;
            yield return new WaitForSeconds (SettleDelay);

            var report = string.Empty;

            // First, so the very first frame each camera sends already carries its final
            // name.
            if (DisambiguateNames && OcislyApi.Resolved) {
                try {
                    var renamed = CameraNames.Apply ();
                    if (renamed > 0)
                        report += "renamed[" + renamed + "] ";
                } catch (Exception e) {
                    BridgeLog.Error ("OCISLY", "could not disambiguate camera names: " + e.Message);
                }
            }

            if (AutoRearm && OcislyApi.Resolved && Snapshot.Count > 0) {
                var filter = string.Join (",", Snapshot.ToArray ());
                var opened = 0;
                try {
                    opened = OcislyService.OpenMatching (filter);
                } catch (Exception e) {
                    BridgeLog.Error ("OCISLY", "auto-rearm could not open cameras: " + e.Message);
                }

                // A repaint has to happen before streaming can be enabled: OCISLY assigns
                // the camera's Name inside its repaint and pushes it straight to protobuf.
                yield return new WaitForSeconds (RepaintDelay);

                var enabled = 0;
                try {
                    enabled = OcislyService.EnableMatching (filter);
                } catch (Exception e) {
                    BridgeLog.Error ("OCISLY", "auto-rearm could not enable streams: " + e.Message);
                }
                report += string.Format ("rearm[filter={0} opened={1} streaming={2}] ",
                                         filter, opened, enabled);
                EventBus.Record ("ocisly.rearmed", string.Empty,
                                 "opened=" + opened + " streaming=" + enabled);
            } else if (AutoRearm) {
                report += "rearm[nothing was on air] ";
            } else {
                report += "rearm[disabled] ";
            }

            if (HideUiOnSceneLoad) {
                yield return new WaitForSeconds (HideUiDelay);
                report += "hideUI[" + (Hud.Hide () ? "done" : "already hidden") + "]";
            } else {
                report += "hideUI[disabled]";
            }

            report += string.Format (" ({0:0.0}s)", Time.realtimeSinceStartup - startedAt);
            LastRestore = report;
            BridgeLog.Info ("OCISLY", "scene restore: " + report);
        }
    }
}
