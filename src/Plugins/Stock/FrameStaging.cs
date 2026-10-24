using KSP.UI.Screens;
using UnityEngine;

namespace KRPC.Bridge.Stock
{
    /// <summary>
    /// Activates the next stage from the frame loop (Update), where the space bar does it
    /// (FlightInputHandler.Update calls StageManager.ActivateNextStage), instead of the physics
    /// loop where kRPC runs every call (KRPC Addon.FixedUpdate, so Control.ActivateNextStage).
    ///
    /// WHY. A radial decoupler pushes the separated part with a coroutine that waits two frames
    /// (ModuleAnchoredDecoupler.ApplyEjectionForces, yield return null). On 02/10/2026 the side
    /// boosters of a three-core launcher, staged by kRPC, left nose towards the core and hit it,
    /// with neutral inherited controls, continuous core thrust, rigid attachments and the same
    /// delay after burnout as by hand; staged with the space bar, they left nose outwards. The only
    /// difference left was which loop ran the staging.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Flight, false)]
    public class FrameStaging : MonoBehaviour
    {
        static int requested;
        static int done;

        /// <summary>Asks for one staging at the next frame; returns the count of stagings done
        /// so far (the caller waits for it to grow).</summary>
        internal static int Request ()
        {
            requested++;
            return done;
        }

        internal static int Done {
            get { return done; }
        }

        void Update ()
        {
            if (requested <= 0)
                return;
            if (FlightGlobals.ActiveVessel == null || !StageManager.CanSeparate)
                return;
            requested--;
            StageManager.ActivateNextStage ();
            done++;
        }

        void OnDestroy ()
        {
            requested = 0;
        }
    }
}
