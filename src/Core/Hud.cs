namespace KRPC.Bridge.Core
{
    /// <summary>
    /// Showing and hiding the game's HUD the way the F2 key does.
    ///
    /// The distinction that matters: calling UIMasterController.HideUI() directly blanks
    /// the stock canvases without telling anyone, so every mod window stays on screen.
    /// Firing GameEvents.onHideUI is what F2 actually does, and it is what mods listen
    /// for - so their windows go away too. For a recorded flight with FMRS, OCISLY,
    /// MechJeb and a toolbar all drawing, that is the whole difference between a clean
    /// frame and a cluttered one.
    /// </summary>
    public static class Hud
    {
        /// <summary>Hide the HUD. Returns false if it was already hidden.</summary>
        public static bool Hide ()
        {
            var controller = KSP.UI.UIMasterController.Instance;
            if (controller == null || !controller.IsUIShowing)
                return false;
            GameEvents.onHideUI.Fire ();
            return true;
        }

        /// <summary>Bring the HUD back. Returns false if it was already visible.</summary>
        public static bool Show ()
        {
            var controller = KSP.UI.UIMasterController.Instance;
            if (controller == null || controller.IsUIShowing)
                return false;
            GameEvents.onShowUI.Fire ();
            return true;
        }

        /// <summary>Whether the HUD is currently showing.</summary>
        public static bool Visible {
            get {
                var controller = KSP.UI.UIMasterController.Instance;
                return controller != null && controller.IsUIShowing;
            }
        }
    }
}
