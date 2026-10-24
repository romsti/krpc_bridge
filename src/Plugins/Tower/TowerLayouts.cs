using System;
using UnityEngine;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// Declares the schema descriptor of the hinge_lock_at_rest_v1 status into the Core's layout
    /// registry at load (TowerLayoutDeclaration: by reflection, this assembly has no reference to
    /// the Core).
    ///
    /// Startup.Instantly, once: every plugin DLL is loaded before the first addon starts, so the
    /// Core assembly is there whatever the load order; its registry is static and replaces a
    /// layout declared twice. Awake only: no GameEvent, no per-frame work. One log line says what
    /// was declared, or why nothing was.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Instantly, true)]
    public sealed class TowerLayouts : MonoBehaviour
    {
        void Awake ()
        {
            try {
                Debug.Log ("[KRPC.Bridge.Tower] " + TowerLayoutDeclaration.Declare ());
            } catch (Exception exc) {
                Debug.LogError ("[KRPC.Bridge.Tower] layout registry: " + exc.Message);
            }
        }
    }
}
