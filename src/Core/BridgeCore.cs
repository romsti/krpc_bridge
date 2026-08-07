using UnityEngine;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// The one MonoBehaviour the whole mod owns.
    ///
    /// Two jobs:
    ///   1. bootstrap once per KSP session (claim the main thread, hook GameEvents, run
    ///      every plugin's resolver),
    ///   2. pump the main-thread queue every FixedUpdate.
    ///
    /// DontDestroyOnLoad matters. Without it this object dies on every scene change, the
    /// queue stops draining mid-flight, and MainThread.Invoke starts timing out during
    /// exactly the transitions where a plugin most wants it (an FMRS jump, for instance).
    ///
    /// KSPAddon.Startup.Instantly runs before the main menu, so plugins registering from
    /// their own Instantly addon and Core resolving at the first Awake are ordered
    /// correctly by KSP's assembly load order, not by luck.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Instantly, true)]
    public sealed class BridgeCore : MonoBehaviour
    {
        static BridgeCore instance;

        /// <summary>True once bootstrap has completed.</summary>
        public static bool Ready { get; private set; }

        /// <summary>Physics ticks since bootstrap. A cheap heartbeat for Python to check the pump is alive.</summary>
        public static long Ticks { get; private set; }

        void Awake ()
        {
            if (instance != null) {
                Destroy (gameObject);
                return;
            }
            instance = this;
            DontDestroyOnLoad (gameObject);

            MainThread.Claim ();
            BridgeLog.Info ("core starting (assembly " +
                            typeof (BridgeCore).Assembly.GetName ().Version + ")");

            EventBus.Subscribe ();
        }

        /// <summary>
        /// Resolve the plugins, in Start rather than Awake.
        ///
        /// THIS ORDERING IS NOT COSMETIC. Plugins register from their own
        /// [KSPAddon(Instantly)] Awake, and KSP loads the Core FIRST because every
        /// plugin declares a KSPAssemblyDependency on it - which means Core's Awake also
        /// runs first, before a single plugin has registered. Resolving there would
        /// iterate an empty list, latch, and leave every service reporting
        /// "not resolved yet" while the Core cheerfully answered ping with pong.
        ///
        /// Unity runs every Awake in a frame before any Start, so Start is after the
        /// last registration regardless of the order KSP instantiated the addons in.
        /// ModRegistry also resolves lazily on first read, so nothing depends on this
        /// being the only path.
        /// </summary>
        void Start ()
        {
            if (Ready)
                return;

            ModRegistry.ResolveAll ();
            Ready = true;
            BridgeLog.Info ("core ready, " + ModRegistry.All ().Count + " plugin(s) registered");
        }

        void FixedUpdate ()
        {
            Ticks++;
            MainThread.Pump ();

            // Sweeping is cheap and only actually walks the dictionary when jobs exist,
            // but there is no reason to do it at 60 Hz.
            if ((Ticks & 0xFF) == 0)
                Jobs.Sweep ();
        }

        void OnDestroy ()
        {
            if (instance == this)
                instance = null;
        }
    }
}
