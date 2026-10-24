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

        // Core v2: the counter is hosted by the game clock (same increments, same value); the
        // clock records the master tick of each increment (conn.bridge.clock_v1).
        static readonly ClockCounter fixedUpdates = GameClock.Counter ("core.fixed_updates");

        /// <summary>Physics ticks since bootstrap. A cheap heartbeat for Python to check the pump is alive.</summary>
        public static long Ticks {
            get { return fixedUpdates.Value; }
        }

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
                            typeof (BridgeCore).Assembly.GetName ().Version + ", " +
                            BridgeManifest.StampLine (typeof (BridgeCore).Assembly) + ")");

            EventBus.Subscribe ();
            // Nothing in Awake may throw past this point: an exception here leaves the Core
            // without Start (Ready stays false, ping answers "loading") and without FixedUpdate
            // (MainThread never pumps). Deployed 06/10 with a static-delegate hook: exactly that.
            try {
                VesselRegistry.Subscribe ();
            } catch (System.Exception exc) {
                BridgeLog.Error ("vessel registry: " + exc.Message);
            }
            // Refactor R1: the schema descriptors of the Core's own tables (layout_v1).
            try {
                CoreLayouts.Declare ();
            } catch (System.Exception exc) {
                BridgeLog.Error ("layout registry: " + exc.Message);
            }
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
            // Opens the physics step on the game clock if its ObscenelyEarly callback did not.
            GameClock.FromCoreFixedUpdate ();
            fixedUpdates.Increment ();
            MainThread.Pump ();

            // Sweeping is cheap and only actually walks the dictionary when jobs exist,
            // but there is no reason to do it at 60 Hz.
            if ((Ticks & 0xFF) == 0)
                Jobs.Sweep ();
        }

        void OnDestroy ()
        {
            if (instance != this)
                return;     // a duplicate destroyed in its own Awake: it subscribed nothing
            instance = null;
            // Refactor lot 3: the event bus V2 hooks are removed symmetrically (the first log's
            // hooks, as always, live as long as the session). The Core persists across scenes:
            // this runs when the game quits.
            try {
                EventBus.UnsubscribeV2 ();
            } catch (System.Exception exc) {
                BridgeLog.Error ("event bus v2: " + exc.Message);
            }
        }
    }
}
