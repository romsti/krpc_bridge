using System;
using UnityEngine;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// The game clock: one master tick per physics step, and UT, fixedTime, dt, warp rate and
    /// real time frozen when the step opens.
    ///
    /// When does a step open? At the first of these that runs in it:
    ///   * a TimingManager callback at stage ObscenelyEarly (before Precalc, Earlyish and every
    ///     part's FixedUpdate), added as soon as a TimingManager exists;
    ///   * BridgeCore.FixedUpdate, which runs once per physics step in every scene.
    /// A step is recognised by (Time.frameCount, Time.fixedTime): several physics steps of one
    /// rendered frame have distinct fixedTimes, and steps of different frames distinct frame
    /// counts. So the step opens exactly once whatever the order of those two calls. Limit:
    /// fixedTime is a float, whose spacing passes the 0.02 s step after about 72 h of one
    /// session; past that, two steps of one rendered frame could be counted as one.
    ///
    /// Nothing that flies reads it in refactor R1: the frames keep their own counters, byte for
    /// byte, now hosted here as <see cref="ClockCounter"/>s (clock_v1 publishes them with the
    /// master tick). New contracts count in master ticks (a lease of N physics steps, the
    /// fingerprint memo of <see cref="VesselRegistry"/>).
    ///
    /// UT is Planetarium.GetUniversalTime() when Planetarium.fetch exists, NaN otherwise (main
    /// menu, editors: reading it there would throw).
    /// </summary>
    public static class GameClock
    {
        /// <summary>Protocol of clock_v1.</summary>
        public const int Protocol = 1;

        static readonly ClockCore core = new ClockCore ();
        static readonly TimingManager.UpdateAction earlyAction = OnObscenelyEarly;
        static bool registered;
        static bool live;
        static int lastFrame = -1;
        static float lastFixedTime = float.NaN;
        static long earlyOpens;
        static long coreOpens;
        static long earlyCalls;
        static long earlyCallsAtCheck;
        static int stepsSinceCheck;

        /// <summary>The pure clock (tick, frozen values, hosted counters).</summary>
        public static ClockCore State {
            get { return core; }
        }

        /// <summary>Master physics tick.</summary>
        public static long Tick {
            get { return core.Tick; }
        }

        /// <summary>Steps opened by the ObscenelyEarly callback.</summary>
        public static long EarlyOpens {
            get { return earlyOpens; }
        }

        /// <summary>Steps opened by BridgeCore.FixedUpdate (no TimingManager callback before it).</summary>
        public static long CoreOpens {
            get { return coreOpens; }
        }

        /// <summary>True while the ObscenelyEarly callback is called (seen within its last 50-step check).</summary>
        public static bool Hooked {
            get { return live; }
        }

        /// <summary>
        /// The master tick of the physics step running NOW, read without opening it (refactor R3:
        /// the tick every frame carries). The open step's tick; or, for a caller that runs in a
        /// step neither opener has reached yet (TimingManager's Earlyish before BridgeCore's
        /// FixedUpdate while the ObscenelyEarly callback is not registered), the tick that step
        /// will get when it opens: the next one. A pure read: no counter moves.
        /// </summary>
        public static long StepTick {
            get {
                return Time.frameCount == lastFrame && Time.fixedTime == lastFixedTime
                    ? core.Tick : core.Tick + 1;
            }
        }

        /// <summary>A legacy counter hosted by the clock (created on first use).</summary>
        public static ClockCounter Counter (string name)
        {
            return core.Counter (name);
        }

        static void OnObscenelyEarly ()
        {
            // Timing0 invokes its whole multicast inside one call: nothing may escape.
            try {
                earlyCalls++;
                if (Sync ())
                    earlyOpens++;
            } catch (Exception exc) {
                BridgeLog.Error ("GameClock ObscenelyEarly: " + exc.Message);
            }
        }

        /// <summary>BridgeCore.FixedUpdate, once per physics step.</summary>
        internal static void FromCoreFixedUpdate ()
        {
            if (Sync ())
                coreOpens++;
            EnsureHook ();
        }

        /// <summary>Opens the step if this physics step is not the one already open. True if it did.</summary>
        static bool Sync ()
        {
            int frame = Time.frameCount;
            float fixedTime = Time.fixedTime;
            if (frame == lastFrame && fixedTime == lastFixedTime)
                return false;
            lastFrame = frame;
            lastFixedTime = fixedTime;
            double ut = double.NaN;
            if (Planetarium.fetch != null)
                ut = Planetarium.GetUniversalTime ();
            core.Open (fixedTime, ut, TimeWarp.fixedDeltaTime, TimeWarp.CurrentRate,
                Time.realtimeSinceStartup);
            return true;
        }

        /// <summary>
        /// TimingManager.Instance is protected: a plugin cannot see whether a TimingManager
        /// exists, and FixedUpdateAdd returns silently while there is none. So: register once,
        /// then, every 50 steps, if the callback was not called in those steps (no
        /// TimingManager yet, or a new one without our callback), register again. Remove first,
        /// so a live registration is never doubled (Sync is idempotent within a step anyway).
        /// </summary>
        static void EnsureHook ()
        {
            if (registered && ++stepsSinceCheck < 50)
                return;
            bool called = earlyCalls != earlyCallsAtCheck;
            earlyCallsAtCheck = earlyCalls;
            stepsSinceCheck = 0;
            live = called;
            if (registered && called)
                return;
            try {
                TimingManager.FixedUpdateRemove (TimingManager.TimingStage.ObscenelyEarly, earlyAction);
                TimingManager.FixedUpdateAdd (TimingManager.TimingStage.ObscenelyEarly, earlyAction);
                registered = true;
            } catch (Exception exc) {
                BridgeLog.Error ("GameClock: TimingManager hook: " + exc.Message);
            }
        }
    }
}
