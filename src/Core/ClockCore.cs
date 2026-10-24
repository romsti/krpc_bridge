using System;
using System.Collections.Generic;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// The physics-step clock, without Unity: one master tick per physics step and the values
    /// of that step frozen when it opens (fixedTime, UT, dt, warp rate, real time).
    ///
    /// PURE: System only, so build/tests/core tests it as the Core compiles it. GameClock is
    /// the Unity side that feeds it once per step.
    ///
    /// The mod had four tick counters (BridgeCore.Ticks, the Actuators pump tick, the
    /// AttitudeV1 tick, plus fixedTime and UT read at different moments of the step). They are
    /// published in frames that must stay byte-identical, so they are not renumbered: each
    /// becomes a <see cref="ClockCounter"/> HOSTED by the clock, with exactly its old
    /// increments, and the clock records the master tick of its last increment. clock_v1 lists
    /// them side by side: that is the alias table between the old counters and the one tick.
    /// </summary>
    public sealed class ClockCore
    {
        long tick;
        double fixedTime = double.NaN;
        double ut = double.NaN;
        double dt = double.NaN;
        double warpRate = double.NaN;
        double realTime = double.NaN;
        readonly List<ClockCounter> counters = new List<ClockCounter> ();

        /// <summary>Master physics tick: 0 before the first step, then +1 per step.</summary>
        public long Tick {
            get { return tick; }
        }

        /// <summary>Unity Time.fixedTime when the step opened (s).</summary>
        public double FixedTime {
            get { return fixedTime; }
        }

        /// <summary>Universal time when the step opened (s); NaN where no Planetarium exists.</summary>
        public double Ut {
            get { return ut; }
        }

        /// <summary>TimeWarp.fixedDeltaTime of the step (s).</summary>
        public double Dt {
            get { return dt; }
        }

        /// <summary>TimeWarp.CurrentRate of the step (1 without warp).</summary>
        public double WarpRate {
            get { return warpRate; }
        }

        /// <summary>Time.realtimeSinceStartup when the step opened (s, wall clock).</summary>
        public double RealTime {
            get { return realTime; }
        }

        /// <summary>
        /// Opens the next step with the values sampled now and returns its tick. The caller
        /// guarantees one call per physics step (GameClock does).
        /// </summary>
        public long Open (double fixedTimeNow, double utNow, double dtNow, double warpNow,
            double realNow)
        {
            tick++;
            fixedTime = fixedTimeNow;
            ut = utNow;
            dt = dtNow;
            warpRate = warpNow;
            realTime = realNow;
            return tick;
        }

        /// <summary>The counter of that name, created on first use. Never null.</summary>
        public ClockCounter Counter (string name)
        {
            if (string.IsNullOrEmpty (name))
                throw new ArgumentException ("counter name", nameof (name));
            for (int i = 0; i < counters.Count; i++)
                if (counters [i].Name == name)
                    return counters [i];
            var counter = new ClockCounter (name, this);
            counters.Add (counter);
            return counter;
        }

        /// <summary>Every hosted counter, in creation order.</summary>
        public IList<ClockCounter> Counters {
            get { return counters.AsReadOnly (); }
        }
    }

    /// <summary>
    /// One legacy tick counter hosted by the clock. <see cref="Value"/> moves exactly as the
    /// counter it replaces did (same increments, same places); <see cref="LastTick"/> is the
    /// master tick of its last increment.
    /// </summary>
    public sealed class ClockCounter
    {
        readonly ClockCore clock;

        /// <summary>Name, as clock_v1 publishes it.</summary>
        public readonly string Name;

        /// <summary>The counter's own value (the number it always published).</summary>
        public long Value;

        /// <summary>Master tick of the last increment (0: never incremented).</summary>
        public long LastTick;

        internal ClockCounter (string name, ClockCore clock)
        {
            Name = name;
            this.clock = clock;
        }

        /// <summary>Value + 1, recorded against the master tick. Returns the new value.</summary>
        public long Increment ()
        {
            Value++;
            LastTick = clock.Tick;
            return Value;
        }
    }
}
