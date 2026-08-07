using System;
using KRPC.Service;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// Making a Python call block on game state WITHOUT freezing the game.
    ///
    /// A kRPC service method runs inside FixedUpdate. A <c>while (!ready) { }</c> loop or
    /// a Thread.Sleep in one therefore hangs Unity, not just the client. The correct
    /// mechanism is kRPC's own continuation: throw a <see cref="YieldException"/> and the
    /// server re-invokes the supplied delegate on a later update, leaving the game to
    /// run normally in between. The client sees one long-running call.
    ///
    /// FmrsService already does this by hand for the scene reload. These helpers are the
    /// same idea, generalised, so no plugin has to hand-roll the deadline bookkeeping
    /// again.
    ///
    /// Deadlines are in REAL time (Time.realtimeSinceStartup), deliberately. In-game
    /// clocks stop during a scene load, which is exactly when you most need a timeout to
    /// still fire.
    /// </summary>
    public static class Wait
    {
        /// <summary>
        /// Block the calling RPC until <paramref name="condition"/> is true, yielding a
        /// physics tick between polls.
        /// </summary>
        /// <param name="condition">Evaluated on the main thread, once per tick.</param>
        /// <param name="timeoutSeconds">Real-time budget. Throws when exceeded.</param>
        /// <param name="what">Named in the timeout message. Make it specific.</param>
        public static void Until (Func<bool> condition, float timeoutSeconds, string what)
        {
            if (condition == null)
                throw new ArgumentNullException ("condition");
            Poll (condition, UnityEngine.Time.realtimeSinceStartup + Bound (timeoutSeconds), what);
        }

        /// <summary>Longest a yielded RPC may block, whatever the caller asked for.</summary>
        public const float MaxTimeoutSeconds = 3600f;

        /// <summary>
        /// Clamp a caller-supplied timeout into something that can actually expire.
        ///
        /// This exists because the timeout usually comes off the wire, and kRPC has no
        /// client-side timeout of its own: an infinite or NaN deadline makes
        /// <c>now &gt; deadline</c> never true - NaN compares false against everything - so
        /// the continuation yields forever and the Python call never returns. There is no
        /// way to cancel it and no error to see.
        /// </summary>
        static float Bound (float timeoutSeconds)
        {
            if (float.IsNaN (timeoutSeconds) || timeoutSeconds <= 0f)
                return 1f;
            return timeoutSeconds > MaxTimeoutSeconds ? MaxTimeoutSeconds : timeoutSeconds;
        }

        static void Poll (Func<bool> condition, float deadline, string what)
        {
            if (condition ())
                return;
            if (UnityEngine.Time.realtimeSinceStartup > deadline)
                throw new InvalidOperationException ("timed out waiting for " + what);
            throw new YieldException<Action> (() => Poll (condition, deadline, what));
        }

        /// <summary>
        /// Block the calling RPC until <paramref name="condition"/> has been true for
        /// <paramref name="consecutiveTicks"/> ticks in a row.
        ///
        /// Use this rather than <see cref="Until"/> whenever the condition can flicker.
        /// A vessel is briefly "landed" mid-bounce, and briefly "loaded" mid scene swap.
        /// Requiring a run of true readings is what makes a handoff reliable.
        /// </summary>
        public static void Settled (Func<bool> condition, int consecutiveTicks,
                                    float timeoutSeconds, string what)
        {
            if (condition == null)
                throw new ArgumentNullException ("condition");
            PollSettled (condition, consecutiveTicks, 0,
                         UnityEngine.Time.realtimeSinceStartup + Bound (timeoutSeconds), what);
        }

        static void PollSettled (Func<bool> condition, int needed, int seen, float deadline, string what)
        {
            if (UnityEngine.Time.realtimeSinceStartup > deadline)
                throw new InvalidOperationException ("timed out waiting for " + what);
            var now = condition () ? seen + 1 : 0;
            if (now >= needed)
                return;
            throw new YieldException<Action> (() => PollSettled (condition, needed, now, deadline, what));
        }

        /// <summary>Yield <paramref name="ticks"/> physics ticks, then return.</summary>
        public static void Ticks (int ticks)
        {
            if (ticks <= 0)
                return;
            throw new YieldException<Action> (() => Ticks (ticks - 1));
        }
    }
}
