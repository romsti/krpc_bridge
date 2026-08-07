using System;
using System.Collections.Generic;
using System.Threading;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// Main-thread dispatcher.
    ///
    /// READ THIS BEFORE USING IT, because the obvious use is the wrong one.
    ///
    /// kRPC already executes RPC bodies on the Unity main thread, inside FixedUpdate.
    /// (That is what the server's "max time per update" setting bounds.) So an ordinary
    /// service method may touch Vessel, Part, GameObject and friends directly, with no
    /// dispatch at all. Wrapping every RPC in <see cref="Invoke{T}"/> would be pure
    /// overhead, and calling it from the main thread would deadlock if it were not
    /// guarded below.
    ///
    /// What this class is actually for is the three cases where you are NOT already on
    /// the main thread:
    ///   1. a background job (see <see cref="Jobs"/>) that needs one main-thread read
    ///      before or after its computation,
    ///   2. a callback handed to you by a third-party mod that fires on its own thread,
    ///   3. your own socket / timer / file watcher.
    ///
    /// The queue is drained by <see cref="BridgeCore"/> in FixedUpdate, with a time
    /// budget, so a flood of queued work degrades throughput instead of framerate.
    /// </summary>
    public static class MainThread
    {
        static readonly Queue<Action> queue = new Queue<Action> ();
        static readonly object gate = new object ();
        static int mainThreadId = -1;

        /// <summary>Milliseconds of work the pump will do per FixedUpdate before deferring the rest.</summary>
        public static double BudgetMillisecondsPerTick = 2.0;

        /// <summary>Set once, from the main thread, by <see cref="BridgeCore"/>.</summary>
        internal static void Claim ()
        {
            mainThreadId = Thread.CurrentThread.ManagedThreadId;
        }

        /// <summary>True when the calling thread is Unity's main thread.</summary>
        public static bool IsMainThread {
            get { return Thread.CurrentThread.ManagedThreadId == mainThreadId; }
        }

        /// <summary>How many actions are waiting. Exposed for diagnostics.</summary>
        public static int PendingCount {
            get { lock (gate) return queue.Count; }
        }

        /// <summary>
        /// Queue an action to run on the main thread and return immediately.
        /// Exceptions thrown by the action are logged, never rethrown here.
        /// </summary>
        public static void Post (Action action)
        {
            if (action == null)
                return;
            lock (gate)
                queue.Enqueue (action);
        }

        /// <summary>
        /// Run a function on the main thread and wait for its result.
        ///
        /// When called FROM the main thread it runs inline, because blocking the main
        /// thread on a queue the main thread is responsible for draining is a deadlock.
        /// This is not a nicety: any kRPC service method that calls Invoke is already on
        /// the main thread and would hang the entire game without this branch.
        /// </summary>
        public static T Invoke<T> (Func<T> function, int timeoutMilliseconds = 5000)
        {
            if (function == null)
                throw new ArgumentNullException ("function");
            if (IsMainThread)
                return function ();

            T result = default (T);
            Exception error = null;

            // The wait handle is NOT disposed on the timeout path, and the closure checks
            // a flag before touching it. The queued action outlives an abandoned Invoke:
            // disposing the handle here and letting the pump run the action later would
            // execute work the caller has given up on and then throw
            // ObjectDisposedException inside the pump, which logs as "queued action threw"
            // and hides the real cause. Letting the GC take the handle costs nothing.
            var done = new ManualResetEventSlim (false);
            var abandoned = new[] { false };

            Post (() => {
                bool giveUp;
                lock (gate)
                    giveUp = abandoned [0];
                if (giveUp)
                    return;
                try {
                    result = function ();
                } catch (Exception e) {
                    error = e;
                } finally {
                    try {
                        done.Set ();
                    } catch (ObjectDisposedException) {
                    }
                }
            });

            if (!done.Wait (timeoutMilliseconds)) {
                lock (gate)
                    abandoned [0] = true;
                throw new TimeoutException (
                    "MainThread.Invoke timed out after " + timeoutMilliseconds +
                    " ms. Is the game paused, loading, or is BridgeCore not running?");
            }
            done.Dispose ();

            if (error != null)
                throw new InvalidOperationException ("main-thread work failed: " + error.Message, error);
            return result;
        }

        /// <summary>Void overload of <see cref="Invoke{T}"/>.</summary>
        public static void Invoke (Action action, int timeoutMilliseconds = 5000)
        {
            Invoke<bool> (() => { action (); return true; }, timeoutMilliseconds);
        }

        /// <summary>
        /// Drain the queue, bounded by <see cref="BudgetMillisecondsPerTick"/>.
        /// Called only by <see cref="BridgeCore.FixedUpdate"/>.
        /// </summary>
        internal static void Pump ()
        {
            var started = UnityEngine.Time.realtimeSinceStartup;
            var budgetSeconds = (float)(BudgetMillisecondsPerTick / 1000.0);

            while (true) {
                Action next;
                lock (gate) {
                    if (queue.Count == 0)
                        return;
                    next = queue.Dequeue ();
                }
                try {
                    next ();
                } catch (Exception e) {
                    BridgeLog.Error ("queued main-thread action threw: " + e);
                }
                if (UnityEngine.Time.realtimeSinceStartup - started > budgetSeconds)
                    return;
            }
        }
    }
}
