using System;
using System.Collections.Generic;
using System.Threading;

namespace KRPC.Bridge.Core
{
    /// <summary>State of a background job.</summary>
    public enum JobState
    {
        /// <summary>Queued, not started.</summary>
        Pending = 0,
        /// <summary>Running on a worker thread.</summary>
        Running = 1,
        /// <summary>Finished, result available.</summary>
        Done = 2,
        /// <summary>Threw. See the error message.</summary>
        Failed = 3,
        /// <summary>Cancellation requested or applied.</summary>
        Cancelled = 4
    }

    /// <summary>
    /// Off-main-thread computation with a pollable result.
    ///
    /// The problem this solves: RPC bodies run inside FixedUpdate. An RPC that spends
    /// 300 ms integrating a descent trajectory drops the game to 3 FPS for that frame.
    /// The kRPC server's own rate control cannot help, because the cost is inside a
    /// single call it cannot subdivide.
    ///
    /// The pattern:
    ///   1. On the main thread, capture everything the computation needs into PLAIN DATA
    ///      (doubles, arrays, structs). Never a Vessel, Part, Orbit, GameObject or any
    ///      other engine object.
    ///   2. Hand that snapshot to <see cref="Start"/>.
    ///   3. The worker thread computes on the snapshot alone.
    ///   4. Python polls, or the RPC uses <see cref="Wait.Until"/> so it looks synchronous
    ///      while the game keeps running.
    ///
    /// THE RULE, and it is absolute: the worker delegate must not touch Unity or KSP.
    /// Unity's API is not thread safe and most of it does not merely misbehave off the
    /// main thread, it hard-crashes the process with no managed exception and no stack
    /// trace in KSP.log. If the worker needs a live reading mid-computation, it must go
    /// back through <see cref="MainThread.Invoke{T}"/>, which costs a full tick.
    /// </summary>
    public static class Jobs
    {
        sealed class Entry
        {
            internal long Id;
            internal string Kind;
            internal JobState State;
            internal double Progress;
            internal string Error;
            internal double[] Result;
            internal float StartedRealtime;
            // Stamped when the job LEAVES a running state. Sweeping on the start time
            // instead would make any job that outlives RetentionSeconds eligible for
            // collection the instant it finishes - so AwaitJob would return from its wait
            // and then find the result already gone.
            internal float FinishedRealtime;
            internal volatile bool CancelRequested;
        }

        static readonly Dictionary<long, Entry> jobs = new Dictionary<long, Entry> ();
        static readonly object gate = new object ();
        static long nextId = 1;

        /// <summary>Completed jobs older than this are swept, so a long session does not leak.</summary>
        public static float RetentionSeconds = 300f;

        /// <summary>
        /// Start a background job.
        /// </summary>
        /// <param name="kind">Short label, for diagnostics and for Python to sort on.</param>
        /// <param name="work">
        /// Pure computation. Receives a progress reporter (0 to 1) and a cancellation
        /// check, returns the result as a flat double array. Must not touch Unity or KSP.
        /// </param>
        /// <returns>The job id, immediately.</returns>
        public static long Start (string kind, Func<Action<double>, Func<bool>, double[]> work)
        {
            if (work == null)
                throw new ArgumentNullException ("work");

            Entry entry;
            lock (gate) {
                entry = new Entry {
                    Id = nextId++,
                    Kind = kind ?? "job",
                    State = JobState.Pending,
                    StartedRealtime = UnityEngine.Time.realtimeSinceStartup
                };
                jobs [entry.Id] = entry;
            }

            ThreadPool.QueueUserWorkItem (_ => {
                lock (gate)
                    entry.State = JobState.Running;
                try {
                    // Progress is written under the lock: a double write is not
                    // guaranteed atomic, and every reader takes the lock.
                    var result = work (p => { lock (gate) entry.Progress = p; },
                                       () => entry.CancelRequested);
                    lock (gate) {
                        entry.Result = result ?? new double[0];
                        entry.State = entry.CancelRequested ? JobState.Cancelled : JobState.Done;
                        entry.Progress = 1.0;
                    }
                } catch (Exception e) {
                    lock (gate) {
                        entry.Error = e.Message;
                        entry.State = JobState.Failed;
                    }
                    BridgeLog.Error ("job " + entry.Id + " (" + entry.Kind + ") failed: " + e);
                }
                // Note: the worker does NOT stamp a finish time. Reading
                // UnityEngine.Time off the main thread is a Unity call, and off the main
                // thread Unity does not raise a managed exception - it takes the process
                // down. Sweep, which runs on the main thread, stamps it instead.
            });

            return entry.Id;
        }

        /// <summary>Current state of a job. Unknown ids report <see cref="JobState.Failed"/>.</summary>
        public static JobState State (long id)
        {
            lock (gate) {
                Entry e;
                return jobs.TryGetValue (id, out e) ? e.State : JobState.Failed;
            }
        }

        /// <summary>Progress from 0 to 1, as last reported by the worker.</summary>
        public static double Progress (long id)
        {
            lock (gate) {
                Entry e;
                return jobs.TryGetValue (id, out e) ? e.Progress : 0.0;
            }
        }

        /// <summary>Error message of a failed job, or the empty string.</summary>
        public static string Error (long id)
        {
            lock (gate) {
                Entry e;
                return jobs.TryGetValue (id, out e) && e.Error != null ? e.Error : string.Empty;
            }
        }

        /// <summary>
        /// The result. Throws while the job is still running, so Python gets a clear
        /// error instead of a silently empty array.
        /// </summary>
        public static IList<double> Result (long id)
        {
            lock (gate) {
                Entry e;
                if (!jobs.TryGetValue (id, out e))
                    throw new InvalidOperationException (
                        "no job with id " + id + " - it was never started, or it finished more than "
                        + RetentionSeconds + " s ago and has been swept");
                if (e.State == JobState.Running || e.State == JobState.Pending)
                    throw new InvalidOperationException ("job " + id + " is still " + e.State);
                if (e.State == JobState.Failed)
                    throw new InvalidOperationException ("job " + id + " failed: " + e.Error);
                return e.Result ?? new double[0];
            }
        }

        /// <summary>
        /// Ask a job to stop. Cooperative only: the worker sees it through its
        /// cancellation check. A worker that never checks will run to completion.
        /// </summary>
        public static void Cancel (long id)
        {
            lock (gate) {
                Entry e;
                if (jobs.TryGetValue (id, out e))
                    e.CancelRequested = true;
            }
        }

        /// <summary>
        /// Drop finished jobs older than <see cref="RetentionSeconds"/>. Called from
        /// FixedUpdate, which is what makes it the right place to read Unity's clock.
        ///
        /// Retention is counted from when the job FINISHED, not from when it started.
        /// Counting from the start would make a job that ran longer than
        /// <see cref="RetentionSeconds"/> collectable the instant it completed, so
        /// <c>await_job</c> would return from its wait and then find the result gone.
        /// Since the worker cannot safely read Unity's clock, the finish time is stamped
        /// here, the first time a sweep observes a terminal state.
        /// </summary>
        internal static void Sweep ()
        {
            var now = UnityEngine.Time.realtimeSinceStartup;
            lock (gate) {
                if (jobs.Count == 0)
                    return;
                List<long> dead = null;
                foreach (var kv in jobs) {
                    var e = kv.Value;
                    if (e.State == JobState.Running || e.State == JobState.Pending)
                        continue;
                    if (e.FinishedRealtime <= 0f) {
                        e.FinishedRealtime = now;
                        continue;
                    }
                    if (now - e.FinishedRealtime < RetentionSeconds)
                        continue;
                    (dead ?? (dead = new List<long> ())).Add (kv.Key);
                }
                if (dead != null)
                    foreach (var id in dead)
                        jobs.Remove (id);
            }
        }
    }
}
