using System;
using System.Collections.Generic;

namespace KRPC.Bridge.Core
{
    /// <summary>One recorded game event.</summary>
    public sealed class GameEventRecord
    {
        /// <summary>Monotonic sequence number. Pass the highest one you have seen back to Poll.</summary>
        public long Id { get; internal set; }

        /// <summary>Event name, for example "vessel.destroy" or "stage.activate".</summary>
        public string Kind { get; internal set; }

        /// <summary>In-game universal time when it fired.</summary>
        public double UT { get; internal set; }

        /// <summary>Real time since startup when it fired. Survives a paused game clock.</summary>
        public double Realtime { get; internal set; }

        /// <summary>Vessel id when the event concerns a vessel, otherwise empty.</summary>
        public string VesselId { get; internal set; }

        /// <summary>Free-form detail: part name, situation, stage number. Never null.</summary>
        public string Detail { get; internal set; }
    }

    /// <summary>
    /// A durable, ordered log of KSP GameEvents, pollable from Python.
    ///
    /// WHY A LOG AND NOT A kRPC EVENT. kRPC does have an event primitive
    /// (<c>KRPC.Service.Event</c>, exposed here as <see cref="Signal"/>), but it is a
    /// latch riding the stream channel: it tells a client THAT something happened and
    /// carries no payload, and two occurrences between polls collapse into one. A kRPC
    /// stream is weaker still - it samples a value once per update, so it reports what
    /// something IS, never that something HAPPENED. A part destroyed between two samples
    /// leaves no trace; a stage that fires and completes inside one tick is invisible.
    ///
    /// So the two are complements, and the intended pattern uses both: block on
    /// <see cref="Signal"/> to be woken the instant something matches, then call Poll to
    /// read every record since your last id, in order, with payloads and timestamps and
    /// no possibility of having missed one.
    ///
    /// Add hooks in <see cref="Subscribe"/>. Keep the handlers trivial: they run inside
    /// KSP's own event dispatch, and an exception thrown there can take out unrelated
    /// mods listening on the same event.
    /// </summary>
    public static class EventBus
    {
        const int Capacity = 512;

        /// <summary>Most event signals that may be live at once. See <see cref="Signal"/>.</summary>
        const int MaxSignals = 64;

        static readonly Queue<GameEventRecord> ring = new Queue<GameEventRecord> (Capacity);
        static readonly object gate = new object ();
        static long nextId = 1;
        static bool subscribed;

        sealed class Subscription
        {
            internal string Filter;
            internal KRPC.Service.Event Event;
        }

        static readonly List<Subscription> subscriptions = new List<Subscription> ();

        /// <summary>
        /// A kRPC event that fires whenever a record whose Kind matches
        /// <paramref name="kinds"/> is written.
        ///
        /// The client blocks on it rather than polling, which is the difference between
        /// noticing a separation within a millisecond and noticing it within however long
        /// your poll loop sleeps. It carries no payload by design - call
        /// <see cref="Poll"/> afterwards for the records themselves.
        /// </summary>
        /// <param name="kinds">
        /// Comma-separated substrings matched case-insensitively against the event kind.
        /// Empty or "*" matches every event.
        /// </param>
        public static KRPC.Service.Event Signal (string kinds)
        {
            // A manually triggered event: Record() fires it. The polled-predicate form of
            // KRPC.Service.Event would re-evaluate every update and could still coalesce
            // two events inside one tick, which is the failure this whole class exists to
            // avoid.
            var subscription = new Subscription {
                Filter = kinds ?? string.Empty,
                Event = new KRPC.Service.Event ()
            };
            lock (gate) {
                // kRPC gives us no "this client went away" callback, so a script that
                // reconnects leaves its old signal behind. Record() drops one whose
                // Trigger throws, but a client that vanished cleanly may never throw -
                // hence a hard cap. Every subscription costs a filter match on every
                // game event, on the main thread, so an unbounded list is a slow leak
                // into a hot path.
                if (subscriptions.Count >= MaxSignals) {
                    var evicted = subscriptions [0];
                    subscriptions.RemoveAt (0);
                    BridgeLog.Warn ("more than " + MaxSignals + " event signals registered; "
                                    + "dropping the oldest (filter '" + evicted.Filter + "')");
                }
                subscriptions.Add (subscription);
            }
            return subscription.Event;
        }


        /// <summary>Total events recorded this session, including ones already dropped from the ring.</summary>
        public static long TotalRecorded { get { lock (gate) return nextId - 1; } }

        /// <summary>
        /// Every event with an id strictly greater than <paramref name="sinceId"/>.
        /// Pass 0 on the first call. Pass the last Id you received on subsequent ones.
        /// </summary>
        public static IList<GameEventRecord> Poll (long sinceId)
        {
            lock (gate) {
                var found = new List<GameEventRecord> ();
                foreach (var record in ring)
                    if (record.Id > sinceId)
                        found.Add (record);
                return found;
            }
        }

        /// <summary>
        /// Record an event. Plugins may call this to publish their own, so a plugin that
        /// wraps a mod exposing C# events can surface them on the same channel
        /// (StageRecovery's RecoveryProcessingComplete, for instance).
        /// </summary>
        public static void Record (string kind, string vesselId = "", string detail = "")
        {
            var name = kind ?? "unknown";

            // Read the clocks BEFORE taking the lock. Both are Unity calls, and holding
            // our own lock across a call into another subsystem is how lock cycles get
            // built - the more so because Record is public API a plugin might reach from
            // an odd context.
            var ut = UniversalTime ();
            var realtime = UnityEngine.Time.realtimeSinceStartup;

            List<KRPC.Service.Event> toFire = null;

            lock (gate) {
                if (ring.Count >= Capacity)
                    ring.Dequeue ();
                ring.Enqueue (new GameEventRecord {
                    Id = nextId++,
                    Kind = name,
                    UT = ut,
                    Realtime = realtime,
                    VesselId = vesselId ?? string.Empty,
                    Detail = detail ?? string.Empty
                });

                foreach (var subscription in subscriptions)
                    if (ModRegistry.MatchesFilter (name, subscription.Filter))
                        (toFire ?? (toFire = new List<KRPC.Service.Event> ())).Add (subscription.Event);
            }

            if (toFire == null)
                return;

            List<KRPC.Service.Event> dead = null;
            foreach (var evnt in toFire) {
                try {
                    evnt.Trigger ();
                } catch (Exception e) {
                    // A client that disconnected leaves an event whose stream is gone.
                    // Drop it rather than throwing on every subsequent game event for the
                    // rest of the session.
                    BridgeLog.Warn ("event signal could not fire, dropping it: " + e.Message);
                    (dead ?? (dead = new List<KRPC.Service.Event> ())).Add (evnt);
                }
            }
            if (dead != null)
                Forget (dead);
        }

        /// <summary>
        /// Universal time, or 0 before the planetarium exists.
        ///
        /// <c>Planetarium.GetUniversalTime</c> dereferences a singleton that is null
        /// during early loading, and this class hooks onGameSceneLoadRequested from
        /// Startup.Instantly - so it genuinely does get called before then. An exception
        /// here would propagate into KSP's own event dispatch and could take out
        /// unrelated mods listening on the same event.
        /// </summary>
        static double UniversalTime ()
        {
            try {
                return Planetarium.GetUniversalTime ();
            } catch (Exception) {
                return 0.0;
            }
        }

        static void Forget (List<KRPC.Service.Event> dead)
        {
            lock (gate) {
                subscriptions.RemoveAll (s => dead.Contains (s.Event));
            }
        }

        /// <summary>Hook the stock GameEvents. Called once by <see cref="BridgeCore"/>.</summary>
        internal static void Subscribe ()
        {
            if (subscribed)
                return;
            subscribed = true;

            // Deliberately a short, curated list. Every hook costs a delegate call on a
            // hot path, and a bus nobody reads is just overhead. Extend it as plugins
            // need to, not speculatively.
            //
            // Each hook is guarded on its own. GameEvents members do move between KSP
            // versions, and losing one hook must cost you that one event, not the bus.

            Hook ("vessel.destroy", () => GameEvents.onVesselDestroy.Add (v =>
                Record ("vessel.destroy", Id (v), v != null ? v.vesselName : "")));

            Hook ("vessel.change", () => GameEvents.onVesselChange.Add (v =>
                Record ("vessel.change", Id (v), v != null ? v.vesselName : "")));

            Hook ("vessel.recovered", () => GameEvents.onVesselRecovered.Add ((pv, _) =>
                Record ("vessel.recovered",
                        pv != null ? pv.vesselID.ToString () : "",
                        pv != null ? pv.vesselName : "")));

            Hook ("vessel.situation", () => GameEvents.onVesselSituationChange.Add (data =>
                Record ("vessel.situation", Id (data.host), data.from + " -> " + data.to)));

            Hook ("vessel.soi", () => GameEvents.onVesselSOIChanged.Add (data =>
                Record ("vessel.soi", Id (data.host),
                        (data.from != null ? data.from.bodyName : "?") + " -> " +
                        (data.to != null ? data.to.bodyName : "?"))));

            Hook ("stage.activate", () => GameEvents.onStageActivate.Add (stage =>
                Record ("stage.activate", ActiveId (), stage.ToString ())));

            Hook ("part.die", () => GameEvents.onPartDie.Add (p =>
                Record ("part.die",
                        p != null && p.vessel != null ? Id (p.vessel) : "",
                        p != null && p.partInfo != null ? p.partInfo.name : "")));

            Hook ("part.crash", () => GameEvents.onCrash.Add (report =>
                Record ("part.crash", OriginVesselId (report), OriginPartName (report))));

            Hook ("part.splashdown", () => GameEvents.onCrashSplashdown.Add (report =>
                Record ("part.splashdown", OriginVesselId (report), OriginPartName (report))));

            Hook ("scene.load", () => GameEvents.onGameSceneLoadRequested.Add (scene =>
                Record ("scene.load", "", scene.ToString ())));

            BridgeLog.Info ("event bus subscribed");
        }

        static void Hook (string name, Action subscribe)
        {
            try {
                subscribe ();
            } catch (Exception e) {
                BridgeLog.Warn ("event bus could not hook " + name + ": " + e.Message);
            }
        }

        static string OriginVesselId (EventReport report)
        {
            return report != null && report.origin != null && report.origin.vessel != null
                ? Id (report.origin.vessel) : string.Empty;
        }

        static string OriginPartName (EventReport report)
        {
            return report != null && report.origin != null && report.origin.partInfo != null
                ? report.origin.partInfo.name : string.Empty;
        }

        static string Id (Vessel v)
        {
            return v == null ? string.Empty : v.id.ToString ();
        }

        static string ActiveId ()
        {
            return FlightGlobals.ActiveVessel == null
                ? string.Empty : FlightGlobals.ActiveVessel.id.ToString ();
        }
    }
}
