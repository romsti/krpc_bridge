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
    ///
    /// VERSION 2 (refactor lot 3, line 3.3; audit C, R6). A second ring, <see cref="EventLogV2"/>,
    /// read by conn.bridge.poll_events_v2 in ONE call per client tick: every record carries the
    /// master tick of the game clock (<see cref="GameClock.Tick"/>, the last physics step opened
    /// when the event fired) and the UT, and names its vessels and part by persistentId. It
    /// receives every record of the first log (same kinds, same detail, plus the identities the
    /// v1 row never had) and the events of R6 that the first log never had: decoupling with both
    /// vessels, joint breaks with the force, part count changes, physics easing, rails, warp,
    /// loading, and the contact transitions KRPC.Bridge.Telemetry publishes. The first log, its
    /// rows, its ids and its signals are unchanged byte for byte: poll_events and on_event see
    /// exactly what they saw. The V2-only kinds fire no signal.
    ///
    /// (06/10, it killed the Core in game): GameEvents' EventData.Add dereferences the
    /// delegate's Target, so a delegate to a STATIC method throws there. Every hook below is a
    /// lambda (compiled onto the closure singleton: a Target), each in its own try/catch, and the
    /// V2 hooks are kept in fields so <see cref="UnsubscribeV2"/> removes exactly what was added.
    /// </summary>
    public static class EventBus
    {
        const int Capacity = 512;

        // The V2 ring (protocol and schema: EventLogV2.Protocol, EventLogV2.Schema).
        static readonly EventLogV2 logV2 = new EventLogV2 ();

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
        /// Records one event into the V2 ring only (no row in the first log, no signal), stamped
        /// with the UT now. A plugin publishes its own V2 events here (KRPC.Bridge.Telemetry: the
        /// contact transitions). <paramref name="tick"/> is the master tick the event belongs to
        /// (a plugin running inside a physics step passes <see cref="GameClock.StepTick"/>); a
        /// negative value takes <see cref="GameClock.Tick"/>. Returns the sequence number, 0 when
        /// nothing could be recorded. Never throws.
        /// </summary>
        /// <param name="kind">Event kind, e.g. "contact.first". Prefix a plugin's own kinds with its domain.</param>
        /// <param name="tick">Master tick of the event, or -1 for the current one.</param>
        /// <param name="vesselPid">persistentId of the vessel it concerns, 0 when none.</param>
        /// <param name="otherPid">A second persistentId the kind names (documented per kind), 0 when none.</param>
        /// <param name="partPid">persistentId of the part it concerns, 0 when none.</param>
        /// <param name="value">A number the kind documents (SI), NaN when none.</param>
        /// <param name="detail">Free text, never parsed by the bridge.</param>
        public static long RecordV2 (string kind, long tick, uint vesselPid, uint otherPid, uint partPid,
            double value, string detail)
        {
            try {
                var ut = UniversalTime ();
                if (tick < 0)
                    tick = GameClock.Tick;
                lock (gate)
                    return logV2.Append (kind, tick, ut, vesselPid, string.Empty, otherPid, partPid, value, detail);
            } catch (Exception) {
                return 0;
            }
        }

        /// <summary>poll_events_v2's text table (<see cref="EventLogV2.Lines"/>), at the current tick and UT.</summary>
        public static List<string> PollV2 (long sinceSeq, int maxRows)
        {
            var ut = UniversalTime ();
            lock (gate)
                return logV2.Lines (sinceSeq, maxRows, GameClock.Tick, ut);
        }

        /// <summary>The highest V2 sequence number given this session (0 before the first record).</summary>
        public static long LastSeqV2 {
            get { lock (gate) return logV2.LastSeq; }
        }

        /// <summary>
        /// Record an event. Plugins may call this to publish their own, so a plugin that
        /// wraps a mod exposing C# events can surface them on the same channel
        /// (StageRecovery's RecoveryProcessingComplete, for instance).
        /// </summary>
        public static void Record (string kind, string vesselId = "", string detail = "")
        {
            RecordBoth (kind, vesselId, detail, 0u, 0u, 0u, double.NaN);
        }

        /// <summary>
        /// One record of the first log (exactly what <see cref="Record"/> always wrote, signals
        /// included) and its copy in the V2 ring, with the identities the v1 row has no column for.
        /// </summary>
        static void RecordBoth (string kind, string vesselId, string detail, uint vesselPid, uint otherPid,
            uint partPid, double value)
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
                // Version 2: the same record, stamped with the master tick and the persistentIds.
                logV2.Append (name, GameClock.Tick, ut, vesselPid, vesselId, otherPid, partPid, value, detail);

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

            // The first log's hooks: the same v1 row as always (kind, Guid, detail), and in the V2
            // ring the persistentIds and a number the v1 row has no column for.
            Hook ("vessel.destroy", () => GameEvents.onVesselDestroy.Add (v =>
                RecordBoth ("vessel.destroy", Id (v), v != null ? v.vesselName : "", Pid (v), 0u, 0u, double.NaN)));

            Hook ("vessel.change", () => GameEvents.onVesselChange.Add (v =>
                RecordBoth ("vessel.change", Id (v), v != null ? v.vesselName : "", Pid (v), 0u, 0u, double.NaN)));

            Hook ("vessel.recovered", () => GameEvents.onVesselRecovered.Add ((pv, _) =>
                RecordBoth ("vessel.recovered",
                        pv != null ? pv.vesselID.ToString () : "",
                        pv != null ? pv.vesselName : "",
                        pv != null ? pv.persistentId : 0u, 0u, 0u, double.NaN)));

            Hook ("vessel.situation", () => GameEvents.onVesselSituationChange.Add (data =>
                RecordBoth ("vessel.situation", Id (data.host), data.from + " -> " + data.to,
                        Pid (data.host), 0u, 0u, (int)data.to)));

            Hook ("vessel.soi", () => GameEvents.onVesselSOIChanged.Add (data =>
                RecordBoth ("vessel.soi", Id (data.host),
                        (data.from != null ? data.from.bodyName : "?") + " -> " +
                        (data.to != null ? data.to.bodyName : "?"), Pid (data.host), 0u, 0u, double.NaN)));

            Hook ("stage.activate", () => GameEvents.onStageActivate.Add (stage =>
                RecordBoth ("stage.activate", ActiveId (), stage.ToString (), Pid (FlightGlobals.ActiveVessel),
                        0u, 0u, stage)));

            Hook ("part.die", () => GameEvents.onPartDie.Add (p =>
                RecordBoth ("part.die",
                        p != null && p.vessel != null ? Id (p.vessel) : "",
                        p != null && p.partInfo != null ? p.partInfo.name : "",
                        p != null ? Pid (p.vessel) : 0u, 0u, p != null ? p.persistentId : 0u, double.NaN)));

            Hook ("part.crash", () => GameEvents.onCrash.Add (report =>
                RecordBoth ("part.crash", OriginVesselId (report), OriginPartName (report),
                        OriginVesselPid (report), 0u, OriginPartPid (report), double.NaN)));

            Hook ("part.splashdown", () => GameEvents.onCrashSplashdown.Add (report =>
                RecordBoth ("part.splashdown", OriginVesselId (report), OriginPartName (report),
                        OriginVesselPid (report), 0u, OriginPartPid (report), double.NaN)));

            Hook ("scene.load", () => GameEvents.onGameSceneLoadRequested.Add (scene =>
                RecordBoth ("scene.load", "", scene.ToString (), 0u, 0u, 0u, (int)scene)));

            BridgeLog.Info ("event bus subscribed");

            SubscribeV2 ();
        }

        // The V2-only hooks, kept so UnsubscribeV2 removes exactly them (EventData.Remove matches
        // the delegate). Lambdas, never method groups.
        static EventData<Vessel, Vessel>.OnEvent onDecouple;
        static EventData<Vessel, Vessel>.OnEvent onUndock;
        static EventData<PartJoint, float>.OnEvent onJointBreak;
        static EventData<Vessel>.OnEvent onPartCount;
        static EventData<Vessel>.OnEvent onEaseStart;
        static EventData<Vessel>.OnEvent onEaseStop;
        static EventData<Vessel>.OnEvent onOffRails;
        static EventData<Vessel>.OnEvent onOnRails;
        static EventData<Vessel>.OnEvent onCreate;
        static EventData<Vessel>.OnEvent onLoaded;
        static EventData<Vessel>.OnEvent onUnloaded;
        static EventVoid.OnEvent onWarp;
        static bool subscribedV2;

        /// <summary>
        /// The events of R6 that the first log never had, into the V2 ring only (no v1 row, no
        /// signal). Members (KSP 1.12.5, installed Assembly-CSharp, decompiled
        /// GameEvents): onPartDeCoupleNewVesselComplete EventData&lt;Vessel, Vessel&gt; (old, new),
        /// onVesselsUndocking EventData&lt;Vessel, Vessel&gt;, onPartJointBreak
        /// EventData&lt;PartJoint, float&gt; (fired by PartJoint.OnJointBreak with Unity's break
        /// force), onVesselPartCountChanged, onPhysicsEaseStart, onPhysicsEaseStop,
        /// onVesselGoOffRails, onVesselGoOnRails, onVesselCreate, onVesselLoaded, onVesselUnloaded
        /// EventData&lt;Vessel&gt;, onTimeWarpRateChanged EventVoid. Each hook on its own: a
        /// moved member costs that one event.
        /// </summary>
        static void SubscribeV2 ()
        {
            if (subscribedV2)
                return;
            subscribedV2 = true;
            // (old vessel, new vessel) at the end of Part.decouple: THE event that binds a separated
            // booster's persistentId to its tick (double flights, ascent).
            onDecouple = (a, b) => V2 ("vessel.decouple", a, Pid (b), 0u, double.NaN, b != null ? b.vesselName : "");
            Hook ("vessel.decouple", () => GameEvents.onPartDeCoupleNewVesselComplete.Add (onDecouple));
            onUndock = (a, b) => V2 ("vessel.undock", a, Pid (b), 0u, double.NaN, b != null ? b.vesselName : "");
            Hook ("vessel.undock", () => GameEvents.onVesselsUndocking.Add (onUndock));
            // Unity's break force is in KSP's force unit, kN (masses in tonnes): published in N.
            onJointBreak = (joint, force) => JointBreak (joint, force);
            Hook ("part.joint_break", () => GameEvents.onPartJointBreak.Add (onJointBreak));
            onPartCount = v => V2 ("vessel.part_count", v, 0u, 0u,
                v != null && v.parts != null ? v.parts.Count : double.NaN, "");
            Hook ("vessel.part_count", () => GameEvents.onVesselPartCountChanged.Add (onPartCount));
            // StartEasing (unpacking): KSP ramps gravity from 0 and LOCKS the active vessel's
            // commands; nothing written to a vessel between ease_start and ease_stop is applied.
            onEaseStart = v => V2 ("vessel.ease_start", v, 0u, 0u, double.NaN, "");
            Hook ("vessel.ease_start", () => GameEvents.onPhysicsEaseStart.Add (onEaseStart));
            onEaseStop = v => V2 ("vessel.ease_stop", v, 0u, 0u, double.NaN, "");
            Hook ("vessel.ease_stop", () => GameEvents.onPhysicsEaseStop.Add (onEaseStop));
            onOffRails = v => V2 ("vessel.off_rails", v, 0u, 0u, double.NaN, "");
            Hook ("vessel.off_rails", () => GameEvents.onVesselGoOffRails.Add (onOffRails));
            onOnRails = v => V2 ("vessel.on_rails", v, 0u, 0u, double.NaN, "");
            Hook ("vessel.on_rails", () => GameEvents.onVesselGoOnRails.Add (onOnRails));
            onCreate = v => V2 ("vessel.create", v, 0u, 0u, double.NaN, v != null ? v.vesselName : "");
            Hook ("vessel.create", () => GameEvents.onVesselCreate.Add (onCreate));
            onLoaded = v => V2 ("vessel.load", v, 0u, 0u, double.NaN, "");
            Hook ("vessel.load", () => GameEvents.onVesselLoaded.Add (onLoaded));
            onUnloaded = v => V2 ("vessel.unload", v, 0u, 0u, double.NaN, "");
            Hook ("vessel.unload", () => GameEvents.onVesselUnloaded.Add (onUnloaded));
            onWarp = () => V2 ("warp.rate", FlightGlobals.ActiveVessel, 0u, 0u, TimeWarp.CurrentRate, "");
            Hook ("warp.rate", () => GameEvents.onTimeWarpRateChanged.Add (onWarp));
            BridgeLog.Info ("event bus v2 subscribed");
        }

        /// <summary>
        /// Removes the V2-only hooks (BridgeCore.OnDestroy: the Core persists across scenes, so this
        /// runs when the game quits). Each removal on its own. The first log's hooks stay for the
        /// session, as they always did.
        /// </summary>
        internal static void UnsubscribeV2 ()
        {
            if (!subscribedV2)
                return;
            subscribedV2 = false;
            Unhook ("vessel.decouple", () => GameEvents.onPartDeCoupleNewVesselComplete.Remove (onDecouple));
            Unhook ("vessel.undock", () => GameEvents.onVesselsUndocking.Remove (onUndock));
            Unhook ("part.joint_break", () => GameEvents.onPartJointBreak.Remove (onJointBreak));
            Unhook ("vessel.part_count", () => GameEvents.onVesselPartCountChanged.Remove (onPartCount));
            Unhook ("vessel.ease_start", () => GameEvents.onPhysicsEaseStart.Remove (onEaseStart));
            Unhook ("vessel.ease_stop", () => GameEvents.onPhysicsEaseStop.Remove (onEaseStop));
            Unhook ("vessel.off_rails", () => GameEvents.onVesselGoOffRails.Remove (onOffRails));
            Unhook ("vessel.on_rails", () => GameEvents.onVesselGoOnRails.Remove (onOnRails));
            Unhook ("vessel.create", () => GameEvents.onVesselCreate.Remove (onCreate));
            Unhook ("vessel.load", () => GameEvents.onVesselLoaded.Remove (onLoaded));
            Unhook ("vessel.unload", () => GameEvents.onVesselUnloaded.Remove (onUnloaded));
            Unhook ("warp.rate", () => GameEvents.onTimeWarpRateChanged.Remove (onWarp));
            BridgeLog.Info ("event bus v2 unsubscribed");
        }

        /// <summary>One V2-only record from a hook. Never throws into KSP's dispatch.</summary>
        static void V2 (string kind, Vessel vessel, uint otherPid, uint partPid, double value, string detail)
        {
            try {
                RecordV2 (kind, -1, Pid (vessel), otherPid, partPid, value, detail);
            } catch (Exception) {
            }
        }

        /// <summary>
        /// part.joint_break: the vessel and the part of the child side (PartJoint.Child, else the
        /// host's vessel), the parent PART's persistentId as other_pid, the force in N.
        /// </summary>
        static void JointBreak (PartJoint joint, float breakForce)
        {
            try {
                Part child = joint != null ? joint.Child : null;
                Part parent = joint != null ? joint.Parent : null;
                Part host = joint != null ? joint.Host : null;
                Vessel vessel = child != null && child.vessel != null ? child.vessel
                    : (host != null ? host.vessel : null);
                string detail = child != null && child.partInfo != null ? child.partInfo.name : "";
                RecordV2 ("part.joint_break", -1, Pid (vessel), parent != null ? parent.persistentId : 0u,
                    child != null ? child.persistentId : 0u, breakForce * Frames.KilonewtonToNewton, detail);
            } catch (Exception) {
            }
        }

        static void Unhook (string name, Action unsubscribe)
        {
            try {
                unsubscribe ();
            } catch (Exception e) {
                BridgeLog.Warn ("event bus could not unhook " + name + ": " + e.Message);
            }
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

        static uint Pid (Vessel v)
        {
            return v == null ? 0u : v.persistentId;
        }

        static uint OriginVesselPid (EventReport report)
        {
            return report != null && report.origin != null && report.origin.vessel != null
                ? report.origin.vessel.persistentId : 0u;
        }

        static uint OriginPartPid (EventReport report)
        {
            return report != null && report.origin != null ? report.origin.persistentId : 0u;
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
