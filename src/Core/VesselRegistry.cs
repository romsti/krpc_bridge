using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// The vessels the mod sees, by persistentId: active or not, loaded, packed, a topology
    /// generation moved by GameEvents, and the topology FINGERPRINT (the hash every Actuators
    /// transport uses to decide that its actuator rows still mean the same modules).
    ///
    /// The fingerprint walks every part and module of a vessel. Three Actuators paths computed
    /// it, up to three times per vessel per physics step (A §3.2). Here it is computed at most
    /// ONCE per vessel and per WINDOW: a window is one uninterrupted stretch of our own code
    /// (the Actuators pump, the AttitudeV1 Earlyish tick), opened and closed by its owner, in
    /// which no game code runs between two reads, so the memo returns exactly what a new walk
    /// would. Outside a window (an RPC body) every call walks. That is what keeps refactor R1
    /// byte-identical.
    ///
    /// One fingerprint per vessel and per STEP would need the memo to survive from Earlyish to
    /// the pump, across part FixedUpdates and kRPC's RPC window (where a staging RPC can split
    /// the vessel). The registry measures whether GameEvents would be enough to invalidate it:
    /// when a window recomputes a fingerprint another window of the same step already had, with
    /// no topology event in between, <see cref="VesselRecord.UneventedChanges"/> counts the
    /// times the value moved anyway. Zero over enough flights is the evidence to arm a per-step
    /// memo; until then nothing reuses a value across windows.
    /// </summary>
    public static class VesselRegistry
    {
        /// <summary>Protocol of vessels_v1.</summary>
        public const int Protocol = 1;

        /// <summary>Columns of vessels_v1, appended only.</summary>
        public static readonly string[] Columns = {
            "persistent_id", "active", "loaded", "packed", "event_generation", "last_event_tick",
            "fingerprint", "fingerprint_tick", "walks", "memo_hits", "cross_window_checks",
            "unevented_changes", "evented_changes", "name"
        };

        const int MaxRecords = 64;

        static readonly Dictionary<uint, VesselRecord> records = new Dictionary<uint, VesselRecord> ();
        static readonly List<uint> pruneScratch = new List<uint> ();
        static int window;
        static int windowSerial;
        static bool subscribed;

        /// <summary>
        /// A persistentId given as a decimal string, the way every bridge RPC names a vessel
        /// (refactor R3: one parser for Control, Telemetry and Trajectories). The refusal is
        /// the wire text the Actuators RPCs always gave.
        /// </summary>
        public static uint ParsePersistentId (string persistentId)
        {
            uint id;
            if (!uint.TryParse (persistentId, NumberStyles.None, CultureInfo.InvariantCulture, out id))
                throw new ArgumentException ("persistentId doit etre un uint decimal",
                    nameof (persistentId));
            return id;
        }

        /// <summary>The loaded vessel of that persistentId (FlightGlobals.VesselsLoaded), or null.</summary>
        public static Vessel FindLoaded (uint persistentId)
        {
            var loaded = FlightGlobals.VesselsLoaded;
            if (loaded == null)
                return null;
            for (int i = 0; i < loaded.Count; i++) {
                var vessel = loaded [i];
                if (vessel != null && vessel.loaded && vessel.persistentId == persistentId)
                    return vessel;
            }
            return null;
        }

        /// <summary>The record of a persistentId, or null.</summary>
        public static VesselRecord Find (uint persistentId)
        {
            VesselRecord record;
            return records.TryGetValue (persistentId, out record) ? record : null;
        }

        /// <summary>Records kept now.</summary>
        public static int Count {
            get { return records.Count; }
        }

        // -------------------------------------------------------------------- windows

        /// <summary>
        /// Opens a window and returns the previous one, to give back to <see cref="CloseWindow"/>.
        /// Windows nest (a fallback tick inside the pump): an inner window shares no memo with
        /// the outer one.
        /// </summary>
        public static int OpenWindow ()
        {
            int previous = window;
            windowSerial++;
            if (windowSerial <= 0)
                windowSerial = 1;
            window = windowSerial;
            return previous;
        }

        /// <summary>Closes the current window and restores the one OpenWindow returned.</summary>
        public static void CloseWindow (int previous)
        {
            window = previous;
        }

        // ---------------------------------------------------------------- fingerprint

        /// <summary>
        /// The topology fingerprint of a loaded vessel: memoized within the open window (same
        /// vessel object, same master tick), walked otherwise. The value is
        /// <see cref="ComputeTopologyHash"/>'s, always.
        /// </summary>
        public static ulong TopologyHash (Vessel vessel)
        {
            if (vessel == null)
                throw new ArgumentNullException (nameof (vessel));
            long tick = GameClock.Tick;
            VesselRecord record = Record (vessel);
            if (window != 0 && record.MemoWindow == window && record.FingerprintTick == tick &&
                    ReferenceEquals (record.MemoVessel, vessel)) {
                record.MemoHits++;
                return record.Fingerprint;
            }
            ulong hash = ComputeTopologyHash (vessel);
            record.Walks++;
            // Would a value from another window of this step have been right?
            if (record.FingerprintTick == tick && record.MemoWindow != window &&
                    ReferenceEquals (record.MemoVessel, vessel)) {
                record.CrossWindowChecks++;
                if (hash != record.Fingerprint) {
                    if (record.EventSerialAtMemo != record.EventGeneration)
                        record.EventedChanges++;
                    else
                        record.UneventedChanges++;
                }
            }
            record.Fingerprint = hash;
            record.FingerprintTick = tick;
            record.MemoWindow = window;
            record.MemoVessel = vessel;
            record.EventSerialAtMemo = record.EventGeneration;
            return hash;
        }

        /// <summary>
        /// The fingerprint itself (moved here unchanged from ActuatorsAddon.ComputeTopologyHash):
        /// FNV-1a over persistentId, the part count, and per part its flightID, its module
        /// count and, per engine and gimbal module, its thrust/gimbal transform count.
        /// </summary>
        public static ulong ComputeTopologyHash (Vessel vessel)
        {
            unchecked {
                ulong hash = 1469598103934665603UL;
                MixHash (ref hash, vessel.persistentId);
                MixHash (ref hash, (uint)vessel.parts.Count);
                for (int p = 0; p < vessel.parts.Count; p++) {
                    var part = vessel.parts [p];
                    if (part == null) {
                        MixHash (ref hash, UInt32.MaxValue);
                        continue;
                    }
                    MixHash (ref hash, part.flightID);
                    MixHash (ref hash, (uint)part.Modules.Count);
                    for (int m = 0; m < part.Modules.Count; m++) {
                        var module = part.Modules [m];
                        if (module == null) {
                            MixHash (ref hash, UInt32.MaxValue);
                            continue;
                        }
                        var engine = module as ModuleEngines;
                        if (engine != null)
                            MixHash (ref hash, (uint)(engine.thrustTransforms == null
                                ? 0 : engine.thrustTransforms.Count));
                        var gimbal = module as ModuleGimbal;
                        if (gimbal != null)
                            MixHash (ref hash, (uint)(gimbal.gimbalTransforms == null
                                ? 0 : gimbal.gimbalTransforms.Count));
                    }
                }
                return hash;
            }
        }

        static void MixHash (ref ulong hash, uint value)
        {
            unchecked {
                hash ^= value;
                hash *= 1099511628211UL;
            }
        }

        // -------------------------------------------------------------------- records

        static VesselRecord Record (Vessel vessel)
        {
            uint pid = vessel.persistentId;
            VesselRecord record;
            if (!records.TryGetValue (pid, out record)) {
                if (records.Count >= MaxRecords)
                    Prune ();
                record = new VesselRecord { PersistentId = pid };
                records.Add (pid, record);
            }
            record.Vessel = vessel;
            return record;
        }

        /// <summary>Drops records of vessels no longer loaded (and the oldest if still full).</summary>
        static void Prune ()
        {
            pruneScratch.Clear ();
            foreach (var pair in records) {
                var v = pair.Value.Vessel;
                if (v == null || !v.loaded)
                    pruneScratch.Add (pair.Key);
            }
            for (int i = 0; i < pruneScratch.Count; i++)
                records.Remove (pruneScratch [i]);
            if (records.Count >= MaxRecords)
                records.Clear ();
        }

        static void Bump (Vessel vessel)
        {
            if (vessel == null)
                return;
            var record = Record (vessel);
            record.EventGeneration++;
            record.LastEventTick = GameClock.Tick;
        }

        // --------------------------------------------------------------------- events

        /// <summary>Hooks the topology GameEvents once per session (BridgeCore.Awake).</summary>
        internal static void Subscribe ()
        {
            if (subscribed)
                return;
            subscribed = true;
            // KSP's EventData.Add dereferences the delegate's Target (EvtDelegate keeps its
            // originator): a delegate to a STATIC method throws a NullReferenceException there.
            // Lambdas are compiled onto the compiler's closure singleton, so they carry a Target.
            // Each hook on its own, so one moved GameEvents member costs that one hook only.
            Hook ("onVesselWasModified", () => GameEvents.onVesselWasModified.Add (v => OnVessel (v)));
            Hook ("onVesselPartCountChanged", () => GameEvents.onVesselPartCountChanged.Add (v => OnVessel (v)));
            Hook ("onVesselCreate", () => GameEvents.onVesselCreate.Add (v => OnVessel (v)));
            Hook ("onVesselGoOnRails", () => GameEvents.onVesselGoOnRails.Add (v => OnVessel (v)));
            Hook ("onVesselGoOffRails", () => GameEvents.onVesselGoOffRails.Add (v => OnVessel (v)));
            Hook ("onVesselChange", () => GameEvents.onVesselChange.Add (v => OnVessel (v)));
            Hook ("onVesselDestroy", () => GameEvents.onVesselDestroy.Add (v => OnVesselDestroy (v)));
            Hook ("onPartDie", () => GameEvents.onPartDie.Add (p => OnPartDie (p)));
            Hook ("onPartDeCoupleNewVesselComplete",
                  () => GameEvents.onPartDeCoupleNewVesselComplete.Add ((a, b) => OnPair (a, b)));
            Hook ("onVesselsUndocking", () => GameEvents.onVesselsUndocking.Add ((a, b) => OnPair (a, b)));
            Hook ("onGameSceneLoadRequested", () => GameEvents.onGameSceneLoadRequested.Add (s => OnSceneLoad (s)));
        }

        static void Hook (string name, Action subscribe)
        {
            try {
                subscribe ();
            } catch (Exception e) {
                BridgeLog.Warn ("vessel registry could not hook " + name + ": " + e.Message);
            }
        }

        static void OnVessel (Vessel vessel)
        {
            try {
                Bump (vessel);
            } catch (Exception) {
                // A registry update must never break a GameEvent chain.
            }
        }

        static void OnPair (Vessel first, Vessel second)
        {
            try {
                Bump (first);
                Bump (second);
            } catch (Exception) {
            }
        }

        static void OnPartDie (Part part)
        {
            try {
                if (part != null)
                    Bump (part.vessel);
            } catch (Exception) {
            }
        }

        static void OnVesselDestroy (Vessel vessel)
        {
            try {
                if (vessel != null)
                    records.Remove (vessel.persistentId);
            } catch (Exception) {
            }
        }

        static void OnSceneLoad (GameScenes scene)
        {
            records.Clear ();
        }

        // ---------------------------------------------------------------- vessels_v1

        /// <summary>
        /// vessels_v1: header line (protocol, tick, active persistentId, n), column line, one row
        /// per record (flags refreshed from FlightGlobals now). Read-only.
        /// </summary>
        public static List<string> Lines ()
        {
            var active = FlightGlobals.ActiveVessel;
            var lines = new List<string> (records.Count + 2);
            var head = new StringBuilder ();
            head.Append ("protocol=").Append (Protocol.ToString (CultureInfo.InvariantCulture));
            head.Append ("\ttick=").Append (GameClock.Tick.ToString (CultureInfo.InvariantCulture));
            head.Append ("\tactive=").Append (active != null
                ? active.persistentId.ToString (CultureInfo.InvariantCulture) : string.Empty);
            head.Append ("\tn=").Append (records.Count.ToString (CultureInfo.InvariantCulture));
            lines.Add (head.ToString ());
            lines.Add (string.Join ("\t", Columns));
            foreach (var pair in records) {
                var r = pair.Value;
                var v = r.Vessel;
                bool loaded = v != null && v.loaded;
                r.Active = v != null && ReferenceEquals (v, active);
                r.Loaded = loaded;
                r.Packed = v != null && v.packed;
                string name = string.Empty;
                try {
                    if (v != null)
                        name = v.vesselName ?? string.Empty;
                } catch (Exception) {
                }
                lines.Add (string.Join ("\t", new[] {
                    r.PersistentId.ToString (CultureInfo.InvariantCulture),
                    r.Active ? "1" : "0",
                    r.Loaded ? "1" : "0",
                    r.Packed ? "1" : "0",
                    r.EventGeneration.ToString (CultureInfo.InvariantCulture),
                    r.LastEventTick.ToString (CultureInfo.InvariantCulture),
                    r.Fingerprint.ToString ("x16", CultureInfo.InvariantCulture),
                    r.FingerprintTick.ToString (CultureInfo.InvariantCulture),
                    r.Walks.ToString (CultureInfo.InvariantCulture),
                    r.MemoHits.ToString (CultureInfo.InvariantCulture),
                    r.CrossWindowChecks.ToString (CultureInfo.InvariantCulture),
                    r.UneventedChanges.ToString (CultureInfo.InvariantCulture),
                    r.EventedChanges.ToString (CultureInfo.InvariantCulture),
                    name.Replace ('\t', ' ').Replace ('\n', ' ').Replace ('\r', ' ')
                }));
            }
            return lines;
        }
    }

    /// <summary>What the registry knows of one vessel (persistentId).</summary>
    public sealed class VesselRecord
    {
        public uint PersistentId;
        public Vessel Vessel;
        public bool Active;
        public bool Loaded;
        public bool Packed;

        /// <summary>+1 at every topology GameEvent naming this vessel.</summary>
        public long EventGeneration;

        /// <summary>Master tick of the last such event.</summary>
        public long LastEventTick;

        public ulong Fingerprint;
        public long FingerprintTick = -1;
        internal int MemoWindow;
        internal Vessel MemoVessel;
        internal long EventSerialAtMemo;

        /// <summary>Fingerprints walked (part and module loops run).</summary>
        public long Walks;

        /// <summary>Fingerprints served from the window memo.</summary>
        public long MemoHits;

        /// <summary>Walks that another window of the same step had already done.</summary>
        public long CrossWindowChecks;

        /// <summary>Of those, the value had changed with a topology event in between.</summary>
        public long EventedChanges;

        /// <summary>Of those, the value had changed WITHOUT any topology event: a per-step memo would have been wrong.</summary>
        public long UneventedChanges;
    }
}
