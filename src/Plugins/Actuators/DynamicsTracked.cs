using System;
using System.Collections.Generic;
using System.Globalization;
using KRPC.Service.Attributes;
using UnityEngine;

namespace KRPC.Bridge.Actuators
{
    /// <summary>
    /// Dynamics of a loaded vessel designated by persistentId, active or not.
    ///
    /// DynamicsSnapshotV3 follows FlightGlobals.ActiveVessel, so in a flight with two
    /// boosters the process flying the non-active one reads the OTHER booster's state.
    /// A tracked channel follows one vessel by identity instead, with its own ring, its
    /// own finite differences and its own topology generation, in exactly the v3 frame
    /// format. The active-vessel stream is unchanged.
    ///
    /// Born closed: nothing is tracked until a client calls TrackVesselV3, and each
    /// registration is a real-time lease the client must renew. The persistentId of a
    /// kRPC Vessel handle comes from conn.ident.vessel_ids(vessel) (first tab field).
    /// </summary>
    public static partial class ActuatorsService
    {
        /// <summary>
        /// Starts or renews tracking of a vessel by persistentId (decimal string) for
        /// leaseSeconds of real time (0.5-60 s). At most four vessels are tracked at once.
        /// Returns whether that vessel is loaded right now.
        /// </summary>
        [KRPCProcedure]
        public static bool TrackVesselV3 (string persistentId, float leaseSeconds = 5f)
        {
            return DynamicsTracked.Track (ParsePersistentId (persistentId), leaseSeconds);
        }

        /// <summary>Stops tracking a vessel. Returns false if it was not tracked.</summary>
        [KRPCProcedure]
        public static bool UntrackVesselV3 (string persistentId)
        {
            return DynamicsTracked.Untrack (ParsePersistentId (persistentId));
        }

        /// <summary>
        /// Header protocol, count, stride (6), then one row per tracked vessel:
        /// persistent_id, loaded, lease_remaining_s, topology_generation, latest_tick,
        /// history_count.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> TrackedVesselsV3 ()
        {
            return DynamicsTracked.ReadRegistry ();
        }

        /// <summary>
        /// Latest v3 frame of a tracked vessel (DynamicsSnapshotV3 layout). Empty while the
        /// vessel is not loaded. Raises if the vessel is not tracked.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> TrackedDynamicsSnapshotV3 (string persistentId)
        {
            return DynamicsTracked.Channel (ParsePersistentId (persistentId)).ReadLatest ();
        }

        /// <summary>
        /// History of a tracked vessel newer than sinceTick, in the DynamicsFramesV3 layout.
        /// Raises if the vessel is not tracked.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> TrackedDynamicsFramesV3 (
            string persistentId, int sinceTick = 0, int maxFrames = 64)
        {
            return DynamicsTracked.Channel (ParsePersistentId (persistentId))
                .ReadHistory (sinceTick, maxFrames);
        }

        /// <summary>
        /// DynamicsExtV1 history of a tracked vessel (captured only while DynamicsExtV1 is
        /// armed). Raises if the vessel is not tracked.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> TrackedDynamicsExtFramesV1 (
            string persistentId, int sinceTick = 0, int maxFrames = 64)
        {
            return DynamicsTracked.Channel (ParsePersistentId (persistentId))
                .ext.ReadHistory (sinceTick, maxFrames);
        }

        static uint ParsePersistentId (string persistentId)
        {
            uint id;
            if (!uint.TryParse (persistentId, NumberStyles.None, CultureInfo.InvariantCulture, out id))
                throw new ArgumentException (
                    "persistentId doit etre un uint decimal", nameof (persistentId));
            return id;
        }
    }

    internal static class DynamicsTracked
    {
        internal const int ProtocolVersion = 3;
        internal const int MaxTracked = 4;
        internal const int RegistryStride = 6;
        const float MinLeaseSeconds = 0.5f;
        const float MaxLeaseSeconds = 60f;

        sealed class Entry
        {
            internal uint PersistentId;
            internal float ExpiresAt;
            internal readonly DynamicsChannel Channel = new DynamicsChannel ();
            internal Guid VesselId = Guid.Empty;
            internal ulong TopologyHash;
            internal long TopologyGeneration;
            internal bool Loaded;
        }

        static readonly Dictionary<uint, Entry> entries = new Dictionary<uint, Entry> ();

        internal static bool Track (uint persistentId, float leaseSeconds)
        {
            if (float.IsNaN (leaseSeconds) || float.IsInfinity (leaseSeconds) ||
                    leaseSeconds < MinLeaseSeconds || leaseSeconds > MaxLeaseSeconds)
                throw new ArgumentOutOfRangeException (
                    nameof (leaseSeconds), "bail de suivi attendu entre 0.5 et 60 s");
            Entry entry;
            if (!entries.TryGetValue (persistentId, out entry)) {
                if (entries.Count >= MaxTracked)
                    throw new InvalidOperationException (
                        "deja " + MaxTracked.ToString (CultureInfo.InvariantCulture) +
                        " vaisseaux suivis : en liberer un avant");
                entry = new Entry { PersistentId = persistentId };
                entries.Add (persistentId, entry);
            }
            entry.ExpiresAt = Time.realtimeSinceStartup + leaseSeconds;
            return FindLoaded (persistentId) != null;
        }

        internal static bool Untrack (uint persistentId)
        {
            Entry entry;
            if (!entries.TryGetValue (persistentId, out entry))
                return false;
            entry.Channel.Reset ();
            entries.Remove (persistentId);
            return true;
        }

        internal static DynamicsChannel Channel (uint persistentId)
        {
            Entry entry;
            if (!entries.TryGetValue (persistentId, out entry))
                throw new InvalidOperationException (
                    "vaisseau " + persistentId.ToString (CultureInfo.InvariantCulture) +
                    " non suivi : appeler track_vessel_v3 d'abord");
            return entry.Channel;
        }

        internal static IList<double> ReadRegistry ()
        {
            float now = Time.realtimeSinceStartup;
            var output = new List<double> (3 + entries.Count * RegistryStride) {
                ProtocolVersion,
                entries.Count,
                RegistryStride
            };
            foreach (var pair in entries) {
                var entry = pair.Value;
                output.Add (entry.PersistentId);
                output.Add (entry.Loaded ? 1.0 : 0.0);
                output.Add (Math.Max (0f, entry.ExpiresAt - now));
                output.Add (entry.TopologyGeneration);
                output.Add (entry.Channel.latestTick);
                output.Add (entry.Channel.history.Count);
            }
            return output;
        }

        /// <summary>
        /// Called by ActuatorsAddon.Pump after the active-vessel capture, same FixedUpdate
        /// and physics tick. Controller metadata is copied into a tracked frame only when
        /// that vessel is the one under v2 control; otherwise those four header fields are 0.
        /// </summary>
        internal static void Pump (
            long physicsTick, Guid controllerVesselId, int lastAcceptedSequence,
            int lastAppliedSequence, long lastAppliedTick, int lastResult)
        {
            if (entries.Count == 0)
                return;
            float now = Time.realtimeSinceStartup;
            List<uint> expired = null;
            foreach (var pair in entries) {
                var entry = pair.Value;
                if (now >= entry.ExpiresAt) {
                    if (expired == null)
                        expired = new List<uint> ();
                    expired.Add (pair.Key);
                    continue;
                }
                try {
                    CaptureOne (entry, physicsTick, controllerVesselId, lastAcceptedSequence,
                        lastAppliedSequence, lastAppliedTick, lastResult);
                } catch {
                    // Observability only: a tracked vessel must never disturb the Pump.
                    entry.Channel.Reset ();
                }
            }
            if (expired == null)
                return;
            for (int i = 0; i < expired.Count; i++) {
                entries [expired [i]].Channel.Reset ();
                entries.Remove (expired [i]);
            }
        }

        static void CaptureOne (
            Entry entry, long physicsTick, Guid controllerVesselId, int lastAcceptedSequence,
            int lastAppliedSequence, long lastAppliedTick, int lastResult)
        {
            var vessel = FindLoaded (entry.PersistentId);
            if (vessel == null) {
                if (entry.Loaded || entry.Channel.latest.Length > 0)
                    entry.Channel.Reset ();
                entry.Loaded = false;
                entry.VesselId = Guid.Empty;
                entry.TopologyHash = 0;
                return;
            }
            ulong hash = ActuatorsAddon.ComputeTopologyHash (vessel);
            if (entry.VesselId != vessel.id || entry.TopologyHash != hash) {
                // Same rule as the active stream: never mix actuator row layouts.
                entry.TopologyGeneration++;
                entry.VesselId = vessel.id;
                entry.TopologyHash = hash;
                entry.Channel.Reset ();
            }
            entry.Loaded = true;
            bool controlled = controllerVesselId != Guid.Empty && vessel.id == controllerVesselId;
            DynamicsV3.CaptureInto (
                entry.Channel, vessel, physicsTick, entry.TopologyGeneration,
                controlled ? lastAcceptedSequence : 0,
                controlled ? lastAppliedSequence : 0,
                controlled ? lastAppliedTick : 0,
                controlled ? lastResult : 0,
                false);
        }

        static Vessel FindLoaded (uint persistentId)
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

        /// <summary>Leaving the flight scene: every tracked vessel is gone.</summary>
        internal static void ResetForScene ()
        {
            foreach (var pair in entries)
                pair.Value.Channel.Reset ();
            entries.Clear ();
        }
    }
}
