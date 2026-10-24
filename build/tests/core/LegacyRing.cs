// The frame rings as they flew until refactor R1, VERBATIM, kept here as the
// reference the Core v2 Recorder is compared with. Do not "fix" anything in this file: its
// whole value is that it is the old code.
//
//   * Push / trim: DynamicsV3.CaptureInto, DynamicsExt.Capture, AeroActuatorV1.Capture,
//     AttitudeV1.WriteFrame, AttitudeV1Translation.WriteTranslationFrame:
//         history.AddLast (frame); while (history.Count > HistoryCapacity) history.RemoveFirst ();
//   * ReadRing: src/Plugins/Actuators/DynamicsV3.cs, now Telemetry (V3, tracked, Ext, AttitudeV1,
//     translation);
//   * ReadHistoryAero: src/Plugins/Actuators/AeroActuatorV1.cs, now Telemetry (its own copy).

using System;
using System.Collections.Generic;

namespace KRPC.Bridge.Core
{
    internal static class LegacyRing
    {
        internal static void Push (LinkedList<double[]> history, double[] frame, int capacity)
        {
            history.AddLast (frame);
            while (history.Count > capacity)
                history.RemoveFirst ();
        }

        internal static IList<double> ReadRing (
            LinkedList<double[]> history, int protocol, int schema,
            int sinceTick, int maxFrames)
        {
            int limit = Math.Min (128, Math.Max (1, maxFrames));
            long oldestTick = history.First == null ? 0 : FrameTick (history.First.Value);
            long newestTick = history.Last == null ? 0 : FrameTick (history.Last.Value);
            bool droppedBefore = sinceTick > 0 && oldestTick > 0 && sinceTick < oldestTick - 1;

            var selected = new List<double[]> (limit);
            for (var node = history.First; node != null && selected.Count < limit;
                    node = node.Next) {
                if (FrameTick (node.Value) > sinceTick)
                    selected.Add (node.Value);
            }

            // history header: protocol, schema, frame_count, oldest_tick, newest_tick,
            // dropped_before. Every frame is length-prefixed afterwards.
            var output = new List<double> (6 + selected.Count * 128) {
                protocol,
                schema,
                selected.Count,
                oldestTick,
                newestTick,
                droppedBefore ? 1.0 : 0.0
            };
            for (int i = 0; i < selected.Count; i++) {
                output.Add (selected [i].Length);
                output.AddRange (selected [i]);
            }
            return output;
        }

        static long FrameTick (double[] frame)
        {
            if (frame == null || frame.Length < 3)
                return 0;
            return (long)frame [2];
        }

        internal static IList<double> ReadHistoryAero (LinkedList<double[]> history,
            int protocolVersion, int schemaVersion, int sinceTick, int maxFrames)
        {
            int limit = Math.Min (128, Math.Max (1, maxFrames));
            long oldestTick = history.First == null ? 0 : FrameTickAero (history.First.Value);
            long newestTick = history.Last == null ? 0 : FrameTickAero (history.Last.Value);
            bool droppedBefore = sinceTick > 0 && oldestTick > 0 && sinceTick < oldestTick - 1;

            var selected = new List<double[]> (limit);
            for (var node = history.First; node != null && selected.Count < limit;
                    node = node.Next) {
                if (FrameTickAero (node.Value) > sinceTick)
                    selected.Add (node.Value);
            }

            var output = new List<double> (6 + selected.Count * 64) {
                protocolVersion, schemaVersion, selected.Count,
                oldestTick, newestTick, droppedBefore ? 1.0 : 0.0
            };
            for (int i = 0; i < selected.Count; i++) {
                output.Add (selected [i].Length);
                output.AddRange (selected [i]);
            }
            return output;
        }

        static long FrameTickAero (double[] frame)
        {
            return frame != null && frame.Length >= 3 ? (long)frame [2] : 0;
        }

        // ---------------------------------------------------------------- leases
        // The legacy lease arithmetic, one expression per site (all floats).

        // ActuatorsAddon: controllerExpiresAt = Time.realtimeSinceStartup + leaseSeconds;
        // DynamicsExt / DynamicsTracked / AttitudeV1: expiresAt = realtime + leaseSeconds.
        internal static float ExpiresAt (float now, float seconds)
        {
            float stored = now + seconds;
            return stored;
        }

        // CommandInto: Time.realtimeSinceStartup + Math.Min (1f, Math.Max (0.05f, leaseSeconds))
        internal static float ThrottleExpiresAt (float now, float leaseSeconds)
        {
            float stored = now + Math.Min (1f, Math.Max (0.05f, leaseSeconds));
            return stored;
        }

        // Pump: if (now >= controllerExpiresAt) ...; HoldsForAttitude: realtime < expiresAt.
        internal static bool Expired (float now, float expiresAt)
        {
            return now >= expiresAt;
        }

        // ControlStatus: Math.Max (0f, controllerExpiresAt - Time.realtimeSinceStartup)
        internal static double Remaining (float expiresAt, float now)
        {
            List<double> wire = new List<double> { Math.Max (0f, expiresAt - now) };
            return wire [0];
        }
    }
}
