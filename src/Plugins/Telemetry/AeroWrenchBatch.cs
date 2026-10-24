using System;
using System.Collections.Generic;
using System.Globalization;
using System.Runtime.CompilerServices;
using KRPC.Service.Attributes;
using SCBody = KRPC.SpaceCenter.Services.CelestialBody;
using SCFlight = KRPC.SpaceCenter.Services.Flight;
using SCVessel = KRPC.SpaceCenter.Services.Vessel;
using KRPC.Bridge.Control;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Telemetry
{
    public static partial class TelemetryService
    {
        /// <summary>
        /// Evaluates kRPC's Flight.SimulateAerodynamicWrenchAt for up to 32 hypothetical
        /// states in ONE call. states holds rows of 14 doubles: position xyz (m), velocity
        /// xyz (m/s), rotation xyzw, angular velocity xyz (rad/s), ut; all in the main
        /// body's non-rotating frame (nonRotatingFrame = true, recommended by kRPC for
        /// prediction) or its rotating frame. Returns protocol, row count, stride (7),
        /// frame code, then rows ok, force xyz (N), torque xyz (N.m about the CoM) in the
        /// same frame. persistentId empty = active vessel, else a loaded vessel. Read-only.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> SimulateAerodynamicWrenchBatchV1 (
            IList<double> states, bool nonRotatingFrame = true, string persistentId = "")
        {
            return AeroWrenchBatch.Simulate (states, nonRotatingFrame, persistentId);
        }
    }

    internal static class AeroWrenchBatch
    {
        internal const int ProtocolVersion = 1;
        internal const int InputStride = 14;
        internal const int OutputStride = 7;
        // Each state walks every part's drag cubes on the Unity thread: keep one RPC short.
        internal const int MaxStates = 32;

        internal static IList<double> Simulate (
            IList<double> states, bool nonRotatingFrame, string persistentId)
        {
            if (states == null)
                throw new ArgumentNullException (nameof (states));
            if (states.Count % InputStride != 0)
                throw new ArgumentException (
                    "states doit contenir des lignes de 14 doubles", nameof (states));
            int count = states.Count / InputStride;
            if (count > MaxStates)
                throw new ArgumentOutOfRangeException (
                    nameof (states), "au plus 32 etats par appel");
            var vessel = ResolveVessel (persistentId);

            var output = new List<double> (4 + count * OutputStride) {
                ProtocolVersion,
                count,
                OutputStride,
                nonRotatingFrame ? 1.0 : 0.0
            };
            if (count == 0)
                return output;
            EvaluateAll (vessel, states, count, nonRotatingFrame, output);
            return output;
        }

        static Vessel ResolveVessel (string persistentId)
        {
            if (string.IsNullOrEmpty (persistentId))
                return VesselLookup.RequireActive ();
            var vessel = VesselLookup.FindLoaded (VesselLookup.ParsePersistentId (persistentId));
            if (vessel != null)
                return vessel;
            throw new InvalidOperationException (KRPC.Bridge.Core.BridgeErrors.Tag (
                KRPC.Bridge.Core.BridgeErrors.NotLoaded, "vaisseau " + persistentId + " absent ou non charge"));
        }

        // Kept out of line: the kRPC member is resolved when THIS method is compiled, so a
        // kRPC build without it fails this RPC only, never the rest of the service.
        [MethodImpl (MethodImplOptions.NoInlining)]
        static void EvaluateAll (
            Vessel vessel, IList<double> states, int count, bool nonRotatingFrame,
            List<double> output)
        {
            var scVessel = new SCVessel (vessel);
            var body = new SCBody (vessel.mainBody);
            var frame = nonRotatingFrame ? body.NonRotatingReferenceFrame : body.ReferenceFrame;
            SCFlight flight = scVessel.Flight (frame);
            for (int row = 0; row < count; row++) {
                int o = row * InputStride;
                bool finite = true;
                for (int k = 0; k < InputStride; k++) {
                    double value = states [o + k];
                    if (double.IsNaN (value) || double.IsInfinity (value)) {
                        finite = false;
                        break;
                    }
                }
                if (!finite) {
                    AddFailed (output);
                    continue;
                }
                try {
                    var wrench = flight.SimulateAerodynamicWrenchAt (
                        body,
                        Tuple.Create (states [o], states [o + 1], states [o + 2]),
                        Tuple.Create (states [o + 3], states [o + 4], states [o + 5]),
                        Tuple.Create (states [o + 6], states [o + 7], states [o + 8], states [o + 9]),
                        Tuple.Create (states [o + 10], states [o + 11], states [o + 12]),
                        states [o + 13]);
#if KRPC_07
                    // kRPC 0.7: SpaceCenter.Wrench is a struct (Force, Torque)
                    var force = wrench.Force;
                    var torque = wrench.Torque;
#else
                    // kRPC 0.6: a pair of tuples (force, torque)
                    var force = wrench.Item1;
                    var torque = wrench.Item2;
#endif
                    output.Add (1.0);
                    output.Add (force.Item1);
                    output.Add (force.Item2);
                    output.Add (force.Item3);
                    output.Add (torque.Item1);
                    output.Add (torque.Item2);
                    output.Add (torque.Item3);
                } catch {
                    AddFailed (output);
                }
            }
        }

        static void AddFailed (List<double> output)
        {
            output.Add (0.0);
            for (int k = 0; k < OutputStride - 1; k++)
                output.Add (double.NaN);
        }
    }
}
