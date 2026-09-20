using System;
using System.Collections.Generic;
using System.Globalization;
using System.Runtime.CompilerServices;
using KRPC.Service.Attributes;
using SCBody = KRPC.SpaceCenter.Services.CelestialBody;
using SCFlight = KRPC.SpaceCenter.Services.Flight;
using SCVessel = KRPC.SpaceCenter.Services.Vessel;

namespace KRPC.Bridge.Actuators
{
    public static partial class ActuatorsService
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
            if (string.IsNullOrEmpty (persistentId)) {
                var active = FlightGlobals.ActiveVessel;
                if (active == null || !active.loaded)
                    throw new InvalidOperationException ("aucun vaisseau actif et charge");
                return active;
            }
            uint id;
            if (!uint.TryParse (persistentId, NumberStyles.None, CultureInfo.InvariantCulture, out id))
                throw new ArgumentException (
                    "persistentId doit etre un uint decimal", nameof (persistentId));
            var loaded = FlightGlobals.VesselsLoaded;
            for (int i = 0; loaded != null && i < loaded.Count; i++) {
                var vessel = loaded [i];
                if (vessel != null && vessel.loaded && vessel.persistentId == id)
                    return vessel;
            }
            throw new InvalidOperationException (
                "vaisseau " + persistentId + " absent ou non charge");
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
                    output.Add (1.0);
                    output.Add (wrench.Item1.Item1);
                    output.Add (wrench.Item1.Item2);
                    output.Add (wrench.Item1.Item3);
                    output.Add (wrench.Item2.Item1);
                    output.Add (wrench.Item2.Item2);
                    output.Add (wrench.Item2.Item3);
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
