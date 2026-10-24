using System;
using System.Globalization;
using KRPC.Bridge.Core;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// How an RPC names a vessel and its actuators, written once for Control and Telemetry
    /// (refactor R3: the copies of ParsePersistentId in AttitudeV1, DynamicsTracked and
    /// AeroWrenchBatch, and the active-vessel lookups of the former ActuatorsService, are this
    /// class). A vessel is named by its persistentId as a decimal string; an engine or a
    /// gimbal by its part's flightID (decimal string) and the ordinal of the module among the
    /// part's modules of that kind. The refusal messages are wire text and stay as they were.
    /// </summary>
    internal static class VesselLookup
    {
        /// <summary>The active vessel, loaded; NOT_LOADED otherwise.</summary>
        internal static Vessel RequireActive ()
        {
            var vessel = FlightGlobals.ActiveVessel;
            if (vessel == null || !vessel.loaded)
                throw new InvalidOperationException (BridgeErrors.Tag (
                    BridgeErrors.NotLoaded, "aucun vaisseau actif et charge"));
            return vessel;
        }

        /// <summary>A persistentId given as a decimal string (Core: VesselRegistry.ParsePersistentId).</summary>
        internal static uint ParsePersistentId (string persistentId)
        {
            return VesselRegistry.ParsePersistentId (persistentId);
        }

        /// <summary>A part flightID given as a decimal string.</summary>
        internal static uint ParseFlightId (string flightId)
        {
            uint id;
            if (!uint.TryParse (flightId, NumberStyles.None, CultureInfo.InvariantCulture, out id))
                throw new ArgumentException ("flightId doit etre un uint decimal", nameof (flightId));
            return id;
        }

        /// <summary>The loaded vessel of that persistentId, or null (Core: VesselRegistry.FindLoaded).</summary>
        internal static Vessel FindLoaded (uint persistentId)
        {
            return VesselRegistry.FindLoaded (persistentId);
        }

        /// <summary>The wantedOrdinal-th ModuleEngines of part flightId on the active vessel.</summary>
        internal static ModuleEngines FindEngine (string flightId, int wantedOrdinal)
        {
            uint id = ParseFlightId (flightId);
            if (wantedOrdinal < 0)
                throw new ArgumentOutOfRangeException (nameof (wantedOrdinal));

            var vessel = RequireActive ();
            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                if (part.flightID != id)
                    continue;
                int ordinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var engine = part.Modules [m] as ModuleEngines;
                    if (engine == null)
                        continue;
                    if (ordinal++ == wantedOrdinal)
                        return engine;
                }
                break;
            }
            throw new InvalidOperationException (BridgeErrors.Tag (
                BridgeErrors.NotFound, "moteur absent du vaisseau actif : part " + flightId +
                ", ordinal " + wantedOrdinal.ToString (CultureInfo.InvariantCulture)));
        }

        /// <summary>The wantedOrdinal-th ModuleGimbal of part flightId on the active vessel.</summary>
        internal static ModuleGimbal FindGimbal (string flightId, int wantedOrdinal)
        {
            uint id = ParseFlightId (flightId);
            if (wantedOrdinal < 0)
                throw new ArgumentOutOfRangeException (nameof (wantedOrdinal));

            var vessel = RequireActive ();
            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                if (part.flightID != id)
                    continue;
                int ordinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var gimbal = part.Modules [m] as ModuleGimbal;
                    if (gimbal == null)
                        continue;
                    if (ordinal++ == wantedOrdinal)
                        return gimbal;
                }
                break;
            }
            throw new InvalidOperationException (BridgeErrors.Tag (
                BridgeErrors.NotFound, "gimbal absent du vaisseau actif : part " + flightId +
                ", ordinal " + wantedOrdinal.ToString (CultureInfo.InvariantCulture)));
        }
    }
}
