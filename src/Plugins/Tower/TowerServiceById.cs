using System;
using System.Collections.Generic;
using KRPC.Service.Attributes;

using SCVessel = KRPC.SpaceCenter.Services.Vessel;
using SCReferenceFrame = KRPC.SpaceCenter.Services.ReferenceFrame;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// The vessel procedures of conn.tower addressed by persistentId (refactor R3, 2.7: state per
    /// vessel everywhere). Same answers as the V1 procedures they call; the vessel is the one of
    /// that persistentId (decimal string, the first field of conn.ident.vessel_ids), whichever
    /// vessel is active. A process that has no kRPC handle of a vessel (an observer, a tool, the
    /// non-active booster's own process after a reload) names it the way every other bridge
    /// service does. The part procedures keep their part handle: a part handle is already one
    /// part of one vessel.
    ///
    /// No Core reference (see the csproj): the lookup is the Stock one, over FlightGlobals.Vessels,
    /// refused with [NOT_FOUND] when the game has no such vessel.
    /// </summary>
    public static partial class TowerService
    {
        static SCVessel ByPersistentId (string persistentId)
        {
            uint id;
            if (!uint.TryParse (persistentId, System.Globalization.NumberStyles.None,
                    System.Globalization.CultureInfo.InvariantCulture, out id))
                throw new ArgumentException ("persistentId must be a decimal uint", "persistentId");
            var vessels = FlightGlobals.Vessels;
            for (int i = 0; vessels != null && i < vessels.Count; i++) {
                var v = vessels [i];
                if (v != null && v.persistentId == id)
                    return new SCVessel (v);
            }
            throw new InvalidOperationException ("[NOT_FOUND] no vessel with persistentId " + persistentId);
        }

        /// <summary>PlaceV1 of the vessel of that persistentId (decimal string), the other arguments as PlaceV1.</summary>
        [KRPCProcedure]
        public static bool PlaceV2 (string persistentId, IList<double> position, IList<double> rotation,
                                    SCReferenceFrame referenceFrame, double holdSeconds)
        {
            return PlaceV1 (ByPersistentId (persistentId), position, rotation, referenceFrame, holdSeconds);
        }

        /// <summary>IsAnchored of the vessel of that persistentId (decimal string).</summary>
        [KRPCProcedure]
        public static bool IsAnchoredV2 (string persistentId)
        {
            return IsAnchored (ByPersistentId (persistentId));
        }

        /// <summary>ExhaustDamageV1 of the vessel of that persistentId (decimal string).</summary>
        [KRPCProcedure]
        public static int ExhaustDamageV2 (string persistentId, bool enabled)
        {
            return ExhaustDamageV1 (ByPersistentId (persistentId), enabled);
        }
    }
}
