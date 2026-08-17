using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using KRPC.Service;
using KRPC.Service.Attributes;

using SCVessel = KRPC.SpaceCenter.Services.Vessel;
using SCPart = KRPC.SpaceCenter.Services.Parts.Part;

namespace KRPC.Bridge.Ident
{
    /// <summary>
    /// conn.ident - the KSP identifiers kRPC holds internally but does not publish.
    ///
    /// WHY THIS EXISTS. kRPC's Vessel exposes no identifier at all: no id, no uid, no
    /// persistent_id. Its Part exposes none either. Yet both classes are BUILT on one
    /// internally - kRPC's Part stores the part's flightID as its own identity and
    /// compares on it - so the information is right there, one field away from the wire.
    ///
    /// Two side boosters built from the same craft file carry the same name, the same part
    /// count and the same mass. Without an identifier, a client watching two of them come
    /// down cannot say which is which, and every workaround is a guess: match on the name
    /// and they collide, match on geometry and the answer changes between flights.
    ///
    /// flightID is the one KSP identity that is assigned when the vessel is built from the
    /// craft file, persists into the .sfs, and survives the save-and-load that an FMRS jump
    /// performs. Record which flightIDs belong to which booster before launch, intersect
    /// after separation, and the question is answered exactly rather than inferred.
    ///
    /// The part half also joins straight to OCISLY: CameraNames suffixes every camera name
    /// with "@" and the part's flightID precisely because it is stable, so a flightID from
    /// here names a camera there.
    ///
    /// SIGNATURE NOTES, because a malformed one stops the whole kRPC server rather than
    /// just this service:
    ///   - Guid is not a legal kRPC type, so VesselIds returns it as a string.
    ///   - flightID is a uint. It goes out as a decimal string, not a double: that is the
    ///     repo pattern for integers, and it concatenates directly into the "@" token form
    ///     that OCISLY publishes.
    ///   - IList of a KRPCClass IS a legal parameter type, which is what makes
    ///     PartFlightIds one round trip instead of one per part.
    ///
    /// ★ AND ONE TRAP, MEASURED THE HARD WAY ON 17/08: return a List, never an array.
    ///
    /// The first version of this file returned `new string [n]` from the two bulk methods,
    /// declared as IList of string. The scanner accepted it - the DECLARED type is legal -
    /// and in game the calls never returned at all. No error, no exception at the client,
    /// just a request sent and no reply, forever.
    ///
    /// Isolating one call at a time (python/diag_ident.py) pinned it exactly: VesselIds and
    /// PartFlightId, which return a plain string, worked. PartFlightIds and VesselFlightIds,
    /// which returned an array, hung - including when the array held ONE element, so it is
    /// neither size nor iteration. The two IList of string members elsewhere in this repo
    /// that DO work, Bridge.Plugins and OCISLY.Hullcams, both return a real List.
    ///
    /// The reason it hangs instead of erroring is worth knowing, because it will happen
    /// again: the failure is in encoding the RESPONSE, after the procedure body has already
    /// succeeded. There is no response to send, and the error path cannot send one either.
    /// From Python it is indistinguishable from a dead server.
    /// </summary>
    [KRPCService (Name = "Ident", GameScene = GameScene.Flight)]
    public static class IdentService
    {
        /// <summary>Whether kRPC's SpaceCenter service resolved. Check this before anything else.</summary>
        [KRPCProperty]
        public static bool Available {
            get { return IdentAddon.Available; }
        }

        /// <summary>Health check.</summary>
        [KRPCProcedure]
        public static string Ping ()
        {
            return "pong";
        }

        /// <summary>
        /// KSP's flightID for one part, as a decimal string.
        ///
        /// Prefix it with "@" to get the identity token that OCISLY appends to every
        /// camera name.
        /// </summary>
        /// <param name="part">The part to identify.</param>
        [KRPCProcedure]
        public static string PartFlightId (SCPart part)
        {
            Require ();
            return FlightIdOf (part);
        }

        /// <summary>
        /// KSP's flightID for each of the given parts, as decimal strings, in the same
        /// order as the parts were passed.
        ///
        /// One round trip for a whole group. Ask for the parts of one booster - the ones a
        /// part scan has already grouped by decouple stage - and keep the result as that
        /// booster's identity set.
        /// </summary>
        /// <param name="parts">The parts to identify. An empty list returns an empty list.</param>
        [KRPCProcedure]
        public static IList<string> PartFlightIds (IList<SCPart> parts)
        {
            Require ();
            // A List, not an array - see the trap in the class summary.
            var output = new List<string> ();
            if (parts == null)
                return output;

            for (int i = 0; i < parts.Count; i++)
                output.Add (FlightIdOf (parts [i]));
            return output;
        }

        /// <summary>
        /// KSP's flightID for every part currently on the vessel, as decimal strings.
        ///
        /// Unordered on purpose: this is meant to be intersected with a set recorded
        /// before launch, and an intersection does not care about order. Relying on the
        /// order matching the vessel's own part list would be relying on an implementation
        /// detail of kRPC.
        ///
        /// The vessel must be loaded. An unloaded vessel exists only as orbital data and
        /// has no instantiated parts, so the result is empty rather than an error - which
        /// is also the honest answer: nothing is known, as opposed to nothing is there.
        /// </summary>
        /// <param name="vessel">The vessel whose parts to identify.</param>
        [KRPCProcedure]
        public static IList<string> VesselFlightIds (SCVessel vessel)
        {
            Require ();
            if (vessel == null)
                throw new ArgumentNullException (nameof (vessel));

            // A List, not an array - see the trap in the class summary.
            var output = new List<string> ();

            var internalVessel = vessel.InternalVessel;
            if (internalVessel == null || !internalVessel.loaded)
                return output;

            var parts = internalVessel.parts;
            if (parts == null)
                return output;

            for (int i = 0; i < parts.Count; i++)
                output.Add (parts [i].flightID.ToString (Culture));
            return output;
        }

        /// <summary>
        /// The vessel's two KSP identities, tab separated: persistentId then Guid.
        ///
        /// persistentId is what FMRS reports through dropped_persistent_ids, so this is
        /// what finally lets a client match a dropped stage there to a Vessel it holds
        /// here. The Guid is what every FMRS jump procedure takes as its argument.
        ///
        /// Both as strings: Guid is not a legal kRPC type, and persistentId is a uint that
        /// travels as a decimal string for the same reason flightID does.
        /// </summary>
        /// <param name="vessel">The vessel to identify.</param>
        [KRPCProcedure]
        public static string VesselIds (SCVessel vessel)
        {
            Require ();
            if (vessel == null)
                throw new ArgumentNullException (nameof (vessel));

            var internalVessel = vessel.InternalVessel;
            if (internalVessel == null)
                throw new InvalidOperationException (
                    "KSP no longer has this vessel - it was destroyed or recovered.");

            // vessel.Id is a public C# property on kRPC's Vessel; it simply carries no
            // KRPCProperty attribute, which is the entire reason this plugin exists.
            return internalVessel.persistentId.ToString (Culture)
                   + "\t" + vessel.Id.ToString ();
        }

        static string FlightIdOf (SCPart part)
        {
            if (part == null)
                throw new ArgumentNullException (nameof (part));

            // InternalPart is FlightGlobals.FindPartByID(storedFlightId), so it is a
            // LOOKUP, not a field read - and it returns null once the part is gone. That
            // is why the bulk paths above walk the vessel's part list directly instead of
            // calling this per part.
            var internalPart = part.InternalPart;
            if (internalPart == null)
                throw new InvalidOperationException (
                    "KSP no longer has this part - its vessel was destroyed, recovered or unloaded.");
            return internalPart.flightID.ToString (Culture);
        }

        // Invariant culture, always. A uint formatted under a locale that groups digits
        // would come out as "1 898 164 639" and no int() on the far side would take it.
        static readonly System.Globalization.CultureInfo Culture =
            System.Globalization.CultureInfo.InvariantCulture;

        static void Require ()
        {
            if (!IdentAddon.Available)
                throw new InvalidOperationException (
                    "kRPC's SpaceCenter service did not resolve. " +
                    "Check conn.bridge.plugins before calling this.");
        }
    }
}
