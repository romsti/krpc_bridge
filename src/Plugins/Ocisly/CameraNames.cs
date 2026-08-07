using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;

namespace KRPC.Bridge.Ocisly
{
    /// <summary>
    /// Giving every Hullcam a name that is unique AND stable across a scene reload.
    ///
    /// THE PROBLEM. A Hullcam part's <c>cameraName</c> is a KSPField with
    /// isPersistant=false, so it comes from the PART CONFIG, not from the instance. Two
    /// cameras of the same part type on one vessel therefore report the exact same name,
    /// and anything downstream has to invent a tie-break out of the order frames happen
    /// to arrive in - which can swap from one flight scene to the next, silently
    /// exchanging two feeds in the middle of a broadcast.
    ///
    /// THE FIX, in two parts.
    ///
    /// An ORDINAL makes the name unique. Cameras that share a name on the same vessel are
    /// sorted by <c>part.flightID</c> and numbered 1, 2, 3. flightID is persistent per
    /// part and survives the save-and-load that an FMRS jump performs, so camera 1 is the
    /// same physical camera before and after.
    ///
    /// An IDENTITY TOKEN makes it recognisable. The flightID itself is appended after an
    /// "@", so a downstream consumer can tell a returning camera from a genuinely new one
    /// with certainty instead of guessing from timing - which fails, because a torn-down
    /// camera can still push a frame or two after a reload, before OCISLY purges its
    /// static table.
    ///
    /// cameraName is not persisted, so none of this ever touches a save file.
    /// </summary>
    internal static class CameraNames
    {
        /// <summary>
        /// Separates the human-readable camera name from its stable identity token.
        /// A consumer splits on this and shows only the left half.
        /// </summary>
        internal const string IdentitySeparator = "@";

        /// <summary>Apply the naming to every loaded Hullcam. Returns how many were renamed.</summary>
        internal static int Apply ()
        {
            OcislyApi.Require ();
            if (OcislyApi.HullcamNameField == null)
                return 0;

            var groups = new Dictionary<string, List<PartModule>> ();
            foreach (var cam in OcislyApi.AllHullcams ()) {
                var module = cam as PartModule;
                if (module == null)
                    continue;
                var raw = OcislyApi.HullcamNameField.GetValue (cam) as string;
                var key = VesselKeyOf (module) + "|" + Strip (raw);
                if (!groups.ContainsKey (key))
                    groups [key] = new List<PartModule> ();
                groups [key].Add (module);
            }

            var renamed = 0;
            foreach (var group in groups) {
                var members = group.Value;
                // flightID is persistent per part, so this order is identical before and
                // after a scene reload. That is the whole point.
                members.Sort (CompareByFlightId);
                for (var i = 0; i < members.Count; i++) {
                    // The part can be gone mid-teardown, same reason CompareByFlightId
                    // guards it. A camera with no part cannot be given a stable identity,
                    // so skip it rather than throw out of the whole pass.
                    if (members [i] == null || members [i].part == null)
                        continue;
                    var raw = OcislyApi.HullcamNameField.GetValue (members [i]) as string;
                    var stem = Strip (raw);
                    // A lone camera keeps its plain name; only real duplicates get numbered.
                    var pretty = members.Count > 1 ? stem + " " + (i + 1) : stem;
                    var wanted = pretty + IdentitySeparator + members [i].part.flightID;
                    if (raw == wanted)
                        continue;
                    OcislyApi.HullcamNameField.SetValue (members [i], wanted);
                    renamed++;
                    BridgeLog.Info ("OCISLY", "camera renamed '" + raw + "' -> '" + wanted + "'");
                }
            }
            return renamed;
        }

        static int CompareByFlightId (PartModule a, PartModule b)
        {
            var idA = a.part != null ? a.part.flightID : 0u;
            var idB = b.part != null ? b.part.flightID : 0u;
            return idA.CompareTo (idB);
        }

        static string VesselKeyOf (PartModule module)
        {
            try {
                return module.vessel != null ? module.vessel.id.ToString () : "?";
            } catch (Exception) {
                return "?";
            }
        }

        /// <summary>
        /// Recover the original camera name: drop the "@flightID" identity token, then a
        /// trailing " 12" ordinal. Idempotent, so re-running never stacks suffixes - which
        /// matters because this runs on every scene load.
        /// </summary>
        internal static string Strip (string name)
        {
            if (string.IsNullOrEmpty (name))
                return "Hull";

            var at = name.LastIndexOf (IdentitySeparator, StringComparison.Ordinal);
            if (at > 0) {
                var allDigits = at + 1 < name.Length;
                for (var k = at + 1; k < name.Length && allDigits; k++)
                    allDigits = char.IsDigit (name [k]);
                if (allDigits)
                    name = name.Substring (0, at);
            }

            var i = name.Length;
            while (i > 0 && char.IsDigit (name [i - 1]))
                i--;
            if (i == name.Length || i == 0 || name [i - 1] != ' ')
                return name;
            return name.Substring (0, i - 1);
        }
    }
}
