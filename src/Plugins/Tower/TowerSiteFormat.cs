using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// The wire format of conn.tower_site's two tables, towers_v1 (TowersV1) and tower_status_v1
    /// (TowerStatusV1): tab-separated text like the Core's vessels_v1 and FMRS's dropped_stages_v1.
    /// Row 0 is a header of key=value pairs, row 1 names the columns, then one row per tower or per
    /// hinge. Columns and header keys are only ever appended (schema); a client maps them by name.
    /// Unknown values are empty strings, never zeros; booleans are 1 or 0; numbers are invariant
    /// round-trip ("R"); a name never carries a tab or a line break.
    ///
    /// PURE: System only (build/tests/tower).
    /// </summary>
    internal static class TowerSiteFormat
    {
        /// <summary>Protocol of towers_v1 and tower_status_v1 (a column removed or moved raises it).</summary>
        internal const int Protocol = 1;

        /// <summary>Schema of both tables (a column or a header key appended raises it).</summary>
        internal const int Schema = 1;

        internal const string TowersLayout = "towers_v1";
        internal const string StatusLayout = "tower_status_v1";

        internal static readonly string[] TowersHeader = { "protocol", "schema", "n", "scene" };

        internal static readonly string[] TowersColumns = {
            "persistent_id", "name", "body", "lat_deg", "lon_deg", "alt_m", "situation", "loaded", "packed"
        };

        internal static readonly string[] StatusHeader = {
            "protocol", "schema", "persistent_id", "name", "source", "situation", "loaded", "packed",
            "anchored", "tilt_deg", "ec_amount", "ec_max", "n"
        };

        internal static readonly string[] StatusColumns = {
            "part_persistent_id", "part_flight_id", "angle_deg", "target_angle_deg", "locked", "motor_engaged"
        };

        /// <summary>The source of a status: the vessel as the game simulates it now.</summary>
        internal const string SourceLoaded = "loaded";

        /// <summary>The source of a status: the vessel's saved state (ProtoVessel), not loaded.</summary>
        internal const string SourceSaved = "saved";

        /// <summary>One tower of the game.</summary>
        internal sealed class TowerRow
        {
            internal uint PersistentId;
            internal string Name = "";
            internal string Body = "";
            internal double Latitude = double.NaN, Longitude = double.NaN, Altitude = double.NaN;
            internal string Situation = "";
            internal bool Loaded;
            /// <summary>Null when not loaded (packing means nothing then).</summary>
            internal bool? Packed;
        }

        /// <summary>The vessel part of a status.</summary>
        internal sealed class StatusHead
        {
            internal uint PersistentId;
            internal string Name = "";
            internal string Source = "";
            internal string Situation = "";
            internal bool Loaded;
            internal bool? Packed, Anchored;
            internal double Tilt = double.NaN, ChargeAmount = double.NaN, ChargeMax = double.NaN;
        }

        /// <summary>One hinge of a tower.</summary>
        internal sealed class HingeRow
        {
            internal uint PartPersistentId;
            internal uint PartFlightId;
            internal double Angle = double.NaN, Target = double.NaN;
            internal bool? Locked, MotorEngaged;
        }

        internal static List<string> Towers (string scene, IList<TowerRow> rows)
        {
            int n = rows == null ? 0 : rows.Count;
            var lines = new List<string> (n + 2);
            lines.Add ("protocol=" + Int (Protocol) + "\tschema=" + Int (Schema) + "\tn=" + Int (n)
                       + "\tscene=" + Clean (scene));
            lines.Add (string.Join ("\t", TowersColumns));
            for (int i = 0; i < n; i++) {
                var r = rows [i];
                lines.Add (string.Join ("\t", new[] {
                    Id (r.PersistentId), Clean (r.Name), Clean (r.Body), Num (r.Latitude), Num (r.Longitude),
                    Num (r.Altitude), Clean (r.Situation), Flag (r.Loaded), Flag (r.Packed)
                }));
            }
            return lines;
        }

        internal static List<string> Status (StatusHead head, IList<HingeRow> hinges)
        {
            int n = hinges == null ? 0 : hinges.Count;
            var lines = new List<string> (n + 2);
            var h = new StringBuilder ();
            h.Append ("protocol=").Append (Int (Protocol));
            h.Append ("\tschema=").Append (Int (Schema));
            h.Append ("\tpersistent_id=").Append (Id (head.PersistentId));
            h.Append ("\tname=").Append (Clean (head.Name));
            h.Append ("\tsource=").Append (Clean (head.Source));
            h.Append ("\tsituation=").Append (Clean (head.Situation));
            h.Append ("\tloaded=").Append (Flag (head.Loaded));
            h.Append ("\tpacked=").Append (Flag (head.Packed));
            h.Append ("\tanchored=").Append (Flag (head.Anchored));
            h.Append ("\ttilt_deg=").Append (Num (head.Tilt));
            h.Append ("\tec_amount=").Append (Num (head.ChargeAmount));
            h.Append ("\tec_max=").Append (Num (head.ChargeMax));
            h.Append ("\tn=").Append (Int (n));
            lines.Add (h.ToString ());
            lines.Add (string.Join ("\t", StatusColumns));
            for (int i = 0; i < n; i++) {
                var r = hinges [i];
                lines.Add (string.Join ("\t", new[] {
                    Id (r.PartPersistentId), Id (r.PartFlightId), Num (r.Angle), Num (r.Target),
                    Flag (r.Locked), Flag (r.MotorEngaged)
                }));
            }
            return lines;
        }

        internal static string Int (int v)
        {
            return v.ToString (CultureInfo.InvariantCulture);
        }

        internal static string Id (uint v)
        {
            return v.ToString (CultureInfo.InvariantCulture);
        }

        internal static string Num (double v)
        {
            return double.IsNaN (v) || double.IsInfinity (v) ? string.Empty : v.ToString ("R", CultureInfo.InvariantCulture);
        }

        internal static string Flag (bool v)
        {
            return v ? "1" : "0";
        }

        internal static string Flag (bool? v)
        {
            return v.HasValue ? Flag (v.Value) : string.Empty;
        }

        internal static string Clean (string s)
        {
            if (string.IsNullOrEmpty (s))
                return string.Empty;
            return s.Replace ('\t', ' ').Replace ('\n', ' ').Replace ('\r', ' ');
        }

        /// <summary>
        /// Why a craft name is refused, or null. TowerSpawnV1 reads saves/&lt;game&gt;/Ships/VAB (then
        /// SPH)/&lt;name&gt;.craft and nothing else: the name must be a bare file name, with no
        /// directory separator, no drive, no "." or "..", and no character a file name cannot hold.
        /// </summary>
        /// <summary>
        /// Whether a saved value is the zero quaternion, "(0, 0, 0, 0)" or "0,0,0,0": never a rotation. A part
        /// built from a craft at the Space Center never runs OnStart, so the rotations its modules capture there
        /// (BaseServo's servoTransformRotation and jointParentRotation) keep the type's default; saved, they would
        /// load as given (BaseServo.OnLoad only checks that the value exists) and turn the hinge by nothing.
        /// </summary>
        internal static bool IsZeroQuaternion (string text)
        {
            if (string.IsNullOrEmpty (text))
                return false;
            var parts = text.Trim ().TrimStart ('(').TrimEnd (')').Split (',');
            if (parts.Length != 4)
                return false;
            foreach (var p in parts) {
                double v;
                if (!double.TryParse (p.Trim (), NumberStyles.Float, CultureInfo.InvariantCulture, out v) || v != 0.0)
                    return false;
            }
            return true;
        }

        internal static string InvalidCraftName (string name)
        {
            if (string.IsNullOrEmpty (name) || name.Trim ().Length == 0)
                return "craftName must name a craft of this game (empty)";
            if (name == "." || name == "..")
                return "craftName must be a craft name, got '" + name + "'";
            if (name.IndexOfAny (new[] { '/', '\\', ':' }) >= 0
                || name.IndexOfAny (System.IO.Path.GetInvalidFileNameChars ()) >= 0)
                return "craftName must be a craft name, not a path: '" + Clean (name) + "'";
            return null;
        }

        /// <summary>A persistentId given as a decimal string, or the reason it is refused (null id).</summary>
        internal static string ParseId (string text, out uint id)
        {
            id = 0;
            if (string.IsNullOrEmpty (text))
                return "persistentId must be a decimal uint, got an empty string";
            if (!uint.TryParse (text, NumberStyles.None, CultureInfo.InvariantCulture, out id))
                return "persistentId must be a decimal uint, got '" + Clean (text) + "'";
            return null;
        }
    }
}
