using System;
using System.Reflection;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// The schema descriptor of the hinge_lock_at_rest_v1 status (refactor R1: conn.bridge.layout_v1
    /// and layouts_v1), so that a client reads its field names from the DLL rather than copying
    /// offsets: one section "status", read once, HingeRestWatch.Fields in wire order, versioned by
    /// HingeRestWatch.Protocol and Schema, kind "status".
    ///
    /// BY REFLECTION. This assembly has no reference to KRPC.Bridge.Core on purpose (a ground-test
    /// tool that loads next to any Core, see the csproj). The registry is found by name among the
    /// loaded assemblies; without a Core (or with one older than R1) nothing is declared and the
    /// reason is returned; every conn.tower member works the same.
    ///
    /// conn.tower_site's two text tables are declared the same way (towers_v1 and tower_status_v1,
    /// TowerSiteFormat: kind "text", a "header" section of key=value pairs, then a "columns" row
    /// repeated until the end, as the Core's own vessels_v1), and the status of its tower bubble
    /// (tower_bubble_v1, TowerBubbleWatch: kind "status", one section read once), and those of its ground
    /// joints (TowerGroundWatch): the status tower_ground_v1 (kind "status") and the parts table
    /// tower_ground_parts_v1 (kind "text", as towers_v1).
    ///
    /// PURE: System only (build/tests/tower declares into the real LayoutRegistry source).
    /// </summary>
    internal static class TowerLayoutDeclaration
    {
        internal const string RegistryType = "KRPC.Bridge.Core.LayoutRegistry";
        internal const string CoreAssembly = "KRPC.Bridge.Core";

        /// <summary>Name of the layout of the hinge_lock_at_rest_v1 status.</summary>
        internal const string HingeLockLayout = "hinge_lock_at_rest_v1";

        internal const string HingeLockNote =
            "the return of hinge_lock_at_rest_v1, hinge_lock_status_v1 and hinge_lock_cancel_v1; state: 0 none, " +
            "1 armed, 2 locked, 3 already locked, 4 expired, 5 cancelled, 6 lost, 7 failed";

        /// <summary>Name of the layout of the tower_bubble_v1 status (conn.tower_site).</summary>
        internal const string BubbleLayout = "tower_bubble_v1";

        internal const string BubbleNote =
            "the return of tower_bubble_v1, tower_bubble_status_v1 and tower_bubble_release_v1; state: 0 none, " +
            "1 loading, 2 settling, 3 unpacking, 4 unpacked, 5 released, 6 lost";

        /// <summary>Name of the layout of the tower_ground_v1 status (conn.tower_site).</summary>
        internal const string GroundLayout = "tower_ground_v1";

        internal const string GroundNote =
            "the return of tower_ground_v1, tower_ground_status_v1 and tower_ground_release_v1; state: 0 none, " +
            "1 waiting, 2 held, 3 released, 4 lost";

        /// <summary>Name of the layout of the tower_ground_parts_v1 table (conn.tower_site).</summary>
        internal const string GroundPartsLayout = "tower_ground_parts_v1";

        internal const string GroundPartsNote = "conn.tower_site.tower_ground_parts_v1: the tower in the header, one row "
                                                + "per part KJR's rule picks and per hinge";

        /// <summary>The Core's LayoutRegistry among the loaded assemblies, or null.</summary>
        internal static Type FindRegistry ()
        {
            foreach (var asm in AppDomain.CurrentDomain.GetAssemblies ()) {
                if (asm.GetName ().Name != CoreAssembly)
                    continue;
                var t = asm.GetType (RegistryType, false);
                if (t != null)
                    return t;
            }
            return null;
        }

        /// <summary>Declares into the Core's registry if there is one; returns what was done (for the log).</summary>
        internal static string Declare ()
        {
            var registry = FindRegistry ();
            if (registry == null)
                return "no layout registry (" + CoreAssembly + " absent or older than R1): " + HingeLockLayout + ", "
                       + TowerSiteFormat.TowersLayout + ", " + TowerSiteFormat.StatusLayout + ", " + BubbleLayout + ", "
                       + GroundLayout + " and " + GroundPartsLayout + " not declared";
            return DeclareInto (registry) + "; " + DeclareTextInto (registry) + "; " + DeclareBubbleInto (registry) + "; "
                   + DeclareGroundInto (registry);
        }

        internal const string TowersNote = "conn.tower_site.towers_v1: one row per catch tower of the game";
        internal const string StatusNote = "conn.tower_site.tower_status_v1: the tower in the header, one row per hinge; "
                                           + "source loaded or saved";

        /// <summary>
        /// Declares conn.tower_site's two text tables into <paramref name="registry"/> (a LayoutRegistry:
        /// static Declare, KindText, Layout.Once, Layout.Rows, Expect); returns what was done. A member
        /// missing: nothing or less is declared, and the text says so.
        /// </summary>
        internal static string DeclareTextInto (Type registry)
        {
            var declare = registry.GetMethod ("Declare", BindingFlags.Public | BindingFlags.Static, null,
                new [] { typeof (string), typeof (string), typeof (string), typeof (int), typeof (int), typeof (string), typeof (string) },
                null);
            var kind = registry.GetField ("KindText", BindingFlags.Public | BindingFlags.Static);
            if (declare == null || kind == null)
                return "layout registry without Declare/KindText: " + TowerSiteFormat.TowersLayout + " and "
                       + TowerSiteFormat.StatusLayout + " not declared";
            var said = new System.Collections.Generic.List<string> ();
            said.Add (Text (registry, declare, (string) kind.GetValue (null), TowerSiteFormat.TowersLayout, TowersNote,
                TowerSiteFormat.Protocol, TowerSiteFormat.Schema, TowerSiteFormat.TowersHeader, TowerSiteFormat.TowersColumns));
            said.Add (Text (registry, declare, (string) kind.GetValue (null), TowerSiteFormat.StatusLayout, StatusNote,
                TowerSiteFormat.Protocol, TowerSiteFormat.Schema, TowerSiteFormat.StatusHeader, TowerSiteFormat.StatusColumns));
            return string.Join ("; ", said.ToArray ());
        }

        static string Text (Type registry, MethodInfo declare, string kind, string name, string note, int protocol,
                            int schema, string[] header, string[] columns)
        {
            var layout = declare.Invoke (null, new object [] {
                name, "TowerSite", "Tower", protocol, schema, kind, note
            });
            if (layout == null)
                return "layout registry returned no layout: " + name + " not declared";
            var once = layout.GetType ().GetMethod ("Once", BindingFlags.Public | BindingFlags.Instance, null,
                new [] { typeof (string), typeof (string[]) }, null);
            var rows = layout.GetType ().GetMethod ("Rows", BindingFlags.Public | BindingFlags.Instance, null,
                new [] { typeof (string), typeof (string), typeof (string), typeof (string[]) }, null);
            if (once == null || rows == null)
                return "layout without Once/Rows: " + name + " declared without fields";
            once.Invoke (layout, new object [] { "header", header });
            rows.Invoke (layout, new object [] {
                "columns", "*", columns.Length.ToString (System.Globalization.CultureInfo.InvariantCulture), columns
            });
            var expect = registry.GetMethod ("Expect", BindingFlags.Public | BindingFlags.Static, null,
                new [] { layout.GetType (), typeof (string), typeof (int) }, null);
            if (expect != null) {
                expect.Invoke (null, new object [] { layout, "header", header.Length });
                expect.Invoke (null, new object [] { layout, "columns", columns.Length });
            }
            return "layout " + name + " declared (protocol " + protocol + ", schema " + schema
                   + ", " + header.Length + " header keys, " + columns.Length + " columns)";
        }

        /// <summary>
        /// Declares the layout into <paramref name="registry"/> (a LayoutRegistry: static
        /// Declare(name, service, owner, protocol, schema, kind, note), KindStatus, Layout.Once,
        /// Expect); returns what was done. A member missing: nothing or less is declared, and the
        /// text says so.
        /// </summary>
        internal static string DeclareInto (Type registry)
        {
            return DeclareStatus (registry, HingeLockLayout, "Tower", HingeRestWatch.Protocol, HingeRestWatch.Schema,
                                  HingeLockNote, HingeRestWatch.Fields, HingeRestWatch.StatusLength);
        }

        /// <summary>
        /// Declares the tower_bubble_v1 status (conn.tower_site) into <paramref name="registry"/>, as
        /// DeclareInto does the hinge lock's; returns what was done.
        /// </summary>
        internal static string DeclareBubbleInto (Type registry)
        {
            return DeclareStatus (registry, BubbleLayout, "TowerSite", TowerBubbleWatch.Protocol, TowerBubbleWatch.Schema,
                                  BubbleNote, TowerBubbleWatch.Fields, TowerBubbleWatch.StatusLength);
        }

        /// <summary>
        /// Declares the two layouts of the ground joints (conn.tower_site) into <paramref name="registry"/>: the
        /// tower_ground_v1 status (as the bubble's) and the tower_ground_parts_v1 table (as towers_v1); returns what was
        /// done.
        /// </summary>
        internal static string DeclareGroundInto (Type registry)
        {
            string status = DeclareStatus (registry, GroundLayout, "TowerSite", TowerGroundWatch.Protocol,
                                           TowerGroundWatch.Schema, GroundNote, TowerGroundWatch.Fields,
                                           TowerGroundWatch.StatusLength);
            var declare = registry.GetMethod ("Declare", BindingFlags.Public | BindingFlags.Static, null,
                new [] { typeof (string), typeof (string), typeof (string), typeof (int), typeof (int), typeof (string), typeof (string) },
                null);
            var kind = registry.GetField ("KindText", BindingFlags.Public | BindingFlags.Static);
            if (declare == null || kind == null)
                return status + "; layout registry without Declare/KindText: " + GroundPartsLayout + " not declared";
            return status + "; " + Text (registry, declare, (string) kind.GetValue (null), GroundPartsLayout,
                GroundPartsNote, TowerGroundWatch.PartsProtocol, TowerGroundWatch.PartsSchema,
                TowerGroundWatch.PartsHeader, TowerGroundWatch.PartsColumns);
        }

        static string DeclareStatus (Type registry, string name, string service, int protocol, int schema, string note,
                                     string[] fields, int length)
        {
            var declare = registry.GetMethod ("Declare", BindingFlags.Public | BindingFlags.Static, null,
                new [] { typeof (string), typeof (string), typeof (string), typeof (int), typeof (int), typeof (string), typeof (string) },
                null);
            var kind = registry.GetField ("KindStatus", BindingFlags.Public | BindingFlags.Static);
            if (declare == null || kind == null)
                return "layout registry without Declare/KindStatus: " + name + " not declared";
            var layout = declare.Invoke (null, new object [] {
                name, service, "Tower", protocol, schema, (string) kind.GetValue (null), note
            });
            if (layout == null)
                return "layout registry returned no layout: " + name + " not declared";
            var once = layout.GetType ().GetMethod ("Once", BindingFlags.Public | BindingFlags.Instance, null,
                new [] { typeof (string), typeof (string[]) }, null);
            if (once == null)
                return "layout without Once: " + name + " declared without fields";
            once.Invoke (layout, new object [] { "status", fields });
            // The descriptor against the constant that sizes the status: a mismatch is listed by
            // LayoutRegistry.Problems (which the replay harness fails on).
            var expect = registry.GetMethod ("Expect", BindingFlags.Public | BindingFlags.Static, null,
                new [] { layout.GetType (), typeof (string), typeof (int) }, null);
            if (expect != null)
                expect.Invoke (null, new object [] { layout, "status", length });
            return "layout " + name + " declared (protocol " + protocol + ", schema " + schema + ", " + fields.Length
                   + " fields)";
        }
    }
}
