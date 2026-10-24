using System;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using System.Text;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// What is loaded, and where it comes from: the data behind conn.bridge.manifest_v1().
    ///
    /// PURE: System and System.Reflection only, no Unity, no KSP, no kRPC type. That is what lets
    /// build/tests/manifest test it without the game, and build/scan run it against the real
    /// DLLs right after kRPC's signature scan, with no KSP launch.
    ///
    /// For every loaded assembly named KRPC.Bridge*, it reads:
    ///   * the build stamp written by tools/build_stamp.py (AssemblyMetadata KRPC.Bridge.GitSha,
    ///     GitDirty, SourceDate, and AssemblyInformationalVersion);
    ///   * the kRPC services the assembly declares ([KRPCService], read as attribute DATA, by
    ///     name, so no attribute instance is built);
    ///   * the protocol and schema versions this build knows for each named layout, read from
    ///     the const fields that define them (table Known below). A const is metadata:
    ///     FieldInfo.GetRawConstantValue reads it without running any static constructor.
    ///
    /// Nothing here touches the game, and nothing is read more than once per session (the caller
    /// caches the result).
    /// </summary>
    internal static class BridgeManifest
    {
        /// <summary>Wire protocol of ManifestV1. Bumped on any breaking change.</summary>
        internal const int Protocol = 1;

        /// <summary>Column set. Columns are only ever appended; a client maps them by name.</summary>
        internal const int Schema = 1;

        internal const string AssemblyPrefix = "KRPC.Bridge";

        internal const string MetaGitSha = "KRPC.Bridge.GitSha";
        internal const string MetaGitDirty = "KRPC.Bridge.GitDirty";
        internal const string MetaSourceDate = "KRPC.Bridge.SourceDate";

        /// <summary>Written in a protocol or schema cell when the table names a constant this build does not have.</summary>
        internal const string Missing = "?";

        internal const string ServiceAttribute = "KRPC.Service.Attributes.KRPCServiceAttribute";

        internal static readonly string[] Columns = {
            "assembly", "service", "layout", "protocol", "schema", "git_sha", "git_dirty",
            "source_date", "informational_version", "assembly_version"
        };

        /// <summary>One named wire layout and the constants that version it.</summary>
        internal sealed class Layout
        {
            internal string Assembly;
            internal string Service;
            internal string Name;
            internal string Type;
            internal string ProtocolField;
            internal string SchemaField;   // empty: the layout has no schema number

            internal Layout (string assembly, string service, string name, string type,
                             string protocolField, string schemaField)
            {
                Assembly = assembly;
                Service = service;
                Name = name;
                Type = type;
                ProtocolField = protocolField;
                SchemaField = schemaField ?? string.Empty;
            }
        }

        const string Ctl = "KRPC.Bridge.Control";
        const string CtlNs = "KRPC.Bridge.Control.";
        const string Tel = "KRPC.Bridge.Telemetry";
        const string TelNs = "KRPC.Bridge.Telemetry.";
        const string Mdl = "KRPC.Bridge.Model";
        const string MdlNs = "KRPC.Bridge.Model.";

        /// <summary>
        /// The versioned layouts this build knows, by the constant that versions each. Add a row
        /// with every new versioned RPC. build/scan checks, on the real DLLs, that every row
        /// resolves; a row naming a constant that does not exist fails the build. The assembly
        /// of a row is the one that holds the constant (refactor R3: the pure Model holds some of
        /// the Control service's).
        /// </summary>
        internal static readonly Layout[] Known = {
            new Layout (Ctl, "Control", "control_v2", CtlNs + "Actuation", "ProtocolVersion", "ControlStatusSchema"),
            new Layout (Ctl, "Control", "inactive_status_v1", CtlNs + "Actuation", "InactiveStatusVersion", ""),
            new Layout (Tel, "Telemetry", "dynamics_v3", TelNs + "DynamicsV3", "ProtocolVersion", "SchemaVersion"),
            new Layout (Tel, "Telemetry", "dynamics_tracked_v3", TelNs + "DynamicsTracked", "ProtocolVersion", ""),
            new Layout (Tel, "Telemetry", "dynamics_ext_v1", TelNs + "DynamicsExt", "ProtocolVersion", "SchemaVersion"),
            new Layout (Tel, "Telemetry", "aero_actuator_v1", TelNs + "AeroActuatorV1", "ProtocolVersion", "SchemaVersion"),
            new Layout (Tel, "Telemetry", "aero_wrench_batch_v1", TelNs + "AeroWrenchBatch", "ProtocolVersion", ""),
            new Layout (Ctl, "Control", "attitude_v1", CtlNs + "AttitudeV1", "ProtocolVersion", "SchemaVersion"),
            new Layout (Ctl, "Control", "attitude_translation_frames_v1", CtlNs + "AttitudeV1", "TranslationProtocolVersion", "TranslationSchemaVersion"),
            // The newest return layout of attitude_translation_v1 (1, 2 or 3 depending on the mode).
            new Layout (Mdl, "Control", "attitude_translation_v1", MdlNs + "AttitudeTranslation", "ReturnVersionHonest", ""),
            new Layout (Mdl, "Control", "translation_evidence", MdlNs + "AttitudeTranslation", "EvidenceBlockProtocol", "EvidenceBlockSchema"),
            new Layout (Mdl, "Control", "attitude_options_v1", MdlNs + "AttitudeWrite", "OptionsVersion", ""),
            new Layout (Ctl, "Control", "attitude_capabilities_v1", CtlNs + "AttitudeV1", "CapabilitiesVersion", ""),
            new Layout (Mdl, "Control", "attitude_gimbal_pivots_v1", MdlNs + "GimbalPivots", "Version", ""),
            new Layout (Mdl, "Control", "attitude_effectors_v1", MdlNs + "EffectorRegistry", "Protocol", "Schema"),
            new Layout (Ctl, "Control", "attitude_translation_v2", CtlNs + "AttitudeV1", "TranslationReturnV2Protocol", ""),
            new Layout ("KRPC.Bridge.Fmrs", "FMRS", "dropped_stages_v1", "KRPC.Bridge.Fmrs.DroppedStagesFormat", "Protocol", "Schema"),
            // Declared into the LayoutRegistry by reflection (Tower has no Core reference); its
            // version constants are read the same way as every other row.
            new Layout ("KRPC.Bridge.Tower", "Tower", "hinge_lock_at_rest_v1", "KRPC.Bridge.Tower.HingeRestWatch", "Protocol", "Schema"),
            // conn.tower_site (same DLL, same reflective declaration): the towers and a tower's state.
            new Layout ("KRPC.Bridge.Tower", "TowerSite", "towers_v1", "KRPC.Bridge.Tower.TowerSiteFormat", "Protocol", "Schema"),
            new Layout ("KRPC.Bridge.Tower", "TowerSite", "tower_status_v1", "KRPC.Bridge.Tower.TowerSiteFormat", "Protocol", "Schema"),
            // the status of a tower bubble (loaded first, unpacked once its load is finished).
            new Layout ("KRPC.Bridge.Tower", "TowerSite", "tower_bubble_v1", "KRPC.Bridge.Tower.TowerBubbleWatch", "Protocol", "Schema"),
            // the ground joints of a tower (status, and the table of the parts fixed).
            new Layout ("KRPC.Bridge.Tower", "TowerSite", "tower_ground_v1", "KRPC.Bridge.Tower.TowerGroundWatch", "Protocol", "Schema"),
            new Layout ("KRPC.Bridge.Tower", "TowerSite", "tower_ground_parts_v1", "KRPC.Bridge.Tower.TowerGroundWatch", "PartsProtocol", "PartsSchema"),
            new Layout ("KRPC.Bridge.Core", "Bridge", "manifest_v1", "KRPC.Bridge.Core.BridgeManifest", "Protocol", "Schema"),
            new Layout ("KRPC.Bridge.Core", "Bridge", "clock_v1", "KRPC.Bridge.Core.GameClock", "Protocol", ""),
            new Layout ("KRPC.Bridge.Core", "Bridge", "vessels_v1", "KRPC.Bridge.Core.VesselRegistry", "Protocol", ""),
            new Layout ("KRPC.Bridge.Core", "Bridge", "bits_v1", "KRPC.Bridge.Core.BitRegistry", "Protocol", ""),
            new Layout ("KRPC.Bridge.Core", "Bridge", "layout_v1", "KRPC.Bridge.Core.LayoutRegistry", "Protocol", ""),
            // Refactor lot 3: the event bus V2 (3.3) and the contact by leg (3.2).
            new Layout ("KRPC.Bridge.Core", "Bridge", "events_v2", "KRPC.Bridge.Core.EventLogV2", "Protocol", "Schema"),
            new Layout (Tel, "Telemetry", "contact_v1", TelNs + "ContactWatch", "Protocol", "Schema"),
            // Refactor lot 3, 3.1: SurfaceModelV1 (the replica of the control surfaces).
            new Layout (Mdl, "Control", "surface_model_v1", MdlNs + "SurfaceModel", "Protocol", "Schema"),
            // the physical slope kappa of SurfaceModelV1.
            new Layout (Mdl, "Control", "surface_kappa_v1", MdlNs + "SurfaceKappa", "Protocol", "Schema"),
            // the roll excitation's window and state.
            new Layout (Mdl, "Control", "attitude_surfaces_roll_v1", MdlNs + "SurfaceKappaRoll", "Protocol", "Schema"),
            // the gimbals' authority per kN without thrust, the legs' geometry.
            new Layout (Ctl, "Control", "gimbal_authority_v1", CtlNs + "GimbalAuthorityRead", "Protocol", "Schema"),
            new Layout (Tel, "Telemetry", "legs_geometry_v1", TelNs + "LegsGeometry", "Protocol", "Schema"),
            // the main throttle at the physics step (option 512).
            new Layout (Mdl, "Control", "attitude_throttle_v1", MdlNs + "MainThrottle", "Protocol", "Schema"),
            // the reference ring (option 2048).
            new Layout (Mdl, "Control", "attitude_reference_ring_v1", MdlNs + "ReferenceRing", "Protocol", "Schema"),
        };

        /// <summary>The build stamp of one assembly. Empty strings when it has none.</summary>
        internal sealed class Stamp
        {
            internal string GitSha = string.Empty;
            internal string GitDirty = string.Empty;
            internal string SourceDate = string.Empty;
            internal string InformationalVersion = string.Empty;
            internal string AssemblyVersion = string.Empty;
        }

        /// <summary>One row of the manifest.</summary>
        internal sealed class Row
        {
            internal string Assembly = string.Empty;
            internal string Service = string.Empty;
            internal string Layout = string.Empty;
            internal string Protocol = string.Empty;
            internal string Schema = string.Empty;
            internal Stamp Stamp = new Stamp ();
        }

        // ------------------------------------------------------------------ reading

        /// <summary>True for KRPC.Bridge and KRPC.Bridge.* (not KRPC.BridgeFoo).</summary>
        internal static bool IsBridgeAssembly (string name)
        {
            if (string.IsNullOrEmpty (name))
                return false;
            return name == AssemblyPrefix || name.StartsWith (AssemblyPrefix + ".", StringComparison.Ordinal);
        }

        /// <summary>The stamp of an assembly, read as attribute data. Never throws.</summary>
        internal static Stamp ReadStamp (Assembly assembly)
        {
            var stamp = new Stamp ();
            if (assembly == null)
                return stamp;
            try {
                var version = assembly.GetName ().Version;
                stamp.AssemblyVersion = version != null ? version.ToString () : string.Empty;
            } catch (Exception) {
            }
            IList<CustomAttributeData> data;
            try {
                data = CustomAttributeData.GetCustomAttributes (assembly);
            } catch (Exception) {
                return stamp;
            }
            foreach (var attribute in data) {
                string type;
                try {
                    type = attribute.Constructor.DeclaringType.FullName;
                } catch (Exception) {
                    continue;
                }
                var args = attribute.ConstructorArguments;
                if (type == "System.Reflection.AssemblyInformationalVersionAttribute" && args.Count == 1) {
                    stamp.InformationalVersion = Text (args [0].Value);
                } else if (type == "System.Reflection.AssemblyMetadataAttribute" && args.Count == 2) {
                    var key = Text (args [0].Value);
                    var value = Text (args [1].Value);
                    if (key == MetaGitSha)
                        stamp.GitSha = value;
                    else if (key == MetaGitDirty)
                        stamp.GitDirty = value;
                    else if (key == MetaSourceDate)
                        stamp.SourceDate = value;
                }
            }
            return stamp;
        }

        /// <summary>
        /// The kRPC service names an assembly declares, sorted. A [KRPCService] without a Name
        /// is named after its class, as kRPC does. Never throws: a type that cannot be loaded is
        /// skipped.
        /// </summary>
        internal static List<string> ServicesOf (Assembly assembly)
        {
            var names = new List<string> ();
            if (assembly == null)
                return names;
            Type[] types;
            try {
                types = assembly.GetTypes ();
            } catch (ReflectionTypeLoadException e) {
                types = e.Types ?? new Type[0];
            } catch (Exception) {
                return names;
            }
            foreach (var type in types) {
                if (type == null)
                    continue;
                IList<CustomAttributeData> data;
                try {
                    data = CustomAttributeData.GetCustomAttributes (type);
                } catch (Exception) {
                    continue;
                }
                foreach (var attribute in data) {
                    string attributeType;
                    try {
                        attributeType = attribute.Constructor.DeclaringType.FullName;
                    } catch (Exception) {
                        continue;
                    }
                    if (attributeType != ServiceAttribute)
                        continue;
                    var name = type.Name;
                    foreach (var named in attribute.NamedArguments) {
                        if (named.MemberName == "Name") {
                            var value = Text (named.TypedValue.Value);
                            if (value.Length > 0)
                                name = value;
                        }
                    }
                    if (!names.Contains (name))
                        names.Add (name);
                }
            }
            names.Sort (StringComparer.Ordinal);
            return names;
        }

        /// <summary>
        /// The value of a const field, as invariant text; "" when no field is named; Missing when
        /// the type or the field does not exist or is not a constant. Never runs a static
        /// constructor: a static readonly field is NOT read, it reports Missing.
        /// </summary>
        internal static string ReadConst (Assembly assembly, string typeName, string fieldName)
        {
            if (string.IsNullOrEmpty (fieldName))
                return string.Empty;
            if (assembly == null || string.IsNullOrEmpty (typeName))
                return Missing;
            try {
                var type = assembly.GetType (typeName, false);
                if (type == null)
                    return Missing;
                var field = type.GetField (fieldName,
                    BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.DeclaredOnly);
                if (field == null || !field.IsLiteral)
                    return Missing;
                var value = field.GetRawConstantValue ();
                var formattable = value as IFormattable;
                return formattable != null
                    ? formattable.ToString (null, CultureInfo.InvariantCulture)
                    : Text (value);
            } catch (Exception) {
                return Missing;
            }
        }

        /// <summary>
        /// The rows for a set of assemblies, using the Known table. Only assemblies named
        /// KRPC.Bridge* are kept. Per assembly: one row per service (layout empty), then one row
        /// per known layout of that assembly. An assembly with no service still gets one row.
        /// </summary>
        internal static List<Row> Collect (IEnumerable<Assembly> assemblies)
        {
            return Collect (assemblies, Known);
        }

        internal static List<Row> Collect (IEnumerable<Assembly> assemblies, IList<Layout> known)
        {
            var rows = new List<Row> ();
            if (assemblies == null)
                return rows;
            var kept = new List<KeyValuePair<string, Assembly>> ();
            foreach (var assembly in assemblies) {
                if (assembly == null)
                    continue;
                string name;
                try {
                    if (IsDynamic (assembly))
                        continue;
                    name = assembly.GetName ().Name;
                } catch (Exception) {
                    continue;
                }
                if (IsBridgeAssembly (name))
                    kept.Add (new KeyValuePair<string, Assembly> (name, assembly));
            }
            kept.Sort ((a, b) => string.CompareOrdinal (a.Key, b.Key));

            foreach (var entry in kept) {
                var stamp = ReadStamp (entry.Value);
                var services = ServicesOf (entry.Value);
                if (services.Count == 0)
                    rows.Add (new Row { Assembly = entry.Key, Stamp = stamp });
                foreach (var service in services)
                    rows.Add (new Row { Assembly = entry.Key, Service = service, Stamp = stamp });
                if (known == null)
                    continue;
                var layouts = new List<Layout> ();
                foreach (var layout in known)
                    if (layout != null && layout.Assembly == entry.Key)
                        layouts.Add (layout);
                layouts.Sort ((a, b) => {
                    int c = string.CompareOrdinal (a.Service, b.Service);
                    return c != 0 ? c : string.CompareOrdinal (a.Name, b.Name);
                });
                foreach (var layout in layouts) {
                    rows.Add (new Row {
                        Assembly = entry.Key,
                        Service = layout.Service,
                        Layout = layout.Name,
                        Protocol = ReadConst (entry.Value, layout.Type, layout.ProtocolField),
                        Schema = ReadConst (entry.Value, layout.Type, layout.SchemaField),
                        Stamp = stamp
                    });
                }
            }
            return rows;
        }

        static bool IsDynamic (Assembly assembly)
        {
            // Assembly.IsDynamic exists from .NET 4; a dynamic assembly has no stamp to read.
            return assembly.IsDynamic;
        }

        // ------------------------------------------------------------------ wire format

        /// <summary>
        /// Header row, column row, then one row per entry. Every row is tab-separated; the header
        /// is key=value pairs: protocol, schema, n (rows), assemblies, git_sha (the one commit of
        /// every assembly, "mixed" when they differ, "" when none has a stamp), git_dirty (1 when
        /// any assembly is dirty or has no stamp), consistent (1 when every assembly has the same
        /// stamped commit and none is dirty: one clean build is loaded).
        /// </summary>
        internal static List<string> Serialize (IList<Row> rows)
        {
            var n = rows != null ? rows.Count : 0;
            var lines = new List<string> (n + 2);

            var shas = new List<string> ();
            var assemblies = new List<string> ();
            bool dirty = false, unstamped = false;
            if (rows != null) {
                foreach (var row in rows) {
                    if (assemblies.Contains (row.Assembly))
                        continue;
                    assemblies.Add (row.Assembly);
                    var sha = row.Stamp != null ? row.Stamp.GitSha : string.Empty;
                    if (string.IsNullOrEmpty (sha))
                        unstamped = true;
                    else if (!shas.Contains (sha))
                        shas.Add (sha);
                    if (row.Stamp == null || row.Stamp.GitDirty != "0")
                        dirty = true;
                }
            }
            string commit = shas.Count == 0 ? string.Empty : (shas.Count == 1 ? shas [0] : "mixed");
            bool consistent = assemblies.Count > 0 && shas.Count == 1 && !unstamped && !dirty;

            var head = new StringBuilder ();
            head.Append ("protocol=").Append (Protocol.ToString (CultureInfo.InvariantCulture));
            head.Append ("\tschema=").Append (Schema.ToString (CultureInfo.InvariantCulture));
            head.Append ("\tn=").Append (n.ToString (CultureInfo.InvariantCulture));
            head.Append ("\tassemblies=").Append (assemblies.Count.ToString (CultureInfo.InvariantCulture));
            head.Append ("\tgit_sha=").Append (Clean (commit));
            head.Append ("\tgit_dirty=").Append (dirty || unstamped ? "1" : "0");
            head.Append ("\tconsistent=").Append (consistent ? "1" : "0");
            lines.Add (head.ToString ());
            lines.Add (string.Join ("\t", Columns));
            if (rows == null)
                return lines;
            foreach (var r in rows) {
                var s = r.Stamp ?? new Stamp ();
                lines.Add (string.Join ("\t", new[] {
                    Clean (r.Assembly), Clean (r.Service), Clean (r.Layout), Clean (r.Protocol),
                    Clean (r.Schema), Clean (s.GitSha), Clean (s.GitDirty), Clean (s.SourceDate),
                    Clean (s.InformationalVersion), Clean (s.AssemblyVersion)
                }));
            }
            return lines;
        }

        /// <summary>The manifest of a set of assemblies, ready for the wire.</summary>
        internal static List<string> Lines (IEnumerable<Assembly> assemblies)
        {
            return Serialize (Collect (assemblies));
        }

        /// <summary>"commit &lt;sha&gt;[ DIRTY], &lt;date&gt;" for a log line. Never throws.</summary>
        internal static string StampLine (Assembly assembly)
        {
            try {
                var s = ReadStamp (assembly);
                if (string.IsNullOrEmpty (s.GitSha))
                    return "no build stamp";
                return "commit " + s.GitSha + (s.GitDirty == "0" ? string.Empty : " DIRTY")
                    + (string.IsNullOrEmpty (s.SourceDate) ? string.Empty : ", " + s.SourceDate);
            } catch (Exception) {
                return "no build stamp";
            }
        }

        /// <summary>Fields are tab-separated and rows are lines: tabs and newlines must go.</summary>
        internal static string Clean (string text)
        {
            if (string.IsNullOrEmpty (text))
                return string.Empty;
            return text.Replace ('\t', ' ').Replace ('\n', ' ').Replace ('\r', ' ');
        }

        static string Text (object value)
        {
            return value == null ? string.Empty : Convert.ToString (value, CultureInfo.InvariantCulture);
        }
    }
}
