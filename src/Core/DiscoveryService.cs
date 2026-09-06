using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using System.Security.Cryptography;
using System.Text;
using KRPC.Service;
using KRPC.Service.Attributes;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// conn.discovery - read-only reflection over the live AppDomain, and the runtime half
    /// of an offline type index.
    ///
    /// WHY THIS EXISTS, SEPARATELY FROM conn.bridge.describe_type. DescribeType answers "what
    /// does this whole type look like" in one dump, which is right for writing a plugin by
    /// hand and wrong for everything a tool does at scale: it cannot list what is loaded, it
    /// cannot search for a type by a substring of its name, it re-transmits a whole type when
    /// a client only wanted one member, and it gives a client no cheap way to ask "is the
    /// type still the shape my offline index recorded". Those are this service's five extra
    /// shapes, plus a flight-only view of the PartModules actually instantiated on the ship.
    ///
    /// EVERYTHING HERE IS READ-ONLY INTROSPECTION. There is no InvokeMethod, no SetField, no
    /// ExecuteReflection and there never will be: mutating or invoking an arbitrary member
    /// across the kRPC wire is a different and far more dangerous capability, and it belongs
    /// in a dedicated typed service that spells out exactly what it will touch, not in a
    /// discovery endpoint that a script can point at any type in any assembly.
    ///
    /// GameScene.All on purpose: asking what is loaded, or what a type looks like, is useful
    /// from the space centre and before any flight. FindPartModules is the one procedure that
    /// needs a vessel, and it answers with an empty list rather than an error when there is
    /// none, so it too is safe to call in any scene.
    ///
    /// SIGNATURE DISCIPLINE, because one malformed signature in any loaded assembly stops the
    /// entire kRPC server rather than just this service (see build/scan):
    ///   - every parameter and return is a primitive, a string, an int or IList of string,
    ///   - NEVER an array and NEVER a KRPCClass return (both are traps documented on Ident
    ///     and Template respectively - an array return type is accepted by the scanner and
    ///     then hangs forever in game),
    ///   - records are packed tab-separated into strings, the same convention Core uses, so
    ///     the wire stays one flat list and Python splits it for free.
    ///
    /// ★ EVERY BULK METHOD RETURNS A List, NOT AN ARRAY. Declaring IList of string and then
    /// returning `new string[n]` passes build/scan - the DECLARED type is legal - and hangs
    /// in game with no reply ever sent. Measured the hard way on the Ident plugin.
    /// </summary>
    [KRPCService (Name = "Discovery", GameScene = GameScene.All)]
    public static class DiscoveryService
    {
        // Invariant culture, always. An int formatted under a locale that groups digits
        // would come out as "1 234" and no int() on the far side would take it.
        static readonly CultureInfo Culture = CultureInfo.InvariantCulture;

        // Public and non-public, instance and static, flattened up the hierarchy - the same
        // reach conn.bridge.describe_type uses, so the two services agree on what a type has.
        const BindingFlags AllMembers =
            BindingFlags.Public | BindingFlags.NonPublic |
            BindingFlags.Instance | BindingFlags.Static |
            BindingFlags.FlattenHierarchy;

        // ------------------------------------------------------------------
        // What is loaded
        // ------------------------------------------------------------------

        /// <summary>
        /// Loaded assemblies whose simple name matches <paramref name="filter"/>, as
        /// "name\tversion\tfullname", sorted by name.
        ///
        /// The starting point for building an offline index: it says which assemblies are in
        /// the AppDomain, and at which version, so a client knows what SearchTypes and
        /// DescribeMember can even be asked about.
        /// </summary>
        /// <param name="filter">
        /// Comma-separated substrings, matched case-insensitively against the assembly's
        /// simple name (the same filter syntax as everywhere else in the bridge). Empty or
        /// "*" returns every loaded assembly.
        /// </param>
        [KRPCProcedure]
        public static IList<string> ListAssemblies (string filter)
        {
            // A List, not an array - see the class summary.
            var rows = new List<string> ();
            foreach (var loaded in AssemblyLoader.loadedAssemblies) {
                if (loaded == null || loaded.assembly == null)
                    continue;
                try {
                    var name = loaded.assembly.GetName ();
                    if (!ModRegistry.MatchesFilter (name.Name, filter ?? string.Empty))
                        continue;
                    rows.Add (name.Name + "\t"
                              + name.Version + "\t"
                              + loaded.assembly.FullName);
                } catch (Exception) {
                    // One broken assembly must never stop the enumeration.
                }
            }
            rows.Sort ();
            return rows;
        }

        /// <summary>
        /// Full names of types, in the assemblies matched by <paramref name="assemblyFragment"/>,
        /// whose simple or full name contains <paramref name="query"/> (case-insensitive),
        /// capped at <paramref name="limit"/> and sorted.
        ///
        /// The lookup a client does before DescribeMember or TypeHierarchy: it turns a half
        /// remembered name into the exact full names those procedures need.
        /// </summary>
        /// <param name="assemblyFragment">Substring of an assembly's simple name, case-insensitive. Empty matches every loaded assembly.</param>
        /// <param name="query">Substring of the type's simple or full name, case-insensitive. Empty matches every type.</param>
        /// <param name="limit">Maximum rows. Zero or less becomes 50; anything above 500 is capped to 500.</param>
        [KRPCProcedure]
        public static IList<string> SearchTypes (string assemblyFragment, string query, int limit)
        {
            var rows = new List<string> ();

            if (limit <= 0)
                limit = 50;
            if (limit > 500)
                limit = 500;

            var needle = query ?? string.Empty;
            var candidates = ModRegistry.FindAssembliesContaining (assemblyFragment ?? string.Empty);
            var seen = new HashSet<string> ();

            foreach (var assembly in candidates) {
                foreach (var type in ModRegistry.SafeTypes (assembly)) {
                    if (type == null)
                        continue;
                    string full;
                    try {
                        full = type.FullName ?? type.Name;
                    } catch (Exception) {
                        continue;
                    }
                    if (!ContainsCI (type.Name, needle) && !ContainsCI (full, needle))
                        continue;
                    if (seen.Add (full))
                        rows.Add (full);
                }
            }

            rows.Sort ();
            if (rows.Count > limit)
                rows.RemoveRange (limit, rows.Count - limit);
            return rows;
        }

        // ------------------------------------------------------------------
        // One member at a time
        // ------------------------------------------------------------------

        /// <summary>
        /// The member(s) named <paramref name="memberName"/> on the resolved type, one row
        /// each, as nine tab-separated fields:
        ///
        ///   kind, declaring type, visibility, static-or-instance, return/field/property
        ///   type, parameters, accessors (get/set), metadata token (decimal), attributes.
        ///
        /// The precise, small counterpart to conn.bridge.describe_type: when a client already
        /// knows the member it wants, this sends that one member's shape instead of the whole
        /// type. Overloads produce several rows; the metadata token disambiguates them.
        ///
        /// An unknown assembly, type or member returns an EMPTY list rather than an error -
        /// "nothing named that here" is the honest answer, and it never throws across the
        /// wire in the not-found case.
        /// </summary>
        /// <param name="assemblyFragment">Substring of an assembly's simple name, case-insensitive.</param>
        /// <param name="typeName">Full or simple type name.</param>
        /// <param name="memberName">Exact member name. Use ".ctor" for constructors.</param>
        [KRPCProcedure]
        public static IList<string> DescribeMember (string assemblyFragment, string typeName, string memberName)
        {
            var rows = new List<string> ();
            var found = Resolve (assemblyFragment, typeName);
            if (found == null || string.IsNullOrEmpty (memberName))
                return rows;

            MemberInfo[] members;
            try {
                members = found.GetMember (memberName, AllMembers);
            } catch (Exception) {
                return rows;
            }

            foreach (var member in members) {
                try {
                    rows.Add (FormatMember (member));
                } catch (Exception) {
                    // Reflecting one member can throw (a type it names failed to load);
                    // skip it and keep the rest, mirroring ModRegistry.SafeTypes.
                }
            }
            rows.Sort ();
            return rows;
        }

        /// <summary>
        /// The type's ancestry: its base-type chain most-derived first up to System.Object,
        /// then its interfaces, one per row. Base rows are prefixed "base\t" and interface
        /// rows "interface\t", each followed by the full type name.
        ///
        /// The first "base" row is the resolved type itself, since it is the most derived; the
        /// last is System.Object. Interfaces are sorted so the output is stable between calls.
        ///
        /// An unknown assembly or type returns an EMPTY list.
        /// </summary>
        /// <param name="assemblyFragment">Substring of an assembly's simple name, case-insensitive.</param>
        /// <param name="typeName">Full or simple type name.</param>
        [KRPCProcedure]
        public static IList<string> TypeHierarchy (string assemblyFragment, string typeName)
        {
            var rows = new List<string> ();
            var found = Resolve (assemblyFragment, typeName);
            if (found == null)
                return rows;

            for (var t = found; t != null; t = t.BaseType)
                rows.Add ("base\t" + (t.FullName ?? t.Name));

            var interfaces = new List<string> ();
            try {
                foreach (var iface in found.GetInterfaces ())
                    if (iface != null)
                        interfaces.Add (iface.FullName ?? iface.Name);
            } catch (Exception) {
                // A partially loaded type can throw here; report the bases we did resolve.
            }
            interfaces.Sort ();
            foreach (var iface in interfaces)
                rows.Add ("interface\t" + iface);

            return rows;
        }

        // ------------------------------------------------------------------
        // Live vessel
        // ------------------------------------------------------------------

        /// <summary>
        /// FLIGHT ONLY. The distinct PartModule types instantiated on the active vessel whose
        /// type name contains <paramref name="query"/>, one row each, as
        /// "fullTypeName\tassemblySimpleName\tinstanceCount\tsamplePartTitles".
        ///
        /// samplePartTitles is a few representative part titles the module sits on, so a
        /// client can tell which craft the module belongs to without a second query. This is
        /// how you learn what a mod actually exposes on THIS ship, as opposed to what its
        /// assembly declares - a module can be present in an assembly and used by no part.
        ///
        /// No active vessel returns an EMPTY list, not an error: there is simply nothing to
        /// report, which is also the honest answer outside flight.
        /// </summary>
        /// <param name="query">Substring of the module's simple or full type name, case-insensitive. Empty returns every module type.</param>
        [KRPCProcedure]
        public static IList<string> FindPartModules (string query)
        {
            var rows = new List<string> ();
            var vessel = FlightGlobals.ActiveVessel;
            if (vessel == null || vessel.parts == null)
                return rows;

            var needle = query ?? string.Empty;
            var byType = new Dictionary<string, ModuleAgg> ();

            foreach (var part in vessel.parts) {
                if (part == null)
                    continue;
                string title = TitleOf (part);
                var modules = ModulesOf (part);
                if (modules == null)
                    continue;
                foreach (var module in modules) {
                    if (module == null)
                        continue;
                    Type type;
                    string full;
                    try {
                        type = module.GetType ();
                        full = type.FullName ?? type.Name;
                    } catch (Exception) {
                        continue;
                    }
                    if (!ContainsCI (type.Name, needle) && !ContainsCI (full, needle))
                        continue;

                    ModuleAgg agg;
                    if (!byType.TryGetValue (full, out agg)) {
                        agg = new ModuleAgg ();
                        try {
                            agg.Assembly = type.Assembly.GetName ().Name;
                        } catch (Exception) {
                            agg.Assembly = string.Empty;
                        }
                        byType [full] = agg;
                    }
                    agg.Count++;
                    if (!string.IsNullOrEmpty (title) && agg.Titles.Count < 3 && !agg.Titles.Contains (title))
                        agg.Titles.Add (title);
                }
            }

            foreach (var pair in byType)
                rows.Add (pair.Key + "\t"
                          + pair.Value.Assembly + "\t"
                          + pair.Value.Count.ToString (Culture) + "\t"
                          + string.Join (", ", pair.Value.Titles.ToArray ()));
            rows.Sort ();
            return rows;
        }

        // ------------------------------------------------------------------
        // Drift detection
        // ------------------------------------------------------------------

        /// <summary>
        /// A compact, stable fingerprint of the resolved type, as "count=N;sha=&lt;hex&gt;",
        /// where N is the member count and the hex is a SHA-256 over the type's
        /// public+non-public members, each rendered as "kind:name:signature" and sorted.
        ///
        /// The point is drift detection without re-transmission. A client that indexed a type
        /// offline stores this one short string; on the next run it asks for the fingerprint
        /// and compares. Equal means the runtime type is byte-for-byte the shape it indexed
        /// and the whole offline description still holds; different means a mod update moved
        /// something and it is worth pulling DescribeMember or describe_type again. Sorting
        /// the member lines first makes the hash independent of reflection's ordering, so it
        /// is stable across runs of the same build.
        ///
        /// An unknown assembly or type returns "error\t&lt;message&gt;" rather than throwing.
        /// </summary>
        /// <param name="assemblyFragment">Substring of an assembly's simple name, case-insensitive.</param>
        /// <param name="typeName">Full or simple type name.</param>
        [KRPCProcedure]
        public static string RuntimeTypeFingerprint (string assemblyFragment, string typeName)
        {
            var found = Resolve (assemblyFragment, typeName);
            if (found == null)
                return "error\ttype not found";

            var lines = new List<string> ();
            MemberInfo[] members;
            try {
                members = found.GetMembers (AllMembers);
            } catch (Exception e) {
                return "error\t" + Flatten (e.Message);
            }

            foreach (var member in members) {
                try {
                    lines.Add (KindOf (member) + ":" + member.Name + ":" + SignatureOf (member));
                } catch (Exception) {
                    // Skip a member that will not reflect, rather than fail the whole hash.
                }
            }
            lines.Sort ();

            var joined = string.Join ("\n", lines.ToArray ());
            string hex;
            using (var sha = SHA256.Create ()) {
                var digest = sha.ComputeHash (Encoding.UTF8.GetBytes (joined));
                var builder = new StringBuilder (digest.Length * 2);
                foreach (var b in digest)
                    builder.Append (b.ToString ("x2", Culture));
                hex = builder.ToString ();
            }
            return "count=" + lines.Count.ToString (Culture) + ";sha=" + hex;
        }

        // ==================================================================
        // Helpers - private plumbing, never exposed to kRPC.
        // ==================================================================

        // Per-module-type aggregation for FindPartModules.
        sealed class ModuleAgg
        {
            internal string Assembly = string.Empty;
            internal int Count;
            internal readonly List<string> Titles = new List<string> ();
        }

        /// <summary>Resolve a type by trying each assembly the fragment matches, in order. Null if none holds it.</summary>
        static Type Resolve (string assemblyFragment, string typeName)
        {
            var candidates = ModRegistry.FindAssembliesContaining (assemblyFragment ?? string.Empty);
            foreach (var candidate in candidates) {
                var found = ModRegistry.FindType (candidate, typeName);
                if (found != null)
                    return found;
            }
            return null;
        }

        /// <summary>Case-insensitive substring test. An empty needle matches everything, a null haystack nothing.</summary>
        static bool ContainsCI (string haystack, string needle)
        {
            if (string.IsNullOrEmpty (needle))
                return true;
            if (haystack == null)
                return false;
            return haystack.IndexOf (needle, StringComparison.OrdinalIgnoreCase) >= 0;
        }

        /// <summary>A representative human title for a part, or the empty string.</summary>
        static string TitleOf (Part part)
        {
            var info = part.partInfo;
            if (info == null)
                return string.Empty;
            if (!string.IsNullOrEmpty (info.title))
                return info.title;
            return info.name ?? string.Empty;
        }

        /// <summary>
        /// The part's PartModule instances, read reflectively so this file needs no compile
        /// time knowledge of KSP's PartModuleList. Part.Modules is a public field in stock
        /// KSP; a property fallback is tried too. Null when the part exposes neither.
        /// </summary>
        static IEnumerable ModulesOf (Part part)
        {
            try {
                var type = part.GetType ();
                var field = type.GetField ("Modules", BindingFlags.Public | BindingFlags.Instance);
                if (field != null)
                    return field.GetValue (part) as IEnumerable;
                var property = type.GetProperty ("Modules", BindingFlags.Public | BindingFlags.Instance);
                if (property != null && property.GetIndexParameters ().Length == 0)
                    return property.GetValue (part, null) as IEnumerable;
            } catch (Exception) {
                // A part whose module list will not read is simply skipped.
            }
            return null;
        }

        /// <summary>One member as the nine tab-separated columns DescribeMember returns.</summary>
        static string FormatMember (MemberInfo member)
        {
            var field = member as FieldInfo;
            if (field != null)
                return Row ("field", member, VisibilityOf (field), field.IsStatic,
                            field.FieldType.Name, string.Empty, string.Empty);

            var property = member as PropertyInfo;
            if (property != null) {
                var getter = property.GetGetMethod (true);
                var setter = property.GetSetMethod (true);
                var accessor = getter ?? setter;
                var access = (getter != null ? "get" : string.Empty)
                             + (getter != null && setter != null ? " " : string.Empty)
                             + (setter != null ? "set" : string.Empty);
                return Row ("property", member,
                            accessor != null ? VisibilityOf (accessor) : string.Empty,
                            accessor != null && accessor.IsStatic,
                            property.PropertyType.Name,
                            ParamList (property.GetIndexParameters ()),
                            access);
            }

            var method = member as MethodInfo;
            if (method != null)
                return Row ("method", member, VisibilityOf (method), method.IsStatic,
                            method.ReturnType.Name, ParamList (method.GetParameters ()), string.Empty);

            var ctor = member as ConstructorInfo;
            if (ctor != null)
                return Row ("constructor", member, VisibilityOf (ctor), ctor.IsStatic,
                            "void", ParamList (ctor.GetParameters ()), string.Empty);

            var evt = member as EventInfo;
            if (evt != null)
                return Row ("event", member, string.Empty, false,
                            evt.EventHandlerType != null ? evt.EventHandlerType.Name : string.Empty,
                            string.Empty, string.Empty);

            var nested = member as Type;
            if (nested != null)
                return Row ("nestedtype", member, nested.IsNestedPublic ? "public" : "private",
                            false, nested.Name, string.Empty, string.Empty);

            return Row (member.MemberType.ToString ().ToLowerInvariant (), member,
                        string.Empty, false, string.Empty, string.Empty, string.Empty);
        }

        /// <summary>Assemble the fixed nine-column row so every kind splits the same in Python.</summary>
        static string Row (string kind, MemberInfo member, string visibility, bool isStatic,
                           string valueType, string parameters, string accessors)
        {
            var declaring = member.DeclaringType != null
                ? (member.DeclaringType.FullName ?? member.DeclaringType.Name)
                : string.Empty;
            return kind + "\t"
                   + declaring + "\t"
                   + visibility + "\t"
                   + (isStatic ? "static" : "instance") + "\t"
                   + (valueType ?? string.Empty) + "\t"
                   + parameters + "\t"
                   + accessors + "\t"
                   + TokenOf (member) + "\t"
                   + AttributesOf (member);
        }

        /// <summary>"kind:name:signature" building blocks reused for the fingerprint.</summary>
        static string KindOf (MemberInfo member)
        {
            if (member is FieldInfo) return "field";
            if (member is PropertyInfo) return "property";
            if (member is ConstructorInfo) return "constructor";
            if (member is MethodInfo) return "method";
            if (member is EventInfo) return "event";
            if (member is Type) return "nestedtype";
            return member.MemberType.ToString ().ToLowerInvariant ();
        }

        /// <summary>A compact, name-free signature for the fingerprint - shape, not parameter names.</summary>
        static string SignatureOf (MemberInfo member)
        {
            var field = member as FieldInfo;
            if (field != null)
                return field.FieldType.Name;

            var property = member as PropertyInfo;
            if (property != null) {
                var getter = property.GetGetMethod (true);
                var setter = property.GetSetMethod (true);
                return property.PropertyType.Name
                       + (getter != null ? " get" : string.Empty)
                       + (setter != null ? " set" : string.Empty);
            }

            var method = member as MethodInfo;
            if (method != null)
                return method.ReturnType.Name + " (" + ParamTypes (method.GetParameters ()) + ")";

            var ctor = member as ConstructorInfo;
            if (ctor != null)
                return "void (" + ParamTypes (ctor.GetParameters ()) + ")";

            var evt = member as EventInfo;
            if (evt != null)
                return evt.EventHandlerType != null ? evt.EventHandlerType.Name : string.Empty;

            var nested = member as Type;
            if (nested != null)
                return nested.Name;

            return string.Empty;
        }

        /// <summary>Parameter list with types and names, e.g. "Single value, Boolean flag".</summary>
        static string ParamList (ParameterInfo[] parameters)
        {
            if (parameters == null || parameters.Length == 0)
                return string.Empty;
            var builder = new StringBuilder ();
            foreach (var parameter in parameters) {
                if (builder.Length > 0)
                    builder.Append (", ");
                builder.Append (parameter.ParameterType.Name).Append (' ').Append (parameter.Name);
            }
            return builder.ToString ();
        }

        /// <summary>Parameter types only, for the shape-only fingerprint signature.</summary>
        static string ParamTypes (ParameterInfo[] parameters)
        {
            if (parameters == null || parameters.Length == 0)
                return string.Empty;
            var builder = new StringBuilder ();
            foreach (var parameter in parameters) {
                if (builder.Length > 0)
                    builder.Append (", ");
                builder.Append (parameter.ParameterType.Name);
            }
            return builder.ToString ();
        }

        static string VisibilityOf (FieldInfo field)
        {
            if (field.IsPublic) return "public";
            if (field.IsPrivate) return "private";
            if (field.IsFamily) return "protected";
            if (field.IsFamilyOrAssembly) return "protected internal";
            if (field.IsFamilyAndAssembly) return "private protected";
            if (field.IsAssembly) return "internal";
            return string.Empty;
        }

        static string VisibilityOf (MethodBase method)
        {
            if (method.IsPublic) return "public";
            if (method.IsPrivate) return "private";
            if (method.IsFamily) return "protected";
            if (method.IsFamilyOrAssembly) return "protected internal";
            if (method.IsFamilyAndAssembly) return "private protected";
            if (method.IsAssembly) return "internal";
            return string.Empty;
        }

        /// <summary>Metadata token as a decimal string, or "0" if the member will not yield one.</summary>
        static string TokenOf (MemberInfo member)
        {
            try {
                return member.MetadataToken.ToString (Culture);
            } catch (Exception) {
                return "0";
            }
        }

        /// <summary>
        /// Sorted, comma-separated attribute type names. Read from CustomAttributeData rather
        /// than GetCustomAttributes so a missing attribute assembly never throws by trying to
        /// construct the attribute.
        /// </summary>
        static string AttributesOf (MemberInfo member)
        {
            var names = new List<string> ();
            try {
                foreach (var data in member.GetCustomAttributesData ())
                    if (data != null && data.AttributeType != null)
                        names.Add (data.AttributeType.Name);
            } catch (Exception) {
                // Attribute metadata that will not read leaves the column empty.
            }
            names.Sort ();
            return string.Join (",", names.ToArray ());
        }

        /// <summary>Tabs and newlines would corrupt a packed row, so they are neutralised.</summary>
        static string Flatten (string text)
        {
            if (string.IsNullOrEmpty (text))
                return string.Empty;
            return text.Replace ('\t', ' ').Replace ('\n', ' ').Replace ('\r', ' ');
        }
    }
}
