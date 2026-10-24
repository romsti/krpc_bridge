using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// Schema descriptors of every wire layout the mod publishes: frames, returns, status
    /// replies and text tables, with the names of their fields, so that a client generates its
    /// decoders instead of copying offsets (refactor R1, plan 2.5). Served by
    /// conn.bridge.layouts_v1() / layout_v1(name); the plugin that owns a layout declares it at
    /// load, from the constants that version it.
    ///
    /// PURE: System only (build/tests/core).
    ///
    /// A layout is a run of SECTIONS, each a block of fields read once (count 1) or repeated
    /// (count read from a field of an earlier section), with a stride that is a literal or read
    /// from a field, or a run of length-prefixed sub-frames (a history). The rules every layout
    /// follows, checked where they can be:
    ///   * fields are only ever APPENDED (at the end of a section whose stride the frame
    ///     carries, or of the layout); a field appended raises the schema, a field removed,
    ///     renamed or moved raises the protocol (never done: a new layout is made instead);
    ///   * a layout never depends on a MODE: a field that a mode does not fill is NaN, never
    ///     absent (attitude_translation_v1 broke that rule before R1; attitude_translation_v2
    ///     is its fixed layout);
    ///   * units are SI in every layout created from R1 on (Frames: the "SI in RPCs" rule); a
    ///     field that keeps a native unit for wire compatibility says so in its name.
    /// </summary>
    public static class LayoutRegistry
    {
        /// <summary>Protocol of the layout_v1 / layouts_v1 wire format.</summary>
        public const int Protocol = 1;

        /// <summary>Kinds of layout.</summary>
        public const string KindFrame = "frame";        // one physics step, kept in a ring
        public const string KindHistory = "history";    // the six-value header + length-prefixed frames
        public const string KindReturn = "return";      // the answer of one call
        public const string KindStatus = "status";      // a status reply (streamable)
        public const string KindBlock = "block";        // a block appended to another layout
        public const string KindText = "text";          // tab-separated text: key=value header + columns

        /// <summary>One field: its name and unit ("" when dimensionless or a code).</summary>
        public sealed class Field
        {
            public string Name;
            public string Unit;
        }

        /// <summary>
        /// One section. Count: "1" (once), an integer, or "@field" (read from that field of an
        /// earlier section). Stride: an integer, or "@field". Prefixed: each repetition starts
        /// with its own length (a history's frames), and Frame names the layout of each.
        /// </summary>
        public sealed class Section
        {
            public string Name;
            public string Count = "1";
            public string Stride = "";
            public bool Prefixed;
            public string Frame = "";
            public readonly List<Field> Fields = new List<Field> ();
        }

        /// <summary>One layout.</summary>
        public sealed class Layout
        {
            public string Name;
            public string Service;
            public string Owner;
            public int LayoutProtocol;
            public int LayoutSchema;
            public string Kind;
            public string Note = "";
            public readonly List<Section> Sections = new List<Section> ();

            /// <summary>Adds a section of fields read once (stride = their count).</summary>
            public Section Once (string name, params string[] fields)
            {
                return Add (name, "1", fields.Length.ToString (CultureInfo.InvariantCulture), fields);
            }

            /// <summary>Adds a section repeated `count` times (a field reference or a literal).</summary>
            public Section Rows (string name, string count, string stride, params string[] fields)
            {
                return Add (name, count, stride, fields);
            }

            /// <summary>Adds the length-prefixed frames of a history (each a `frame` layout).</summary>
            public Section Frames (string name, string count, string frame)
            {
                var s = new Section { Name = name, Count = count, Stride = "@length", Prefixed = true, Frame = frame };
                Sections.Add (s);
                return s;
            }

            Section Add (string name, string count, string stride, string[] fields)
            {
                var s = new Section { Name = name, Count = count, Stride = stride };
                foreach (var f in fields)
                    s.Fields.Add (new Field { Name = f, Unit = InferUnit (f) });
                Sections.Add (s);
                return s;
            }

            /// <summary>Every field name, sections in order (the flat list attitude_layout_v1 returns).</summary>
            public List<string> FieldNames ()
            {
                var names = new List<string> ();
                foreach (var s in Sections)
                    foreach (var f in s.Fields)
                        names.Add (f.Name);
                return names;
            }

            /// <summary>The fields of one section, or null.</summary>
            public Section Find (string section)
            {
                foreach (var s in Sections)
                    if (s.Name == section)
                        return s;
                return null;
            }
        }

        static readonly object gate = new object ();
        static readonly List<Layout> layouts = new List<Layout> ();
        static readonly List<string> problems = new List<string> ();

        /// <summary>A new layout, registered (same name again: replaced, so a re-load is harmless).</summary>
        public static Layout Declare (string name, string service, string owner, int protocol, int schema,
            string kind, string note = "")
        {
            if (string.IsNullOrEmpty (name))
                throw new ArgumentException ("layout name", nameof (name));
            var layout = new Layout {
                Name = name, Service = service ?? "", Owner = owner ?? "", LayoutProtocol = protocol,
                LayoutSchema = schema, Kind = kind ?? KindReturn, Note = note ?? ""
            };
            lock (gate) {
                for (int i = 0; i < layouts.Count; i++)
                    if (layouts [i].Name == name) {
                        layouts [i] = layout;
                        return layout;
                    }
                layouts.Add (layout);
            }
            return layout;
        }

        /// <summary>
        /// Records a mismatch between a declared section and the constant that sizes it (field
        /// count != stride). Checked by the replay harness: a descriptor that does not describe
        /// the frame is a build failure, not a client surprise.
        /// </summary>
        public static void Expect (Layout layout, string section, int stride)
        {
            var s = layout.Find (section);
            int n = s == null ? -1 : s.Fields.Count;
            if (n != stride)
                lock (gate)
                    problems.Add (layout.Name + "." + section + ": " + n.ToString (CultureInfo.InvariantCulture) +
                        " fields declared, stride " + stride.ToString (CultureInfo.InvariantCulture));
        }

        /// <summary>The mismatches Expect found (empty when every descriptor fits its constants).</summary>
        public static List<string> Problems ()
        {
            lock (gate)
                return new List<string> (problems);
        }

        /// <summary>The layout of that name, or null.</summary>
        public static Layout Find (string name)
        {
            lock (gate) {
                foreach (var l in layouts)
                    if (l.Name == name)
                        return l;
            }
            return null;
        }

        /// <summary>The names of the declared layouts, sorted.</summary>
        public static List<string> Names ()
        {
            var names = new List<string> ();
            lock (gate)
                foreach (var l in layouts)
                    names.Add (l.Name);
            names.Sort (StringComparer.Ordinal);
            return names;
        }

        /// <summary>layouts_v1: one row per layout, "name\tservice\tprotocol\tschema\tkind\tfields", sorted.</summary>
        public static List<string> NameLines ()
        {
            var lines = new List<string> ();
            foreach (var name in Names ()) {
                var l = Find (name);
                lines.Add (l.Name + "\t" + l.Service + "\t" + l.LayoutProtocol.ToString (CultureInfo.InvariantCulture) +
                    "\t" + l.LayoutSchema.ToString (CultureInfo.InvariantCulture) + "\t" + l.Kind + "\t" +
                    l.FieldNames ().Count.ToString (CultureInfo.InvariantCulture));
            }
            return lines;
        }

        /// <summary>
        /// layout_v1(name): row 0 key=value pairs (protocol of this format, layout, service,
        /// layout_protocol, layout_schema, kind, sections, fields, note); then per section a line
        /// "#section\tname\tcount\tstride\tprefixed\tframe" followed by its fields, one line each:
        /// "section\toffset\tname\tunit" (offset within the section, or within a row of it).
        /// Unknown name: ArgumentException listing the known ones.
        /// </summary>
        public static List<string> Lines (string name)
        {
            var l = Find (name);
            if (l == null)
                throw new ArgumentException ("unknown layout: " + name + " (known: " +
                    string.Join (", ", Names ().ToArray ()) + ")", nameof (name));
            var inv = CultureInfo.InvariantCulture;
            var lines = new List<string> ();
            var head = new StringBuilder ();
            head.Append ("protocol=").Append (Protocol.ToString (inv));
            head.Append ("\tlayout=").Append (l.Name);
            head.Append ("\tservice=").Append (l.Service);
            head.Append ("\tlayout_protocol=").Append (l.LayoutProtocol.ToString (inv));
            head.Append ("\tlayout_schema=").Append (l.LayoutSchema.ToString (inv));
            head.Append ("\tkind=").Append (l.Kind);
            head.Append ("\tsections=").Append (l.Sections.Count.ToString (inv));
            head.Append ("\tfields=").Append (l.FieldNames ().Count.ToString (inv));
            head.Append ("\tnote=").Append (Clean (l.Note));
            lines.Add (head.ToString ());
            foreach (var s in l.Sections) {
                lines.Add ("#section\t" + s.Name + "\t" + s.Count + "\t" + s.Stride + "\t" +
                    (s.Prefixed ? "1" : "0") + "\t" + s.Frame);
                for (int i = 0; i < s.Fields.Count; i++)
                    lines.Add (s.Name + "\t" + i.ToString (inv) + "\t" + s.Fields [i].Name + "\t" + s.Fields [i].Unit);
            }
            return lines;
        }

        /// <summary>The flat field names of a layout (sections in order). Unknown: ArgumentException.</summary>
        public static List<string> FieldNames (string name)
        {
            var l = Find (name);
            if (l == null)
                throw new ArgumentException ("unknown layout: " + name + " (known: " +
                    string.Join (", ", Names ().ToArray ()) + ")", nameof (name));
            return l.FieldNames ();
        }

        // Suffix -> unit, longest first. A trailing _x/_y/_z/_w (component) is ignored.
        static readonly string[][] suffixes = {
            new[] { "_rad_s2", "rad/s2" }, new[] { "_kg_m2", "kg.m2" }, new[] { "_kg_m3", "kg/m3" },
            new[] { "_kg_s", "kg/s" }, new[] { "_deg_s", "deg/s" }, new[] { "_rad_s", "rad/s" },
            new[] { "_ms2", "m/s2" }, new[] { "_deg2", "deg2" }, new[] { "_tonnes", "t" },
            new[] { "_nm", "N.m" }, new[] { "_kn", "kN" }, new[] { "_kpa", "kPa" }, new[] { "_pa", "Pa" },
            new[] { "_ms", "m/s" }, new[] { "_us", "us" }, new[] { "_pct", "%" }, new[] { "_atm", "atm" },
            new[] { "_deg", "deg" }, new[] { "_rad", "rad" }, new[] { "_kg", "kg" }, new[] { "_n", "N" },
            new[] { "_m", "m" }, new[] { "_s", "s" }
        };

        /// <summary>The unit a field name carries in its suffix ("" when none).</summary>
        public static string InferUnit (string name)
        {
            if (string.IsNullOrEmpty (name))
                return string.Empty;
            string n = name;
            if (n.Length > 2 && n [n.Length - 2] == '_' && "xyzw".IndexOf (n [n.Length - 1]) >= 0)
                n = n.Substring (0, n.Length - 2);
            foreach (var pair in suffixes)
                if (n.EndsWith (pair [0], StringComparison.Ordinal))
                    return pair [1];
            return string.Empty;
        }

        /// <summary>prefix + "x", "y", "z" (+ suffix): the three components of a vector field.</summary>
        public static string[] Xyz (string prefix, string suffix = "")
        {
            return new[] { prefix + "x" + suffix, prefix + "y" + suffix, prefix + "z" + suffix };
        }

        /// <summary>Concatenates field groups (single names or Xyz arrays) into one array.</summary>
        public static string[] Join (params object[] parts)
        {
            var all = new List<string> ();
            foreach (var p in parts) {
                var one = p as string;
                if (one != null) {
                    all.Add (one);
                    continue;
                }
                var many = p as string[];
                if (many != null)
                    all.AddRange (many);
            }
            return all.ToArray ();
        }

        static string Clean (string text)
        {
            if (string.IsNullOrEmpty (text))
                return string.Empty;
            return text.Replace ('\t', ' ').Replace ('\n', ' ').Replace ('\r', ' ');
        }

        /// <summary>Test hook: forgets every layout and problem.</summary>
        internal static void ResetForTests ()
        {
            lock (gate) {
                layouts.Clear ();
                problems.Clear ();
            }
        }
    }
}
