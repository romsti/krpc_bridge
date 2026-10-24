using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// The declared meaning of every bit the mod publishes in a mask: motifs (why a channel did
    /// or did not write), reasons (why the lateral force is limited), capabilities (which
    /// quantities a frame carries), options. One table per named set, filled by the plugin
    /// that owns the bits (at load, from its own constants), and exported as is by
    /// conn.bridge.bit_sets_v1() / bits_v1(set).
    ///
    /// PURE: System only. Registration is idempotent per (set, bit): re-registering the same
    /// definition is a no-op, a different definition for a taken bit throws (a build/tests
    /// check, so a clash never reaches a game).
    /// </summary>
    public static class BitRegistry
    {
        /// <summary>Protocol of the bits_v1 wire format.</summary>
        public const int Protocol = 1;

        /// <summary>Columns of bits_v1, appended only.</summary>
        public static readonly string[] Columns = { "bit", "name", "kind", "note" };

        /// <summary>Kinds of bit. A motif is BLOCKING for a channel when it forbids that channel's write.</summary>
        public const string KindBlockingAll = "blocking";            // blocks every channel it applies to
        public const string KindBlockingRoll = "blocking_roll";       // blocks the roll write only
        public const string KindBlockingPitchYaw = "blocking_pitch_yaw";
        public const string KindInformational = "informational";
        public const string KindCapability = "capability";
        public const string KindReason = "reason";
        public const string KindOption = "option";

        /// <summary>One declared bit.</summary>
        public sealed class Bit
        {
            public int Index;
            public string Name;
            public string Kind;
            public string Note;
        }

        sealed class Set
        {
            internal string Name;
            internal string Owner;
            internal readonly SortedDictionary<int, Bit> Bits = new SortedDictionary<int, Bit> ();
        }

        static readonly object gate = new object ();
        static readonly List<Set> sets = new List<Set> ();

        /// <summary>
        /// Declares bit <paramref name="index"/> (0-62) of set <paramref name="set"/>. Same
        /// definition again: no-op. Another definition of a taken bit: InvalidOperationException.
        /// </summary>
        public static void Declare (string set, string owner, int index, string name, string kind,
            string note = "")
        {
            if (string.IsNullOrEmpty (set) || string.IsNullOrEmpty (name) || string.IsNullOrEmpty (kind))
                throw new ArgumentException ("set, name and kind are required");
            if (index < 0 || index > 62)
                throw new ArgumentOutOfRangeException (nameof (index), "0..62 (a long published as a double)");
            lock (gate) {
                Set s = Find (set);
                if (s == null) {
                    s = new Set { Name = set, Owner = owner ?? string.Empty };
                    sets.Add (s);
                }
                Bit existing;
                if (s.Bits.TryGetValue (index, out existing)) {
                    if (existing.Name == name && existing.Kind == kind && existing.Note == (note ?? string.Empty))
                        return;
                    throw new InvalidOperationException ("bit " + index.ToString (CultureInfo.InvariantCulture) +
                        " of " + set + " is already " + existing.Name + ", not " + name);
                }
                foreach (var pair in s.Bits)
                    if (pair.Value.Name == name)
                        throw new InvalidOperationException ("name " + name + " of " + set +
                            " is already bit " + pair.Key.ToString (CultureInfo.InvariantCulture));
                s.Bits.Add (index, new Bit { Index = index, Name = name, Kind = kind, Note = note ?? string.Empty });
            }
        }

        /// <summary>Declares a mask's bits in one call: names[i] is bit i (null or "" skips it).</summary>
        public static void DeclareAll (string set, string owner, string kind, string[] names)
        {
            for (int i = 0; i < names.Length; i++)
                if (!string.IsNullOrEmpty (names [i]))
                    Declare (set, owner, i, names [i], kind);
        }

        static Set Find (string name)
        {
            for (int i = 0; i < sets.Count; i++)
                if (sets [i].Name == name)
                    return sets [i];
            return null;
        }

        /// <summary>"set\towner\tbits" for every declared set, sorted by name.</summary>
        public static List<string> SetLines ()
        {
            var lines = new List<string> ();
            lock (gate) {
                foreach (var s in sets)
                    lines.Add (s.Name + "\t" + s.Owner + "\t" + s.Bits.Count.ToString (CultureInfo.InvariantCulture));
            }
            lines.Sort (StringComparer.Ordinal);
            return lines;
        }

        /// <summary>The declared bits of a set, by index; empty when the set is unknown.</summary>
        public static List<Bit> Bits (string set)
        {
            var bits = new List<Bit> ();
            lock (gate) {
                var s = Find (set);
                if (s != null)
                    foreach (var pair in s.Bits)
                        bits.Add (pair.Value);
            }
            return bits;
        }

        /// <summary>
        /// bits_v1 wire format: a header line of key=value pairs (protocol, set, owner, n), the
        /// column line, then one tab-separated row per bit in index order. Unknown set:
        /// ArgumentException naming the known ones.
        /// </summary>
        public static List<string> Lines (string set)
        {
            Set s;
            List<Bit> bits;
            lock (gate) {
                s = Find (set);
                if (s == null) {
                    var known = new List<string> ();
                    foreach (var x in sets)
                        known.Add (x.Name);
                    known.Sort (StringComparer.Ordinal);
                    throw new ArgumentException ("unknown bit set: " + set + " (known: " +
                        string.Join (", ", known.ToArray ()) + ")", nameof (set));
                }
                bits = new List<Bit> (s.Bits.Values);
            }
            var lines = new List<string> (bits.Count + 2);
            var head = new StringBuilder ();
            head.Append ("protocol=").Append (Protocol.ToString (CultureInfo.InvariantCulture));
            head.Append ("\tset=").Append (Clean (s.Name));
            head.Append ("\towner=").Append (Clean (s.Owner));
            head.Append ("\tn=").Append (bits.Count.ToString (CultureInfo.InvariantCulture));
            lines.Add (head.ToString ());
            lines.Add (string.Join ("\t", Columns));
            foreach (var b in bits)
                lines.Add (b.Index.ToString (CultureInfo.InvariantCulture) + "\t" + Clean (b.Name) + "\t" +
                    Clean (b.Kind) + "\t" + Clean (b.Note));
            return lines;
        }

        /// <summary>The mask with each set bit named, "a|b|c" (an undeclared bit N as "bitN").</summary>
        public static string Describe (string set, long mask)
        {
            var parts = new List<string> ();
            var bits = Bits (set);
            for (int i = 0; i <= 62; i++) {
                if ((mask & (1L << i)) == 0)
                    continue;
                string name = null;
                foreach (var b in bits)
                    if (b.Index == i)
                        name = b.Name;
                parts.Add (name ?? "bit" + i.ToString (CultureInfo.InvariantCulture));
            }
            return string.Join ("|", parts.ToArray ());
        }

        static string Clean (string text)
        {
            if (string.IsNullOrEmpty (text))
                return string.Empty;
            return text.Replace ('\t', ' ').Replace ('\n', ' ').Replace ('\r', ' ');
        }

        /// <summary>Test hook: forgets every set.</summary>
        internal static void ResetForTests ()
        {
            lock (gate)
                sets.Clear ();
        }
    }
}
