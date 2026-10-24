using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// The ring of the event bus V2 (refactor lot 3, line 3.3; audit C, R6): every record stamped
    /// with the master tick of the game clock and the UT, naming its vessels and part by
    /// persistentId, read incrementally from a sequence number in ONE call per client tick
    /// (conn.bridge.poll_events_v2).
    ///
    /// PURE: System only (build/tests/core). The KSP side (which GameEvents feed it, when) is
    /// <see cref="EventBus"/>; a plugin publishes its own records through
    /// <see cref="EventBus.RecordV2"/>.
    ///
    /// The ring is PREALLOCATED: <see cref="Capacity"/> record objects created once and
    /// overwritten in place, so recording allocates nothing but the detail string the caller
    /// built. Sequence numbers start at 1 and never repeat within a session; a reader that
    /// fell more than <see cref="Capacity"/> records behind is told how many it lost (header
    /// field lost), never silently.
    /// </summary>
    public sealed class EventLogV2
    {
        /// <summary>Protocol of poll_events_v2 (the text table). A removed or moved column raises it.</summary>
        public const int Protocol = 1;

        /// <summary>Schema of poll_events_v2: +1 when a column or a header key is appended.</summary>
        public const int Schema = 1;

        /// <summary>Records kept. Past it, the oldest is overwritten.</summary>
        public const int Capacity = 1024;

        /// <summary>The column row of poll_events_v2, appended only.</summary>
        public static readonly string[] Columns = {
            "seq", "tick", "ut", "kind", "vessel_pid", "vessel_id", "other_pid", "part_pid", "value", "detail"
        };

        /// <summary>The header keys of poll_events_v2, appended only.</summary>
        public static readonly string[] HeaderKeys = {
            "protocol", "schema", "since", "first_seq", "last_seq", "lost", "truncated", "tick", "ut",
            "capacity", "n"
        };

        /// <summary>One record. Fields are overwritten when the ring wraps.</summary>
        public sealed class Record
        {
            public long Seq;
            public long Tick;
            public double Ut;
            public string Kind = string.Empty;
            public uint VesselPid;
            public string VesselId = string.Empty;
            public uint OtherPid;
            public uint PartPid;
            public double Value = double.NaN;
            public string Detail = string.Empty;
        }

        readonly Record[] ring;
        readonly int capacity;
        long nextSeq = 1;
        int count;
        int head;           // index of the oldest record

        /// <summary>A ring of <see cref="Capacity"/> records.</summary>
        public EventLogV2 () : this (Capacity)
        {
        }

        /// <summary>A ring of <paramref name="capacity"/> records (tests use a small one).</summary>
        public EventLogV2 (int capacity)
        {
            if (capacity < 1)
                throw new ArgumentOutOfRangeException (nameof (capacity));
            this.capacity = capacity;
            ring = new Record[capacity];
            for (int i = 0; i < capacity; i++)
                ring [i] = new Record ();
        }

        /// <summary>The highest sequence number given (0 before the first record).</summary>
        public long LastSeq {
            get { return nextSeq - 1; }
        }

        /// <summary>The sequence number of the oldest record still kept (LastSeq + 1 when empty).</summary>
        public long FirstSeq {
            get { return count == 0 ? nextSeq : ring [head].Seq; }
        }

        /// <summary>Records kept now.</summary>
        public int Count {
            get { return count; }
        }

        /// <summary>
        /// Appends one record and returns its sequence number. Never throws on a field value: a
        /// null kind becomes "unknown", null strings become empty, tabs and line breaks in the
        /// detail are flattened when the table is written.
        /// </summary>
        public long Append (string kind, long tick, double ut, uint vesselPid, string vesselId, uint otherPid,
            uint partPid, double value, string detail)
        {
            Record r;
            if (count < capacity) {
                r = ring [(head + count) % capacity];
                count++;
            } else {
                r = ring [head];
                head = (head + 1) % capacity;
            }
            r.Seq = nextSeq++;
            r.Tick = tick;
            r.Ut = ut;
            r.Kind = kind ?? "unknown";
            r.VesselPid = vesselPid;
            r.VesselId = vesselId ?? string.Empty;
            r.OtherPid = otherPid;
            r.PartPid = partPid;
            r.Value = value;
            r.Detail = detail ?? string.Empty;
            return r.Seq;
        }

        /// <summary>The record of that sequence number, or null (never given, or already overwritten).</summary>
        public Record Find (long seq)
        {
            if (count == 0 || seq < FirstSeq || seq > LastSeq)
                return null;
            return ring [(head + (int)(seq - FirstSeq)) % capacity];
        }

        /// <summary>Forgets every record (sequence numbers keep counting: they never repeat).</summary>
        public void Clear ()
        {
            count = 0;
            head = 0;
        }

        /// <summary>
        /// poll_events_v2: row 0 tab-separated key=value pairs (<see cref="HeaderKeys"/>): protocol,
        /// schema, since (as asked), first_seq and last_seq (the ring now), lost (records after
        /// <paramref name="since"/> already overwritten: the reader polled too slowly), truncated
        /// (1 when <paramref name="maxRows"/> cut the answer: poll again from the last seq), tick and
        /// ut (when the call ran), capacity, n (rows after the column row); row 1 the
        /// <see cref="Columns"/>; then one row per record with seq greater than since, oldest first,
        /// at most maxRows (0 or less: no limit). Numbers are invariant: tick, seq and ids as
        /// integers, ut and value round-trip ("R"), NaN as "NaN".
        /// </summary>
        public List<string> Lines (long since, int maxRows, long tick, double ut)
        {
            var inv = CultureInfo.InvariantCulture;
            if (since < 0)
                since = 0;
            long first = FirstSeq;
            long last = LastSeq;
            long start = Math.Max (since + 1, first);
            long available = last >= start ? last - start + 1 : 0;
            long lost = since + 1 < first ? Math.Min (first, last + 1) - (since + 1) : 0;
            if (lost < 0)
                lost = 0;
            long n = available;
            bool truncated = false;
            if (maxRows > 0 && n > maxRows) {
                n = maxRows;
                truncated = true;
            }
            var lines = new List<string> ((int)n + 2);
            var head = new StringBuilder (160);
            head.Append ("protocol=").Append (Protocol.ToString (inv));
            head.Append ("\tschema=").Append (Schema.ToString (inv));
            head.Append ("\tsince=").Append (since.ToString (inv));
            head.Append ("\tfirst_seq=").Append (first.ToString (inv));
            head.Append ("\tlast_seq=").Append (last.ToString (inv));
            head.Append ("\tlost=").Append (lost.ToString (inv));
            head.Append ("\ttruncated=").Append (truncated ? "1" : "0");
            head.Append ("\ttick=").Append (tick.ToString (inv));
            head.Append ("\tut=").Append (ut.ToString ("R", inv));
            head.Append ("\tcapacity=").Append (capacity.ToString (inv));
            head.Append ("\tn=").Append (n.ToString (inv));
            lines.Add (head.ToString ());
            lines.Add (string.Join ("\t", Columns));
            var row = new StringBuilder (128);
            for (long s = start; s < start + n; s++) {
                var r = Find (s);
                if (r == null)
                    break;
                row.Length = 0;
                row.Append (r.Seq.ToString (inv)).Append ('\t');
                row.Append (r.Tick.ToString (inv)).Append ('\t');
                row.Append (r.Ut.ToString ("R", inv)).Append ('\t');
                row.Append (Flatten (r.Kind)).Append ('\t');
                row.Append (r.VesselPid.ToString (inv)).Append ('\t');
                row.Append (Flatten (r.VesselId)).Append ('\t');
                row.Append (r.OtherPid.ToString (inv)).Append ('\t');
                row.Append (r.PartPid.ToString (inv)).Append ('\t');
                row.Append (r.Value.ToString ("R", inv)).Append ('\t');
                row.Append (Flatten (r.Detail));
                lines.Add (row.ToString ());
            }
            return lines;
        }

        /// <summary>Tabs and line breaks would corrupt a packed row: they become spaces.</summary>
        public static string Flatten (string text)
        {
            if (string.IsNullOrEmpty (text))
                return string.Empty;
            if (text.IndexOf ('\t') < 0 && text.IndexOf ('\n') < 0 && text.IndexOf ('\r') < 0)
                return text;
            return text.Replace ('\t', ' ').Replace ('\n', ' ').Replace ('\r', ' ');
        }
    }
}
