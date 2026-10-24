// Core v2 pure tests (refactor R1). See CoreTests.csproj.

using System;
using System.Collections.Generic;
using System.Globalization;

namespace KRPC.Bridge.Core
{
    internal static class CoreTests
    {
        static int checks;
        static int failures;

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                if (failures <= 40)
                    Console.WriteLine ("FAIL: " + what);
            }
        }

        public static int Main (string[] args)
        {
            RecorderMatchesLegacy ();
            RecorderReserveIsZeroed ();
            RecorderEdgeCases ();
            LeaseMatchesLegacy ();
            LeaseTicks ();
            ClockCounters ();
            Bits ();
            FramesMaths ();
            Layouts ();
            Errors ();
            EventLog ();
            Console.WriteLine ("core v2: " + checks + " checks, " + failures + " failures");
            return failures == 0 ? 0 : 1;
        }

        // ------------------------------------------------------------------ Recorder

        static bool SameBits (IList<double> a, IList<double> b, out int first)
        {
            first = -1;
            if (a.Count != b.Count) {
                first = Math.Min (a.Count, b.Count);
                return false;
            }
            for (int i = 0; i < a.Count; i++)
                if (BitConverter.DoubleToInt64Bits (a [i]) != BitConverter.DoubleToInt64Bits (b [i])) {
                    first = i;
                    return false;
                }
            return true;
        }

        /// <summary>
        /// Random sequences of pushes (lengths 0-60, ticks mostly +1..+3, sometimes equal, a few
        /// decreasing, NaN or huge), clears and reads (since and max over their whole range, out
        /// of range included), on capacities 1 to 1000: the legacy ring and the Recorder must give
        /// the same doubles, bit for bit, for both history readers, and the same count and
        /// newest frame.
        /// </summary>
        static void RecorderMatchesLegacy ()
        {
            int[] capacities = { 1, 2, 3, 5, 17, 128, 1000 };
            int scenarios = 0, reads = 0;
            for (int seed = 1; seed <= 140; seed++) {
                var rng = new Random (seed);
                int capacity = capacities [seed % capacities.Length];
                bool disorder = seed % 4 == 0;
                var legacy = new LinkedList<double[]> ();
                var recorder = new Recorder (capacity);
                long tick = rng.Next (0, 5);
                int ops = 300 + rng.Next (2500);
                for (int op = 0; op < ops; op++) {
                    int kind = rng.Next (100);
                    if (kind < 70) {
                        int length = rng.Next (10) == 0 ? rng.Next (0, 3) : 3 + rng.Next (58);
                        int step = rng.Next (10) == 0 ? 0 : 1 + rng.Next (3);
                        tick += step;
                        if (disorder && rng.Next (30) == 0)
                            tick -= 1 + rng.Next (6);
                        var frame = new double[length];
                        for (int i = 0; i < length; i++)
                            frame [i] = rng.NextDouble () * 1000.0 - 500.0;
                        if (length >= 3) {
                            frame [2] = tick;
                            if (disorder && rng.Next (60) == 0)
                                frame [2] = double.NaN;
                            if (rng.Next (200) == 0)
                                frame [2] = tick + 0.75;   // fractional tick: truncated by both
                        }
                        if (length > 0 && rng.Next (20) == 0)
                            frame [0] = double.NaN;
                        LegacyRing.Push (legacy, frame, capacity);
                        if (rng.Next (2) == 0) {
                            recorder.Push (frame);
                        } else {
                            var buffer = recorder.Reserve (length);
                            for (int i = 0; i < length; i++)
                                buffer [i] = frame [i];
                            recorder.Commit (length);
                        }
                    } else if (kind < 72) {
                        legacy.Clear ();
                        recorder.Clear ();
                    } else {
                        int since;
                        switch (rng.Next (6)) {
                        case 0: since = 0; break;
                        case 1: since = int.MinValue + rng.Next (3); break;
                        case 2: since = int.MaxValue - rng.Next (3); break;
                        case 3: since = (int)tick - rng.Next (40); break;
                        case 4: since = -rng.Next (10); break;
                        default: since = rng.Next ((int)Math.Max (1, tick + 10)); break;
                        }
                        int max = rng.Next (8) == 0 ? rng.Next (-5, 1) : rng.Next (1, 200);
                        int protocol = 1 + rng.Next (3), schema = rng.Next (4);
                        var a = LegacyRing.ReadRing (legacy, protocol, schema, since, max);
                        var b = recorder.ReadSince (protocol, schema, since, max);
                        int first;
                        Check (SameBits (a, b, out first), "ReadRing seed " + seed + " op " + op +
                            " since " + since + " max " + max + " first difference at " + first);
                        var c = LegacyRing.ReadHistoryAero (legacy, protocol, schema, since, max);
                        Check (SameBits (c, b, out first), "ReadHistoryAero seed " + seed + " op " + op);
                        reads++;
                    }
                    if (legacy.Count != recorder.Count) {
                        Check (false, "count seed " + seed + " op " + op);
                        break;
                    }
                    if (op % 50 == 0) {
                        var latestLegacy = legacy.Last == null ? new double[0] : legacy.Last.Value;
                        int first;
                        Check (SameBits (new List<double> (latestLegacy), recorder.CopyNewest (), out first),
                            "newest seed " + seed + " op " + op);
                        Check (SameBits (latestLegacy, recorder.NewestArray (), out first),
                            "newest array seed " + seed + " op " + op);
                    }
                }
                scenarios++;
            }
            Check (scenarios == 140 && reads > 10000, "recorder scenarios ran (" + reads + " reads)");
        }

        /// <summary>
        /// Reserve gives what new double[n] gives: zeros, even in a reused buffer that held a
        /// longer frame (a writer that leaves an index unwritten publishes 0, as before).
        /// </summary>
        static void RecorderReserveIsZeroed ()
        {
            var r = new Recorder (3);
            for (int k = 0; k < 20; k++) {
                int length = k % 2 == 0 ? 50 : 7;
                var buffer = r.Reserve (length);
                bool zero = true;
                for (int i = 0; i < length; i++)
                    if (buffer [i] != 0.0 || BitConverter.DoubleToInt64Bits (buffer [i]) != 0)
                        zero = false;
                Check (zero, "reserve zeroed, round " + k);
                for (int i = 0; i < buffer.Length; i++)
                    buffer [i] = double.NaN;      // garbage beyond length too
                buffer [2] = k;
                r.Commit (length);
                var read = r.ReadSince (1, 1, k - 1, 1);
                Check (read.Count == 6 + 1 + length && read [6] == length, "committed length " + k);
            }
            // Reserve while full: the oldest frame stays readable until Commit.
            var full = new Recorder (2);
            full.Push (new double[] { 9, 9, 1 });
            full.Push (new double[] { 9, 9, 2 });
            var pending = full.Reserve (3);
            pending [2] = 3;
            var before = full.ReadSince (1, 1, 0, 10);
            Check (before [2] == 2 && before [3] == 1 && before [4] == 2, "oldest readable during reserve");
            full.Commit (3);
            var after = full.ReadSince (1, 1, 0, 10);
            Check (after [2] == 2 && after [3] == 2 && after [4] == 3, "oldest evicted at commit");
            // Many wraps: no buffer handed out twice while still in the ring.
            var wrap = new Recorder (4);
            for (int t = 1; t <= 50; t++) {
                var b = wrap.Reserve (3 + t % 5);
                b [2] = t;
                b [0] = t;
                wrap.Commit (3 + t % 5);
                var all = wrap.ReadSince (1, 1, 0, 10);
                int frames = (int)all [2];
                int at = 6;
                bool ok = frames == Math.Min (4, t);
                for (int f = 0; f < frames; f++) {
                    int len = (int)all [at];
                    long expected = t - frames + 1 + f;
                    ok &= all [at + 1] == expected && all [at + 3] == expected;
                    at += 1 + len;
                }
                Check (ok, "wrap " + t);
            }
        }

        static void RecorderEdgeCases ()
        {
            var r = new Recorder (5);
            var empty = r.ReadSince (3, 2, 0, 64);
            Check (empty.Count == 6 && empty [0] == 3 && empty [1] == 2 && empty [2] == 0 &&
                empty [3] == 0 && empty [4] == 0 && empty [5] == 0, "empty read");
            Check (r.CopyNewest ().Count == 0 && r.NewestArray ().Length == 0, "empty newest");
            r.Push (new double[] { 1, 2, 10 });
            r.Push (new double[] { 1, 2, 11 });
            var dropped = r.ReadSince (1, 1, 3, 64);
            Check (dropped [5] == 1.0, "dropped_before when since < oldest - 1");
            var notDropped = r.ReadSince (1, 1, 9, 64);
            Check (notDropped [5] == 0.0 && notDropped [2] == 2, "since = oldest - 1");
            Check (r.Ordered, "ordered");
            r.Push (new double[] { 1, 2, 5 });
            Check (!r.Ordered, "a decreasing tick switches to the scan");
            r.Clear ();
            Check (r.Ordered && r.Count == 0, "clear resets order");
            bool threw = false;
            try {
                r.Commit (1);
            } catch (InvalidOperationException) {
                threw = true;
            }
            Check (threw, "commit without reserve");
        }

        // ------------------------------------------------------------------ Lease

        static void LeaseMatchesLegacy ()
        {
            var rng = new Random (42);
            var lease = new Lease ();
            int mismatches = 0;
            for (int i = 0; i < 200000; i++) {
                float now = (float)(rng.NextDouble () * (i % 3 == 0 ? 1e6 : 3600.0));
                float seconds = (float)(rng.NextDouble () * 60.0);
                if (i % 7 == 0)
                    seconds = new[] { 0.05f, 0.1f, 0.25f, 0.5f, 1f, 5f, 10f, 60f } [i % 8];
                float legacy = LegacyRing.ExpiresAt (now, seconds);
                lease.RenewReal (now, seconds);
                if (BitConverter.SingleToInt32Bits (legacy) != BitConverter.SingleToInt32Bits (lease.ExpiresAtReal))
                    mismatches++;
                float clamp = Math.Min (1f, Math.Max (0.05f, seconds));
                lease.RenewReal (now, clamp);
                if (BitConverter.SingleToInt32Bits (LegacyRing.ThrottleExpiresAt (now, seconds)) !=
                        BitConverter.SingleToInt32Bits (lease.ExpiresAtReal))
                    mismatches++;
                float later = now + (float)((rng.NextDouble () - 0.2) * 2.0 * seconds);
                if (i % 11 == 0)
                    later = lease.ExpiresAtReal;           // exactly at the end
                if (LegacyRing.Expired (later, lease.ExpiresAtReal) != lease.ExpiredReal (later) ||
                        LegacyRing.Expired (later, lease.ExpiresAtReal) != lease.Expired (later, i) ||
                        LegacyRing.Expired (later, lease.ExpiresAtReal) == lease.ValidReal (later))
                    mismatches++;
                double wireLegacy = LegacyRing.Remaining (lease.ExpiresAtReal, later);
                double wireNew = new List<double> { lease.RemainingReal (later) } [0];
                if (BitConverter.DoubleToInt64Bits (wireLegacy) != BitConverter.DoubleToInt64Bits (wireNew))
                    mismatches++;
            }
            Check (mismatches == 0, "lease arithmetic = legacy, bit for bit (" + mismatches + " mismatches)");
        }

        static void LeaseTicks ()
        {
            var lease = new Lease ();
            Check (!lease.Held && lease.ExpiresAtTick == Lease.NoTickLimit, "new lease");
            lease.Grant ("o", "t", 10f, 1f, 100, 25);
            Check (lease.Held && lease.Owner == "o" && lease.ExpiresAtTick == 125, "grant with ticks");
            Check (!lease.Expired (10.5f, 124) && lease.Expired (10.5f, 125), "tick budget expires");
            Check (lease.Expired (11f, 0), "wall guard still expires");
            Check (lease.RemainingTicks (120) == 5 && lease.RemainingTicks (130) == 0, "remaining ticks");
            lease.Grant ("o", "t", 10f, 1f, 100, 0);
            Check (lease.ExpiresAtTick == Lease.NoTickLimit && !lease.Expired (10.5f, long.MaxValue - 1),
                "no tick budget");
            lease.SetTicks (long.MaxValue - 3, 10);
            Check (lease.ExpiresAtTick == Lease.NoTickLimit, "overflow means no budget");
            lease.Clear ();
            Check (!lease.Held && lease.ExpiresAtReal == 0f && lease.ExpiresAtTick == Lease.NoTickLimit, "clear");
        }

        // ------------------------------------------------------------------ clock

        static void ClockCounters ()
        {
            var clock = new ClockCore ();
            Check (clock.Tick == 0 && double.IsNaN (clock.Ut), "clock starts at 0");
            var pump = clock.Counter ("actuators.pump_tick");
            Check (ReferenceEquals (pump, clock.Counter ("actuators.pump_tick")), "one counter per name");
            pump.Increment ();
            Check (pump.Value == 1 && pump.LastTick == 0, "increment before any step");
            clock.Open (0.02, 100.0, 0.02, 1.0, 5.0);
            clock.Open (0.04, 100.02, 0.02, 1.0, 5.03);
            Check (clock.Tick == 2 && clock.Ut == 100.02 && clock.FixedTime == 0.04, "two steps");
            pump.Increment ();
            pump.Increment ();
            Check (pump.Value == 3 && pump.LastTick == 2, "value kept, master tick recorded");
            Check (clock.Counters.Count == 1, "counters listed");
        }

        // ------------------------------------------------------------------ bits

        static void Bits ()
        {
            BitRegistry.ResetForTests ();
            BitRegistry.Declare ("test.motifs", "Tests", 0, "lease_expired", BitRegistry.KindBlockingAll, "n");
            BitRegistry.Declare ("test.motifs", "Tests", 0, "lease_expired", BitRegistry.KindBlockingAll, "n");
            bool threw = false;
            try {
                BitRegistry.Declare ("test.motifs", "Tests", 0, "other", BitRegistry.KindBlockingAll, "n");
            } catch (InvalidOperationException) {
                threw = true;
            }
            Check (threw, "a taken bit with another name throws");
            threw = false;
            try {
                BitRegistry.Declare ("test.motifs", "Tests", 3, "lease_expired", BitRegistry.KindBlockingAll, "n");
            } catch (InvalidOperationException) {
                threw = true;
            }
            Check (threw, "a name on two bits throws");
            BitRegistry.DeclareAll ("test.caps", "Tests", BitRegistry.KindCapability,
                new[] { "a", null, "c" });
            var lines = BitRegistry.Lines ("test.caps");
            Check (lines.Count == 4 && lines [0] == "protocol=1\tset=test.caps\towner=Tests\tn=2" &&
                lines [1] == "bit\tname\tkind\tnote" && lines [2] == "0\ta\tcapability\t" &&
                lines [3] == "2\tc\tcapability\t", "bits_v1 lines");
            Check (BitRegistry.Describe ("test.caps", 5 | 8) == "a|c|bit3", "describe");
            var sets = BitRegistry.SetLines ();
            Check (sets.Count == 2 && sets [0] == "test.caps\tTests\t2", "set lines");
            threw = false;
            try {
                BitRegistry.Lines ("nope");
            } catch (ArgumentException) {
                threw = true;
            }
            Check (threw, "unknown set throws");
            BitRegistry.ResetForTests ();
        }

        // ------------------------------------------------------------------ layouts

        static void Layouts ()
        {
            LayoutRegistry.ResetForTests ();
            var l = LayoutRegistry.Declare ("test_frame", "Test", "Tests", 3, 2, LayoutRegistry.KindFrame, "a\tnote");
            l.Once ("header", "protocol", "schema", "row_count", "row_stride");
            l.Rows ("rows", "@row_count", "@row_stride", LayoutRegistry.Join ("flight_id",
                LayoutRegistry.Xyz ("force_", "_n"), "lever_m", LayoutRegistry.Xyz ("omega_rad_s_")));
            LayoutRegistry.Expect (l, "header", 4);
            LayoutRegistry.Expect (l, "rows", 8);
            Check (LayoutRegistry.Problems ().Count == 0, "declared sections fit their strides");
            LayoutRegistry.Expect (l, "rows", 9);
            Check (LayoutRegistry.Problems ().Count == 1 &&
                LayoutRegistry.Problems () [0] == "test_frame.rows: 8 fields declared, stride 9", "a mismatch is recorded");
            var lines = LayoutRegistry.Lines ("test_frame");
            Check (lines [0] == "protocol=1\tlayout=test_frame\tservice=Test\tlayout_protocol=3\tlayout_schema=2\t" +
                "kind=frame\tsections=2\tfields=12\tnote=a note", "layout_v1 header");
            Check (lines [1] == "#section\theader\t1\t4\t0\t" && lines [2] == "header\t0\tprotocol\t",
                "section line, first field");
            Check (lines.Contains ("rows\t1\tforce_x_n\tN") && lines.Contains ("rows\t4\tlever_m\tm") &&
                lines.Contains ("rows\t7\tomega_rad_s_z\trad/s"), "units inferred from the suffixes");
            var names = LayoutRegistry.FieldNames ("test_frame");
            Check (names.Count == 12 && names [4] == "flight_id" && names [11] == "omega_rad_s_z", "flat names");
            var h = LayoutRegistry.Declare ("test_history", "Test", "Tests", 1, 1, LayoutRegistry.KindHistory);
            h.Once ("header", "protocol", "schema", "frame_count", "oldest_tick", "newest_tick", "dropped_before");
            h.Frames ("frames", "@frame_count", "test_frame");
            var hl = LayoutRegistry.Lines ("test_history");
            Check (hl.Contains ("#section\tframes\t@frame_count\t@length\t1\ttest_frame"), "prefixed section");
            Check (LayoutRegistry.NameLines ().Count == 2 && LayoutRegistry.NameLines () [0] ==
                "test_frame\tTest\t3\t2\tframe\t12", "layouts_v1 rows, sorted");
            LayoutRegistry.Declare ("test_frame", "Test", "Tests", 3, 3, LayoutRegistry.KindFrame);
            Check (LayoutRegistry.Find ("test_frame").LayoutSchema == 3 && LayoutRegistry.Names ().Count == 2,
                "declaring again replaces");
            bool threw = false;
            try {
                LayoutRegistry.Lines ("nope");
            } catch (ArgumentException exc) {
                threw = exc.Message.Contains ("test_frame");
            }
            Check (threw, "unknown layout: the message lists the known ones");
            string[,] units = {
                { "final_thrust_kn", "kN" }, { "engine_torque_body_nm_x", "N.m" }, { "mass_tonnes", "t" },
                { "moi_body_kg_m2_y", "kg.m2" }, { "alpha_rad_s2_z", "rad/s2" }, { "surface_speed_ms", "m/s" },
                { "lateral_authority_ms2", "m/s2" }, { "mission_time_s", "s" }, { "cost_us", "us" },
                { "dynamic_pressure_pa", "Pa" }, { "static_pressure_kpa", "kPa" }, { "latitude_deg", "deg" },
                { "flight_id", "" }, { "situation", "" }, { "rotation_world_w", "" }, { "authority_x_nm", "N.m" }
            };
            for (int i = 0; i < units.GetLength (0); i++)
                Check (LayoutRegistry.InferUnit (units [i, 0]) == units [i, 1], "unit of " + units [i, 0]);
            LayoutRegistry.ResetForTests ();
        }

        // ------------------------------------------------------------------ errors

        static void Errors ()
        {
            Check (BridgeErrors.Tag (BridgeErrors.NotLoaded, "aucun vaisseau") == "[NOT_LOADED] aucun vaisseau",
                "tag format");
            Check (BridgeErrors.CodeOf ("[TOKEN_EXPIRED] token d'attitude") == BridgeErrors.TokenExpired &&
                BridgeErrors.CodeOf ("[NOPE] x") == "" && BridgeErrors.CodeOf ("plain") == "" &&
                BridgeErrors.CodeOf ("") == "", "code parse");
            Check (BridgeErrors.Codes.Length == 7, "seven codes");
            var memory = new TokenMemory (3);
            Check (memory.CodeOf ("a") == BridgeErrors.TokenInvalid && memory.CodeOf (null) == BridgeErrors.TokenInvalid,
                "unknown token: invalid");
            memory.Remember ("a", BridgeErrors.TokenInvalid);
            memory.Remember ("a", BridgeErrors.TokenExpired);
            Check (memory.CodeOf ("a") == BridgeErrors.TokenExpired, "a later reason replaces");
            memory.Remember ("b", BridgeErrors.TokenExpired);
            memory.Remember ("c", BridgeErrors.TokenInvalid);
            memory.Remember ("d", BridgeErrors.TokenExpired);
            Check (memory.CodeOf ("a") == BridgeErrors.TokenInvalid && memory.CodeOf ("d") == BridgeErrors.TokenExpired &&
                memory.CodeOf ("b") == BridgeErrors.TokenExpired, "bounded: the oldest is forgotten");
            memory.Remember (null, BridgeErrors.TokenExpired);
            Check (memory.CodeOf ("c") == BridgeErrors.TokenInvalid, "a null token is ignored");
        }

        // ------------------------------------------------------------------ event bus V2

        static Dictionary<string, string> Head (string row)
        {
            var d = new Dictionary<string, string> ();
            foreach (var cell in row.Split ('\t')) {
                int eq = cell.IndexOf ('=');
                if (eq > 0)
                    d [cell.Substring (0, eq)] = cell.Substring (eq + 1);
            }
            return d;
        }

        /// <summary>
        /// EventLogV2 (refactor lot 3): sequence numbers, the incremental read from a seq, the ring
        /// that overwrites the oldest and says how many a slow reader lost, maxRows and truncated, a
        /// reader ahead of the log (a restarted game), flattening, the header keys and columns in the
        /// declared order, no allocation of records once the ring has wrapped (same objects).
        /// </summary>
        static void EventLog ()
        {
            var inv = CultureInfo.InvariantCulture;
            var log = new EventLogV2 (4);
            Check (log.LastSeq == 0 && log.FirstSeq == 1 && log.Count == 0, "empty log");
            var empty = log.Lines (0, 0, 7, 12.5);
            var h = Head (empty [0]);
            Check (empty.Count == 2 && h ["n"] == "0" && h ["lost"] == "0" && h ["last_seq"] == "0" &&
                h ["tick"] == "7" && h ["ut"] == "12.5" && h ["capacity"] == "4", "empty table");
            var keys = new List<string> ();
            foreach (var cell in empty [0].Split ('\t'))
                keys.Add (cell.Substring (0, cell.IndexOf ('=')));
            Check (string.Join (",", keys) == string.Join (",", EventLogV2.HeaderKeys), "header keys in the declared order");
            Check (empty [1] == string.Join ("\t", EventLogV2.Columns), "column row");

            Check (log.Append ("a", 10, 1.0, 5u, "g", 0u, 0u, double.NaN, "x\ty") == 1, "first seq is 1");
            log.Append ("b", 11, 1.02, 0u, null, 6u, 7u, 1500.0, null);
            log.Append (null, 12, 1.04, 0u, "", 0u, 0u, 2.0, "line\nbreak");
            var all = log.Lines (0, 0, 12, 1.04);
            Check (all.Count == 5 && Head (all [0]) ["n"] == "3", "three rows");
            var r1 = all [2].Split ('\t');
            Check (r1.Length == 10 && r1 [0] == "1" && r1 [1] == "10" && r1 [3] == "a" && r1 [4] == "5" && r1 [5] == "g" &&
                r1 [8] == "NaN" && r1 [9] == "x y", "row 1: values, NaN, flattened detail");
            var r2 = all [3].Split ('\t');
            Check (r2 [6] == "6" && r2 [7] == "7" && r2 [8] == "1500" && r2 [5] == "" && r2 [9] == "", "row 2: pids, value, empty strings");
            var r3 = all [4].Split ('\t');
            Check (r3 [3] == "unknown" && r3 [9] == "line break" && double.Parse (r3 [2], inv) == 1.04, "row 3: null kind, ut round trip");

            var since2 = log.Lines (2, 0, 12, 1.04);
            Check (since2.Count == 3 && since2 [2].StartsWith ("3\t", StringComparison.Ordinal), "from a seq");
            var cut = log.Lines (0, 2, 12, 1.04);
            Check (cut.Count == 4 && Head (cut [0]) ["truncated"] == "1" && Head (cut [0]) ["n"] == "2", "maxRows truncates");
            var ahead = log.Lines (9, 0, 12, 1.04);
            Check (ahead.Count == 2 && Head (ahead [0]) ["lost"] == "0" && Head (ahead [0]) ["since"] == "9", "a reader ahead: nothing, nothing lost");
            var negative = log.Lines (-5, 0, 12, 1.04);
            Check (Head (negative [0]) ["since"] == "0" && negative.Count == 5, "a negative since reads everything");

            // Wrap: 4 slots, 7 records -> seq 4..7 kept, 1..3 overwritten.
            var slot = log.Find (1);
            for (int i = 0; i < 4; i++)
                log.Append ("w" + i.ToString (inv), 20 + i, 2.0 + i, 0u, "", 0u, 0u, i, "");
            Check (log.FirstSeq == 4 && log.LastSeq == 7 && log.Count == 4, "ring keeps the newest");
            Check (log.Find (3) == null && log.Find (4) != null && log.Find (8) == null, "find inside the ring only");
            Check (ReferenceEquals (log.Find (5), slot), "records are reused, never reallocated");
            var slow = Head (log.Lines (1, 0, 30, 3.0) [0]);
            Check (slow ["lost"] == "2" && slow ["n"] == "4" && slow ["first_seq"] == "4", "a slow reader is told what it lost");
            var fromZero = Head (log.Lines (0, 0, 30, 3.0) [0]);
            Check (fromZero ["lost"] == "3", "from 0 after a wrap: three lost");
            var current = Head (log.Lines (5, 0, 30, 3.0) [0]);
            Check (current ["lost"] == "0" && current ["n"] == "2", "a reader inside the ring loses nothing");

            log.Clear ();
            Check (log.Count == 0 && log.LastSeq == 7 && log.FirstSeq == 8, "clear keeps counting");
            Check (log.Append ("after", 40, 4.0, 0u, "", 0u, 0u, 0, "") == 8, "seq never repeats");
            Check (Head (log.Lines (2, 0, 40, 4.0) [0]) ["lost"] == "5", "records cleared count as lost");

            bool threw = false;
            try {
                new EventLogV2 (0);
            } catch (ArgumentOutOfRangeException) {
                threw = true;
            }
            Check (threw, "capacity 0 refused");
            Check (EventLogV2.Capacity == 1024 && EventLogV2.Columns.Length == 10, "constants");
        }

        // ------------------------------------------------------------------ frames

        static void FramesMaths ()
        {
            var x = new double[] { 1, 0, 0 };
            var y = new double[] { 0, 1, 0 };
            var z = new double[3];
            Frames.Cross (x, y, z);
            Check (z [0] == 0 && z [1] == 0 && z [2] == 1, "x cross y");
            double c = Math.Cos (0.3), s = Math.Sin (0.3);
            // A rotation about z: images of the world basis in body.
            var wx = new double[] { c, -s, 0 };
            var wy = new double[] { s, c, 0 };
            var wz = new double[] { 0, 0, 1 };
            var v = new double[] { 1.5, -2.0, 0.25 };
            var body = new double[3];
            var back = new double[3];
            Frames.WorldToBody (wx, wy, wz, v, body);
            Frames.BodyToWorld (wx, wy, wz, body, back);
            Check (Math.Abs (back [0] - v [0]) < 1e-12 && Math.Abs (back [1] - v [1]) < 1e-12 &&
                Math.Abs (back [2] - v [2]) < 1e-12, "world -> body -> world");
            Check (Frames.Finite3 (v) && !Frames.Finite3 (new[] { 0, double.NaN, 0 }), "finite3");
            Check (Frames.KilonewtonToNewton == 1000.0 && Frames.TonneToKilogram == 1000.0, "unit constants");
        }
    }
}
