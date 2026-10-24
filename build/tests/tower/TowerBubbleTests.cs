// -----------------------------------------------------------------------------------
// conn.tower_site.tower_bubble_v1: pure tests of TowerBubbleWatch, the watcher that loads a
// catch tower first (packed) and unpacks it only once its load is finished, and of its layout
// declaration. See TowerTests.csproj; run by Program.Main.
//
// The case of (KSP.log of a test install, 13/10/2026 19:08:19): "[Tour Catch Sud]: landed - waiting
// for ground contact to resume physics..." at .248, "Unpacking Tour Catch Sud" at .279, anchored at
// .305: loaded and unpacked within one or two frames, before Part.Start had built its joints.
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;

namespace KRPC.Bridge.Tower
{
    public static partial class Program
    {
        const double BubbleR = 100000.0;
        // A physics step of KSP (s).
        const double Step02 = 0.02;

        static void TowerBubbleTests ()
        {
            TestBubbleInvalidRange ();
            TestBubbleLoadsFirstThenWaits ();
            TestBubbleNeedsEveryPartAndServo ();
            TestBubbleMoveResetsStillness ();
            TestBubbleFloatNoiseIsStill ();
            TestBubbleSamePhysicsStepCountsOnce ();
            TestBubbleUnpackAfterReady ();
            TestBubbleEarlyUnpackIsTheD227Case ();
            TestBubbleAlreadyUnpackedWhenPosed ();
            TestBubbleRepackAndUnload ();
            TestBubbleLostAndReleased ();
            TestBubbleRaise ();
            TestBubbleStatus ();
            TestBubbleLayout ();
        }

        // ------------------------------------------------------------------ helpers

        /// <summary>A tower of n rigid bodies, every one started and still, at that world scale.</summary>
        static TowerBubbleWatch.Observation Tower (bool loaded, bool packed, int parts = 40, int started = 40,
                                                   int servos = 2, int ready = 2, double[] pos = null,
                                                   double scale = 3300.0)
        {
            if (pos == null)
                pos = Shape (parts);
            return new TowerBubbleWatch.Observation {
                Exists = true, Loaded = loaded, Packed = packed, Parts = parts, PartsStarted = started,
                Servos = servos, ServosReady = ready, Positions = pos, Count = pos.Length, Scale = scale,
                Distance = 3243.0
            };
        }

        /// <summary>Root-relative positions of n bodies: a mast 40 m high, arms out to 17 m.</summary>
        static double[] Shape (int n)
        {
            var p = new double [3 * n];
            for (int i = 0; i < n; i++) {
                p [3 * i] = (i % 5) * 4.0;
                p [3 * i + 1] = i * 1.0;
                p [3 * i + 2] = (i % 3) * 2.0;
            }
            return p;
        }

        static double[] Moved (double[] p, int body, double dy)
        {
            var q = (double[]) p.Clone ();
            q [3 * body + 1] += dy;
            return q;
        }

        // ------------------------------------------------------------------ tests

        static void TestBubbleInvalidRange ()
        {
            foreach (var r in new[] { double.NaN, double.PositiveInfinity, 0.0, -1.0, 1e40 })
                Check (Throws (() => TowerBubbleWatch.Pose (r, 0.0)), "range " + r + " refused");
            Check (TowerBubbleWatch.Invalid (8000.0) == null, "8 km accepted");
            var w = TowerBubbleWatch.Pose (BubbleR, 10.0);
            Check (Throws (() => w.Raise (double.NaN)), "raise NaN refused");
        }

        static void TestBubbleLoadsFirstThenWaits ()
        {
            var w = TowerBubbleWatch.Pose (BubbleR, 100.0);
            Equal (TowerBubbleWatch.StateLoading, w.State, "posed: loading");
            Equal (TowerBubbleWatch.Ranges.LoadOnly, w.Wanted, "posed: load and unload only");
            double t = 0.0, ut = 100.0;
            Equal (TowerBubbleWatch.Event.None, w.Step (ut, t, Tower (false, true)), "not loaded yet: nothing");
            Equal (TowerBubbleWatch.StateLoading, w.State, "still loading");
            t += Step02; ut += Step02;
            Equal (TowerBubbleWatch.Event.Loaded, w.Step (ut, t, Tower (true, true)), "loaded, packed: Loaded");
            Equal (TowerBubbleWatch.StateSettling, w.State, "settling");
            Equal (TowerBubbleWatch.Ranges.LoadOnly, w.Wanted, "settling: still load and unload only");
            Near (0.02, w.LoadedAt - w.PosedAt, 1e-9, "loaded 0.02 s after the pose");
            Equal (0, w.StillTicks, "first look: a reference, not a still step");
            // Every part started and still: Ready after SettleTicks still steps, not before.
            for (int k = 1; k < TowerBubbleWatch.SettleTicks; k++) {
                t += Step02; ut += Step02;
                Equal (TowerBubbleWatch.Event.None, w.Step (ut, t, Tower (true, true)), "still step " + k + ": not yet");
                Equal (k, w.StillTicks, "still steps " + k);
            }
            t += Step02; ut += Step02;
            Equal (TowerBubbleWatch.Event.Ready, w.Step (ut, t, Tower (true, true)), "Ready at the third still step");
            Equal (TowerBubbleWatch.StateUnpacking, w.State, "unpacking");
            Equal (TowerBubbleWatch.Ranges.Full, w.Wanted, "now pack and unpack too");
            Near (0.08, w.ReadyAt - w.PosedAt, 1e-9, "ready 0.08 s after the pose (4 steps)");
            Near (0.0, w.Shift, 0.0, "nothing moved");
            Equal (1, w.Loads, "one load");
            Check (!w.EarlyUnpack, "no early unpack");
        }

        static void TestBubbleNeedsEveryPartAndServo ()
        {
            var w = TowerBubbleWatch.Pose (BubbleR, 0.0);
            double t = 0.0;
            w.Step (0.0, t, Tower (true, true, started: 0));
            // Part.Start over three frames: no part started, then some, then all but the servos.
            for (int k = 0; k < 6; k++) {
                t += Step02;
                w.Step (t, t, Tower (true, true, started: 39));
                Equal (0, w.StillTicks, "a part not started: never still (" + k + ")");
            }
            for (int k = 0; k < 6; k++) {
                t += Step02;
                w.Step (t, t, Tower (true, true, ready: 1));
                Equal (0, w.StillTicks, "a servo not ready: never still (" + k + ")");
            }
            Equal (TowerBubbleWatch.StateSettling, w.State, "still settling");
            t += Step02;
            w.Step (t, t, Tower (true, true));
            Equal (1, w.StillTicks, "everything ready: counting");
            // An empty tower (no part yet) is never ready.
            var e = TowerBubbleWatch.Pose (BubbleR, 0.0);
            for (int k = 0; k < 8; k++)
                e.Step (k * Step02, k * Step02, Tower (true, true, parts: 0, started: 0, servos: 0, ready: 0,
                                                       pos: new double [0]));
            Equal (TowerBubbleWatch.StateSettling, e.State, "no part: never ready");
        }

        static void TestBubbleMoveResetsStillness ()
        {
            var w = TowerBubbleWatch.Pose (BubbleR, 0.0);
            var p = Shape (40);
            double t = 0.0;
            w.Step (t, t, Tower (true, true, pos: p));
            t += Step02;
            w.Step (t, t, Tower (true, true, pos: p));
            t += Step02;
            w.Step (t, t, Tower (true, true, pos: p));
            Equal (2, w.StillTicks, "two still steps");
            // An arm moves 2 mm (a joint made, a coroutine's ResetJoints): back to zero.
            var q = Moved (p, 30, -0.002);
            t += Step02;
            w.Step (t, t, Tower (true, true, pos: q));
            Equal (0, w.StillTicks, "a move above the tolerance resets the count");
            Near (0.002, w.Shift, 1e-12, "the shift measured");
            Near (0.002, w.SettleShiftMax, 1e-12, "the largest shift kept");
            for (int k = 0; k < TowerBubbleWatch.SettleTicks - 1; k++) {
                t += Step02;
                w.Step (t, t, Tower (true, true, pos: q));
            }
            Equal (TowerBubbleWatch.StateSettling, w.State, "not ready before three still steps after the move");
            t += Step02;
            Equal (TowerBubbleWatch.Event.Ready, w.Step (t, t, Tower (true, true, pos: q)), "ready after them");
            // A body list that changes length (a part died) is no comparison: not still.
            var c = TowerBubbleWatch.Pose (BubbleR, 0.0);
            c.Step (0.0, 0.0, Tower (true, true, parts: 40, started: 40));
            c.Step (Step02, Step02, Tower (true, true, parts: 39, started: 39));
            Check (double.IsNaN (c.Shift), "different body counts: no shift");
            Equal (0, c.StillTicks, "different body counts: not still");
        }

        static void TestBubbleFloatNoiseIsStill ()
        {
            // The tolerance: 4 ulp of the scale, from float32 world positions.
            Near (4.0 * 3300.0 / 8388608.0, TowerBubbleWatch.ToleranceFor (3300.0), 1e-15, "tolerance at 3.3 km");
            Check (TowerBubbleWatch.ToleranceFor (3300.0) < 0.0016, "under 1.6 mm at 3.3 km");
            Check (double.IsNaN (TowerBubbleWatch.ToleranceFor (double.NaN)), "no scale: no tolerance");
            // A still tower 3.2 km from the origin, its world positions rounded to float32 at every step,
            // the origin shifted between steps (floating origin): the root-relative positions, differences
            // of float32 world coordinates, never move by more than the tolerance.
            var rnd = new Random (227);
            var shape = Shape (40);
            double worst = 0.0, tolMin = double.MaxValue, scaleBefore = double.NaN;
            double[] before = null;
            for (int s = 0; s < 400; s++) {
                double ox = 3243.0 + rnd.NextDouble () * 6000.0 - 3000.0;
                double oy = -1200.0 + rnd.NextDouble () * 600.0, oz = 800.0 + rnd.NextDouble () * 400.0;
                float rx = (float) ox, ry = (float) oy, rz = (float) oz;
                var rel = new double [shape.Length];
                double maxCoord = Math.Max (Math.Abs (ox), Math.Max (Math.Abs (oy), Math.Abs (oz))), extent = 0.0;
                for (int i = 0; i < shape.Length; i += 3) {
                    float wx = (float) (ox + shape [i]), wy = (float) (oy + shape [i + 1]), wz = (float) (oz + shape [i + 2]);
                    rel [i] = (double) wx - rx;
                    rel [i + 1] = (double) wy - ry;
                    rel [i + 2] = (double) wz - rz;
                    maxCoord = Math.Max (maxCoord, Math.Max (Math.Abs (wx), Math.Max (Math.Abs (wy), Math.Abs (wz))));
                    extent = Math.Max (extent, Math.Sqrt (shape [i] * shape [i] + shape [i + 1] * shape [i + 1]
                                                          + shape [i + 2] * shape [i + 2]));
                }
                double scale = maxCoord + extent;
                if (before != null) {
                    // the scale of both looks, as the watcher takes it
                    double tol = TowerBubbleWatch.ToleranceFor (Math.Max (scale, scaleBefore));
                    tolMin = Math.Min (tolMin, tol);
                    double sh = TowerBubbleWatch.MaxShift (rel, rel.Length, before, before.Length);
                    worst = Math.Max (worst, sh / tol);
                }
                before = rel;
                scaleBefore = scale;
            }
            Check (worst <= 1.0, "float32 noise under the tolerance (worst " + worst.ToString ("F3") + " of it)");
            Check (worst > 0.0, "the noise is there (it is measured, not zero)");
            // The sag of (22 cm at the rails) is two orders of magnitude above it.
            Check (0.22 > 100.0 * tolMin, "the sag is far above the tolerance");
            Check (double.IsNaN (TowerBubbleWatch.MaxShift (new double [3], 3, new double [6], 6)), "counts differ: NaN");
            Check (double.IsNaN (TowerBubbleWatch.MaxShift (null, 0, new double [0], 0)), "no list: NaN");
        }

        static void TestBubbleSamePhysicsStepCountsOnce ()
        {
            // The pose RPC and TowerAddon.FixedUpdate look in the same physics step: one look only.
            var w = TowerBubbleWatch.Pose (BubbleR, 0.0);
            w.Step (0.0, 1.00, Tower (true, true));
            for (int k = 0; k < 10; k++)
                Equal (TowerBubbleWatch.Event.None, w.Step (0.0, 1.00, Tower (true, true)), "same step " + k + ": nothing");
            Equal (0, w.StillTicks, "no still step counted within one physics step");
            w.Step (0.02, 1.02, Tower (true, true));
            Equal (1, w.StillTicks, "the next physics step counts");
            Equal (TowerBubbleWatch.Event.None, w.Step (0.02, 1.01, Tower (true, false)), "an older step: ignored");
            Equal (TowerBubbleWatch.StateSettling, w.State, "nothing changed");
        }

        static TowerBubbleWatch Ready (out double t)
        {
            var w = TowerBubbleWatch.Pose (BubbleR, 0.0);
            t = 0.0;
            w.Step (t, t, Tower (true, true));
            while (w.State == TowerBubbleWatch.StateSettling) {
                t += Step02;
                w.Step (t, t, Tower (true, true));
            }
            return w;
        }

        static void TestBubbleUnpackAfterReady ()
        {
            double t;
            var w = Ready (out t);
            Equal (TowerBubbleWatch.StateUnpacking, w.State, "ready");
            t += Step02;
            Equal (TowerBubbleWatch.Event.None, w.Step (t, t, Tower (true, true)), "KSP has not unpacked it yet");
            t += Step02;
            Equal (TowerBubbleWatch.Event.Unpacked, w.Step (t, t, Tower (true, false)), "KSP unpacked it: Unpacked");
            Equal (TowerBubbleWatch.StateUnpacked, w.State, "unpacked");
            Equal (TowerBubbleWatch.Ranges.Full, w.Wanted, "held unpacked (pack = R)");
            Equal (1, w.Unpacks, "one unpack");
            Check (!w.EarlyUnpack, "after ready: not early");
            t += Step02;
            Equal (TowerBubbleWatch.Event.None, w.Step (t, t, Tower (true, false)), "unpacked: nothing more");
        }

        static void TestBubbleEarlyUnpackIsTheD227Case ()
        {
            // physics_range's case: loaded and unpacked together, the parts not started. The watcher would
            // never ask for it; seen anyway (KSP's own unpack range, another writer), it is said.
            var w = TowerBubbleWatch.Pose (BubbleR, 0.0);
            w.Step (0.0, 0.0, Tower (false, true));
            Equal (TowerBubbleWatch.Event.EarlyUnpack, w.Step (0.02, 0.02, Tower (true, false, started: 0, ready: 0)),
                   "loaded and unpacked at once: EarlyUnpack");
            Check (w.EarlyUnpack, "flagged");
            Equal (TowerBubbleWatch.StateUnpacked, w.State, "held unpacked anyway");
            // Unpacked during the watch of its load (not ready yet): early too.
            var s = TowerBubbleWatch.Pose (BubbleR, 0.0);
            s.Step (0.0, 0.0, Tower (true, true, started: 10));
            Equal (TowerBubbleWatch.Event.EarlyUnpack, s.Step (0.02, 0.02, Tower (true, false, started: 20)),
                   "unpacked while settling: EarlyUnpack");
        }

        static void TestBubbleAlreadyUnpackedWhenPosed ()
        {
            // Posed on a tower KSP had already unpacked (its own ranges load first): held, not early.
            var w = TowerBubbleWatch.Pose (BubbleR, 0.0);
            Equal (TowerBubbleWatch.Event.AlreadyUnpacked, w.Step (0.0, 0.0, Tower (true, false)), "already unpacked");
            Check (!w.EarlyUnpack, "not early");
            Equal (TowerBubbleWatch.Ranges.Full, w.Wanted, "held unpacked");
        }

        static void TestBubbleRepackAndUnload ()
        {
            double t;
            var w = Ready (out t);
            t += Step02;
            w.Step (t, t, Tower (true, false));
            // An active vessel switch: KSP packs every vessel, then unpacks it again.
            t += Step02;
            Equal (TowerBubbleWatch.Event.Repacked, w.Step (t, t, Tower (true, true)), "packed while loaded: Repacked");
            Equal (TowerBubbleWatch.StateUnpacking, w.State, "waits for KSP");
            Equal (TowerBubbleWatch.Ranges.Full, w.Wanted, "its parts stay built: the full ranges stay");
            t += Step02;
            Equal (TowerBubbleWatch.Event.Unpacked, w.Step (t, t, Tower (true, false)), "unpacked again");
            Equal (2, w.Unpacks, "two unpacks");
            Check (!w.EarlyUnpack, "not early");
            // Unloaded (beyond R): back to loading, so the next load is watched again.
            t += Step02;
            Equal (TowerBubbleWatch.Event.Unloaded, w.Step (t, t, Tower (false, true)), "unloaded");
            Equal (TowerBubbleWatch.StateLoading, w.State, "loading again");
            Equal (TowerBubbleWatch.Ranges.LoadOnly, w.Wanted, "pack and unpack back to the original");
            t += Step02;
            Equal (TowerBubbleWatch.Event.Loaded, w.Step (t, t, Tower (true, true)), "loaded again");
            Equal (2, w.Loads, "two loads");
            Equal (0, w.StillTicks, "the new load is watched from zero");
        }

        static void TestBubbleLostAndReleased ()
        {
            var w = TowerBubbleWatch.Pose (BubbleR, 5.0);
            w.Step (5.0, 0.0, Tower (true, true));
            var gone = new TowerBubbleWatch.Observation { Exists = false, Distance = double.NaN };
            Equal (TowerBubbleWatch.Event.Lost, w.Step (6.0, 0.02, gone), "gone: Lost");
            Equal (TowerBubbleWatch.StateLost, w.State, "lost");
            Equal (TowerBubbleWatch.Ranges.None, w.Wanted, "lost: nothing to write");
            Check (!w.Active, "lost: not held");
            Equal (TowerBubbleWatch.Event.None, w.Step (7.0, 0.04, Tower (true, true)), "lost is final");
            Near (1.0, w.Elapsed (100.0), 1e-12, "elapsed frozen at the loss");
            var r = TowerBubbleWatch.Pose (BubbleR, 0.0);
            r.Step (0.0, 0.0, Tower (true, false));
            r.Release (3.0);
            Equal (TowerBubbleWatch.StateReleased, r.State, "released");
            Equal (TowerBubbleWatch.Ranges.Original, r.Wanted, "released: the original ranges");
            Equal (TowerBubbleWatch.Event.None, r.Step (4.0, 0.02, Tower (false, true)), "released is final");
            r.Release (9.0);
            Near (3.0, r.Elapsed (50.0), 1e-12, "a second release changes nothing");
        }

        static void TestBubbleRaise ()
        {
            var w = TowerBubbleWatch.Pose (8000.0, 0.0);
            Check (w.Raise (100000.0), "a larger range is kept");
            Near (100000.0, w.Range, 0.0, "R raised");
            Check (!w.Raise (50000.0), "a smaller one is not");
            Near (100000.0, w.Range, 0.0, "R kept");
            w.Release (1.0);
            Check (!w.Raise (200000.0), "released: no raise");
        }

        static void TestBubbleStatus ()
        {
            Equal (TowerBubbleWatch.StatusLength, TowerBubbleWatch.Fields.Length, "fields and length");
            double t;
            var w = Ready (out t);
            t += Step02;
            w.Step (t, t, Tower (true, false));
            var s = w.Status (t + 1.0, 1.0, 0.0, 3243.0);
            Equal (TowerBubbleWatch.StatusLength, s.Count, "status length");
            var names = TowerBubbleWatch.Fields;
            Func<string, double> f = n => s [Array.IndexOf (names, n)];
            Near (1, f ("protocol"), 0, "protocol");
            Near (TowerBubbleWatch.StateUnpacked, f ("state"), 0, "state");
            Near (BubbleR, f ("range_m"), 0, "range");
            Near (t + 1.0, f ("elapsed_s"), 1e-9, "elapsed");
            Near (1, f ("loaded"), 0, "loaded (Unity side)");
            Near (0, f ("packed"), 0, "packed (Unity side)");
            Near (40, f ("parts"), 0, "parts");
            Near (40, f ("parts_started"), 0, "parts started");
            Near (2, f ("servos"), 0, "servos");
            Near (2, f ("servos_ready"), 0, "servos ready");
            Near (TowerBubbleWatch.SettleTicks, f ("settle_ticks"), 0, "settle ticks");
            Near (0.0, f ("loaded_after_s"), 1e-12, "loaded at the pose");
            Near (0.06, f ("ready_after_s"), 1e-9, "ready after 3 steps");
            Near (0.08, f ("unpacked_after_s"), 1e-9, "unpacked after 4");
            Near (0, f ("early_unpack"), 0, "not early");
            Near (0, f ("overwritten"), 0, "not overwritten");
            Near (1, f ("loads"), 0, "loads");
            Near (1, f ("unpacks"), 0, "unpacks");
            Near (3243.0, f ("distance_m"), 0, "distance (Unity side)");
            w.MarkOverwritten ();
            Near (1, w.Status (t, 1, 0, 0) [Array.IndexOf (names, "overwritten")], 0, "overwritten flagged");
            var none = TowerBubbleWatch.NoneStatus (0.0, double.NaN, 9000.0);
            Equal (TowerBubbleWatch.StatusLength, none.Count, "none: length");
            Near (TowerBubbleWatch.StateNone, none [1], 0, "none: state");
            Check (double.IsNaN (none [2]) && double.IsNaN (none [6]), "none: NaN range and counts");
            Near (9000.0, none [22], 0, "none: distance");
        }

        static void TestBubbleLayout ()
        {
            string said = TowerLayoutDeclaration.DeclareBubbleInto (typeof (LayoutRegistry));
            Check (said.StartsWith ("layout tower_bubble_v1 declared", StringComparison.Ordinal), "declared: " + said);
            Equal (0, LayoutRegistry.Problems ().Count, "the descriptor fits StatusLength");
            var lines = LayoutRegistry.Lines (TowerLayoutDeclaration.BubbleLayout);
            var head = Header (lines [0]);
            Equal ("tower_bubble_v1", head ["layout"], "layout");
            Equal ("TowerSite", head ["service"], "service");
            Equal ("1", head ["layout_protocol"], "protocol");
            Equal ("1", head ["layout_schema"], "schema");
            Equal (LayoutRegistry.KindStatus, head ["kind"], "kind");
            Equal (TowerBubbleWatch.StatusLength.ToString (), head ["fields"], "fields");
            Equal ("#section\tstatus\t1\t" + TowerBubbleWatch.StatusLength + "\t0\t", lines [1], "one section, read once");
            var units = new Dictionary<string, string> ();
            for (int i = 0; i < TowerBubbleWatch.Fields.Length; i++) {
                var cells = lines [2 + i].Split ('\t');
                Equal (TowerBubbleWatch.Fields [i], cells [2], "field " + i);
                units [cells [2]] = cells [3];
            }
            Equal ("m", units ["range_m"], "unit of range_m");
            Equal ("s", units ["elapsed_s"], "unit of elapsed_s");
            Equal ("m", units ["tolerance_m"], "unit of tolerance_m");
            Equal ("", units ["parts"], "parts: a count");
        }
    }
}
