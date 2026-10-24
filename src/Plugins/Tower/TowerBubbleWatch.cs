using System;
using System.Collections.Generic;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// The pure part of conn.tower_site.tower_bubble_v1: the bubble of one catch tower. The tower
    /// is LOADED first, out to a range R, and stays packed (on rails); it is UNPACKED only once its load
    /// has finished. One watcher per tower, fed one observation per physics step; it says which ranges
    /// the tower must have and when to switch them.
    ///
    /// WHY (ground test of 13/10/2026 on a test install, without KJR, 22 unpacks out of 22). A tower
    /// LOADED AND UNPACKED AT THE SAME MOMENT sags: its rails 0.22 m lower over its foot than in a good
    /// tower (38.59 m against 38.81 m), 2.5 cm further from the mast, its locked hinges scattered by
    /// +-1.4 deg, its arms swinging (up to 0.31 m sideways in 3 s). A tower loaded earlier (packed, on
    /// rails) and unpacked later is good, 2.4 km or 600 m from the active vessel. kRPC 0.6.0's
    /// Vessel.physics_range ties the two (VesselPhysicsRangeAddon.Apply, decompiled: load = unpack =
    /// 0.9 R, pack = unload = R): Vessel.Update loads a vessel nearer than its load range and, in the
    /// same frame, OrbitPhysicsManager.LateUpdate unpacks it when it is nearer than its unpack range.
    ///
    /// THE MECHANISM (KSP 1.12.5, decompiled). Vessel.Load instantiates the parts and only logs
    /// "landed - waiting for ground contact to resume physics..."; Vessel.GoOffRails unpacks a loaded
    /// vessel once its physics hold is over (75 frames after Vessel.Awake, i.e. after the scene start
    /// for a tower of the scene) and, landed, once Vessel.CheckGroundCollision has found the ground
    /// (skipped for a tower saved with the body's PQS levels) - it never asks whether the parts have
    /// started. A part builds itself over three frames (Part.Start, a coroutine): its modules' OnStart
    /// (a servo creates its moving rigid body and its joint, not yet connected), then its rigid body
    /// (isKinematic = packed), then ModulesBeforePartAttachJoint (the servo joint connected to the
    /// part, its lock angle read from the moving part's current angle) and CreateAttachJoint, then
    /// Part.started = true. Unpacked before that, every part's body is already dynamic when it is
    /// created, and the FlightIntegrator pulls it down (it integrates every rigid body of an unpacked
    /// vessel from the root, joint or no joint) before its joints exist. Loaded first, every joint is
    /// made between kinematic bodies at the saved pose, and the unpack frees a finished vessel.
    ///
    /// THE RULE. Loading: load = 0.9 R and unload = R, pack and unpack the vessel's own. Ready to unpack
    /// when the tower is loaded and packed, every part has started (Part.started: rigid body and attach
    /// joint made, servo joints connected), every robotic servo has finished its init
    /// (BaseServo.ServoInitComplete), and the pose of every rigid body against the root has not moved,
    /// within the float32 resolution of its world position (Tolerance), over SettleTicks consecutive
    /// physics steps with all of that true. Then pack = R and unpack = 0.9 R as well, and KSP unpacks it
    /// on its own path (GoOffRails, ground check included). No delay of its own: SettleTicks is the
    /// longest joint work KSP still defers once a part has started.
    ///
    /// Never shorter than the vessel's own ranges (each range is the larger of the bubble's and the
    /// original); a second pose keeps the larger R. The released tower gets its original ranges back.
    ///
    /// PURE: System only (build/tests/tower). The Unity side (TowerBubbles) observes the vessel, writes
    /// the ranges and logs.
    /// </summary>
    public sealed class TowerBubbleWatch
    {
        /// <summary>Protocol of the tower_bubble_v1 status (a field removed or moved raises it).</summary>
        public const int Protocol = 1;

        /// <summary>Schema of the status (a field appended raises it).</summary>
        public const int Schema = 1;

        /// <summary>No bubble was ever posed on this tower in this scene.</summary>
        public const int StateNone = 0;
        /// <summary>Loading ranges written; the tower is not loaded yet.</summary>
        public const int StateLoading = 1;
        /// <summary>Loaded and packed; waiting for its load to finish (parts, servos, a still pose).</summary>
        public const int StateSettling = 2;
        /// <summary>Ready: the unpack ranges written; waiting for KSP to unpack it.</summary>
        public const int StateUnpacking = 3;
        /// <summary>Loaded and unpacked, the bubble held (pack = R).</summary>
        public const int StateUnpacked = 4;
        /// <summary>Released: the original ranges written back.</summary>
        public const int StateReleased = 5;
        /// <summary>The vessel is gone (destroyed, recovered, or no longer in FlightGlobals.Vessels).</summary>
        public const int StateLost = 6;

        /// <summary>
        /// The ratio of the load and unpack ranges to R: kRPC 0.6.0's (VesselPhysicsRangeAddon.hysteresis,
        /// the stock 2250/2500), so that a client computes R as for physics_range.
        /// </summary>
        public const float Hysteresis = 0.9f;

        /// <summary>
        /// Consecutive physics steps of a finished and still tower before its unpack. KSP still defers
        /// joint work once a part has started (decompiled): Part.SetJoints waits three WaitForFixedUpdate
        /// before ResetJoints, and the autostruts of Part.UpdateAutoStrut are secured one WaitForFixedUpdate
        /// later (Part.SecureAutoStruts). Three steps, all with every part started, cover both.
        /// </summary>
        public const int SettleTicks = 3;

        /// <summary>The relative spacing of float32 numbers, 2^-23: ulp(x) &lt;= |x| x Float32Epsilon.</summary>
        public const double Float32Epsilon = 1.0 / 8388608.0;

        /// <summary>The status fields, in wire order (layout tower_bubble_v1, section "status").</summary>
        public static readonly string[] Fields = {
            "protocol", "state", "range_m", "elapsed_s", "loaded", "packed", "parts", "parts_started",
            "servos", "servos_ready", "still_ticks", "settle_ticks", "shift_m", "tolerance_m",
            "settle_shift_max_m", "loaded_after_s", "ready_after_s", "unpacked_after_s", "early_unpack",
            "overwritten", "loads", "unpacks", "distance_m"
        };

        /// <summary>Length of the status (Fields.Length).</summary>
        public const int StatusLength = 23;

        /// <summary>The ranges the tower must have now.</summary>
        public enum Ranges
        {
            /// <summary>load = 0.9 R, unload = R; pack and unpack the original.</summary>
            LoadOnly,
            /// <summary>load = unpack = 0.9 R, unload = pack = R (as kRPC's physics_range).</summary>
            Full,
            /// <summary>The original ranges (released).</summary>
            Original,
            /// <summary>Nothing to write (lost).</summary>
            None
        }

        /// <summary>What happened at a step (for the log).</summary>
        public enum Event
        {
            None,
            /// <summary>The tower is loaded (packed): its load is being watched.</summary>
            Loaded,
            /// <summary>The load is finished: the unpack ranges are written.</summary>
            Ready,
            /// <summary>KSP unpacked the tower after it was ready.</summary>
            Unpacked,
            /// <summary>The tower was unpacked when the bubble was posed (KSP's own ranges did it).</summary>
            AlreadyUnpacked,
            /// <summary>The tower got unpacked BEFORE it was ready (someone else's ranges, or KSP's own).</summary>
            EarlyUnpack,
            /// <summary>The tower was unloaded: back to the loading ranges.</summary>
            Unloaded,
            /// <summary>The tower was packed while loaded (KSP packs every vessel at an active vessel switch).</summary>
            Repacked,
            /// <summary>The vessel is gone.</summary>
            Lost
        }

        /// <summary>One look at the tower, at one physics step (from the Unity side).</summary>
        public struct Observation
        {
            /// <summary>The vessel still exists.</summary>
            public bool Exists;
            public bool Loaded;
            public bool Packed;
            /// <summary>Parts of the vessel, and how many have Part.started.</summary>
            public int Parts;
            public int PartsStarted;
            /// <summary>Robotic servo modules (BaseServo), and how many have ServoInitComplete.</summary>
            public int Servos;
            public int ServosReady;
            /// <summary>Root-relative positions of the rigid bodies (x, y, z each, m), the first Count values.</summary>
            public double[] Positions;
            public int Count;
            /// <summary>The largest absolute world coordinate of those bodies plus the tower's extent (m).</summary>
            public double Scale;
            /// <summary>Distance to the active vessel (m), NaN when there is none.</summary>
            public double Distance;
        }

        public double Range { get; private set; }
        public readonly double PosedAt;

        public int State { get; private set; }
        public double EndedAt { get; private set; }
        public double LoadedAt { get; private set; }
        public double ReadyAt { get; private set; }
        public double UnpackedAt { get; private set; }
        public bool EarlyUnpack { get; private set; }
        public bool Overwritten { get; private set; }
        public int Loads { get; private set; }
        public int Unpacks { get; private set; }
        public int StillTicks { get; private set; }
        public double Shift { get; private set; }
        public double Tolerance { get; private set; }
        public double SettleShiftMax { get; private set; }
        public Observation Last { get; private set; }
        public bool Observed { get; private set; }

        double[] previous = new double[0];
        int previousCount = -1;
        double previousScale = double.NaN;
        double lastStepTime = double.NaN;

        TowerBubbleWatch (double range, double now)
        {
            Range = range;
            PosedAt = now;
            State = StateLoading;
            EndedAt = double.NaN;
            LoadedAt = double.NaN;
            ReadyAt = double.NaN;
            UnpackedAt = double.NaN;
            Shift = double.NaN;
            Tolerance = double.NaN;
            SettleShiftMax = double.NaN;
        }

        /// <summary>Why this range is refused, or null: finite and strictly positive (m).</summary>
        public static string Invalid (double range)
        {
            if (double.IsNaN (range) || double.IsInfinity (range) || range <= 0.0)
                return "range_m must be finite and > 0";
            if (range > float.MaxValue)
                return "range_m must fit a float";
            return null;
        }

        /// <summary>A bubble posed at <paramref name="now"/> (game time, s) with the range R.</summary>
        public static TowerBubbleWatch Pose (double range, double now)
        {
            var why = Invalid (range);
            if (why != null)
                throw new ArgumentException (why);
            return new TowerBubbleWatch (range, now);
        }

        /// <summary>Whether the bubble is still held (loading, settling, unpacking or unpacked).</summary>
        public bool Active {
            get { return State >= StateLoading && State <= StateUnpacked; }
        }

        /// <summary>A second pose on a held bubble: the larger range is kept. True when R grew.</summary>
        public bool Raise (double range)
        {
            var why = Invalid (range);
            if (why != null)
                throw new ArgumentException (why);
            if (!Active || range <= Range)
                return false;
            Range = range;
            return true;
        }

        /// <summary>The ranges the tower must have in the current state.</summary>
        public Ranges Wanted {
            get {
                switch (State) {
                case StateLoading:
                case StateSettling:
                    return Ranges.LoadOnly;
                case StateUnpacking:
                case StateUnpacked:
                    return Ranges.Full;
                case StateReleased:
                    return Ranges.Original;
                default:
                    return Ranges.None;
                }
            }
        }

        /// <summary>
        /// The tolerance of a still pose (m): a root-relative position is the difference of two float32
        /// world positions, each within half an ulp per axis, so each look is off by at most one ulp per
        /// axis (sqrt(3) in norm) and two looks of a still tower differ by at most 2 sqrt(3) &lt; 4 ulps;
        /// ulp(x) &lt;= |x| eps32. <paramref name="scale"/>: the largest absolute world coordinate of the
        /// two looks plus the extent (the root's own float32 rotation turns a point of the tower by at most
        /// eps32 x its distance to the root).
        /// </summary>
        public static double ToleranceFor (double scale)
        {
            if (double.IsNaN (scale) || double.IsInfinity (scale) || scale < 0.0)
                return double.NaN;
            return 4.0 * Float32Epsilon * scale;
        }

        /// <summary>The largest displacement between two position lists of the same count (m), NaN otherwise.</summary>
        public static double MaxShift (double[] a, int countA, double[] b, int countB)
        {
            if (a == null || b == null || countA != countB || countA < 0 || countA % 3 != 0
                || countA > a.Length || countB > b.Length)
                return double.NaN;
            double max = 0.0;
            for (int i = 0; i < countA; i += 3) {
                double dx = a [i] - b [i], dy = a [i + 1] - b [i + 1], dz = a [i + 2] - b [i + 2];
                double d = Math.Sqrt (dx * dx + dy * dy + dz * dz);
                if (double.IsNaN (d))
                    return double.NaN;
                if (d > max)
                    max = d;
            }
            return max;
        }

        /// <summary>The Unity side found the ranges changed by someone else since its last write.</summary>
        public void MarkOverwritten ()
        {
            Overwritten = true;
        }

        /// <summary>
        /// One physics step. <paramref name="now"/>: game time (UT, s); <paramref name="stepTime"/>: the
        /// physics clock of this step (Time.fixedTime, s): a second look within one physics step (the pose
        /// RPC and the FixedUpdate of the same step) changes nothing, a still pose is counted between two
        /// different steps only. In this order: the vessel gone ends the bubble (Lost); not loaded: back to
        /// loading (after a load: Unloaded); loaded and unpacked: Unpacked (EarlyUnpack if it was not
        /// ready yet, AlreadyUnpacked at the first look); loaded and packed: after an unpack, Repacked
        /// (KSP unpacks it again with the full ranges); otherwise the load is watched and, after
        /// SettleTicks still steps with every part and servo ready, Ready.
        /// </summary>
        public Event Step (double now, double stepTime, Observation o)
        {
            if (!Active)
                return Event.None;
            if (Observed && !(stepTime > lastStepTime))
                return Event.None;
            bool first = !Observed;
            Observed = true;
            lastStepTime = stepTime;
            Last = o;
            if (!o.Exists) {
                End (StateLost, now);
                return Event.Lost;
            }
            if (!o.Loaded) {
                ResetStill ();
                if (State == StateLoading)
                    return Event.None;
                State = StateLoading;
                return Event.Unloaded;
            }
            if (!o.Packed) {
                ResetStill ();
                if (State == StateUnpacked)
                    return Event.None;
                var was = State;
                State = StateUnpacked;
                Unpacks++;
                if (double.IsNaN (UnpackedAt))
                    UnpackedAt = now;
                if (first)
                    return Event.AlreadyUnpacked;
                if (was == StateUnpacking)
                    return Event.Unpacked;
                EarlyUnpack = true;
                return Event.EarlyUnpack;
            }
            // loaded and packed
            if (State == StateUnpacked) {
                State = StateUnpacking;
                return Event.Repacked;
            }
            if (State == StateUnpacking)
                return Event.None;
            var ev = Event.None;
            if (State == StateLoading) {
                State = StateSettling;
                Loads++;
                if (double.IsNaN (LoadedAt))
                    LoadedAt = now;
                ResetStill ();
                ev = Event.Loaded;
            }
            // Settling: the load is watched.
            bool done = o.Parts > 0 && o.PartsStarted == o.Parts && o.ServosReady == o.Servos;
            // The scale of both looks: a floating origin shift between them moves every world coordinate.
            Tolerance = ToleranceFor (previousCount < 0 ? o.Scale : Math.Max (o.Scale, previousScale));
            Shift = MaxShift (o.Positions, o.Count, previous, previousCount);
            Remember (o.Positions, o.Count, o.Scale);
            if (done && !double.IsNaN (Shift) && (double.IsNaN (SettleShiftMax) || Shift > SettleShiftMax))
                SettleShiftMax = Shift;
            if (done && !double.IsNaN (Shift) && !double.IsNaN (Tolerance) && Shift <= Tolerance)
                StillTicks++;
            else
                StillTicks = 0;
            if (StillTicks >= SettleTicks) {
                State = StateUnpacking;
                if (double.IsNaN (ReadyAt))
                    ReadyAt = now;
                return Event.Ready;
            }
            return ev;
        }

        void ResetStill ()
        {
            StillTicks = 0;
            previousCount = -1;
        }

        void Remember (double[] positions, int count, double scale)
        {
            if (positions == null || count < 0 || count > positions.Length) {
                previousCount = -1;
                return;
            }
            if (previous.Length < count)
                previous = new double [count];
            Array.Copy (positions, previous, count);
            previousCount = count;
            previousScale = scale;
        }

        /// <summary>Released by a client (the original ranges are written back by the Unity side).</summary>
        public void Release (double now)
        {
            End (StateReleased, now);
        }

        void End (int state, double now)
        {
            if (!Active)
                return;
            State = state;
            EndedAt = now;
        }

        /// <summary>Game time since the pose, frozen at the end.</summary>
        public double Elapsed (double now)
        {
            return (Active ? now : EndedAt) - PosedAt;
        }

        static double Since (double t, double origin)
        {
            return double.IsNaN (t) ? double.NaN : t - origin;
        }

        static double Flag (bool b)
        {
            return b ? 1.0 : 0.0;
        }

        /// <summary>
        /// The status (layout tower_bubble_v1). <paramref name="loaded"/>, <paramref name="packed"/>
        /// (1, 0 or NaN) and <paramref name="distance"/> are read now by the Unity side; the counts are
        /// those of the last step.
        /// </summary>
        public List<double> Status (double now, double loaded, double packed, double distance)
        {
            var o = Last;
            var r = new List<double> (StatusLength) {
                Protocol, State, Range, Elapsed (now), loaded, packed,
                Observed ? o.Parts : double.NaN, Observed ? o.PartsStarted : double.NaN,
                Observed ? o.Servos : double.NaN, Observed ? o.ServosReady : double.NaN,
                StillTicks, SettleTicks, Shift, Tolerance, SettleShiftMax,
                Since (LoadedAt, PosedAt), Since (ReadyAt, PosedAt), Since (UnpackedAt, PosedAt),
                Flag (EarlyUnpack), Flag (Overwritten), Loads, Unpacks, distance
            };
            return r;
        }

        /// <summary>The status of a tower that has no bubble: protocol, StateNone, NaN elsewhere.</summary>
        public static List<double> NoneStatus (double loaded, double packed, double distance)
        {
            var r = new List<double> (StatusLength);
            for (int i = 0; i < StatusLength; i++)
                r.Add (double.NaN);
            r [0] = Protocol;
            r [1] = StateNone;
            r [4] = loaded;
            r [5] = packed;
            r [11] = SettleTicks;
            r [18] = 0;
            r [19] = 0;
            r [20] = 0;
            r [21] = 0;
            r [22] = distance;
            return r;
        }
    }
}
