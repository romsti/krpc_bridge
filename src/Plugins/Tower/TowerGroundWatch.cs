using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// The pure part of conn.tower_site.tower_ground_v1: which parts of a catch tower are fixed to the world,
    /// and the life of those joints (made at the unpack, made again at every floating origin shift, released at the
    /// pack, the end of the scene or on demand), counted for the status.
    ///
    /// WHY (ground tests of 16/10 and 19/10/2026 on a test install). The same north tower,
    /// not active, unpacked by the bubble (tower_bubble_v1), its arms opened, locked, closed: 3.2 km from the active
    /// vessel, the hinges of the arms slip, the arm's tip still - at maxMotorOutput 20000 a sustained vibration of
    /// 0.13 to 0.21 deg (period 1.2 s) and echec:temps, at 80000 the hinge runs against its own motor to its stop in
    /// 0.38 s; 400 m from the active vessel the same tower closes in 4.8-5.0 s (20000) and 3.00 s (80000), no second
    /// move, no vibration. Both capture flights that failed (15/10 18h35, 16/10 17h21) had the tower far away.
    ///
    /// THE MECHANISM (Kerbal Joint Reinforcement Continued, decompiled: KJRManager, KJRGroundJointModule). A landed
    /// vessel with launch clamps gets one FixedJoint to the world (connectedBody null, unbreakable) on each of its
    /// PartsToPick heaviest parts (Part.physicsMass, not a clamp, physicalSignificance not NONE; the symmetry
    /// counterparts of a pick are added) - only if the vessel is within MaxDistanceFromOrigin (500 m) of the floating
    /// origin when its physics starts ("Vessel ... is N m from origin, skipping ground joint creation" otherwise), and
    /// KJR destroys those joints at the first floating origin shift for good (OnFloatingOriginShift removes the
    /// module). On the v5 tower the three heaviest parts are the root TG.TowerBase (160 t, which TowerAnchor already
    /// fixes at any distance) and the two arm hinges TG.TowerHinge: 8.4 t each at maxMotorOutput 20000 (0.4 t of
    /// part, plus BaseServo.GetModuleMass = motorizedMassPerKN 0.0004 x driveUnit 20000), 32.4 t at 80000, against
    /// 8.0-8.04 t for the mast sections (8 t, plus their physicsless lights and flags). FlightIntegrator.UpdateMassStats
    /// gives half of physicsMass to each rigid body of a servo part (rb and servoRb), and kRPC's Part.mass reads rb.mass
    /// alone: the 4.2 t "read in game" at 20000 (16.2 t at 80000) is half of the 8.4 t (32.4 t) KJR sorts by. The
    /// joint is added to the part's own game object, i.e. to Part.rb: for a hinge, its fixed half; the arm hangs on its
    /// moving half (Part.servoRb, BaseServo's movingPartObject), which no joint here touches. So near the origin KJR
    /// holds the hinges' fixed halves to the world; far away nothing does, the hinge's reaction torque goes down the
    /// arm mount, the carriage and the mast, and the hinge's own angle (fixed half against moving half) moves while
    /// the arm stays.
    ///
    /// THE RULE (Select). Fixed: the parts KJR's rule picks (PartsToPick heaviest, same order, same exclusions,
    /// symmetry included), at any distance from the origin, and the fixed half (Part.rb) of every hinge
    /// (ModuleRoboticServoHinge) whose parent is attached to a node of its fixed half. NEVER fixed: a part on the
    /// moving side of any robotic servo (a child attached to a servo node of a servo, or below one), a servo whose
    /// parent holds it by a servo node (its own body turns against the tower), and anything attached by a surface node
    /// to or from a servo (its side is not known): those are "mobile", and a KJR pick that is mobile is dropped
    /// (KJR itself would fix it). A part without a rigid body gets no joint. On the v5 tower: the root and the two
    /// hinges, at 20000 as at 80000; with a light motor (stock 1200: 0.88 t) KJR's rule would pick two mast sections,
    /// and the hinge rule still fixes both hinges.
    ///
    /// PURE: System only (build/tests/tower). The Unity side (TowerGround) reads the parts, makes and destroys the
    /// joints, and logs.
    /// </summary>
    public sealed class TowerGroundWatch
    {
        /// <summary>Protocol of the tower_ground_v1 status (a field removed or moved raises it).</summary>
        public const int Protocol = 1;

        /// <summary>Schema of the status (a field appended raises it).</summary>
        public const int Schema = 1;

        /// <summary>Protocol of the tower_ground_parts_v1 table (a column or a header key removed or moved raises it).</summary>
        public const int PartsProtocol = 1;

        /// <summary>Schema of the table (a column or a header key appended raises it).</summary>
        public const int PartsSchema = 1;

        /// <summary>No ground joints were ever posed on this tower in this scene.</summary>
        public const int StateNone = 0;
        /// <summary>Posed; the tower is not loaded, packed, not landed, or has no root body: nothing fixed now.</summary>
        public const int StateWaiting = 1;
        /// <summary>Posed and fixed: the joints are in place.</summary>
        public const int StateHeld = 2;
        /// <summary>Released on demand: every joint destroyed.</summary>
        public const int StateReleased = 3;
        /// <summary>The vessel is gone (destroyed, recovered, or no longer in FlightGlobals.Vessels).</summary>
        public const int StateLost = 4;

        /// <summary>KJRGroundJointModule.PartsToPick (decompiled): the heaviest parts KJR fixes.</summary>
        public const int PartsToPick = 3;

        /// <summary>KJRGroundJointModule.MaxDistanceFromOrigin (m, decompiled): beyond it KJR fixes nothing.</summary>
        public const double KjrMaxDistanceFromOrigin = 500.0;

        /// <summary>The side of a servo a node belongs to.</summary>
        public const int SideFixed = 0;
        /// <summary>A node of the servo's moving half (BaseServo.servoAttachNodes).</summary>
        public const int SideMoving = 1;
        /// <summary>Not known: a surface attachment, or no node found.</summary>
        public const int SideUnknown = 2;

        /// <summary>The status fields, in wire order (layout tower_ground_v1, section "status").</summary>
        public static readonly string[] Fields = {
            "protocol", "state", "joints", "parts_fixed", "kjr_parts", "kjr_excluded", "hinges", "hinges_fixed",
            "hinges_skipped", "mass_fixed_tonnes", "makes", "remade_after_shift", "shifts", "broken", "releases",
            "elapsed_s", "held_after_s", "loaded", "packed", "origin_distance_m", "frame_rotating",
            "frame_speed_ms", "last_shift_m"
        };

        /// <summary>Length of the status (Fields.Length).</summary>
        public const int StatusLength = 23;

        /// <summary>The header keys of tower_ground_parts_v1, in order.</summary>
        public static readonly string[] PartsHeader = { "protocol", "schema", "persistent_id", "name", "state", "n" };

        /// <summary>The columns of tower_ground_parts_v1, in order.</summary>
        public static readonly string[] PartsColumns = {
            "part_persistent_id", "part_flight_id", "part_name", "physics_mass_tonnes", "kjr", "hinge", "mobile", "held"
        };

        // ------------------------------------------------------------------ the rule

        /// <summary>One part of the vessel, in the vessel's part order (Vessel.parts), as the Unity side reads it.</summary>
        public struct PartInfo
        {
            /// <summary>Index of the parent part in the list, -1 for the root (or a parent outside the vessel).</summary>
            public int Parent;
            /// <summary>Part.physicsMass (t): the part, its resources and its physicsless children.</summary>
            public double PhysicsMass;
            /// <summary>physicalSignificance is FULL (KJR picks only those).</summary>
            public bool Significant;
            /// <summary>The part has its own rigid body (Part.rb): a joint can hold it.</summary>
            public bool Body;
            /// <summary>A launch clamp (Part.isLaunchClamp): KJR never picks one.</summary>
            public bool Clamp;
            /// <summary>Carries a robotic servo (BaseServo: hinge, piston, rotation servo, rotor).</summary>
            public bool Servo;
            /// <summary>Carries a ModuleRoboticServoHinge.</summary>
            public bool Hinge;
            /// <summary>For a servo: the side of its own node that holds its parent (SideFixed for the root).</summary>
            public int ParentSide;
            /// <summary>When the parent is a servo: the side of the parent's node that holds this part.</summary>
            public int SideOnParent;
            /// <summary>Symmetry group (the smallest index of the part and its counterparts), -1 without counterparts.</summary>
            public int Symmetry;
        }

        /// <summary>What Select decided.</summary>
        public sealed class Selection
        {
            /// <summary>The parts to fix, by index: KJR's picks in KJR's order, then the hinges' fixed halves.</summary>
            public readonly List<int> Fixed = new List<int> ();
            /// <summary>Per part: picked by KJR's rule (mobile or not).</summary>
            public bool[] Kjr = new bool [0];
            /// <summary>Per part: a hinge (ModuleRoboticServoHinge), fixed or skipped.</summary>
            public bool[] Hinge = new bool [0];
            /// <summary>Per part: a hinge whose fixed half is fixed by the hinge rule.</summary>
            public bool[] HingeBase = new bool [0];
            /// <summary>Per part: on a moving side, never fixed.</summary>
            public bool[] Mobile = new bool [0];
            /// <summary>Parts KJR's rule picks (symmetry counterparts included).</summary>
            public int KjrPicked;
            /// <summary>Of those, the ones not fixed: mobile, or without a rigid body.</summary>
            public int KjrExcluded;
            /// <summary>Hinges of the vessel, those fixed, those skipped (mobile, or their side not known).</summary>
            public int Hinges, HingesFixed, HingesSkipped;
            /// <summary>Sum of the physics masses of the parts to fix (t).</summary>
            public double MassFixed;
        }

        /// <summary>Whether <paramref name="nodeId"/> is one of the comma-separated BaseServo.servoAttachNodes.</summary>
        public static bool IsServoNode (string servoAttachNodes, string nodeId)
        {
            if (string.IsNullOrEmpty (servoAttachNodes) || string.IsNullOrEmpty (nodeId))
                return false;
            foreach (var n in servoAttachNodes.Split (',')) {
                if (string.Equals (n.Trim (), nodeId.Trim (), StringComparison.Ordinal))
                    return true;
            }
            return false;
        }

        /// <summary>
        /// The rule (see the class): KJR's picks and the hinges' fixed halves, never a mobile part. Pure; the input is
        /// not modified. A parent index out of range is read as no parent; a parent chain that loops makes every part
        /// on it mobile (never fixed).
        /// </summary>
        public static Selection Select (IList<PartInfo> parts)
        {
            int n = parts == null ? 0 : parts.Count;
            var s = new Selection { Kjr = new bool [n], Hinge = new bool [n], HingeBase = new bool [n], Mobile = new bool [n] };
            if (n == 0)
                return s;
            ComputeMobile (parts, s.Mobile);
            // KJR's picks: OrderByDescending (physicsMass) is a stable sort, ties in the vessel's part order.
            var order = new int [n];
            for (int i = 0; i < n; i++)
                order [i] = i;
            Array.Sort (order, (a, b) => {
                int c = parts [b].PhysicsMass.CompareTo (parts [a].PhysicsMass);
                return c != 0 ? c : a.CompareTo (b);
            });
            var kjrOrder = new List<int> ();
            int picks = 0;
            for (int k = 0; k < n && picks < PartsToPick; k++) {
                int i = order [k];
                var p = parts [i];
                if (!p.Significant || p.Clamp || s.Kjr [i])
                    continue;
                s.Kjr [i] = true;
                kjrOrder.Add (i);
                picks++;
                if (p.Symmetry < 0)
                    continue;
                for (int j = 0; j < n; j++) {
                    if (j != i && parts [j].Symmetry == p.Symmetry && !s.Kjr [j]) {
                        s.Kjr [j] = true;
                        kjrOrder.Add (j);
                    }
                }
            }
            s.KjrPicked = kjrOrder.Count;
            foreach (int i in kjrOrder) {
                if (s.Mobile [i] || !parts [i].Body) {
                    s.KjrExcluded++;
                    continue;
                }
                s.Fixed.Add (i);
            }
            for (int i = 0; i < n; i++) {
                var p = parts [i];
                if (!p.Hinge)
                    continue;
                s.Hinge [i] = true;
                s.Hinges++;
                if (s.Mobile [i] || !p.Body) {
                    s.HingesSkipped++;
                    continue;
                }
                s.HingeBase [i] = true;
                s.HingesFixed++;
                if (!s.Fixed.Contains (i))
                    s.Fixed.Add (i);
            }
            foreach (int i in s.Fixed)
                s.MassFixed += parts [i].PhysicsMass;
            return s;
        }

        /// <summary>Mobile: on the moving side of a servo, below one, or not known (see the class). Iterative, loop-safe.</summary>
        static void ComputeMobile (IList<PartInfo> parts, bool[] mobile)
        {
            int n = parts.Count;
            // 0 unknown, 1 computing, 2 done
            var mark = new int [n];
            var chain = new List<int> ();
            for (int start = 0; start < n; start++) {
                if (mark [start] == 2)
                    continue;
                // climb to the first part already known (or the root), then resolve downwards
                chain.Clear ();
                int i = start;
                bool loop = false;
                while (i >= 0 && i < n && mark [i] != 2) {
                    if (mark [i] == 1) {
                        loop = true;
                        break;
                    }
                    mark [i] = 1;
                    chain.Add (i);
                    int parent = parts [i].Parent;
                    i = parent >= 0 && parent < n ? parent : -1;
                }
                for (int k = chain.Count - 1; k >= 0; k--) {
                    int c = chain [k];
                    mobile [c] = loop || Own (parts, c, mobile);
                    mark [c] = 2;
                }
            }
        }

        /// <summary>A part's own mobility once its parent's is known.</summary>
        static bool Own (IList<PartInfo> parts, int i, bool[] mobile)
        {
            var p = parts [i];
            int parent = p.Parent >= 0 && p.Parent < parts.Count ? p.Parent : -1;
            bool m = false;
            if (parent >= 0) {
                m = mobile [parent];
                if (parts [parent].Servo && p.SideOnParent != SideFixed)
                    m = true;
            }
            if (p.Servo && parent >= 0 && p.ParentSide != SideFixed)
                m = true;
            return m;
        }

        // ------------------------------------------------------------------ the life of the joints

        /// <summary>What the Unity side must do after a step.</summary>
        public enum Action
        {
            None,
            /// <summary>Fix the tower: a new selection, its joints made at the current pose.</summary>
            Make,
            /// <summary>Destroy every joint, then make them again (a joint destroyed by someone else, the parts changed).</summary>
            Remake,
            /// <summary>Destroy every joint: the tower can no longer be held (Why says why).</summary>
            Release,
            /// <summary>The vessel is gone: destroy whatever is left.</summary>
            Lost
        }

        /// <summary>One look at the tower, at one physics step (from the Unity side).</summary>
        public struct Observation
        {
            public bool Exists;
            public bool Loaded;
            public bool Packed;
            /// <summary>Situation LANDED or PRELAUNCH (TowerAnchor's rule).</summary>
            public bool Landed;
            /// <summary>The root part has its rigid body.</summary>
            public bool RootBody;
            /// <summary>Parts of the vessel now.</summary>
            public int Parts;
            /// <summary>Joints of this tower still alive (a destroyed Unity component reads null).</summary>
            public int Joints;
        }

        public readonly double PosedAt;
        public int State { get; private set; }
        public double EndedAt { get; private set; }
        public double HeldAt { get; private set; }
        /// <summary>Why the last Release, Remake or drop happened (for the log).</summary>
        public string Why { get; private set; }
        public int Made { get; private set; }
        public int PartsAtMake { get; private set; }
        public int Makes { get; private set; }
        public int RemadeAfterShift { get; private set; }
        public int Shifts { get; private set; }
        public int Broken { get; private set; }
        public int Releases { get; private set; }
        public double LastShift { get; private set; }
        public Selection Last { get; private set; }
        /// <summary>The part count of a make that fixed nothing (-1 otherwise): not tried again until it changes.</summary>
        public int EmptyParts { get; private set; }

        TowerGroundWatch (double now)
        {
            PosedAt = now;
            State = StateWaiting;
            EndedAt = double.NaN;
            HeldAt = double.NaN;
            LastShift = double.NaN;
            EmptyParts = -1;
            Why = "";
        }

        /// <summary>Ground joints posed at <paramref name="now"/> (game time, s): waiting until the first step.</summary>
        public static TowerGroundWatch Pose (double now)
        {
            return new TowerGroundWatch (now);
        }

        /// <summary>Whether the joints are still wanted (waiting or held).</summary>
        public bool Active {
            get { return State == StateWaiting || State == StateHeld; }
        }

        /// <summary>Whether the joints are in place.</summary>
        public bool Held {
            get { return State == StateHeld; }
        }

        /// <summary>
        /// One physics step. In this order: the vessel gone (Lost); not loaded, packed, not landed or no root body:
        /// Release if held (Why: unloaded, packed, not landed, no root body), else wait; held: a joint destroyed by
        /// someone else (counted Broken) or the part count changed since the joints were made: Remake; not held: Make.
        /// </summary>
        public Action Step (double now, Observation o)
        {
            if (!Active)
                return Action.None;
            if (!o.Exists) {
                End (StateLost, now);
                Why = "the vessel is gone";
                return Action.Lost;
            }
            if (!o.Loaded || o.Packed || !o.Landed || !o.RootBody) {
                EmptyParts = -1;
                if (State != StateHeld)
                    return Action.None;
                State = StateWaiting;
                Releases++;
                Why = !o.Loaded ? "unloaded" : o.Packed ? "packed" : !o.Landed ? "not landed" : "no root body";
                return Action.Release;
            }
            if (State == StateHeld) {
                bool broken = o.Joints < Made;
                if (broken)
                    Broken += Made - o.Joints;
                bool changed = o.Parts != PartsAtMake;
                if (!broken && !changed)
                    return Action.None;
                Why = changed ? "the part count changed (" + PartsAtMake + " -> " + o.Parts + ")"
                    : (Made - o.Joints) + " joint(s) destroyed by someone else";
                return Action.Remake;
            }
            // a make that fixed nothing (no part could take a joint) is not tried again for the same parts
            if (EmptyParts == o.Parts)
                return Action.None;
            Why = "";
            return Action.Make;
        }

        /// <summary>
        /// The Unity side made the joints of <paramref name="selection"/> (after Make or Remake): <paramref name="joints"/>
        /// made, the vessel had <paramref name="parts"/> parts. Held when at least one joint was made, waiting otherwise.
        /// </summary>
        public void MarkMade (double now, Selection selection, int joints, int parts)
        {
            if (!Active)
                return;
            Record (now, selection, joints, parts);
            Makes++;
        }

        /// <summary>
        /// A floating origin shift of <paramref name="offset"/> (m, the move of a landed vessel). True when the joints are
        /// held: the Unity side destroys them at once and makes them again (MarkRemadeAfterShift, or MarkDropped).
        /// </summary>
        public bool Shift (double now, double offset)
        {
            if (!Active)
                return false;
            Shifts++;
            LastShift = offset;
            return State == StateHeld;
        }

        /// <summary>The joints made again right after a floating origin shift.</summary>
        public void MarkRemadeAfterShift (double now, Selection selection, int joints, int parts)
        {
            if (!Active)
                return;
            Record (now, selection, joints, parts);
            RemadeAfterShift++;
        }

        /// <summary>The joints destroyed and not made again (the tower could not be held at that moment).</summary>
        public void MarkDropped (string why)
        {
            if (State != StateHeld)
                return;
            State = StateWaiting;
            Releases++;
            Made = 0;
            Why = why ?? "";
        }

        void Record (double now, Selection selection, int joints, int parts)
        {
            if (!Active)
                return;
            Last = selection;
            Made = Math.Max (0, joints);
            PartsAtMake = parts;
            EmptyParts = Made > 0 ? -1 : parts;
            State = Made > 0 ? StateHeld : StateWaiting;
            if (State == StateHeld && double.IsNaN (HeldAt))
                HeldAt = now;
        }

        /// <summary>Released by a client (the Unity side destroys the joints).</summary>
        public void Release (double now)
        {
            Why = "released on demand";
            End (StateReleased, now);
        }

        void End (int state, double now)
        {
            if (!Active)
                return;
            State = state;
            EndedAt = now;
            Made = 0;
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

        static double Count (Selection s, Func<Selection, int> f)
        {
            return s == null ? double.NaN : f (s);
        }

        /// <summary>
        /// The status (layout tower_ground_v1). <paramref name="loaded"/>, <paramref name="packed"/> (1, 0 or NaN),
        /// <paramref name="joints"/> (alive now), <paramref name="originDistance"/> (|root position|, m, KJR's
        /// measure), <paramref name="frameRotating"/> (1, 0) and <paramref name="frameSpeed"/> (Krakensbane, m/s) are
        /// read now by the Unity side; the selection counts are those of the last make.
        /// </summary>
        public List<double> Status (double now, double loaded, double packed, double joints, double originDistance,
                                    double frameRotating, double frameSpeed)
        {
            var s = Last;
            return new List<double> (StatusLength) {
                Protocol, State, joints,
                s == null ? double.NaN : s.Fixed.Count, Count (s, x => x.KjrPicked), Count (s, x => x.KjrExcluded),
                Count (s, x => x.Hinges), Count (s, x => x.HingesFixed), Count (s, x => x.HingesSkipped),
                s == null ? double.NaN : s.MassFixed,
                Makes, RemadeAfterShift, Shifts, Broken, Releases,
                Elapsed (now), Since (HeldAt, PosedAt), loaded, packed, originDistance, frameRotating, frameSpeed,
                LastShift
            };
        }

        /// <summary>The status of a tower without ground joints: protocol, StateNone, zero counts, NaN elsewhere.</summary>
        public static List<double> NoneStatus (double loaded, double packed, double originDistance, double frameRotating,
                                               double frameSpeed)
        {
            var r = new List<double> (StatusLength);
            for (int i = 0; i < StatusLength; i++)
                r.Add (double.NaN);
            r [0] = Protocol;
            r [1] = StateNone;
            r [2] = 0;
            for (int i = 10; i <= 14; i++)
                r [i] = 0;
            r [17] = loaded;
            r [18] = packed;
            r [19] = originDistance;
            r [20] = frameRotating;
            r [21] = frameSpeed;
            return r;
        }

        // ------------------------------------------------------------------ the parts table

        /// <summary>One row of tower_ground_parts_v1: a part KJR's rule picks, or a hinge.</summary>
        public sealed class PartRow
        {
            public uint PartPersistentId;
            public uint PartFlightId;
            public string Name = "";
            public double PhysicsMass = double.NaN;
            public bool Kjr, Hinge, Mobile, Held;
        }

        /// <summary>
        /// The rows of a selection, by part index: every part KJR's rule picks and every hinge, the fixed ones first (in
        /// the order they are fixed), then the others in part order.
        /// </summary>
        public static List<int> RowOrder (Selection s)
        {
            var r = new List<int> ();
            if (s == null)
                return r;
            foreach (int i in s.Fixed)
                r.Add (i);
            for (int i = 0; i < s.Kjr.Length; i++)
                if ((s.Kjr [i] || (i < s.Hinge.Length && s.Hinge [i])) && !r.Contains (i))
                    r.Add (i);
            return r;
        }

        /// <summary>
        /// The table tower_ground_parts_v1: row 0 the header protocol, schema, persistent_id (the tower), name, state,
        /// n; row 1 the columns part_persistent_id, part_flight_id, part_name, physics_mass_tonnes, kjr (picked by KJR's
        /// rule), hinge (a hinge), mobile (on a moving side: never fixed), held (a joint holds it now); one row per part.
        /// </summary>
        public static List<string> PartsTable (uint towerPid, string towerName, int state, IList<PartRow> rows)
        {
            int n = rows == null ? 0 : rows.Count;
            var lines = new List<string> (n + 2);
            var h = new StringBuilder ();
            h.Append ("protocol=").Append (TowerSiteFormat.Int (PartsProtocol));
            h.Append ("\tschema=").Append (TowerSiteFormat.Int (PartsSchema));
            h.Append ("\tpersistent_id=").Append (TowerSiteFormat.Id (towerPid));
            h.Append ("\tname=").Append (TowerSiteFormat.Clean (towerName));
            h.Append ("\tstate=").Append (state.ToString (CultureInfo.InvariantCulture));
            h.Append ("\tn=").Append (TowerSiteFormat.Int (n));
            lines.Add (h.ToString ());
            lines.Add (string.Join ("\t", PartsColumns));
            for (int i = 0; i < n; i++) {
                var r = rows [i];
                lines.Add (string.Join ("\t", new[] {
                    TowerSiteFormat.Id (r.PartPersistentId), TowerSiteFormat.Id (r.PartFlightId),
                    TowerSiteFormat.Clean (r.Name), TowerSiteFormat.Num (r.PhysicsMass), TowerSiteFormat.Flag (r.Kjr),
                    TowerSiteFormat.Flag (r.Hinge), TowerSiteFormat.Flag (r.Mobile), TowerSiteFormat.Flag (r.Held)
                }));
            }
            return lines;
        }
    }
}
