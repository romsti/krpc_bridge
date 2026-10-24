// -----------------------------------------------------------------------------------
// conn.tower_site.tower_ground_v1: pure tests of TowerGroundWatch - which parts of a catch tower
// are fixed to the world (KJR's rule at any distance from the floating origin, the hinges' fixed halves,
// never a moving side), the life of the joints (made at the unpack, made again at a floating origin
// shift, released at the pack, broken, lost, released on demand), the status, the parts table and their
// layouts. See TowerTests.csproj; run by Program.Main.
//
// The tower is the shipped v5 (distribution/TowerCatch/Crafts/Tour Catch v5.craft, 48 parts in the craft's order),
// with the masses of Patches/TowerCatch.cfg (distribution/TowerCatch/) and of the stock parts: what KJRGroundJointModule sorts by
// (Part.physicsMass: the part, its resources, its physicsless children).
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;

namespace KRPC.Bridge.Tower
{
    public static partial class Program
    {
        static void TowerGroundTests ()
        {
            TestGroundServoNodes ();
            TestGroundV5AtEveryMotor ();
            TestGroundLightMotorKeepsTheHinges ();
            TestGroundNeverAMovingSide ();
            TestGroundInvertedAndSurfaceHinges ();
            TestGroundKjrExclusionsAndSymmetry ();
            TestGroundParentLoopIsMobile ();
            TestGroundLifeCycle ();
            TestGroundShift ();
            TestGroundBrokenAndChanged ();
            TestGroundEmptyMake ();
            TestGroundReleaseAndLost ();
            TestGroundStatus ();
            TestGroundPartsTable ();
            TestGroundLayouts ();
        }

        // ------------------------------------------------------------------ the v5 tower

        // Indexes of the v5 craft's parts (the craft's PART order, Vessel.parts after a load).
        const int V5Root = 0, V5Core1 = 1, V5Core2 = 2, V5Core3 = 3, V5Core4 = 4, V5Top = 5, V5Carriage = 6;
        const int V5Battery = 7, V5Probe = 8, V5MountN = 9, V5HingeN = 10, V5ArmN = 11, V5TrussN = 12, V5TipN = 13;
        const int V5MountS = 18, V5HingeS = 19, V5ArmS = 20, V5TrussS = 21, V5TipS = 22;
        const int V5Clamp0 = 27, V5CamTop = 31, V5CamRailN = 32, V5CamRailS = 33, V5CamBase = 34;
        const int V5Parts = 48;

        /// <summary>
        /// The physics mass of a TG.TowerHinge at a motor: 0.4 t of part (hinge_04.cfg) plus the motor
        /// (BaseServo.GetModuleMass = motorizedMassPerKN 0.0004 x driveUnit, driveUnit = maxMotorOutput at 100 %
        /// motor size). The game's rb.mass is half of it (FlightIntegrator.UpdateMassStats: rb and servoRb).
        /// </summary>
        static double HingeMass (double maxMotorOutput)
        {
            return 0.4 + 0.0004 * maxMotorOutput;
        }

        static TowerGroundWatch.PartInfo P (int parent, double mass, bool body = true)
        {
            return new TowerGroundWatch.PartInfo {
                Parent = parent, PhysicsMass = body ? mass : 0.0, Significant = body, Body = body, Symmetry = -1,
                ParentSide = TowerGroundWatch.SideFixed, SideOnParent = TowerGroundWatch.SideFixed
            };
        }

        /// <summary>
        /// The v5 tower at a motor. Physicsless parts (lights, flags: PhysicsSignificance = 1) have no body and add
        /// their mass to their parent's physicsMass: mast section 2 carries two spotlights (0.015 t) and three flags
        /// (0.0005 t), section 4 two spotlights and four nav lights (0.001 t), each arm's tip one nav light.
        /// </summary>
        static List<TowerGroundWatch.PartInfo> V5 (double maxMotorOutput)
        {
            var l = new List<TowerGroundWatch.PartInfo> ();
            l.Add (P (-1, 160.0));                          // 0 TG.TowerBase
            l.Add (P (0, 8.0));                             // 1 TG.TowerCore 10 m
            l.Add (P (1, 8.0 + 2 * 0.015 + 3 * 0.0005));    // 2 TG.TowerCore 20 m
            l.Add (P (2, 8.0));                             // 3 TG.TowerCore 30 m
            l.Add (P (3, 8.0 + 2 * 0.015 + 4 * 0.001));     // 4 TG.TowerCore 40 m
            l.Add (P (4, 1.0));                             // 5 TG.TowerTop
            l.Add (P (3, 2.325));                           // 6 TG.TowerTruss 5 m: the carriage
            l.Add (P (6, 0.2));                             // 7 batteryBankLarge
            l.Add (P (7, 0.1));                             // 8 probeCoreOcto.v2
            for (int side = 0; side < 2; side++) {
                int mount = l.Count;
                l.Add (P (6, 0.6));                         // TG.TowerArm Short: the arm mount
                var h = P (mount, HingeMass (maxMotorOutput));
                h.Servo = h.Hinge = true;                   // TG.TowerHinge, parent on its "bottom" node
                l.Add (h);
                var arm = P (mount + 1, 0.6);               // TG.TowerArm Short on the hinge's "top" (servo) node
                arm.SideOnParent = TowerGroundWatch.SideMoving;
                l.Add (arm);
                l.Add (P (mount + 2, 4.65));                // TG.TowerTruss 10 m
                l.Add (P (mount + 3, 2.325 + 0.001));       // TG.TowerTruss 5 m, the tip
                for (int k = 0; k < 4; k++)                 // structuralPanel2, the rails
                    l.Add (P (k < 3 ? mount + 3 : mount + 4, 0.3));
            }
            for (int k = 0; k < 4; k++) {                   // 27-30 TG.TowerClamp
                var c = P (0, 0.1);
                c.Clamp = true;
                l.Add (c);
            }
            l.Add (P (4, 0.004));                           // 31 TG.TowerCamTop
            l.Add (P (13, 0.004));                          // 32 TG.TowerCamRailN on the north tip
            l.Add (P (22, 0.004));                          // 33 TG.TowerCamRailS on the south tip
            l.Add (P (0, 0.004));                           // 34 TG.TowerCamBase
            foreach (int parent in new[] { 4, 4, 2, 2 })    // 35-38 spotLight2.v2
                l.Add (P (parent, 0.015, false));
            foreach (int parent in new[] { 4, 4, 4, 4, 13, 22 })  // 39-44 navLight1
                l.Add (P (parent, 0.001, false));
            for (int k = 0; k < 3; k++)                     // 45-47 flagPartSize3
                l.Add (P (2, 0.0005, false));
            return l;
        }

        static string Ids (List<int> l)
        {
            return string.Join (",", l.ConvertAll (i => i.ToString ()).ToArray ());
        }

        // ------------------------------------------------------------------ the rule

        static void TestGroundServoNodes ()
        {
            Check (TowerGroundWatch.IsServoNode ("top", "top"), "servo node: top");
            Check (TowerGroundWatch.IsServoNode (" top , bottom2", "bottom2"), "servo node: a list, trimmed");
            Check (!TowerGroundWatch.IsServoNode ("top", "bottom"), "not a servo node: bottom");
            Check (!TowerGroundWatch.IsServoNode ("", "top"), "no servo node at all");
            Check (!TowerGroundWatch.IsServoNode ("top", null), "no node id");
            Check (!TowerGroundWatch.IsServoNode ("Top", "top"), "case-sensitive, as KSP's node ids");
        }

        static void TestGroundV5AtEveryMotor ()
        {
            var v5 = V5 (20000.0);
            Equal (V5Parts, v5.Count, "the v5 craft has 48 parts");
            Near (8.4, v5 [V5HingeN].PhysicsMass, 1e-12, "hinge at 20000: 8.4 t (4.2 t read on its rb)");
            foreach (var motor in new[] { 20000.0, 40000.0, 80000.0 }) {
                var s = TowerGroundWatch.Select (V5 (motor));
                string at = " at maxMotorOutput " + motor;
                // KJR's rule: the root, then the two hinges (heavier than every mast section, 8.0-8.034 t)
                Equal ("0,10,19", Ids (s.Fixed), "fixed: root, hinge N, hinge S" + at);
                Equal (3, s.KjrPicked, "KJR's three picks" + at);
                Equal (0, s.KjrExcluded, "none excluded" + at);
                Check (s.Kjr [V5Root] && s.Kjr [V5HingeN] && s.Kjr [V5HingeS], "KJR picks root and hinges" + at);
                Equal (2, s.Hinges, "two hinges" + at);
                Equal (2, s.HingesFixed, "both hinges fixed" + at);
                Equal (0, s.HingesSkipped, "no hinge skipped" + at);
                Check (s.HingeBase [V5HingeN] && s.HingeBase [V5HingeS], "the hinge rule holds them too" + at);
                Near (160.0 + 2 * HingeMass (motor), s.MassFixed, 1e-9, "mass fixed" + at);
            }
            var m = TowerGroundWatch.Select (V5 (20000.0)).Mobile;
            foreach (int i in new[] { V5ArmN, V5TrussN, V5TipN, 14, 15, 16, 17, V5CamRailN, 43,
                                      V5ArmS, V5TrussS, V5TipS, 23, 24, 25, 26, V5CamRailS, 44 })
                Check (m [i], "the arms are mobile: part " + i);
            foreach (int i in new[] { V5Root, V5Core1, V5Core2, V5Core3, V5Core4, V5Top, V5Carriage, V5Battery, V5Probe,
                                      V5MountN, V5HingeN, V5MountS, V5HingeS, V5Clamp0, V5CamTop, V5CamBase })
                Check (!m [i], "the mast, the carriage and the hinges' fixed halves are not: part " + i);
        }

        static void TestGroundLightMotorKeepsTheHinges ()
        {
            // the stock G-32W motor (1200): 0.88 t; KJR's rule picks the two heaviest mast sections instead
            var s = TowerGroundWatch.Select (V5 (1200.0));
            Equal ("0,4,2,10,19", Ids (s.Fixed), "root, sections 4 and 2 (KJR), then both hinges (hinge rule)");
            Check (s.Kjr [V5Core4] && s.Kjr [V5Core2] && !s.Kjr [V5HingeN], "KJR's picks without the hinges");
            Equal (2, s.HingesFixed, "both hinges fixed by the hinge rule");
            Check (!s.Kjr [V5Core1] && !s.Kjr [V5Core3], "ties broken by part order after the physicsless masses");
        }

        static void TestGroundNeverAMovingSide ()
        {
            // a heavy load on the north arm (a caught booster's mass would never be part of the tower, but a heavy
            // truss could): KJR would fix it, the rule drops it
            var l = V5 (20000.0);
            var t = l [V5TrussN];
            t.PhysicsMass = 50.0;
            l [V5TrussN] = t;
            var s = TowerGroundWatch.Select (l);
            Check (s.Kjr [V5TrussN], "KJR's rule picks the heavy truss");
            Check (s.Mobile [V5TrussN], "on the arm: mobile");
            Check (!s.Fixed.Contains (V5TrussN), "never fixed");
            Equal (1, s.KjrExcluded, "one KJR pick excluded");
            Equal ("0,10,19", Ids (s.Fixed), "the hinge S lost its KJR pick but the hinge rule fixes both");
            Check (!s.Kjr [V5HingeS] && s.HingeBase [V5HingeS], "hinge S: hinge rule only");
            foreach (int i in s.Fixed)
                Check (!s.Mobile [i], "nothing fixed is mobile: part " + i);
        }

        static void TestGroundInvertedAndSurfaceHinges ()
        {
            // the north hinge held by its parent through its servo node: its own body turns against the tower
            var l = V5 (20000.0);
            var h = l [V5HingeN];
            h.ParentSide = TowerGroundWatch.SideMoving;
            l [V5HingeN] = h;
            var s = TowerGroundWatch.Select (l);
            Check (s.Mobile [V5HingeN] && s.Mobile [V5ArmN] && s.Mobile [V5TipN], "inverted hinge and its arm: mobile");
            Check (!s.Fixed.Contains (V5HingeN), "inverted hinge: not fixed");
            Equal (1, s.HingesSkipped, "one hinge skipped");
            Equal (1, s.HingesFixed, "the other fixed");
            Equal (1, s.KjrExcluded, "its KJR pick excluded");
            // a hinge surface-attached to its parent: its side is not known
            l = V5 (20000.0);
            h = l [V5HingeS];
            h.ParentSide = TowerGroundWatch.SideUnknown;
            l [V5HingeS] = h;
            s = TowerGroundWatch.Select (l);
            Check (s.Mobile [V5HingeS] && !s.Fixed.Contains (V5HingeS), "surface-attached hinge: not fixed");
            // a part surface-attached to a hinge: mobile
            l = V5 (20000.0);
            var probe = P (V5HingeN, 30.0);
            probe.SideOnParent = TowerGroundWatch.SideUnknown;
            l.Add (probe);
            s = TowerGroundWatch.Select (l);
            Check (s.Mobile [V5Parts] && s.Kjr [V5Parts] && !s.Fixed.Contains (V5Parts), "surface-attached to a hinge: mobile");
            // a part on the hinge's fixed node: fixed side, as the hinge
            l = V5 (20000.0);
            var bracket = P (V5HingeN, 30.0);
            l.Add (bracket);
            s = TowerGroundWatch.Select (l);
            Check (!s.Mobile [V5Parts] && s.Fixed.Contains (V5Parts), "on the hinge's fixed node: fixed by KJR's rule");
            // a servo that is the root: its own body is the base
            var root = new List<TowerGroundWatch.PartInfo> { P (-1, 10.0), P (0, 1.0) };
            var r0 = root [0];
            r0.Servo = r0.Hinge = true;
            root [0] = r0;
            var c1 = root [1];
            c1.SideOnParent = TowerGroundWatch.SideMoving;
            root [1] = c1;
            s = TowerGroundWatch.Select (root);
            Check (!s.Mobile [0] && s.Mobile [1], "root hinge: its base fixed, its moving child mobile");
            Equal ("0", Ids (s.Fixed), "root hinge: fixed once");
        }

        static void TestGroundKjrExclusionsAndSymmetry ()
        {
            var l = new List<TowerGroundWatch.PartInfo> {
                P (-1, 100.0),          // 0 root
                P (0, 50.0),            // 1 a clamp, heaviest: never picked
                P (0, 40.0, false),     // 2 physicsless (no body, not significant): never picked
                P (0, 9.0),             // 3 symmetric pair 3/4
                P (0, 9.0),             // 4
                P (0, 8.0),             // 5
                P (0, 7.0),             // 6
            };
            var c = l [1];
            c.Clamp = true;
            l [1] = c;
            var a = l [3];
            a.Symmetry = 3;
            l [3] = a;
            var b = l [4];
            b.Symmetry = 3;
            l [4] = b;
            var s = TowerGroundWatch.Select (l);
            Check (!s.Kjr [1] && !s.Kjr [2], "no clamp, no physicsless part");
            // picks: the root, 3 (and its counterpart 4), 5: three picks, four parts
            Equal ("0,3,4,5", Ids (s.Fixed), "KJR's order, the counterpart right after its pick");
            Equal (4, s.KjrPicked, "four parts for three picks");
            Check (!s.Kjr [6], "the fourth heaviest is not picked");
            // a significant part without a rigid body is picked by KJR's rule but cannot be fixed
            var d = l [5];
            d.Body = false;
            l [5] = d;
            s = TowerGroundWatch.Select (l);
            Check (s.Kjr [5] && !s.Fixed.Contains (5), "no body: picked, not fixed");
            Equal (1, s.KjrExcluded, "counted excluded");
            Equal (0, TowerGroundWatch.Select (new List<TowerGroundWatch.PartInfo> ()).Fixed.Count, "no parts, nothing");
            Equal (0, TowerGroundWatch.Select (null).Fixed.Count, "null, nothing");
        }

        static void TestGroundParentLoopIsMobile ()
        {
            var l = new List<TowerGroundWatch.PartInfo> { P (-1, 1.0), P (2, 50.0), P (1, 40.0), P (7, 30.0) };
            var s = TowerGroundWatch.Select (l);
            Check (s.Mobile [1] && s.Mobile [2], "a parent loop: mobile, never fixed");
            Check (!s.Mobile [0] && !s.Mobile [3], "the root and a parent out of range: not mobile");
            Equal ("3", Ids (s.Fixed), "fixed: the loop excluded (the root is the fourth heaviest)");
            Equal (2, s.KjrExcluded, "the loop's two picks excluded");
        }

        // ------------------------------------------------------------------ the life of the joints

        static TowerGroundWatch.Observation Ground (bool loaded = true, bool packed = false, bool landed = true,
                                                    int joints = 0, int parts = V5Parts, bool exists = true,
                                                    bool rootBody = true)
        {
            return new TowerGroundWatch.Observation {
                Exists = exists, Loaded = loaded, Packed = packed, Landed = landed, RootBody = rootBody, Parts = parts,
                Joints = joints
            };
        }

        static void TestGroundLifeCycle ()
        {
            var w = TowerGroundWatch.Pose (100.0);
            Equal (TowerGroundWatch.StateWaiting, w.State, "posed: waiting");
            Equal (TowerGroundWatch.Action.None, w.Step (100.0, Ground (loaded: false)), "not loaded: wait");
            Equal (TowerGroundWatch.Action.None, w.Step (100.02, Ground (packed: true)), "packed: wait");
            Equal (TowerGroundWatch.Action.None, w.Step (100.04, Ground (landed: false)), "not landed: wait");
            Equal (TowerGroundWatch.Action.None, w.Step (100.06, Ground (rootBody: false)), "no root body: wait");
            Equal (TowerGroundWatch.Action.Make, w.Step (100.08, Ground ()), "unpacked and landed: make");
            var s = TowerGroundWatch.Select (V5 (20000.0));
            w.MarkMade (100.08, s, 3, V5Parts);
            Check (w.Held, "made: held");
            Near (0.08, w.HeldAt - w.PosedAt, 1e-9, "held 0.08 s after the pose");
            Equal (TowerGroundWatch.Action.None, w.Step (100.10, Ground (joints: 3)), "held, intact: nothing");
            Equal (TowerGroundWatch.Action.Release, w.Step (100.12, Ground (packed: true, joints: 3)), "packed: release");
            Equal ("packed", w.Why, "why: packed");
            Equal (TowerGroundWatch.StateWaiting, w.State, "released at the pack: waiting");
            Equal (TowerGroundWatch.Action.None, w.Step (100.14, Ground (packed: true)), "still packed: wait");
            Equal (TowerGroundWatch.Action.Make, w.Step (100.16, Ground ()), "unpacked again: make again");
            w.MarkMade (100.16, s, 3, V5Parts);
            Equal (TowerGroundWatch.Action.Release, w.Step (100.18, Ground (loaded: false, joints: 3)), "unloaded: release");
            Equal ("unloaded", w.Why, "why: unloaded");
            Equal (TowerGroundWatch.Action.Make, w.Step (100.20, Ground ()), "back: make");
            w.MarkMade (100.20, s, 3, V5Parts);
            Equal (TowerGroundWatch.Action.Release, w.Step (100.22, Ground (landed: false, joints: 3)), "off the ground");
            Equal ("not landed", w.Why, "why: not landed");
            Equal (3, w.Releases, "three releases");
            Equal (3, w.Makes, "three makes");
            Near (0.08, w.HeldAt - w.PosedAt, 1e-9, "held_after_s is the first time");
        }

        static void TestGroundShift ()
        {
            var w = TowerGroundWatch.Pose (0.0);
            Check (!w.Shift (0.01, 650.0), "a shift while waiting: nothing to remake");
            Equal (1, w.Shifts, "but counted");
            Near (650.0, w.LastShift, 0, "its move");
            w.Step (0.02, Ground ());
            var s = TowerGroundWatch.Select (V5 (80000.0));
            w.MarkMade (0.02, s, 3, V5Parts);
            Check (w.Shift (0.04, 3243.5), "a shift while held: remade at once");
            w.MarkRemadeAfterShift (0.04, s, 3, V5Parts);
            Check (w.Held, "held after the shift");
            Equal (1, w.RemadeAfterShift, "remade once");
            Equal (1, w.Makes, "a shift is not a make");
            Equal (2, w.Shifts, "two shifts");
            Equal (TowerGroundWatch.Action.None, w.Step (0.06, Ground (joints: 3)), "intact after the shift");
            Check (w.Shift (0.08, 500.0), "another shift");
            w.MarkDropped ("floating origin shift while it cannot be held");
            Equal (TowerGroundWatch.StateWaiting, w.State, "dropped: waiting");
            Equal (1, w.Releases, "counted released");
            Equal (TowerGroundWatch.Action.Make, w.Step (0.10, Ground ()), "made again at the next step");
        }

        static void TestGroundBrokenAndChanged ()
        {
            var w = TowerGroundWatch.Pose (0.0);
            w.Step (0.0, Ground ());
            var s = TowerGroundWatch.Select (V5 (20000.0));
            w.MarkMade (0.0, s, 3, V5Parts);
            Equal (TowerGroundWatch.Action.Remake, w.Step (0.02, Ground (joints: 2)), "a joint destroyed: remake");
            Equal (1, w.Broken, "one broken");
            Check (w.Why.Contains ("destroyed"), "why: " + w.Why);
            w.MarkMade (0.02, s, 3, V5Parts);
            Equal (TowerGroundWatch.Action.Remake, w.Step (0.04, Ground (joints: 3, parts: 47)), "a part gone: remake");
            Check (w.Why.Contains ("48 -> 47"), "why: " + w.Why);
            Equal (1, w.Broken, "no joint missing: not broken");
            w.MarkMade (0.04, s, 3, 47);
            Equal (TowerGroundWatch.Action.None, w.Step (0.06, Ground (joints: 3, parts: 47)), "stable again");
            Equal (3, w.Makes, "three makes");
        }

        static void TestGroundEmptyMake ()
        {
            var w = TowerGroundWatch.Pose (0.0);
            Equal (TowerGroundWatch.Action.Make, w.Step (0.0, Ground ()), "make");
            w.MarkMade (0.0, new TowerGroundWatch.Selection (), 0, V5Parts);
            Equal (TowerGroundWatch.StateWaiting, w.State, "nothing made: waiting");
            Equal (TowerGroundWatch.Action.None, w.Step (0.02, Ground ()), "not tried again for the same parts");
            Equal (TowerGroundWatch.Action.Make, w.Step (0.04, Ground (parts: 49)), "tried again when the parts change");
            w.MarkMade (0.04, new TowerGroundWatch.Selection (), 0, 49);
            w.Step (0.06, Ground (packed: true, parts: 49));
            Equal (TowerGroundWatch.Action.Make, w.Step (0.08, Ground (parts: 49)), "tried again at the next unpack");
        }

        static void TestGroundReleaseAndLost ()
        {
            var w = TowerGroundWatch.Pose (10.0);
            w.Step (10.0, Ground ());
            w.MarkMade (10.0, TowerGroundWatch.Select (V5 (20000.0)), 3, V5Parts);
            w.Release (12.5);
            Equal (TowerGroundWatch.StateReleased, w.State, "released on demand");
            Check (!w.Active, "no longer active");
            Equal (TowerGroundWatch.Action.None, w.Step (13.0, Ground (joints: 0)), "released: steps do nothing");
            Check (!w.Shift (13.0, 600.0), "released: a shift does nothing");
            Equal (0, w.Shifts, "not counted");
            Near (2.5, w.Elapsed (99.0), 1e-12, "elapsed frozen at the release");
            w.MarkMade (14.0, TowerGroundWatch.Select (V5 (20000.0)), 3, V5Parts);
            Equal (TowerGroundWatch.StateReleased, w.State, "a late make cannot hold a released tower");
            var g = TowerGroundWatch.Pose (0.0);
            Equal (TowerGroundWatch.Action.Lost, g.Step (1.0, Ground (exists: false)), "the vessel gone: lost");
            Equal (TowerGroundWatch.StateLost, g.State, "lost");
            Equal (TowerGroundWatch.Action.None, g.Step (2.0, Ground ()), "lost is final");
        }

        static void TestGroundStatus ()
        {
            var w = TowerGroundWatch.Pose (100.0);
            w.Step (100.0, Ground (packed: true));
            w.Step (100.5, Ground ());
            w.MarkMade (100.5, TowerGroundWatch.Select (V5 (20000.0)), 3, V5Parts);
            w.Shift (101.0, 3243.0);
            w.MarkRemadeAfterShift (101.0, TowerGroundWatch.Select (V5 (20000.0)), 3, V5Parts);
            w.Step (101.5, Ground (joints: 2));
            var s = w.Status (102.0, 1, 0, 3, 3243.0, 1, 0);
            Equal (TowerGroundWatch.StatusLength, s.Count, "status length");
            Equal (TowerGroundWatch.StatusLength, TowerGroundWatch.Fields.Length, "fields length");
            var names = TowerGroundWatch.Fields;
            Func<string, double> f = n => s [Array.IndexOf (names, n)];
            Near (1, f ("protocol"), 0, "protocol");
            Near (TowerGroundWatch.StateHeld, f ("state"), 0, "state");
            Near (3, f ("joints"), 0, "joints (Unity side)");
            Near (3, f ("parts_fixed"), 0, "parts fixed");
            Near (3, f ("kjr_parts"), 0, "KJR's picks");
            Near (0, f ("kjr_excluded"), 0, "none excluded");
            Near (2, f ("hinges"), 0, "hinges");
            Near (2, f ("hinges_fixed"), 0, "hinges fixed");
            Near (0, f ("hinges_skipped"), 0, "hinges skipped");
            Near (176.8, f ("mass_fixed_tonnes"), 1e-9, "mass fixed");
            Near (1, f ("makes"), 0, "makes");
            Near (1, f ("remade_after_shift"), 0, "remade after a shift");
            Near (1, f ("shifts"), 0, "shifts");
            Near (1, f ("broken"), 0, "broken");
            Near (0, f ("releases"), 0, "releases");
            Near (2.0, f ("elapsed_s"), 1e-12, "elapsed");
            Near (0.5, f ("held_after_s"), 1e-12, "held after");
            Near (1, f ("loaded"), 0, "loaded (Unity side)");
            Near (0, f ("packed"), 0, "packed (Unity side)");
            Near (3243.0, f ("origin_distance_m"), 0, "origin distance (Unity side)");
            Near (1, f ("frame_rotating"), 0, "rotating frame (Unity side)");
            Near (0, f ("frame_speed_ms"), 0, "frame speed (Unity side)");
            Near (3243.0, f ("last_shift_m"), 0, "last shift");
            var fresh = TowerGroundWatch.Pose (0.0).Status (0.0, 0, double.NaN, 0, double.NaN, 1, 0);
            Check (double.IsNaN (fresh [3]) && double.IsNaN (fresh [9]) && double.IsNaN (fresh [16]),
                   "before any make: NaN selection counts and held_after_s");
            var none = TowerGroundWatch.NoneStatus (1.0, 0.0, 412.0, 1.0, 0.0);
            Equal (TowerGroundWatch.StatusLength, none.Count, "none: length");
            Near (TowerGroundWatch.StateNone, none [1], 0, "none: state");
            Near (0, none [2], 0, "none: no joint");
            Check (double.IsNaN (none [3]) && double.IsNaN (none [9]) && double.IsNaN (none [15]),
                   "none: NaN counts of a selection and times");
            Near (0, none [10] + none [11] + none [12] + none [13] + none [14], 0, "none: zero counters");
            Near (412.0, none [19], 0, "none: origin distance");
        }

        static void TestGroundPartsTable ()
        {
            var s = TowerGroundWatch.Select (V5 (20000.0));
            var order = TowerGroundWatch.RowOrder (s);
            Equal ("0,10,19", Ids (order), "rows: the fixed parts (KJR's picks are the hinges here)");
            var rows = new List<TowerGroundWatch.PartRow> ();
            foreach (int i in order)
                rows.Add (new TowerGroundWatch.PartRow {
                    PartPersistentId = (uint) (1000 + i), PartFlightId = (uint) (2000 + i),
                    Name = i == 0 ? "TG.TowerBase" : "TG.TowerHinge", PhysicsMass = i == 0 ? 160.0 : 8.4,
                    Kjr = s.Kjr [i], Hinge = s.Hinge [i], Mobile = s.Mobile [i], Held = true
                });
            var t = TowerGroundWatch.PartsTable (2637070260u, "Tour\tCatch Nord", TowerGroundWatch.StateHeld, rows);
            Equal (5, t.Count, "header, columns, three rows");
            Equal ("protocol=1\tschema=1\tpersistent_id=2637070260\tname=Tour Catch Nord\tstate=2\tn=3", t [0], "header");
            Equal (string.Join ("\t", TowerGroundWatch.PartsColumns), t [1], "columns");
            Equal ("1000\t2000\tTG.TowerBase\t160\t1\t0\t0\t1", t [2], "the root");
            Equal ("1010\t2010\tTG.TowerHinge\t8.4\t1\t1\t0\t1", t [3], "a hinge");
            // a light motor: the hinges are fixed by the hinge rule, the mast sections by KJR's
            var light = TowerGroundWatch.Select (V5 (1200.0));
            Equal ("0,4,2,10,19", Ids (TowerGroundWatch.RowOrder (light)), "rows of the light motor");
            // an inverted hinge: listed although not fixed
            var l = V5 (20000.0);
            var h = l [V5HingeN];
            h.ParentSide = TowerGroundWatch.SideMoving;
            l [V5HingeN] = h;
            Equal ("0,19,10", Ids (TowerGroundWatch.RowOrder (TowerGroundWatch.Select (l))),
                   "fixed first, then the skipped hinge (also KJR's excluded pick)");
            var empty = TowerGroundWatch.PartsTable (5u, "", TowerGroundWatch.StateNone, null);
            Equal (2, empty.Count, "no rows: header and columns");
            Equal (0, TowerGroundWatch.RowOrder (null).Count, "no selection: no row");
        }

        static void TestGroundLayouts ()
        {
            string said = TowerLayoutDeclaration.DeclareGroundInto (typeof (LayoutRegistry));
            Check (said.StartsWith ("layout tower_ground_v1 declared", StringComparison.Ordinal), "declared: " + said);
            Check (said.Contains ("layout tower_ground_parts_v1 declared"), "parts declared: " + said);
            Equal (0, LayoutRegistry.Problems ().Count, "the descriptors fit their constants");
            var lines = LayoutRegistry.Lines (TowerLayoutDeclaration.GroundLayout);
            var head = Header (lines [0]);
            Equal ("tower_ground_v1", head ["layout"], "layout");
            Equal ("TowerSite", head ["service"], "service");
            Equal ("1", head ["layout_protocol"], "protocol");
            Equal ("1", head ["layout_schema"], "schema");
            Equal (LayoutRegistry.KindStatus, head ["kind"], "kind");
            Equal (TowerGroundWatch.StatusLength.ToString (), head ["fields"], "fields");
            Equal ("#section\tstatus\t1\t" + TowerGroundWatch.StatusLength + "\t0\t", lines [1], "one section, read once");
            var units = new Dictionary<string, string> ();
            for (int i = 0; i < TowerGroundWatch.Fields.Length; i++) {
                var cells = lines [2 + i].Split ('\t');
                Equal (TowerGroundWatch.Fields [i], cells [2], "field " + i);
                units [cells [2]] = cells [3];
            }
            Equal ("t", units ["mass_fixed_tonnes"], "unit of mass_fixed_tonnes");
            Equal ("m/s", units ["frame_speed_ms"], "unit of frame_speed_ms");
            Equal ("m", units ["origin_distance_m"], "unit of origin_distance_m");
            Equal ("s", units ["elapsed_s"], "unit of elapsed_s");
            Equal ("", units ["joints"], "joints: a count");
            var parts = LayoutRegistry.Lines (TowerLayoutDeclaration.GroundPartsLayout);
            var ph = Header (parts [0]);
            Equal ("tower_ground_parts_v1", ph ["layout"], "parts layout");
            Equal (LayoutRegistry.KindText, ph ["kind"], "parts kind");
            Equal ("TowerSite", ph ["service"], "parts service");
        }
    }
}
