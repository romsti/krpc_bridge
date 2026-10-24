// Lot 3, capacity 3.1: the pure replica of a stock ModuleControlSurface (SurfaceModel.cs) against
// independent hand computations, its column against the analytic derivative, the command law and
// its gradient, the actuator step, the identity metric, the response-lag rule, the mono-flight
// scale guard, and the free-torque input of the law (born false: the allocation unchanged without
// it). Writes nothing into the golden vectors.

using System;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    internal static class SurfaceTests
    {
        static int checks;
        static int failures;

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.WriteLine ("FAIL (surfaces): " + what);
            }
        }

        static bool Near (double a, double b, double tol)
        {
            return Math.Abs (a - b) <= tol * Math.Max (1.0, Math.Max (Math.Abs (a), Math.Abs (b)));
        }

        /// <summary>Smooth analytic curves (the game's are FloatCurves; any function will do here).</summary>
        sealed class Curves : ISurfaceCurves
        {
            internal double DragBase = 0.2;
            public double Lift (double x) { return Math.Sin (2.0 * x) * 0.9 + 0.1 * x; }
            public double LiftMach (double m) { return 1.0 - 0.2 * m; }
            public double Drag (double x) { return DragBase + 1.1 * x * x; }
            public double DragMach (double m) { return 1.0 + 0.3 * m; }
        }

        internal static int Run (out int checkCount)
        {
            CommandLaw ();
            Actuator ();
            Forces ();
            ColumnAgainstAnalytic ();
            IdentityMetric ();
            Lag ();
            ScaleGuard ();
            ProvenAxes ();
            RotationBand ();
            StepUncertainty ();
            TorqueFreeInput ();
            checkCount = checks;
            return failures;
        }

        static SurfaceCommand Command ()
        {
            return new SurfaceCommand {
                CPitch = 1.0, CYaw = 0.0, Flip = false, KRoll = 0.0, Range = 35.0, Authority = 100.0,
                Direction = 1.0, DeployTerm = 0.0, Speed = 40.0, Exponential = false
            };
        }

        static void CommandLaw ()
        {
            var c = Command ();
            Check (Near (SurfaceModel.Target (c, 0.5, 0, 0), 17.5, 1e-12), "pitch 0.5 -> 17.5 deg");
            Check (Near (SurfaceModel.Target (c, 2.0, 0, 0), 35.0, 1e-12), "inner clamp at the range");
            c.Flip = true;
            Check (Near (SurfaceModel.Target (c, 0.5, 0, 0), -17.5, 1e-12), "CoM below: sign flips");
            c.Flip = false;
            c.KRoll = 2.0;
            Check (Near (SurfaceModel.Target (c, 0, 0.3, 0), -21.0, 1e-12), "roll term subtracted");
            c.KRoll = 0.0;
            c.Direction = 0.0;
            Check (SurfaceModel.Target (c, 0.7, 0.2, -0.4) == 0.0, "deflectionDirection 0 (held): target 0");
            c.Direction = -1.0;
            Check (Near (SurfaceModel.Target (c, 0.5, 0, 0), 17.5, 1e-12), "direction -1: squared out of the command");
            c.DeployTerm = 10.0;
            // -1 * clamp(-1 * 0.5 * 35 + 10) = -1 * (-7.5) = 7.5
            Check (Near (SurfaceModel.Target (c, 0.5, 0, 0), 7.5, 1e-12), "deploy term inside the outer clamp, direction outside");
            c.Direction = 1.0;
            c.Authority = 150.0;
            c.DeployTerm = 30.0;
            Check (Near (SurfaceModel.Target (c, 1.0, 0, 0), 52.5, 1e-12), "outer clamp at 1.5 range");
            c.DeployTerm = 0.0;
            c.Authority = 100.0;
            c.CYaw = -0.5;
            Check (Near (SurfaceModel.Target (c, 0.2, 0, 0.4), 0.0, 1e-12), "pitch and yaw projected on the hinge");

            // Gradient against finite differences, away from the clamps; zero at a clamp.
            var rng = new Random (31);
            var g = new double[3];
            for (int trial = 0; trial < 200; trial++) {
                var k = new SurfaceCommand {
                    CPitch = rng.NextDouble () * 2 - 1, CYaw = rng.NextDouble () * 2 - 1,
                    Flip = rng.NextDouble () < 0.5, KRoll = rng.NextDouble () * 3 - 1.5,
                    Range = 10 + 30 * rng.NextDouble (), Authority = 20 + 80 * rng.NextDouble (),
                    Direction = rng.NextDouble () < 0.5 ? 1.0 : -1.0, DeployTerm = 0.0, Speed = 40
                };
                double p = rng.NextDouble () * 0.6 - 0.3, r = rng.NextDouble () * 0.6 - 0.3,
                    y = rng.NextDouble () * 0.6 - 0.3;
                SurfaceModel.TargetGradient (k, p, r, y, g);
                double h = 1e-6;
                double fp = (SurfaceModel.Target (k, p + h, r, y) - SurfaceModel.Target (k, p - h, r, y)) / (2 * h);
                double fr = (SurfaceModel.Target (k, p, r + h, y) - SurfaceModel.Target (k, p, r - h, y)) / (2 * h);
                double fy = (SurfaceModel.Target (k, p, r, y + h) - SurfaceModel.Target (k, p, r, y - h)) / (2 * h);
                double x = (k.Flip ? -1 : 1) * (k.CPitch * p + k.CYaw * y) - k.KRoll * r;
                if (Math.Abs (x) < 0.999) {
                    Check (Near (g [0], fp, 1e-6) && Near (g [1], fr, 1e-6) && Near (g [2], fy, 1e-6),
                        "gradient = finite differences (trial " + trial + ")");
                } else if (Math.Abs (x) > 1.001) {
                    Check (g [0] == 0 && g [1] == 0 && g [2] == 0, "saturated: zero gradient (trial " + trial + ")");
                    Check (fp == 0 && fr == 0 && fy == 0, "saturated: the target does not move");
                }
            }
        }

        static void Actuator ()
        {
            var c = Command ();
            Check (Near (SurfaceModel.Step (c, 0.0, 10.0, 0.02), 0.8, 1e-12), "40 deg/s: 0.8 deg per step");
            Check (SurfaceModel.Step (c, 0.0, 0.5, 0.02) == 0.5, "reached inside the step");
            Check (Near (SurfaceModel.Step (c, 5.0, -10.0, 0.02), 4.2, 1e-12), "toward a lower target");
            c.Exponential = true;
            Check (Near (SurfaceModel.Step (c, 0.0, 10.0, 0.02), 8.0, 1e-12), "exponential: Lerp by speed dt");
            c.Speed = 100.0;
            Check (SurfaceModel.Step (c, 0.0, 10.0, 0.02) == 10.0, "Lerp clamped at 1");
            Check (SurfaceModel.MoveTowards (1.0, 1.0, 0.0) == 1.0, "MoveTowards at the target");
            Check (SurfaceModel.Sign (0.0) == 1.0 && SurfaceModel.Sign (-0.0) == 1.0, "Mathf.Sign(0) = 1");
        }

        static SurfaceFlow Flow (double alphaDeg)
        {
            var f = new SurfaceFlow {
                Area = 0.9, LiftCoeff = 0.5, LiftMultiplier = 0.036, DragMultiplier = 0.015,
                PerpendicularOnly = false, Omnidirectional = true, UseDrag = true, NodeBlocked = false,
                QLift = 20000.0, QDrag = 21000.0, Mach = 0.8
            };
            double a = alphaDeg * Math.PI / 180.0;
            // The flow comes along -y with an incidence alpha toward +z; the lift axis is +z.
            f.NVel [0] = 0.0; f.NVel [1] = -Math.Cos (a); f.NVel [2] = Math.Sin (a);
            f.LiftVector0 [0] = 0.0; f.LiftVector0 [1] = 0.0; f.LiftVector0 [2] = 1.0;
            f.Hinge [0] = 1.0; f.Hinge [1] = 0.0; f.Hinge [2] = 0.0;
            f.LeverLift [0] = 2.0; f.LeverLift [1] = 20.0; f.LeverLift [2] = 0.5;
            f.LeverDrag [0] = 2.1; f.LeverDrag [1] = 19.5; f.LeverDrag [2] = 0.4;
            return f;
        }

        /// <summary>The module's force, written out independently of SurfaceModel.Forces.</summary>
        static void Hand (SurfaceFlow f, Curves c, double deltaDeg, double[] lift, double[] drag)
        {
            double[] lv0 = f.LiftVector0, n = f.NVel;
            double d0 = n [0] * lv0 [0] + n [1] * lv0 [1] + n [2] * lv0 [2];
            double a0 = Math.Abs (d0);
            double t = deltaDeg * Math.PI / 180.0;
            // About +x (the hinge): y' = y cos - z sin, z' = y sin + z cos.
            double[] lv = { lv0 [0], lv0 [1] * Math.Cos (t) - lv0 [2] * Math.Sin (t), lv0 [1] * Math.Sin (t) + lv0 [2] * Math.Cos (t) };
            double d1 = n [0] * lv [0] + n [1] * lv [1] + n [2] * lv [2];
            double a1 = Math.Abs (d1);
            double s0 = f.QLift * f.LiftMultiplier * (d0 >= 0 ? 1 : -1) * c.Lift (a0) * c.LiftMach (f.Mach) * f.LiftCoeff;
            double s1 = f.QLift * f.LiftMultiplier * (d1 >= 0 ? 1 : -1) * c.Lift (a1) * c.LiftMach (f.Mach) * f.LiftCoeff;
            double q0 = f.QDrag * f.DragMultiplier * c.Drag (a0) * c.DragMach (f.Mach) * f.LiftCoeff;
            double q1 = f.QDrag * f.DragMultiplier * c.Drag (a1) * c.DragMach (f.Mach) * f.LiftCoeff;
            for (int k = 0; k < 3; k++) {
                lift [k] = -lv0 [k] * s0 * (1 - f.Area) - lv [k] * s1 * f.Area;
                drag [k] = -n [k] * q0 * (1 - f.Area) - n [k] * q1 * f.Area;
            }
        }

        static void Forces ()
        {
            var c = new Curves ();
            var lift = new double[3];
            var drag = new double[3];
            var hl = new double[3];
            var hd = new double[3];
            foreach (double alpha in new[] { -30.0, -5.0, 0.0, 3.0, 12.0, 40.0 })
                foreach (double delta in new[] { -35.0, -10.0, -0.5, 0.0, 0.7, 20.0, 35.0 }) {
                    var f = Flow (alpha);
                    SurfaceModel.Forces (f, c, delta, true, lift, drag);
                    Hand (f, c, delta, hl, hd);
                    bool same = true;
                    for (int k = 0; k < 3; k++)
                        same &= Near (lift [k], hl [k], 1e-12) && Near (drag [k], hd [k], 1e-12);
                    Check (same, "replica = hand computation (alpha " + alpha + ", delta " + delta + ")");
                }
            // Rotation: +z about +x by 90 deg is -y (Quaternion.AngleAxis * v, q v q*).
            var v = new double[3];
            SurfaceModel.Rotate (new[] { 0.0, 0.0, 1.0 }, new[] { 2.0, 0.0, 0.0 }, 90.0, v);
            Check (Near (v [0], 0, 1e-15) && Near (v [1], -1, 1e-15) && Near (v [2], 0, 1e-15), "rotation sense, axis normalized");
            // Not rotated (localRotation == neutral): the moving part uses the base axis.
            {
                var f = Flow (12.0);
                SurfaceModel.Forces (f, c, 0.1, false, lift, drag);
                Hand (f, c, 0.0, hl, hd);
                Check (Near (lift [2], hl [2], 1e-12) && Near (drag [1], hd [1], 1e-12), "not rotated: as at zero deflection");
            }
            // One-sided surface: absDot = clamp01 for the fixed part only; |f| for the moving part.
            {
                var f = Flow (-20.0);
                f.Omnidirectional = false;
                SurfaceModel.Forces (f, c, 0.0, false, lift, drag);
                // liftDot < 0: absDot 0 -> Lift(0) = 0, sign -1: no lift at all when not rotated.
                Check (lift [0] == 0 && lift [1] == 0 && Near (lift [2], 0, 1e-12), "one-sided: no lift behind");
                SurfaceModel.Forces (f, c, 5.0, true, lift, drag);
                Check (Math.Abs (lift [2]) > 0, "one-sided: the turned part uses |f|");
            }
            // perpendicularOnly: the lift has no component along the flow.
            {
                var f = Flow (15.0);
                f.PerpendicularOnly = true;
                SurfaceModel.Forces (f, c, 10.0, true, lift, drag);
                double along = lift [0] * f.NVel [0] + lift [1] * f.NVel [1] + lift [2] * f.NVel [2];
                Check (Math.Abs (along) < 1e-9 * Math.Max (1, Math.Abs (lift [2])), "perpendicularOnly: lift normal to the flow");
            }
            // No drag model: no drag; node lift: nothing.
            {
                var f = Flow (10.0);
                f.UseDrag = false;
                SurfaceModel.Forces (f, c, 10.0, true, lift, drag);
                Check (drag [0] == 0 && drag [1] == 0 && drag [2] == 0 && lift [2] != 0, "useInternalDragModel off: no drag");
                f.NodeBlocked = true;
                SurfaceModel.Forces (f, c, 10.0, true, lift, drag);
                Check (lift [0] == 0 && lift [1] == 0 && lift [2] == 0, "node lift blocked: nothing");
            }
            // Torque about the CoM: lever x force for the lift and the drag.
            {
                var f = Flow (8.0);
                SurfaceModel.Forces (f, c, 12.0, true, lift, drag);
                var t = new double[3];
                SurfaceModel.Torque (f, lift, drag, t);
                double[] l = f.LeverLift, d = f.LeverDrag;
                double tx = l [1] * lift [2] - l [2] * lift [1] + d [1] * drag [2] - d [2] * drag [1];
                double ty = l [2] * lift [0] - l [0] * lift [2] + d [2] * drag [0] - d [0] * drag [2];
                double tz = l [0] * lift [1] - l [1] * lift [0] + d [0] * drag [1] - d [1] * drag [0];
                Check (Near (t [0], tx, 1e-12) && Near (t [1], ty, 1e-12) && Near (t [2], tz, 1e-12), "torque = lever x force");
            }
            // Zero dynamic pressure: no force (the pad).
            {
                var f = Flow (10.0);
                f.QLift = 0.0;
                f.QDrag = 0.0;
                SurfaceModel.Forces (f, c, 30.0, true, lift, drag);
                Check (lift [2] == 0 && drag [1] == 0, "q = 0: no force");
            }
        }

        static void ColumnAgainstAnalytic ()
        {
            var c = new Curves ();
            var column = new double[3];
            var fine = new double[3];
            var tp = new double[3];
            var tm = new double[3];
            foreach (double alpha in new[] { -10.0, 4.0, 25.0 })
                foreach (double delta in new[] { -20.0, -3.0, 6.0, 30.0 }) {
                    var f = Flow (alpha);
                    SurfaceModel.Column (f, c, delta, column);
                    double h = 1e-4;
                    SurfaceModel.TorqueAt (f, c, delta + h, tp);
                    SurfaceModel.TorqueAt (f, c, delta - h, tm);
                    for (int k = 0; k < 3; k++)
                        fine [k] = (tp [k] - tm [k]) / (2 * h);
                    double scale = Math.Sqrt (fine [0] * fine [0] + fine [1] * fine [1] + fine [2] * fine [2]);
                    bool ok = true;
                    for (int k = 0; k < 3; k++)
                        ok &= Math.Abs (column [k] - fine [k]) <= 1e-4 * scale + 1e-9;
                    Check (ok, "column (+-0.1 deg) = derivative (alpha " + alpha + ", delta " + delta + ")");
                }
            // the torque in the state the module has at a deflection. Turned is the default;
            // not turned is the force at zero deflection (the moving part's lift vector is lv0), so
            // the step at Unity's threshold is the turned torque at the threshold minus that one.
            var lift = new double[3];
            var drag = new double[3];
            var t0 = new double[3];
            var t1 = new double[3];
            foreach (double alpha in new[] { -10.0, 4.0, 25.0 }) {
                var f = Flow (alpha);
                SurfaceModel.TorqueAt (f, c, 0.16, t0);
                SurfaceModel.TorqueAt (f, c, 0.16, true, t1);
                Check (t0 [0] == t1 [0] && t0 [1] == t1 [1] && t0 [2] == t1 [2], "TorqueAt: turned by default (alpha " + alpha + ")");
                SurfaceModel.TorqueAt (f, c, 0.16, false, t1);
                SurfaceModel.Forces (f, c, 0.16, false, lift, drag);
                SurfaceModel.Torque (f, lift, drag, tp);
                Check (t1 [0] == tp [0] && t1 [1] == tp [1] && t1 [2] == tp [2], "TorqueAt not turned = Forces(not turned) (alpha " + alpha + ")");
                SurfaceModel.TorqueAt (f, c, 0.0, true, tm);
                bool same = true;
                for (int k = 0; k < 3; k++)
                    same &= Math.Abs (t1 [k] - tm [k]) <= 1e-12 * (1 + Math.Abs (tm [k]));
                Check (same, "not turned at 0.16 deg = turned at 0 (alpha " + alpha + ")");
            }
        }

        static void IdentityMetric ()
        {
            var a = new[] { 100.0, 0.0, 0.0 };
            var b = new[] { 0.0, 50.0, 0.0 };
            Check (SurfaceModel.Identity (a, b, a, b, 1.0) == 0.0, "identical: 0");
            var a2 = new[] { 100.1, 0.0, 0.0 };
            Check (Near (SurfaceModel.Identity (a2, b, a, b, 1.0), 0.1 / 150.0, 1e-12), "relative to |L| + |D|");
            var z = new[] { 0.0, 0.0, 0.0 };
            Check (double.IsNaN (SurfaceModel.Identity (a, b, z, z, 1.0)), "below the floor: not judged");
        }

        static void Lag ()
        {
            Check (SurfaceModel.LagOutcome (10.0, 10.0, 9.0) == 0, "answered this step's command");
            Check (SurfaceModel.LagOutcome (9.0, 10.0, 9.0) == 1, "answered the previous step's");
            Check (SurfaceModel.LagOutcome (5.0, 10.0, 9.0) == 2, "neither");
            Check (SurfaceModel.LagOutcome (10.0, 10.0, 10.01) == -1, "too close to tell");
            Check (SurfaceModel.LagOutcome (double.NaN, 10.0, 9.0) == -1, "NaN: not told");
            Check (!SurfaceModel.LagEstablished (24, 0, 0) && SurfaceModel.LagEstablished (25, 0, 0) &&
                SurfaceModel.LagEstablished (40, 2, 0) && !SurfaceModel.LagEstablished (40, 1, 2), "established rule");
        }

        static void ScaleGuard ()
        {
            var g = new SurfaceScaleGuard ();
            var p = new double[3];
            var r = new double[3];
            // the third argument is each axis's uncertainty; a step counts when it is below
            // IdentityTolerance x |predicted| (here |predicted| above 10 N.m).
            var thr = new[] { 0.01, 0.01, 0.01 };
            var rng = new Random (7);
            for (int i = 0; i < 200; i++) {
                p [0] = 100.0 * Math.Sin (0.1 * i);
                r [0] = p [0] * (1.0 + 0.05 * (rng.NextDouble () - 0.5));
                p [1] = 0.5;          // not predicted to the replica's precision: never counted
                r [1] = 50.0;
                p [2] = 80.0 * Math.Cos (0.07 * i);
                r [2] = 2.6 * p [2];  // the model under-predicts by 2.6: tripped
                g.Update (p, r, thr);
            }
            Check (Near (g.K (0), 1.0, 0.02) && (g.Tripped & 1) == 0, "k = 1 when the model holds");
            Check (g.Steps [1] == 0 && g.K (1) == 1.0 && (g.Tripped & 2) == 0, "no excitation: no evidence, k = 1");
            Check (Near (g.K (2), 2.6, 1e-9) && (g.Tripped & 4) != 0, "k out of [0.5, 2]: tripped");
            // Sticky until the reset (mono-flight: reset at every arming).
            for (int i = 0; i < 300; i++) {
                p [2] = 80.0 * Math.Cos (0.07 * i);
                r [2] = p [2];
                g.Update (p, r, thr);
            }
            Check ((g.Tripped & 4) != 0, "a tripped axis stays tripped for the arming");
            g.Reset ();
            Check (g.Tripped == 0 && g.K (0) == 1.0 && g.Steps [2] == 0, "reset: k = 1, no evidence");
            // Not before the evidence.
            var h = new SurfaceScaleGuard ();
            for (int i = 0; i < SurfaceModel.ScaleEvidenceSteps - 1; i++) {
                p [0] = 20.0;
                r [0] = 80.0;
                h.Update (p, r, thr);
            }
            Check ((h.Tripped & 1) == 0, "no trip before the evidence");
            h.Update (p, r, thr);
            Check ((h.Tripped & 1) != 0, "trip at the evidence");
            // The rule is strict, and an unknown uncertainty counts nothing.
            var u = new SurfaceScaleGuard ();
            p [0] = 10.0;
            r [0] = 10.0;
            u.Update (p, r, thr);
            thr [1] = double.NaN;
            p [1] = 1e6;
            r [1] = 1e6;
            u.Update (p, r, thr);
            Check (u.Steps [0] == 0 && u.Steps [1] == 0, "uncertainty = tolerance x |predicted|, or NaN: not counted");
            thr [1] = 0.01;
        }

        /// <summary>
        /// an axis is proven with the guard's ScaleEvidenceSteps counted steps and no trip; a
        /// surface contributes to the axes where its column is above the identity tolerance of its
        /// norm (whatever the command, its torque increment lies along the column).
        /// </summary>
        static void ProvenAxes ()
        {
            var g = new SurfaceScaleGuard ();
            Check (g.Judged () == 0 && g.Proven () == 0, "no evidence: nothing judged, nothing proven (k = 1 is not a proof)");
            var p = new double[3];
            var r = new double[3];
            var u = new[] { 0.0, 0.0, 0.0 };
            for (int i = 0; i < SurfaceModel.ScaleEvidenceSteps - 1; i++) {
                p [0] = 50.0 + i;
                r [0] = p [0];
                p [1] = 0.0;          // never excited: never counted
                r [1] = 0.0;
                p [2] = 30.0 + i;
                r [2] = 2.5 * p [2];  // a real defect
                g.Update (p, r, u);
            }
            Check (g.Judged () == 0 && g.Proven () == 0, "one step short of the evidence: nothing proven");
            g.Update (p, r, u);
            Check (g.Judged () == 5 && (g.Tripped & 4) != 0, "at the evidence: x and z judged, z tripped");
            Check (g.Proven () == 1, "proven: judged and not tripped (x only)");
            for (int i = 0; i < 400; i++) {
                p [0] = 50.0;
                r [0] = 50.0;
                g.Update (p, r, u);
            }
            Check (g.Proven () == 1 && g.Steps [1] == 0, "an axis never excited is never proven");
            g.Reset ();
            Check (g.Judged () == 0 && g.Proven () == 0, "reset at every arming: the proof goes with it");

            var c = new double[3];
            c [0] = 100.0; c [1] = 0.0; c [2] = 0.0;
            Check (SurfaceModel.ContributedAxes (c) == 1, "a pure pitch column: x only");
            c [1] = 0.05;
            Check (SurfaceModel.ContributedAxes (c) == 1, "a roll share under the identity tolerance: not an axis of it");
            c [1] = 0.2;
            Check (SurfaceModel.ContributedAxes (c) == 3, "a roll share above it: x and y");
            c [0] = 1000.0; c [1] = 1.0;
            Check (SurfaceModel.ContributedAxes (c) == 1, "at the tolerance exactly (1 of a norm above 1000): not above it");
            c [0] = -890.4; c [1] = -186.0; c [2] = 113.0;
            Check (SurfaceModel.ContributedAxes (c) == 7, "a grid fin (the replay's stock fin 0): all three");
            c [0] = 0.0; c [1] = 0.0; c [2] = 0.0;
            Check (SurfaceModel.ContributedAxes (c) == 7, "no column: every axis");
            c [0] = double.NaN;
            Check (SurfaceModel.ContributedAxes (c) == 7, "NaN column: every axis");
            c [0] = double.PositiveInfinity;
            Check (SurfaceModel.ContributedAxes (c) == 7, "infinite column: every axis");
            Check ((SurfaceModel.SanctionUnproven >> SurfaceModel.SanctionUnprovenShift) == 7 &&
                (SurfaceModel.SanctionUnproven & (SurfaceModel.SanctionScaleX | SurfaceModel.SanctionScaleY |
                SurfaceModel.SanctionScaleZ | SurfaceModel.SanctionLag | SurfaceModel.SanctionIdentity |
                SurfaceModel.SanctionNotArmed)) == 0, "the unproven bits are new bits 6-8");
        }

        /// <summary>
        /// Unity's rotation band, computed from 0.999999f and the float spacing; its edges
        /// hold every flip and every bare/stored disagreement of a float emulation of the module's
        /// chain (AngleAxis, product, the transform's storage renormalized, Dot rounded at its
        /// return) over many neutrals.
        /// </summary>
        static void RotationBand ()
        {
            double lo = SurfaceModel.RotationBandLoDeg, hi = SurfaceModel.RotationBandHiDeg;
            double t = SurfaceModel.RotationThresholdDeg;
            Check (Math.Abs (t - 0.16071) < 5e-5, "exact flip of Unity's test at 0.1607 deg: " + t);
            Check (Math.Abs (lo - 0.1399) < 2e-4 && Math.Abs (hi - 0.1791) < 2e-4, "band 0.140-0.179 deg: " + lo + " " + hi);
            // The flights: the module turned at 0.16226, not at 0.16611 (102058, 230852); an earlier change's
            // three captures at 0.1639-0.1653; the step at +-0.157 and +-0.173.
            foreach (double d in new[] { 0.16226, 0.16611, 0.1639, 0.1653, 0.157, -0.1574, 0.173 })
                Check (SurfaceModel.InRotationBand (d), "in the band: " + d);
            foreach (double d in new[] { 0.0, 0.12, -0.139, 0.18, 0.75, -35.0, double.NaN })
                Check (!SurfaceModel.InRotationBand (d), "outside the band: " + d);
            // Float emulation (the CLR rounds every float operation, as a Mono store does).
            var rng = new Random (159);
            int flips = 0, disagreements = 0, outside = 0;
            float th = SurfaceModel.UnityEqualDot;
            for (int n = 0; n < 60; n++) {
                float[] neutral = n == 0 ? new[] { 0f, 0f, 0f, 1f } : AxisAngle ((float)(rng.NextDouble () * 360.0),
                    rng.NextDouble () - 0.5, rng.NextDouble () - 0.5, rng.NextDouble () - 0.5);
                bool havePrevious = false, previousBare = false, previousStored = false;
                for (int i = 0; i <= 1200; i++) {
                    float delta = (float)(0.120 + 1e-4 * i) * (n % 2 == 0 ? 1f : -1f);
                    float[] q = Multiply (AxisAngle (delta, 1.0, 0.0, 0.0), neutral);
                    bool bare = !(Dot (q, neutral) > th);
                    bool stored = !(Dot (Normalize (q), neutral) > th);
                    bool inBand = SurfaceModel.InRotationBand (delta);
                    if (bare != stored) {
                        disagreements++;
                        if (!inBand)
                            outside++;
                    }
                    if (havePrevious && (bare != previousBare || stored != previousStored)) {
                        flips++;
                        if (!inBand && !SurfaceModel.InRotationBand (delta - 1e-4f * Math.Sign (delta)))
                            outside++;
                    }
                    havePrevious = true;
                    previousBare = bare;
                    previousStored = stored;
                }
            }
            Check (flips > 60 && disagreements > 0 && outside == 0, "every flip and disagreement inside the band: flips " +
                flips + ", disagreements " + disagreements + ", outside " + outside);
            Console.WriteLine ("  rotation band " + lo.ToString ("F5") + "-" + hi.ToString ("F5") + " deg (flip " +
                t.ToString ("F5") + "): " + flips + " flips and " + disagreements +
                " bare/stored disagreements over 60 neutrals, " + outside + " outside");
        }

        // Float quaternions (x, y, z, w), every operation rounded to float.
        static float[] AxisAngle (float degrees, double ax, double ay, double az)
        {
            double n = Math.Sqrt (ax * ax + ay * ay + az * az);
            float x = (float)(ax / n), y = (float)(ay / n), z = (float)(az / n);
            double half = degrees * Math.PI / 360.0;
            float s = (float)Math.Sin (half);
            return new[] { x * s, y * s, z * s, (float)Math.Cos (half) };
        }

        static float[] Multiply (float[] a, float[] b)
        {
            return new[] {
                a [3] * b [0] + a [0] * b [3] + a [1] * b [2] - a [2] * b [1],
                a [3] * b [1] - a [0] * b [2] + a [1] * b [3] + a [2] * b [0],
                a [3] * b [2] + a [0] * b [1] - a [1] * b [0] + a [2] * b [3],
                a [3] * b [3] - a [0] * b [0] - a [1] * b [1] - a [2] * b [2]
            };
        }

        static float[] Normalize (float[] q)
        {
            float m = (float)Math.Sqrt (q [0] * q [0] + q [1] * q [1] + q [2] * q [2] + q [3] * q [3]);
            return new[] { q [0] / m, q [1] / m, q [2] / m, q [3] / m };
        }

        /// <summary>Unity's Dot: the sum (double under Mono) rounded to float at the return.</summary>
        static float Dot (float[] a, float[] b)
        {
            return (float)((double)a [0] * b [0] + (double)a [1] * b [1] + (double)a [2] * b [2] + (double)a [3] * b [3]);
        }

        /// <summary>
        /// the step uncertainty's pieces, and the flight's case on the guard -- a weakly
        /// excited axis whose only counted step was a turned-state jump at the threshold.
        /// </summary>
        static void StepUncertainty ()
        {
            var c = new Curves ();
            var jump = new double[3];
            var t1 = new double[3];
            var t0 = new double[3];
            foreach (double alpha in new[] { -10.0, 4.0, 25.0 }) {
                var f = Flow (alpha);
                SurfaceModel.TurnJump (f, c, 0.16, jump);
                SurfaceModel.TorqueAt (f, c, 0.16, true, t1);
                SurfaceModel.TorqueAt (f, c, 0.16, false, t0);
                Check (jump [0] == t1 [0] - t0 [0] && jump [1] == t1 [1] - t0 [1] && jump [2] == t1 [2] - t0 [2],
                    "TurnJump = turned - not turned (alpha " + alpha + ")");
                var col = new double[3];
                SurfaceModel.Column (f, c, 0.16, col);
                Check (Math.Abs (jump [0]) > 0.5 * Math.Abs (col [0] * 0.16), "the jump is about column x 0.16 deg (alpha " + alpha + ")");
            }
            Check (Near (SurfaceModel.DeflectionResolution (0.75, 0.7500001, 0.74, 0.7500001, 0.75),
                1e-7 + 2.0 * 0.7500001 / 16777216.0, 1e-9), "deflection resolution: command gap + one float spacing");
            Check (double.IsPositiveInfinity (SurfaceModel.DeflectionResolution (double.NaN, 1.0, 1.0, 1.0, 1.0)),
                "deflection resolution: unknown -> infinite");
            // The flight (a recorded flight, roll): two fins at +-0.157 deg, roll column
            // 14.7 N.m/deg each; the prediction took one fin's turned-state jump (2.4 N.m), the
            // module did not: realized half the predicted. The rule before (1e-3 of the
            // authority, 3.9 N.m) counted that step alone: k_y 0.50.
            var old = new SurfaceScaleGuard ();
            var guard = new SurfaceScaleGuard ();
            var p = new[] { 0.0, 4.8, 0.0 };
            var r = new[] { 0.0, 2.4, 0.0 };
            var before = new[] { 3.9, 3.9, 3.9 };
            var uncertainty = new[] { 0.0, 2.0 * 14.7 * 0.157 + 1e-5, 0.0 };
            for (int i = 0; i < SurfaceModel.ScaleEvidenceSteps + 5; i++) {
                old.Update (p, r, new[] { -1.0, -1.0, -1.0 });
                guard.Update (p, r, uncertainty);
            }
            Check (old.Steps [1] > 0 && Near (old.K (1), 0.5, 1e-12), "without the band term the jump is counted: k 0.5");
            Check (guard.Steps [1] == 0 && guard.K (1) == 1.0 && guard.Tripped == 0, "with it: no evidence, k = 1");
            // A real scale defect, well excited (x 0.4 and x 2.5), small uncertainty: trips.
            foreach (double scale in new[] { 0.4, 2.5 }) {
                var d = new SurfaceScaleGuard ();
                for (int i = 0; i < SurfaceModel.ScaleEvidenceSteps; i++) {
                    double inc = 40.0 * Math.Sin (0.3 * i + 0.1);
                    d.Update (new[] { inc, inc, inc }, new[] { scale * inc, scale * inc, scale * inc }, new[] { 1e-4, 1e-4, 1e-4 });
                }
                Check (d.Tripped == 7 && Near (d.K (0), scale, 1e-12), "scale " + scale + " well excited: tripped on the three axes");
            }
        }

        static void TorqueFreeInput ()
        {
            // One step of the law with three ctrlState-like columns; with the flag off the target is
            // M_d - M (unchanged), with it on the free torque comes off, nothing else moves.
            AttitudeOutput o1 = RunLaw (false, 0.0), o2 = RunLaw (true, 0.0), o3 = RunLaw (true, 3.0);
            bool same = true;
            for (int a = 0; a < 3; a++)
                same &= o1.TorqueAllocTarget [a] == o2.TorqueAllocTarget [a] && o1.DeltaU [a] == o2.DeltaU [a];
            Check (same, "free torque 0 with the flag: bit-identical allocation");
            bool shifted = true;
            for (int a = 0; a < 3; a++)
                shifted &= Near (o3.TorqueAllocTarget [a], o1.TorqueAllocTarget [a] - 3.0 * (a + 1), 1e-12);
            Check (shifted, "free torque taken off the allocation target");
            Check (o3.Nu [0] == o1.Nu [0] && o3.Alpha [0] == o1.Alpha [0], "reference and authority untouched");
        }

        static AttitudeOutput RunLaw (bool useFree, double free)
        {
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            inp.Resize (3);
            inp.Dt = 0.02;
            for (int a = 0; a < 3; a++) {
                inp.Inertia [a] = a == 1 ? 2e4 : 1e5;
                inp.Torque [a] = 10.0 * a;
                for (int j = 0; j < 3; j++)
                    inp.B [a * 3 + j] = a == j ? -5000.0 : 0.0;
                inp.UMin [a] = -1.0;
                inp.UMax [a] = 1.0;
                inp.Priority [a] = 3.0;
                inp.TorqueFree [a] = free * (a + 1);
            }
            inp.UseTorqueFree = useFree;
            inp.OmegaDotValid = true;
            inp.ReferenceValid = true;
            inp.Error [0] = 0.02;
            inp.Error [2] = -0.01;
            // One step: the hedge has not integrated anything yet, so only the allocation can move.
            AttitudeMath.Step (g, inp, st, work, o);
            return o;
        }
    }
}
