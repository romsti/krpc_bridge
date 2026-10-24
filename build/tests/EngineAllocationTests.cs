// -----------------------------------------------------------------------------------
// option 4096 of AttitudeOptionsV1, the engine-by-engine allocation IN SHADOW
// (EngineAllocation.cs, CascadeWork). What these tests pin down:
//   * option 4096 is a new bit (supported 8191; the client) and changes NO existing rule
//     (blocking sets, write masks, frozen axes, neutralization, roll gate, reference age) for any older option set;
//   * the horizon arithmetic: phi = 1 - (1 - f)^H is KSP's Lerp held H steps (integer H, step by step); the box of the
//     realized change and the command that realizes it, both sides;
//   * one stage of the cascade (weighted rows only) equals AttitudeMath.SolveBoxLeastSquares on the same stacked
//     problem (random draws); with equality rows the KKT conditions hold and the solution matches the law's solver
//     with the equalities heavily weighted;
//   * the heavy booster (nine engines of 2 MN on a 3 m ring 10.8 m below the CoM, gimbals +-2 deg, an RCS ring 37 m
//     above): a reachable torque met exactly with D's axial force kept; translation WITHOUT inclining (a lateral
//     force with zero pitch/yaw torque: common tilt + differential + RCS); the strict priorities (an unreachable
//     lateral demand leaves pitch/yaw and the axial force exact, a main throttle at 1 leaves every throttle at 1);
//     the commands realize the allocation at the horizon (the Lerp simulated); the authority along each direction,
//     free and from the trim, in function of the thrust;
//   * no allocation per solve, the iteration counts, and the cost per step (printed).
// -----------------------------------------------------------------------------------

using System;
using System.Diagnostics;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static partial class EngineAllocationTests
    {
        static int checks;
        static int failures;

        const int Option = AttitudeWrite.OptionEngineAllocation;
        const int Armed = AttitudeWrite.ModeArmed;
        const int Both = AttitudeWrite.ArmRoll | AttitudeWrite.ArmPitchYaw;
        const int Earlyish = AttitudeWrite.StageEarlyish;


        public static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestBits ();
            TestOptionChangesNoExistingRule ();
            TestHorizon ();
            TestStageAgainstLaw ();
            TestStageEqualities ();
            TestHeavyTorque ();
            TestHeavyTranslation ();
            TestHeavyPriorities ();
            TestHeavyHorizon ();
            TestHeavyAuthority ();
            TestCost ();
            // Step 2 (option 8192): EngineAllocationWriteTests.cs.
            TestWriteBits ();
            TestDeadZoneArithmetic ();
            TestDeadZoneSolve ();
            TestDeadZoneHeavyLoop ();
            TestWriteRule ();
            TestWriteRuleSequence ();
            checkCount = checks;
            Console.WriteLine ("engine allocation checks: " + checks + ", failures: " + failures +
"");
            return failures;
        }

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.Error.WriteLine ("FAIL: " + what);
            }
        }

        // ------------------------------------------------------------------ the option

        static void TestBits ()
        {
            Check (Option == 4096 && (AttitudeWrite.OptionsSupported & 8191) == 8191,
                "option 4096, supported 8191 and above (mirrored by the client)");
            Check (AttitudeWrite.ValidOptions (4096) && AttitudeWrite.ValidOptions (8191) &&
                !AttitudeWrite.ValidOptions (AttitudeWrite.OptionsSupported + 1) && !AttitudeWrite.ValidOptions (-1),
                "valid options 0-8191 (8192 since step 2)");
            for (int o = 0; o < 4096; o++)
                Check ((o & Option) == 0, "no older option holds bit 4096 (" + o + ")");
            Check ((AttitudeWrite.AllowedWriteMask (Option) & ~AttitudeWrite.WriteMaskBase) == 0,
                "option 4096 allows no write-mask bit");
        }

        static void TestOptionChangesNoExistingRule ()
        {
            var rng = new Random (2641);
            for (int k = 0; k < 4096; k += 7) {
                int options = k;
                int o = options | Option;
                Check (AttitudeWrite.RollBlockingSet (o) == AttitudeWrite.RollBlockingSet (options),
                    "option 4096: roll blocking set of options " + options);
                Check (AttitudeWrite.PitchYawBlockingSet (o) == AttitudeWrite.PitchYawBlockingSet (options),
                    "option 4096: pitch/yaw blocking set of options " + options);
                Check (AttitudeWrite.AllowedWriteMask (o) == AttitudeWrite.AllowedWriteMask (options),
                    "option 4096: allowed write mask of options " + options);
                for (int w = -2; w < 32; w++)
                    Check (AttitudeWrite.ValidWriteMask (w, o) == AttitudeWrite.ValidWriteMask (w, options),
                        "option 4096: write mask " + w + " (options " + options + ")");
                for (int w = 0; w < 32; w++) {
                    Check (AttitudeWrite.EffectiveWriteMask (w, o) == AttitudeWrite.EffectiveWriteMask (w, options),
                        "option 4096: stored mask " + w);
                    foreach (double age in new[] { 0.1, 0.31, 1.0, double.PositiveInfinity, double.NaN }) {
                        Check (AttitudeWrite.PitchYawReferenceOld (age, w, o) ==
                            AttitudeWrite.PitchYawReferenceOld (age, w, options), "option 4096: reference age rule");
                        Check (AttitudeWrite.PitchYawReferenceHeld (age, w, o) ==
                            AttitudeWrite.PitchYawReferenceHeld (age, w, options), "option 4096: reference held");
                    }
                    for (int last = 0; last < 8; last++)
                        for (int stage = 1; stage < 3; stage++)
                            Check (AttitudeWrite.SurfaceOwnedAxes (Armed, Both, w, o, stage, last) ==
                                AttitudeWrite.SurfaceOwnedAxes (Armed, Both, w, options, stage, last),
                                "option 4096: surfaces' owned axes");
                }
            }
            for (int i = 0; i < 20000; i++) {
                long motif = ((long)rng.Next () << 31) ^ rng.Next () ^ ((long)rng.Next (0, 4096) << 32);
                if (i < 64)
                    motif = 1L << i % 44;
                int options = rng.Next (0, 4096);
                int o = options | Option;
                int unneutralizable = rng.Next (0, 8);
                Check (AttitudeWrite.FrozenCtrlAxes (o, motif, unneutralizable) ==
                    AttitudeWrite.FrozenCtrlAxes (options, motif, unneutralizable), "option 4096: frozen axes identical");
                int now = rng.Next (0, 8), last = rng.Next (0, 8), answering = rng.Next (0, 8);
                Check (AttitudeWrite.NeutralizeSurfaces (o, Armed, Earlyish, now, last, answering) ==
                    AttitudeWrite.NeutralizeSurfaces (options, Armed, Earlyish, now, last, answering),
                    "option 4096: neutralization identical");
                Check (AttitudeWrite.RollSurfaceGate (o, answering, (i & 1) == 0) ==
                    AttitudeWrite.RollSurfaceGate (options, answering, (i & 1) == 0), "option 4096: roll gate identical");
                for (int mode = 0; mode < 2; mode++)
                    for (int axes = 0; axes < 4; axes++)
                        for (int stage = 1; stage < 3; stage++) {
                            bool computed = (i & 2) == 0;
                            Check (AttitudeWrite.MayWritePitchYaw (mode, axes, stage, computed, motif, o) ==
                                AttitudeWrite.MayWritePitchYaw (mode, axes, stage, computed, motif, options),
                                "option 4096: MayWritePitchYaw identical");
                            Check (AttitudeWrite.MayWrite (mode, axes, stage, computed, motif, o) ==
                                AttitudeWrite.MayWrite (mode, axes, stage, computed, motif, options),
                                "option 4096: MayWrite identical");
                            Check (AttitudeWrite.ContactHold (o, mode, axes, stage, computed, true, motif) ==
                                AttitudeWrite.ContactHold (options, mode, axes, stage, computed, true, motif),
                                "option 4096: ContactHold identical");
                        }
            }
        }

        // ------------------------------------------------------------------ horizon

        static void TestHorizon ()
        {
            // phi is KSP's float Lerp held H steps.
            foreach (double f in new[] { 0.04, 0.16, 0.5, 0.9 }) {
                for (int h = 1; h <= 30; h++) {
                    float u = 0.2f, target = 0.9f;
                    for (int s = 0; s < h; s++)
                        u = u + (target - u) * (float)f;
                    double model = 0.2 + EngineAllocation.Phi (f, h) * (0.9 - 0.2);
                    Check (Math.Abs (model - u) < 1e-5, "phi = Lerp held " + h + " steps at f " + f + ": " + model + " vs " + u);
                }
            }
            Check (EngineAllocation.Phi (1.0, 3.0) == 1.0 && EngineAllocation.Phi (2.0, 3.0) == 1.0, "instant actuator: phi 1");
            Check (EngineAllocation.Phi (0.0, 3.0) == 0.0 && EngineAllocation.Phi (-1.0, 3.0) == 0.0 &&
                EngineAllocation.Phi (double.NaN, 3.0) == 0.0, "an actuator that does not move: phi 0");
            Check (EngineAllocation.Phi (0.3, double.PositiveInfinity) == 1.0, "infinite horizon: phi 1 (the law's convention)");
            double[] fs = { 0.16, 1.0, 0.04, 0.0, double.NaN };
            double h0 = EngineAllocation.Horizon (fs, fs.Length);
            Check (Math.Abs (h0 - GimbalSpool.HorizonSteps (0.04)) < 1e-12, "the horizon is the slowest time constant: " + h0);
            Check (EngineAllocation.Horizon (new[] { 1.0, 1.0 }, 2) == 1.0, "instant actuators: one step");
            Check (Math.Abs (EngineAllocation.Phi (0.04, h0) - (1.0 - 1.0 / Math.E)) < 0.01,
                "the slowest realizes 1 - 1/e at the horizon");
            // Both sides: the command realizes x exactly.
            var rng = new Random (11);
            for (int i = 0; i < 2000; i++) {
                double u0 = rng.NextDouble (), up = 0.05 + 0.95 * rng.NextDouble (), down = 0.05 + 0.95 * rng.NextDouble ();
                double c = rng.NextDouble ();
                double x = EngineAllocation.Realized (c, u0, up, down);
                double back = EngineAllocation.CommandOf (x, u0, up, down);
                Check (Math.Abs (back - c) < 1e-12, "command of the realized change, both sides");
                Check ((x >= 0.0) == (c >= u0), "the realized change has the command's side");
            }
        }

        // --------------------------------------------------------- one stage against the law's solver

        static void RandomStage (Random rng, int n, double[] a, double[] b, double[] r2, double[] p, double[] lo,
            double[] hi)
        {
            for (int j = 0; j < n; j++) {
                for (int r = 0; r < 6; r++)
                    a [6 * j + r] = rng.NextDouble () < 0.3 ? 0.0 : 2.0 * rng.NextDouble () - 1.0;
                double s = 0.05 + 0.5 * rng.NextDouble ();
                r2 [j] = s * s;
                lo [j] = -0.2 - rng.NextDouble ();
                hi [j] = 0.2 + rng.NextDouble ();
                p [j] = lo [j] + (hi [j] - lo [j]) * rng.NextDouble ();
            }
            for (int r = 0; r < 6; r++)
                b [r] = 4.0 * (2.0 * rng.NextDouble () - 1.0);
        }

        static double Objective (int n, double[] a, double[] b, bool[] rowIn, double[] r2, double[] p, double[] y,
            double weightEq, double[] q)
        {
            double f = 0.0;
            for (int r = 0; r < 6; r++) {
                if (!rowIn [r])
                    continue;
                double v = -b [r];
                for (int j = 0; j < n; j++)
                    v += a [6 * j + r] * y [j];
                f += (q [r] == 0.0 ? weightEq * weightEq : 1.0) * v * v;
            }
            for (int j = 0; j < n; j++)
                f += r2 [j] * (y [j] - p [j]) * (y [j] - p [j]);
            return f;
        }

        /// <summary>The same problem through AttitudeMath.SolveBoxLeastSquares (rows weighted, equalities at weightEq).</summary>
        static double[] LawSolve (int n, double[] a, double[] b, bool[] rowIn, double[] q, double[] r2, double[] p,
            double[] lo, double[] hi, double weightEq)
        {
            int m = 6 + n;
            var work = new AttitudeWork ();
            work.Resize (m, n);
            for (int i = 0; i < m * n; i++)
                work.A [i] = 0.0;
            for (int r = 0; r < 6; r++) {
                double w = !rowIn [r] ? 0.0 : (q [r] == 0.0 ? weightEq : 1.0);
                for (int j = 0; j < n; j++)
                    work.A [r * n + j] = w * a [6 * j + r];
                work.Bv [r] = w * b [r];
            }
            for (int j = 0; j < n; j++) {
                double s = Math.Sqrt (r2 [j]);
                work.A [(6 + j) * n + j] = s;
                work.Bv [6 + j] = s * p [j];
                work.Lb [j] = lo [j];
                work.Ub [j] = hi [j];
                work.W [j] = AttitudeMath.SetFree;
            }
            bool converged;
            AttitudeMath.SolveBoxLeastSquares (work, m, n, 400, out converged);
            var y = new double[n];
            for (int j = 0; j < n; j++)
                y [j] = work.X [j];
            return y;
        }

        static void TestStageAgainstLaw ()
        {
            var rng = new Random (264);
            var cw = new CascadeWork ();
            for (int draw = 0; draw < 600; draw++) {
                int n = 2 + rng.Next (0, 14);
                var a = new double[6 * n];
                var b = new double[6];
                var r2 = new double[n];
                var p = new double[n];
                var lo = new double[n];
                var hi = new double[n];
                RandomStage (rng, n, a, b, r2, p, lo, hi);
                var q = new double[] { 1, 1, 1, 1, 1, 1 };
                var rowIn = new bool[6];
                for (int r = 0; r < 6; r++)
                    rowIn [r] = rng.NextDouble () < 0.8;
                var y = new double[n];
                var w = new int[n];
                for (int j = 0; j < n; j++) {
                    y [j] = p [j];
                    w [j] = CascadeWork.Free;
                }
                bool converged, jitter;
                cw.Solve (n, a, b, q, rowIn, r2, p, lo, hi, y, w, EngineAllocation.MaxIterations (n), out converged,
                    out jitter);
                Check (converged, "stage converged (draw " + draw + ")");
                var yl = LawSolve (n, a, b, rowIn, q, r2, p, lo, hi, 1.0);
                double fa = Objective (n, a, b, rowIn, r2, p, y, 1.0, q), fl = Objective (n, a, b, rowIn, r2, p, yl, 1.0, q);
                double diff = 0.0;
                for (int j = 0; j < n; j++) {
                    Check (y [j] >= lo [j] - 1e-12 && y [j] <= hi [j] + 1e-12, "box (draw " + draw + ")");
                    diff = Math.Max (diff, Math.Abs (y [j] - yl [j]));
                }
                Check (Math.Abs (fa - fl) <= 1e-9 * (1.0 + Math.Abs (fl)) && diff < 1e-6,
                    "the stage equals the law's solver (draw " + draw + "): objective " + fa + " vs " + fl + ", |dy| " + diff);
            }
        }

        static void TestStageEqualities ()
        {
            var rng = new Random (2642);
            var cw = new CascadeWork ();
            int compared = 0;
            for (int draw = 0; draw < 400; draw++) {
                int n = 8 + rng.Next (0, 12);
                var a = new double[6 * n];
                var b = new double[6];
                var r2 = new double[n];
                var p = new double[n];
                var lo = new double[n];
                var hi = new double[n];
                RandomStage (rng, n, a, b, r2, p, lo, hi);
                var rowIn = new bool[] { true, true, true, true, true, true };
                var q = new double[] { 1, 1, 1, 1, 1, 1 };
                // A feasible start that meets two equality rows: their targets are what that start achieves.
                var y = new double[n];
                var w = new int[n];
                for (int j = 0; j < n; j++) {
                    y [j] = lo [j] + (hi [j] - lo [j]) * rng.NextDouble ();
                    w [j] = CascadeWork.Free;
                }
                for (int r = 0; r < 2; r++) {
                    q [r] = 0.0;
                    double v = 0.0;
                    for (int j = 0; j < n; j++)
                        v += a [6 * j + r] * y [j];
                    b [r] = v;
                }
                bool converged, jitter;
                cw.Solve (n, a, b, q, rowIn, r2, p, lo, hi, y, w, EngineAllocation.MaxIterations (n), out converged,
                    out jitter);
                Check (converged, "equality stage converged (draw " + draw + ")");
                for (int r = 0; r < 2; r++) {
                    double v = 0.0, scale = 0.0;
                    for (int j = 0; j < n; j++) {
                        v += a [6 * j + r] * y [j];
                        scale += Math.Abs (a [6 * j + r]);
                    }
                    Check (Math.Abs (v - b [r]) <= 1e-9 * (1.0 + scale), "equality held (draw " + draw + ", row " + r +
                        "): " + (v - b [r]));
                }
                for (int j = 0; j < n; j++)
                    Check (y [j] >= lo [j] - 1e-12 && y [j] <= hi [j] + 1e-12, "box (equality draw " + draw + ")");
                // The law's solver with the equalities weighted 1e3: the same point within its own bias.
                var yl = LawSolve (n, a, b, rowIn, q, r2, p, lo, hi, 1e3);
                double fa = Objective (n, a, b, rowIn, r2, p, y, 0.0, q);
                double fl = Objective (n, a, b, rowIn, r2, p, yl, 0.0, q);
                // The heavily weighted point may break the equalities by ~1e-6 and so do slightly better on the rest.
                Check (fa <= fl * (1.0 + 1e-3) + 1e-6, "the equality stage is optimal (draw " + draw + "): " + fa + " vs " + fl);
                compared++;
            }
            Check (compared == 400, "every equality draw compared");
        }

        // ------------------------------------------------------------------ the heavy booster

        const int HeavyEngines = 9;
        const double HeavyThrustN = 2.0e6;      // per engine at full throttle
        const double HeavyMassKg = 270e3;
        const double HeavyGimbalDeg = 2.0;
        const double HeavyRcsN = 20e3;          // RCS translation force per unit command, each axis
        const double HeavyRcsTorqueNm = 0.36e6; // RCS rotation column

        /// <summary>
        /// The heavy booster in the allocation's variables: 9 throttles, 9 x 2 gimbal columns, 3 rotation, 2 linear.
        /// Engine i at (r cos, -10.8, r sin), thrust along +y (the nose); gimbal x tilts the thrust toward +z, y toward
        /// -x (the force per degree T pi / 180). main: the main throttle (every engine realized there). f: gimbal and
        /// engine Lerp factors (1: instant) and the horizon (steady when NaN).
        /// </summary>
        static void Heavy (EngineAllocation x, double main, double fGimbal, double fEngine, double horizon,
            double[] gimbalU0 = null)
        {
            int ne = HeavyEngines;
            int n = ne + 2 * ne + 3 + 2;
            x.Resize (n);
            double deg = Math.PI / 180.0;
            int j = 0;
            for (int i = 0; i < ne; i++, j++) {
                double rx = 0.0, rz = 0.0;
                if (i > 0) {
                    double ang = 2.0 * Math.PI * (i - 1) / (ne - 1);
                    rx = 3.0 * Math.Cos (ang);
                    rz = 3.0 * Math.Sin (ang);
                }
                double ry = -10.8;
                double s = HeavyThrustN;
                Set (x, j, 0.0, s, 0.0, rx, ry, rz);
                x.U0 [j] = main;
                x.Lo [j] = 0.0;
                x.Hi [j] = 1.0;
                x.Pref [j] = main;
                x.Priority [j] = 0.5;
                double phi = double.IsNaN (horizon) ? 1.0 : EngineAllocation.Phi (fEngine, horizon);
                x.PhiUp [j] = x.PhiDown [j] = phi;
                x.Kind [j] = EngineAllocation.KindThrottle;
            }
            for (int i = 0; i < ne; i++) {
                double rx = 0.0, rz = 0.0;
                if (i > 0) {
                    double ang = 2.0 * Math.PI * (i - 1) / (ne - 1);
                    rx = 3.0 * Math.Cos (ang);
                    rz = 3.0 * Math.Sin (ang);
                }
                double t = HeavyThrustN * main * deg;
                double phi = double.IsNaN (horizon) ? 1.0 : EngineAllocation.Phi (fGimbal, horizon);
                for (int axis = 0; axis < 2; axis++, j++) {
                    if (axis == 0)
                        Set (x, j, 0.0, 0.0, t, rx, -10.8, rz);
                    else
                        Set (x, j, -t, 0.0, 0.0, rx, -10.8, rz);
                    x.U0 [j] = gimbalU0 == null ? 0.0 : gimbalU0 [2 * i + axis];
                    x.Lo [j] = -HeavyGimbalDeg;
                    x.Hi [j] = HeavyGimbalDeg;
                    x.Pref [j] = 0.0;
                    x.Priority [j] = 1.0;
                    x.PhiUp [j] = x.PhiDown [j] = phi;
                    x.Kind [j] = EngineAllocation.KindGimbal;
                }
            }
            for (int a = 0; a < 3; a++, j++) {
                for (int r = 0; r < 6; r++)
                    x.Col [6 * j + r] = r == 3 + a ? HeavyRcsTorqueNm : 0.0;
                Box (x, j, EngineAllocation.KindRotation);
            }
            // Linear X: +x force at the ring 37 m above, Y: +z force.
            Set (x, j, HeavyRcsN, 0.0, 0.0, 0.0, 37.0, 0.0);
            Box (x, j++, EngineAllocation.KindLinear);
            Set (x, j, 0.0, 0.0, HeavyRcsN, 0.0, 37.0, 0.0);
            Box (x, j++, EngineAllocation.KindLinear);
        }

        static void Set (EngineAllocation x, int j, double fx, double fy, double fz, double rx, double ry, double rz)
        {
            x.Col [6 * j] = fx;
            x.Col [6 * j + 1] = fy;
            x.Col [6 * j + 2] = fz;
            x.Col [6 * j + 3] = ry * fz - rz * fy;
            x.Col [6 * j + 4] = rz * fx - rx * fz;
            x.Col [6 * j + 5] = rx * fy - ry * fx;
        }

        static void Box (EngineAllocation x, int j, int kind)
        {
            x.U0 [j] = 0.0;
            x.Lo [j] = -1.0;
            x.Hi [j] = 1.0;
            x.Pref [j] = 0.0;
            x.Priority [j] = 3.0;
            x.PhiUp [j] = x.PhiDown [j] = 1.0;
            x.Kind [j] = kind;
        }

        static void Targets (EngineAllocation x, double fx, double fy, double fz, double mx, double my, double mz,
            bool lateral)
        {
            x.Target [0] = fx;
            x.Target [1] = fy;
            x.Target [2] = fz;
            x.Target [3] = mx;
            x.Target [4] = my;
            x.Target [5] = mz;
            for (int r = 0; r < 6; r++)
                x.RowOn [r] = true;
            x.RowOn [0] = x.RowOn [2] = lateral;
        }

        static double SumKind (EngineAllocation x, int kind, int row)
        {
            double s = 0.0;
            for (int j = 0; j < x.Count; j++)
                if (x.Kind [j] == kind)
                    s += x.Col [6 * j + row] * x.X [j];
            return s;
        }

        static void TestHeavyTorque ()
        {
            var x = new EngineAllocation ();
            Heavy (x, 0.4, 1.0, 1.0, double.NaN);
            Targets (x, 0.0, 0.0, 0.0, 3.0e6, 0.05e6, -2.0e6, false);
            Check (x.Solve (true), "heavy: solved");
            Check (x.Converged && x.JointTaken && x.Stages == 0, "heavy torque: converged, every level met by the joint solve: " +
                x.Stages);
            // The joint solve IS the cascade's answer when every level is met: the cascade alone (no warm start).
            var cascade = new EngineAllocation ();
            Heavy (cascade, 0.4, 1.0, 1.0, double.NaN);
            Targets (cascade, 0.0, 0.0, 0.0, 3.0e6, 0.05e6, -2.0e6, false);
            Check (cascade.Solve (false) && cascade.Converged && !cascade.JointTaken && cascade.Stages == 3,
                "heavy torque: the cascade alone runs three stages (axial, pitch/yaw, roll): " + cascade.Stages);
            double dc = 0.0;
            for (int j = 0; j < x.Count; j++)
                dc = Math.Max (dc, Math.Abs (cascade.X [j] - x.X [j]) / (0.5 * (x.Hi [j] - x.Lo [j])));
            for (int r = 1; r < 6; r++)
                Check (Math.Abs (cascade.Achieved [r] - x.Achieved [r]) <= 1e-5 * x.RowAuthority [r],
                    "heavy torque: joint and cascade achieve the same row " + r);
            Check (dc < 1e-3, "heavy torque: joint and cascade, the same allocation (share of each box): " + dc);
            Check (Math.Abs (x.Achieved [1]) < 1e-6 * x.RowAuthority [1], "heavy torque: D's axial force kept: " + x.Achieved [1]);
            Check (Math.Abs (x.Achieved [3] - 3.0e6) < 1e-6 * x.RowAuthority [3] &&
                Math.Abs (x.Achieved [5] + 2.0e6) < 1e-6 * x.RowAuthority [5], "heavy torque: pitch/yaw met: " +
                x.Achieved [3] + " " + x.Achieved [5]);
            Check (Math.Abs (x.Achieved [4] - 0.05e6) < 1e-5 * x.RowAuthority [4], "heavy torque: roll met: " + x.Achieved [4]);
            Check ((x.Reasons & (EngineAllocation.ReasonLimitedAxial | EngineAllocation.ReasonLimitedPitchYaw |
                EngineAllocation.ReasonLimitedRoll)) == 0, "heavy torque: nothing limited: " + x.Reasons);
            double diff = Math.Abs (SumKind (x, EngineAllocation.KindThrottle, 3)) + Math.Abs (SumKind (x, EngineAllocation.KindThrottle, 5));
            double gim = Math.Abs (SumKind (x, EngineAllocation.KindGimbal, 3)) + Math.Abs (SumKind (x, EngineAllocation.KindGimbal, 5));
            Check (diff > 0.0 && gim > 0.0, "heavy torque: the differential and the gimbals share the torque: " + diff + ", " + gim);
            double sum = 0.0;
            for (int j = 0; j < HeavyEngines; j++) {
                Check (x.Command [j] >= 0.0 && x.Command [j] <= 1.0, "heavy torque: throttle in [0, 1]");
                sum += x.Command [j] - 0.4;
            }
            Check (Math.Abs (sum) < 1e-6, "heavy torque: the throttles' deviations sum to zero (equal slopes): " + sum);
        }

        static void TestHeavyTranslation ()
        {
            // 0.2 m/s^2 along +z WITHOUT inclining: no pitch/yaw torque wanted.
            var x = new EngineAllocation ();
            Heavy (x, 0.4, 1.0, 1.0, double.NaN);
            double f = 0.2 * HeavyMassKg;
            Targets (x, 0.0, 0.0, f, 0.0, 0.0, 0.0, true);
            Check (x.Solve (true) && x.Converged && x.JointTaken, "heavy translation: every level met at once");
            var cascade = new EngineAllocation ();
            Heavy (cascade, 0.4, 1.0, 1.0, double.NaN);
            Targets (cascade, 0.0, 0.0, f, 0.0, 0.0, 0.0, true);
            Check (cascade.Solve (false) && cascade.Converged && cascade.Stages == 4, "heavy translation: the cascade alone, four stages");
            for (int r = 0; r < 6; r++)
                Check (Math.Abs (cascade.Achieved [r] - x.Achieved [r]) <= 1e-5 * x.RowAuthority [r],
                    "heavy translation: joint and cascade achieve the same row " + r);
            Check (Math.Abs (x.Achieved [2] - f) < 1e-5 * x.RowAuthority [2] && Math.Abs (x.Achieved [0]) < 1e-5 * x.RowAuthority [0],
                "heavy translation: the lateral force met: " + x.Achieved [0] + " " + x.Achieved [2]);
            Check (Math.Abs (x.Achieved [3]) < 1e-6 * x.RowAuthority [3] && Math.Abs (x.Achieved [5]) < 1e-6 * x.RowAuthority [5],
                "heavy translation: no pitch/yaw torque (the body does not incline): " + x.Achieved [3] + " " + x.Achieved [5]);
            Check (Math.Abs (x.Achieved [1]) < 1e-6 * x.RowAuthority [1], "heavy translation: D's axial force kept");
            double common = 0.0;
            int same = 0;
            for (int i = 0; i < HeavyEngines; i++) {
                double gx = x.Command [HeavyEngines + 2 * i];
                common += gx;
                if (gx > 0.0)
                    same++;
            }
            Check (same == HeavyEngines, "heavy translation: every gimbal tilted the same way (common tilt): " + same);
            double gimbalTorque = SumKind (x, EngineAllocation.KindGimbal, 3);
            double cancel = SumKind (x, EngineAllocation.KindThrottle, 3) + SumKind (x, EngineAllocation.KindLinear, 3) +
                SumKind (x, EngineAllocation.KindRotation, 3);
            Check (Math.Abs (gimbalTorque) > 1e3 && Math.Abs (gimbalTorque + cancel) < 1e-5 * x.RowAuthority [3],
                "heavy translation: the tilt's torque cancelled by the differential and the RCS (rotation and translation): " + gimbalTorque + " vs " + cancel);
            Check (SumKind (x, EngineAllocation.KindLinear, 2) > 0.0,
                "heavy translation: the RCS 37 m above pushes the same way (forces add, torques cancel)");
        }

        static void TestHeavyPriorities ()
        {
            // An unreachable lateral demand: pitch/yaw and the axial force stay exact, the lateral is limited.
            var x = new EngineAllocation ();
            Heavy (x, 0.4, 1.0, 1.0, double.NaN);
            Targets (x, 0.0, 0.0, 50.0 * HeavyMassKg, 1.0e6, 0.0, 0.0, true);
            Check (x.Solve (true) && x.Converged && !x.JointTaken && x.Stages == 4,
                "priorities: the joint solve refused, the cascade ran its four stages: " + x.Stages);
            Check (Math.Abs (x.Achieved [3] - 1.0e6) < 1e-6 * x.RowAuthority [3] && Math.Abs (x.Achieved [5]) < 1e-6 * x.RowAuthority [5],
                "priorities: pitch/yaw exact under an unreachable lateral demand: " + x.Achieved [3] + " " + x.Achieved [5]);
            Check (Math.Abs (x.Achieved [1]) < 1e-6 * x.RowAuthority [1], "priorities: the axial force exact");
            Check ((x.Reasons & EngineAllocation.ReasonLimitedLateral) != 0 && (x.Reasons & EngineAllocation.ReasonLimitedPitchYaw) == 0,
                "priorities: the lateral limited, not pitch/yaw: " + x.Reasons);
            Check (x.Achieved [2] > 0.0, "priorities: the lateral force still pushed as far as the attitude allows");
            // A main throttle at 1: D's axial force held means no engine can go down (none can go up).
            Heavy (x, 1.0, 1.0, 1.0, double.NaN);
            Targets (x, 0.0, 0.0, 0.0, 2.0e6, 0.0, 0.0, false);
            Check (x.Solve (true) && x.Converged, "full throttle: solved");
            for (int j = 0; j < HeavyEngines; j++)
                Check (Math.Abs (x.Command [j] - 1.0) < 1e-9, "full throttle: engine " + j + " kept at 1: " + x.Command [j]);
            Check (Math.Abs (x.Achieved [3] - 2.0e6) < 1e-6 * x.RowAuthority [3], "full throttle: the gimbals give the torque");
            // A torque beyond everything: the axial force still exact, pitch/yaw limited, roll after.
            Heavy (x, 0.4, 1.0, 1.0, double.NaN);
            Targets (x, 0.0, 0.0, 0.0, 1.0e9, 1.0e5, 0.0, false);
            Check (x.Solve (true) && x.Converged, "saturation: solved");
            Check (Math.Abs (x.Achieved [1]) < 1e-6 * x.RowAuthority [1], "saturation: the axial force exact: " + x.Achieved [1]);
            Check ((x.Reasons & EngineAllocation.ReasonLimitedPitchYaw) != 0, "saturation: pitch/yaw limited");
        }

        static void TestHeavyHorizon ()
        {
            // Gimbals Lerping 0.16 a step, engines 0.04 up and 0.08 down: the horizon is the engines' time constant.
            double fg = 0.16, fu = 0.04, fd = 0.08;
            double h = EngineAllocation.Horizon (new[] { fg, fu, fd }, 3);
            var gu0 = new double[2 * HeavyEngines];
            for (int k = 0; k < gu0.Length; k++)
                gu0 [k] = 0.3 * Math.Sin (k);
            var x = new EngineAllocation ();
            Heavy (x, 0.5, fg, fu, h, gu0);
            for (int j = 0; j < HeavyEngines; j++)
                x.PhiDown [j] = EngineAllocation.Phi (fd, h);
            double f = 0.15 * HeavyMassKg;
            Targets (x, f, 0.0, 0.0, 0.5e6, 0.0, -0.4e6, true);
            Check (x.Solve (true) && x.Converged, "horizon: solved");
            // Simulate every actuator's Lerp toward its command for the (rounded) horizon: the realized change is x.
            int steps = (int)Math.Round (h);
            Heavy (x, 0.5, fg, fu, steps, gu0);
            for (int j = 0; j < HeavyEngines; j++)
                x.PhiDown [j] = EngineAllocation.Phi (fd, steps);
            Targets (x, f, 0.0, 0.0, 0.5e6, 0.0, -0.4e6, true);
            Check (x.Solve (true) && x.Converged, "horizon (integer): solved");
            var wrench = new double[6];
            for (int j = 0; j < x.Count; j++) {
                double u = x.U0 [j], c = x.Command [j];
                double fac = x.Kind [j] == EngineAllocation.KindGimbal ? fg
                    : x.Kind [j] == EngineAllocation.KindThrottle ? (c >= u ? fu : fd) : 1.0;
                double real = u;
                for (int s = 0; s < steps; s++)
                    real = real + (c - real) * fac;
                double dx = real - u;
                Check (Math.Abs (dx - x.X [j]) < 1e-9 * (1.0 + Math.Abs (x.X [j]) + Math.Abs (c - u)),
                    "horizon: the command realizes the allocation after " + steps + " steps (" + j + "): " + dx + " vs " + x.X [j]);
                for (int r = 0; r < 6; r++)
                    wrench [r] += x.Col [6 * j + r] * dx;
            }
            Check (Math.Abs (wrench [0] - f) < 1e-4 * x.RowAuthority [0] && Math.Abs (wrench [3] - 0.5e6) < 1e-4 * x.RowAuthority [3] &&
                Math.Abs (wrench [5] + 0.4e6) < 1e-4 * x.RowAuthority [5],
                "horizon: the simulated wrench at the horizon is the target (balanced on the REALIZED state): " +
                wrench [0] + " " + wrench [3] + " " + wrench [5]);
            int over = 0;
            for (int j = HeavyEngines; j < 3 * HeavyEngines; j++)
                if (Math.Abs (x.Command [j] - x.U0 [j]) > Math.Abs (x.X [j]) + 1e-12)
                    over++;
            Check (over > 0, "horizon: a gimbal's command leads its realized change (it Lerps)");
        }

        static void TestHeavyAuthority ()
        {
            var x = new EngineAllocation ();
            double[] levels = { 0.25, 0.5, 0.75, 1.0 };
            double previous = 0.0;
            foreach (double level in levels) {
                Heavy (x, level, 1.0, 1.0, double.NaN);
                Targets (x, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
                double minimum = double.PositiveInfinity;
                for (int d = 0; d < 4; d++) {
                    double ux = d == 0 ? 1.0 : d == 1 ? -1.0 : 0.0, uz = d == 2 ? 1.0 : d == 3 ? -1.0 : 0.0;
                    double along = x.AuthorityAlong (ux, uz);
                    Check (along > 0.0 && !double.IsNaN (along), "authority: positive along " + d + " at " + level + ": " + along);
                    Check (Math.Abs (x.Achieved [3]) < 1e-6 * x.RowAuthority [3] && Math.Abs (x.Achieved [5]) < 1e-6 * x.RowAuthority [5],
                        "authority: torque-free at " + level + " along " + d);
                    Check (Math.Abs (x.Achieved [1]) < 1e-6 * x.RowAuthority [1], "authority: the axial force kept at " + level);
                    minimum = Math.Min (minimum, along / HeavyMassKg);
                }
                if (level < 1.0)
                    Check (minimum > previous, "authority grows with the thrust up to the full throttle: " + minimum + " at " + level);
                previous = minimum;
            }
            // At a full main throttle, the differential has no room: the authority is the RCS's balance only.
            Heavy (x, 1.0, 1.0, 1.0, double.NaN);
            Targets (x, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
            double full = x.AuthorityAlong (0.0, 1.0) / HeavyMassKg;
            Heavy (x, 0.75, 1.0, 1.0, double.NaN);
            Targets (x, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
            double three = x.AuthorityAlong (0.0, 1.0) / HeavyMassKg;
            Check (full < three, "authority at full throttle below 3/4 (no differential left): " + full + " vs " + three);
            // Free authority: a pitch torque wanted leaves less along the axis it uses.
            Heavy (x, 0.5, 1.0, 1.0, double.NaN);
            Targets (x, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
            double neutral = x.AuthorityAlong (0.0, 1.0);
            Targets (x, 0.0, 0.0, 0.0, 0.8 * x.RowAuthority [3], 0.0, 0.0, false);
            double busy = x.AuthorityAlong (0.0, 1.0);
            Check (busy < neutral, "free authority: the attitude's need leaves less: " + busy + " vs " + neutral);
            // Nothing thrusting, no RCS: nothing.
            var empty = new EngineAllocation ();
            empty.Resize (2);
            for (int j = 0; j < 2; j++) {
                for (int r = 0; r < 6; r++)
                    empty.Col [6 * j + r] = 0.0;
                Box (empty, j, EngineAllocation.KindRotation);
            }
            Targets (empty, 0, 0, 0, 0, 0, 0, false);
            Check (!empty.Solve (true) && (empty.Reasons & EngineAllocation.ReasonNoActuator) != 0, "no actuator: refused");
            Check (empty.AuthorityAlong (1.0, 0.0) == 0.0, "no actuator: no authority");
        }

        static void TestCost ()
        {
            // The heavy booster (9 engines, 9 gimbal modules: 32 variables) and the super hope (33 engines, 13 gimbal
            // modules: 64 variables). One step = the allocation + ONE authority (the plugin's worst step: it solves one
            // authority every second step).
            CostConfig ("heavy booster", 9, 9, 0);
            CostConfig ("super hope", 33, 13, 1);
        }

        internal static readonly double[] CostMeanUs = new double[2], CostAuthUs = new double[2];
        internal static readonly double[] CostP50Us = new double[2], CostP95Us = new double[2];
        internal static readonly double[] CostIterations = new double[2], CostJoint = new double[2];

        static void CostConfig (string name, int ne, int ng, int slot)
        {
            var x = new EngineAllocation ();
            var auth = new EngineAllocation ();
            int n = ne + 2 * ng + 5;
            var sw = new Stopwatch ();
            const int steps = 1000;
            var perStep = new double[steps];
            double total = 0.0, totalAuth = 0.0;
            int iterations = 0, maxIterations = 0, joint = 0;
            long allocated = 0;
            for (int step = 0; step < steps + 1; step++) {
                // A descent-like demand: a slow torque and a lateral demand, with saturating bursts (10 % of the steps).
                double burst = step % 200 < 20 ? 40.0 : 1.0;
                FillMany (x, ne, ng, 0.5 + 0.1 * Math.Sin (0.05 * step));
                Targets (x, 0.0, 0.0, 0.1 * HeavyMassKg * Math.Sin (0.03 * step), burst * 1e6 * Math.Sin (0.1 * step),
                    1e4 * Math.Cos (0.07 * step), burst * 0.8e6 * Math.Cos (0.11 * step), true);
                FillMany (auth, ne, ng, 0.5);
                Targets (auth, 0, 0, 0, 0, 0, 0, false);
                long before = GC.GetAllocatedBytesForCurrentThread ();
                sw.Restart ();
                x.Solve (true);
                sw.Stop ();
                double us = sw.Elapsed.TotalMilliseconds * 1000.0;
                sw.Restart ();
                auth.AuthorityAlong (step % 2 == 0 ? 1.0 : 0.0, step % 2 == 0 ? 0.0 : 1.0);
                sw.Stop ();
                double ua = sw.Elapsed.TotalMilliseconds * 1000.0;
                long after = GC.GetAllocatedBytesForCurrentThread ();
                if (step == 0)
                    continue;   // the JIT and the first sizing
                allocated += after - before;
                Check (x.Converged, name + " cost run: converged at step " + step);
                total += us;
                totalAuth += ua;
                perStep [step - 1] = us + ua;
                iterations += x.Iterations;
                maxIterations = Math.Max (maxIterations, x.Iterations);
                if (x.JointTaken)
                    joint++;
            }
            Check (allocated == 0, name + ": no allocation per step (solve and authority): " + allocated + " bytes");
            Check (maxIterations <= 5 * EngineAllocation.MaxIterations (n), name + ": iterations bounded");
            Array.Sort (perStep);
            CostMeanUs [slot] = total / steps;
            CostAuthUs [slot] = totalAuth / steps;
            CostP50Us [slot] = perStep [steps / 2];
            CostP95Us [slot] = perStep [(int)(0.95 * steps)];
            CostIterations [slot] = iterations / (double)steps;
            CostJoint [slot] = joint / (double)steps;
            Console.WriteLine ("engine allocation cost, " + name + " (" + n + " variables): allocation mean " +
                CostMeanUs [slot].ToString ("F1") + " us (" + CostIterations [slot].ToString ("F1") + " iterations, max " +
                maxIterations + ", joint solve " + (100.0 * CostJoint [slot]).ToString ("F0") + " % of the steps), one authority " +
                CostAuthUs [slot].ToString ("F1") + " us; a step median " + CostP50Us [slot].ToString ("F1") + " us, p95 " +
                CostP95Us [slot].ToString ("F1") + " us (net8, this machine)");
        }

        static void FillMany (EngineAllocation x, int ne, int ng, double main)
        {
            int n = ne + 2 * ng + 5;
            x.Resize (n);
            double deg = Math.PI / 180.0;
            int j = 0;
            for (int i = 0; i < ne; i++, j++) {
                double ang = 2.0 * Math.PI * i / ne, rad = i < ng ? 1.5 : 4.2;
                Set (x, j, 0.0, 2.3e6, 0.0, rad * Math.Cos (ang), -30.0, rad * Math.Sin (ang));
                x.U0 [j] = main;
                x.Lo [j] = 0.0;
                x.Hi [j] = 1.0;
                x.Pref [j] = main;
                x.Priority [j] = 0.5;
                x.PhiUp [j] = 0.63;
                x.PhiDown [j] = 0.8;
                x.Kind [j] = EngineAllocation.KindThrottle;
            }
            for (int i = 0; i < ng; i++) {
                double ang = 2.0 * Math.PI * i / ne, rad = 1.5;
                double t = 2.3e6 * main * deg;
                for (int axis = 0; axis < 2; axis++, j++) {
                    if (axis == 0)
                        Set (x, j, 0.0, 0.0, t, rad * Math.Cos (ang), -30.0, rad * Math.Sin (ang));
                    else
                        Set (x, j, -t, 0.0, 0.0, rad * Math.Cos (ang), -30.0, rad * Math.Sin (ang));
                    x.U0 [j] = 0.0;
                    x.Lo [j] = -3.0;
                    x.Hi [j] = 3.0;
                    x.Pref [j] = 0.0;
                    x.Priority [j] = 1.0;
                    x.PhiUp [j] = x.PhiDown [j] = 0.98;
                    x.Kind [j] = EngineAllocation.KindGimbal;
                }
            }
            for (int a = 0; a < 3; a++, j++) {
                for (int r = 0; r < 6; r++)
                    x.Col [6 * j + r] = r == 3 + a ? HeavyRcsTorqueNm : 0.0;
                Box (x, j, EngineAllocation.KindRotation);
            }
            Set (x, j, HeavyRcsN, 0.0, 0.0, 0.0, 40.0, 0.0);
            Box (x, j++, EngineAllocation.KindLinear);
            Set (x, j, 0.0, 0.0, HeavyRcsN, 0.0, 40.0, 0.0);
            Box (x, j++, EngineAllocation.KindLinear);
        }
    }
}
