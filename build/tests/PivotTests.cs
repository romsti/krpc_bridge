// -----------------------------------------------------------------------------------
// tests of GimbalPivots.cs, the per-pivot mapping of a gimbal
// module with several non-parallel pivots. What they pin down:
//   * the V1 cases are NOT mapped: one pivot (barge, medium, small:
//     one pivot per module, dynamics_v3 transform_count 1), parallel pivots, a
//     missing pivot 0 -- the V1 path runs, unchanged;
//   * pivot 0's row is exactly the identity; each row of a cluster turned about the thrust
//     axis is a rotation; every pivot turns by the SAME world rotation (the stock rule);
//   * the per-component clamp and its derivative flags; the inverse of a row;
//   * the flight: the heavy booster a recorded flight, one AR-1 Cluster part (flight
//     id 958067724) at UT 19823.708, finalThrust 4372.473 kN, its six nozzle levers and
//     directions from dynamics_v3 -- the V1 rule reproduces the column the DLL PUBLISHED
//     (axis 1: roll -1.014e5 N.m/deg, pitch/yaw 147-362 N.m/deg), and the mapping gives the
//     coherent pitch/yaw column T L pi/180 = 8.27e5 N.m/deg with no roll;
//   * the ground check of 27/09 (sol_d42.json, law in the clamps): the WLS drives a mapped module
//     to its box corner, pivots clamp, the local columns fall to sqrt(31)/6 and sqrt(19)/6 and
//     sum B.u0 is 0.57 of the torque -- the clamp, reproduced; the law's authority reads the
//     unclamped columns (AttitudeInput.BAuthority), alpha from B otherwise;
//   * the translation's force columns fall like the torque ones, but the damage is a
//     tilt-matrix JUMP and a feed-forward shortfall (a total deflection read on local columns),
//     not an authority collapse; the fix (free columns, realized wrench) is continuous and
//     undershoots by under 6 %; the heavy booster refuses the translation (no differential
//     lever); the exact authority (mode 2) against a grid, and the heavy booster's 1.159.
// Writes nothing into the golden vectors.
// -----------------------------------------------------------------------------------

using System;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static class PivotTests
    {
        static int checks;
        static int failures;

        public static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestModes ();
            TestV1CasesNotMapped ();
            TestClusterRowsAreRotations ();
            TestClampAndInverse ();
            TestFlightHeavyBooster ();
            TestSaturationGroundCheck ();
            TestAuthorityFromFreeColumns ();
            TestD65ColumnsAndRealized ();
            TestD65TranslationModel ();
            TestD65HeavyBoosterRefused ();
            TestD65ExactAuthority ();
            TestD65AuthorityColumns ();
            checkCount = checks;
            Console.WriteLine ("pivot checks: " + checks + ", failures: " + failures);
            return failures;
        }

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.Error.WriteLine ("FAIL (pivots): " + what);
            }
        }

        static void TestModes ()
        {
            Check (GimbalPivots.ModeV1 == 0 && GimbalPivots.ModeMapped == 1 &&
                GimbalPivots.ModeExactAuthority == 2,
                "modes: 0 V1, 1 per pivot, 2 per pivot + exact authority (mirrored by the client)");
            Check (GimbalPivots.ValidMode (0) && GimbalPivots.ValidMode (1) &&
                GimbalPivots.ValidMode (2), "0, 1 and 2 valid");
            Check (!GimbalPivots.ValidMode (3) && !GimbalPivots.ValidMode (-1), "others refused");
            Check (!GimbalPivots.Mapped (0) && GimbalPivots.Mapped (1) && GimbalPivots.Mapped (2) &&
                !GimbalPivots.Mapped (3), "modes 1 and 2 map per pivot, 0 does not");
            Check (GimbalPivots.Version == 1, "version 1");
        }

        // Axes of pivot i turned by phi about the body y axis (thrust axis of the booster).
        static void PutTurned (double[] axes, int i, double phiDeg)
        {
            double p = phiDeg * Math.PI / 180.0, c = Math.Cos (p), s = Math.Sin (p);
            int a = GimbalPivots.AxesStride * i;
            axes [a] = c;       // X = Ry(phi) (1, 0, 0)
            axes [a + 1] = 0.0;
            axes [a + 2] = -s;
            axes [a + 3] = s;   // Y = Ry(phi) (0, 0, 1)
            axes [a + 4] = 0.0;
            axes [a + 5] = c;
        }

        static void TestV1CasesNotMapped ()
        {
            var axes = new double[GimbalPivots.AxesStride * 6];
            var map = new double[GimbalPivots.MapStride * 6];
            for (int k = 0; k < map.Length; k++)
                map [k] = 42.0;
            PutTurned (axes, 0, 30.0);
            Check (!GimbalPivots.Build (axes, 1, map), "one pivot: not mapped (V1 path)");
            Check (map [0] == 42.0, "one pivot: map untouched");
            Check (!GimbalPivots.Build (axes, 0, map), "no pivot: not mapped");
            for (int i = 0; i < 6; i++)
                PutTurned (axes, i, 30.0);
            Check (!GimbalPivots.Build (axes, 6, map), "six parallel pivots: not mapped");
            // Parallel up to float noise (Unity quaternions): still V1.
            axes [GimbalPivots.AxesStride * 3 + 2] += 1e-5;
            Check (!GimbalPivots.Build (axes, 6, map), "parallel within float noise: not mapped");
            var missing = new double[GimbalPivots.AxesStride * 2];
            PutTurned (missing, 1, 60.0);
            Check (!GimbalPivots.Build (missing, 2, map), "pivot 0 missing: not mapped");
        }

        static void TestClusterRowsAreRotations ()
        {
            var axes = new double[GimbalPivots.AxesStride * 6];
            var map = new double[GimbalPivots.MapStride * 6];
            for (int i = 0; i < 6; i++)
                PutTurned (axes, i, 17.0 + 60.0 * i);
            Check (GimbalPivots.Build (axes, 6, map), "AR-1 Cluster (6 pivots, 60 deg apart): mapped");
            Check (map [0] == 1.0 && map [1] == 0.0 && map [2] == 0.0 && map [3] == 1.0,
                "pivot 0: exactly the identity");
            for (int i = 1; i < 6; i++) {
                int m = GimbalPivots.MapStride * i;
                double det = map [m] * map [m + 3] - map [m + 1] * map [m + 2];
                double c = Math.Cos (60.0 * i * Math.PI / 180.0);
                Check (Math.Abs (det - 1.0) < 1e-12, "row " + i + " is a rotation (det 1)");
                Check (Math.Abs (map [m] - c) < 1e-12 && Math.Abs (map [m + 3] - c) < 1e-12,
                    "row " + i + " turns by the pivots' angle");
            }
            // Same world rotation: x_i X_i + y_i Y_i = x X0 + y Y0 for every pivot.
            double x = 0.7, y = -1.3;
            double wx = x * axes [0] + y * axes [3], wy = x * axes [1] + y * axes [4],
                wz = x * axes [2] + y * axes [5];
            for (int i = 0; i < 6; i++) {
                double xi, yi;
                bool fx, fy;
                GimbalPivots.PivotCommand (map, i, x, y, -2, 2, -2, 2, out xi, out yi, out fx, out fy);
                int a = GimbalPivots.AxesStride * i;
                double vx = xi * axes [a] + yi * axes [a + 3], vy = xi * axes [a + 1] + yi * axes [a + 4],
                    vz = xi * axes [a + 2] + yi * axes [a + 5];
                Check (fx && fy && Math.Abs (vx - wx) + Math.Abs (vy - wy) + Math.Abs (vz - wz) < 1e-12,
                    "pivot " + i + " turns by the module's world rotation");
            }
            // Pivot 0 passes the command through bit for bit (the lease of pivot 0 = V1).
            double x0, y0;
            bool f0, g0;
            GimbalPivots.PivotCommand (map, 0, 1.2345678f, -0.9876543f, -2, 2, -2, 2,
                out x0, out y0, out f0, out g0);
            Check (x0 == (double)1.2345678f && y0 == (double)-0.9876543f, "pivot 0: command unchanged");
        }

        static void TestClampAndInverse ()
        {
            var axes = new double[GimbalPivots.AxesStride * 2];
            var map = new double[GimbalPivots.MapStride * 2];
            PutTurned (axes, 0, 0.0);
            PutTurned (axes, 1, 45.0);
            Check (GimbalPivots.Build (axes, 2, map), "two pivots 45 deg apart: mapped");
            double xi, yi;
            bool fx, fy;
            // (2, 2) projects on pivot 1 (turned 45 deg) as (0, 2 sqrt2): y clamped to 2, x free.
            GimbalPivots.PivotCommand (map, 1, 2.0, 2.0, -2, 2, -2, 2, out xi, out yi, out fx, out fy);
            Check (Math.Abs (yi - 2.0) < 1e-12 && !fy, "clamped component: at the bound, not free");
            Check (Math.Abs (xi) < 1e-12 && fx, "free component follows");
            double x, y;
            GimbalPivots.PivotCommand (map, 1, 0.5, -1.1, -2, 2, -2, 2, out xi, out yi, out fx, out fy);
            Check (GimbalPivots.ModuleFromPivot (map, 1, xi, yi, out x, out y)
                && Math.Abs (x - 0.5) < 1e-12 && Math.Abs (y + 1.1) < 1e-12, "row inverse");
            var cX = new double[] { 1.0, 2.0, 3.0 };
            var cY = new double[] { -4.0, 5.0, -6.0 };
            var colX = new double[3];
            var colY = new double[3];
            GimbalPivots.AddPivotColumns (map, 0, true, true, cX, cY, colX, colY);
            Check (colX [0] == 1.0 && colX [1] == 2.0 && colX [2] == 3.0 && colY [0] == -4.0 &&
                colY [1] == 5.0 && colY [2] == -6.0, "pivot 0's columns pass unchanged");
            var zX = new double[3];
            var zY = new double[3];
            GimbalPivots.AddPivotColumns (map, 1, false, false, cX, cY, zX, zY);
            Check (zX [0] == 0.0 && zY [2] == 0.0, "clamped pivot: no contribution");
        }

        // The heavy booster a recorded flight, UT 19823.708 (dynamics_v3 snapshot), part
        // 958067724 (NFLV AR-1 Cluster): body-frame levers from the CoM and nozzle.forward.
        static readonly double[,] Levers = {
            { -0.11211109161376953, -10.829963684082031, -1.3683843612670898 },
            { 0.32715296745300293, -10.830023765563965, 1.2467451095581055 },
            { -0.916344165802002, -10.829957008361816, 0.7854137420654297 },
            { 1.131386399269104, -10.83003044128418, -0.9070539474487305 },
            { -1.1395572423934937, -10.829941749572754, -0.5257062911987305 },
            { 1.3539884090423584, -10.830045700073242, 0.4038372039794922 } };
        static readonly double[,] Directions = {
            { -0.004240602254867554, -0.9999526739120483, 0.008770644664764404 },
            { -0.0008677095174789429, -0.9999852180480957, 0.005397796630859375 },
            { -0.0008676797151565552, -0.9999611973762512, 0.008770525455474854 },
            { -0.00424058735370636, -0.999976634979248, 0.0053977370262146 },
            { -0.004259422421455383, -0.9999527335166931, 0.008763492107391357 },
            { -0.001139417290687561, -0.9999854564666748, 0.005295455455780029 } };
        const double FinalThrustKn = 4372.47314453125;

        /// <summary>(GimbalAuthorityTests): the flight's six nozzle levers and directions.</summary>
        internal static double[,] FlightLevers ()
        {
            return (double[,])Levers.Clone ();
        }

        internal static double[,] FlightDirections ()
        {
            return (double[,])Directions.Clone ();
        }
        // What the DLL published for that part at that UT (attitude_v1.jsonl, N.m per degree).
        const double PublishedRollAxis1 = -101402.453125;
        const double PublishedPitchYawAxis0 = 385.2;   // |(-362.39, -130.48)|
        const double PublishedPitchYawAxis1 = 363.8;   // |(-146.75, 333.28)|

        static void TestFlightHeavyBooster ()
        {
            const int n = 6;
            double thrustN = FinalThrustKn * 1000.0 / n;
            // Cluster centre (perpendicular to the thrust axis, body y) and each nozzle's radial
            // unit: the pivots of the AR-1 Cluster are turned to their nozzle's azimuth, the
            // frame the published columns reveal (axis 1 = rotation about the radial: roll).
            double cx = 0.0, cz = 0.0;
            for (int k = 0; k < n; k++) {
                cx += Levers [k, 0] / n;
                cz += Levers [k, 2] / n;
            }
            var axes = new double[GimbalPivots.AxesStride * n];
            for (int k = 0; k < n; k++) {
                double rx = Levers [k, 0] - cx, rz = Levers [k, 2] - cz;
                double r = Math.Sqrt (rx * rx + rz * rz);
                rx /= r;
                rz /= r;
                int a = GimbalPivots.AxesStride * k;
                // X = up x radial (tangential), Y = radial.
                axes [a] = rz;
                axes [a + 1] = 0.0;
                axes [a + 2] = -rx;
                axes [a + 3] = rx;
                axes [a + 4] = 0.0;
                axes [a + 5] = rz;
            }
            var map = new double[GimbalPivots.MapStride * n];
            Check (GimbalPivots.Build (axes, n, map), "the flight's AR-1 Cluster: mapped");

            var v1X = new double[3];
            var v1Y = new double[3];
            var mX = new double[3];
            var mY = new double[3];
            var offset = new double[3];     // pivot at the nozzle: the lever term only
            for (int k = 0; k < n; k++) {
                var ax = new double[] { axes [6 * k], axes [6 * k + 1], axes [6 * k + 2] };
                var ay = new double[] { axes [6 * k + 3], axes [6 * k + 4], axes [6 * k + 5] };
                var d = new double[] { Directions [k, 0], Directions [k, 1], Directions [k, 2] };
                var lever = new double[] { Levers [k, 0], Levers [k, 1], Levers [k, 2] };
                var cX = new double[3];
                var cY = new double[3];
                GimbalPivots.AddNozzleDerivative (ax, d, lever, offset, thrustN, cX, 0, null);
                GimbalPivots.AddNozzleDerivative (ay, d, lever, offset, thrustN, cY, 0, null);
                for (int r = 0; r < 3; r++) {   // V1: the same local angles on every pivot
                    v1X [r] += cX [r];
                    v1Y [r] += cY [r];
                }
                GimbalPivots.AddPivotColumns (map, k, true, true, cX, cY, mX, mY);
            }
            GimbalPivots.PerDegree (v1X);
            GimbalPivots.PerDegree (v1Y);
            GimbalPivots.PerDegree (mX);
            GimbalPivots.PerDegree (mY);
            double coherent = FinalThrustKn * 1000.0 * 10.83 * Math.PI / 180.0;   // T L pi/180
            double v1Roll = Math.Abs (v1Y [1]);
            double v1PitchYaw = Math.Max (Math.Sqrt (v1X [0] * v1X [0] + v1X [2] * v1X [2]),
                Math.Sqrt (v1Y [0] * v1Y [0] + v1Y [2] * v1Y [2]));
            Console.WriteLine ("  flight UT 19823.708, one AR-1 Cluster at 4372 kN:");
            Console.WriteLine ("    V1 (published by the DLL): roll " + PublishedRollAxis1.ToString ("F0") +
                ", pitch/yaw " + PublishedPitchYawAxis0 + " / " + PublishedPitchYawAxis1 + " N.m/deg");
            Console.WriteLine ("    V1 rule recomputed: roll " + v1Roll.ToString ("F0") +
                ", pitch/yaw " + v1PitchYaw.ToString ("F0") + " N.m/deg");
            double mPitchYaw = Math.Sqrt (mX [0] * mX [0] + mX [2] * mX [2]);
            double mPitchYawY = Math.Sqrt (mY [0] * mY [0] + mY [2] * mY [2]);
            Console.WriteLine ("    mapped per pivot: pitch/yaw " + mPitchYaw.ToString ("F0") + " / " +
                mPitchYawY.ToString ("F0") + ", roll " + Math.Abs (mX [1]).ToString ("F0") + " / " +
                Math.Abs (mY [1]).ToString ("F0") + " N.m/deg (T L pi/180 = " +
                coherent.ToString ("F0") + ")");
            Check (Math.Abs (v1Roll / Math.Abs (PublishedRollAxis1) - 1.0) < 0.02,
                "V1 rule reproduces the published roll column within 2 %");
            Check (v1PitchYaw < 0.01 * coherent,
                "V1 rule: pitch/yaw < 1 % of the coherent column (published: 0.05 %)");
            Check (Math.Abs (mPitchYaw / coherent - 1.0) < 0.02 &&
                Math.Abs (mPitchYawY / coherent - 1.0) < 0.02,
                "mapped: both columns give T L pi/180 of pitch/yaw within 2 %");
            Check (Math.Abs (mX [1]) < 0.02 * coherent && Math.Abs (mY [1]) < 0.02 * coherent,
                "mapped: roll < 2 % of it");
            double dotXY = mX [0] * mY [0] + mX [2] * mY [2];
            Check (Math.Abs (dotXY) < 0.02 * mPitchYaw * mPitchYawY,
                "mapped: the two columns are orthogonal (pitch and yaw both reachable)");
        }

        // ------------------------------------------------------ saturation in the clamps

        // Ideal AR-1 Cluster: six pivots 60 deg apart about the thrust axis (body y), each pivot's
        // torque column c X_i and c Y_i (a coherent tilt: torque along the rotation axis).
        const double PivotColumn = 76460.0 / 6.0;   // N.m/deg per pivot: 76 460 per module (stock_pivots)

        static void ClusterMap (double[] axes, double[] map, double phi0)
        {
            for (int i = 0; i < 6; i++)
                PutTurned (axes, i, phi0 + 60.0 * i);
            GimbalPivots.Build (axes, 6, map);
        }

        // Module columns (clamped: the DLL's local derivative; free: the authority's), torque of
        // the clamped pivots and the number of clamped components, in the plane (x, z).
        static int ModuleAt (double[] axes, double[] map, double x, double y, double[] colX,
            double[] colY, double[] freeX, double[] freeY, double[] torque)
        {
            int clamped = 0;
            for (int r = 0; r < 3; r++)
                colX [r] = colY [r] = freeX [r] = freeY [r] = torque [r] = 0.0;
            for (int i = 0; i < 6; i++) {
                double xi, yi;
                bool fx, fy;
                GimbalPivots.PivotCommand (map, i, x, y, -2, 2, -2, 2, out xi, out yi, out fx, out fy);
                clamped += (fx ? 0 : 1) + (fy ? 0 : 1);
                int a = GimbalPivots.AxesStride * i;
                var cX = new double[] { PivotColumn * axes [a], PivotColumn * axes [a + 1], PivotColumn * axes [a + 2] };
                var cY = new double[] { PivotColumn * axes [a + 3], PivotColumn * axes [a + 4], PivotColumn * axes [a + 5] };
                GimbalPivots.AddPivotColumns (map, i, fx, fy, cX, cY, colX, colY);
                GimbalPivots.AddPivotColumns (map, i, true, true, cX, cY, freeX, freeY);
                for (int r = 0; r < 3; r++)
                    torque [r] += cX [r] * xi + cY [r] * yi;
            }
            return clamped;
        }

        static double Plane (double[] v)
        {
            return Math.Sqrt (v [0] * v [0] + v [2] * v [2]);
        }

        static void TestSaturationGroundCheck ()
        {
            var axes = new double[GimbalPivots.AxesStride * 6];
            var map = new double[GimbalPivots.MapStride * 6];
            ClusterMap (axes, map, 0.0);     // X0 along pitch (body x)
            var cX = new double[3];
            var cY = new double[3];
            var fX = new double[3];
            var fY = new double[3];
            var t = new double[3];
            double free = 6.0 * PivotColumn;
            // Inside the 2 deg disc: nothing clamps, columns = free = the module's 76 460.
            int k = ModuleAt (axes, map, 1.2, -1.5, cX, cY, fX, fY, t);
            Check (k == 0 && Math.Abs (Plane (cX) / free - 1.0) < 1e-12 &&
                Math.Abs (Plane (fY) / free - 1.0) < 1e-12, "|w| < 2 deg: no clamp, column 6 c");
            Check (Math.Abs (cX [0] * 1.2 + cY [0] * -1.5 - t [0]) < 1e-6 * free,
                "|w| < 2 deg: sum B.u0 is the torque");
            // (2, 0.8): the two components on the 30 deg line clamp -> sqrt(31)/6 (70 952).
            double xA = 2.0 - 1e-9, yA = 0.8;
            k = ModuleAt (axes, map, xA, yA, cX, cY, fX, fY, t);
            double colA = Math.Max (Plane (cX), Plane (cY));
            Check (k == 2 && Math.Abs (colA / free - Math.Sqrt (31.0) / 6.0) < 1e-9,
                "(2, 0.8): 2 components clamp, column sqrt(31)/6 (ground check: 70 890 for 76 460)");
            Check (Math.Abs (Plane (fX) / free - 1.0) < 1e-12 && Math.Abs (Plane (fY) / free - 1.0) < 1e-12,
                "(2, 0.8): the free columns stay 6 c");
            double realA = t [0], predA = cX [0] * xA + cY [0] * yA;
            // Box corner: four components clamp -> sqrt(19)/6 (55 547).
            double e = 2.0 - 1e-9;
            k = ModuleAt (axes, map, e, e, cX, cY, fX, fY, t);
            double colB = Math.Max (Plane (cX), Plane (cY));
            Check (k == 4 && Math.Abs (colB / free - Math.Sqrt (19.0) / 6.0) < 1e-9,
                "box corner: 4 components clamp, column sqrt(19)/6 (ground check: 55 500 for 76 460)");
            double realB = Plane (t);
            double predB = Math.Sqrt (Math.Pow (cX [0] * e + cY [0] * e, 2) + Math.Pow (cX [2] * e + cY [2] * e, 2));
            // The ground check's pitch: 4 modules at the corner, 4 at (2, 0.8) (X0 along pitch).
            double real = 4.0 * (realB + realA), pred = 4.0 * (predB + predA);
            Console.WriteLine ("  clamps (ideal cluster, 76 460 N.m/deg per module): columns " +
                (colA).ToString ("F0") + " / " + colB.ToString ("F0") + ", realized " +
                (real / 1e6).ToString ("F3") + " MN.m, sum B.u0 " + (pred / 1e6).ToString ("F3") +
                ", ratio " + (pred / real).ToString ("F3") + " (measured 1.342, 0.761, 0.568)");
            Check (Math.Abs (pred / real - 0.568) < 0.01,
                "sum B.u0 / torque = 0.57 in the clamps: the linear check does not hold at saturation");
            Check (Math.Abs (real / 1.342e6 - 1.0) < 0.03 && Math.Abs (pred / 0.761e6 - 1.0) < 0.03,
                "realized and sum B.u0 within 3 % of the ground check");
            Check (predB < realB && predA < realA, "clamped: sum B.u0 under the torque, never over");
        }

        static void TestAuthorityFromFreeColumns ()
        {
            var axes = new double[GimbalPivots.AxesStride * 6];
            var map = new double[GimbalPivots.MapStride * 6];
            // X0 at 45 deg from pitch: its box corner pushes along pitch (the ground check's
            // corner modules).
            ClusterMap (axes, map, 45.0);
            var cX = new double[3];
            var cY = new double[3];
            var fX = new double[3];
            var fY = new double[3];
            var t = new double[3];
            double e = 2.0 - 1e-9;
            ModuleAt (axes, map, e, e, cX, cY, fX, fY, t);
            AttitudeOutput withB = null, withFree = null, noFlag = null;
            for (int pass = 0; pass < 3; pass++) {
                var inp = new AttitudeInput ();
                inp.Resize (2);
                inp.Dt = 0.02;
                inp.OmegaDotValid = true;   // no tick gap: the filters and the authority run
                inp.Inertia [0] = inp.Inertia [1] = inp.Inertia [2] = 1e6;
                for (int r = 0; r < 3; r++) {
                    inp.B [r * 2] = cX [r];
                    inp.B [r * 2 + 1] = cY [r];
                    inp.BAuthority [r * 2] = fX [r];
                    inp.BAuthority [r * 2 + 1] = fY [r];
                }
                for (int j = 0; j < 2; j++) {
                    inp.U0 [j] = e;
                    inp.UMin [j] = -2;
                    inp.UMax [j] = 2;
                    inp.Priority [j] = 1;
                }
                inp.UseBAuthority = pass == 1;
                if (pass == 2) {
                    inp.UseBAuthority = true;
                    inp.BAuthority [0] = double.NaN;
                }
                var o = new AttitudeOutput ();
                AttitudeMath.Step (new AttitudeGains (), inp, new AttitudeLawState (), new AttitudeWork (), o);
                if (pass == 0)
                    withB = o;
                else if (pass == 1)
                    withFree = o;
                else
                    noFlag = o;
            }
            double freeAlpha = (Math.Abs (fX [0]) + Math.Abs (fY [0])) * 2.0 / 1e6;
            double localAlpha = (Math.Abs (cX [0]) + Math.Abs (cY [0])) * 2.0 / 1e6;
            Check (withB.Filtered && Math.Abs (withB.Alpha [0] - localAlpha) < 1e-12,
                "UseBAuthority false: alpha from B, unchanged");
            Check (withFree.Filtered && Math.Abs (withFree.Alpha [0] - freeAlpha) < 1e-12,
                "UseBAuthority: alpha from the unclamped columns");
            Console.WriteLine ("  authority at the corner (module at 45 deg): local " +
                (localAlpha * 1e6).ToString ("F0") + ", free " + (freeAlpha * 1e6).ToString ("F0") +
                " N.m (torque realized " + Plane (t).ToString ("F0") + ")");
            Check (withB.Alpha [0] < 0.5 * Plane (t) / 1e6 && withFree.Alpha [0] > Plane (t) / 1e6,
                "corner: the local columns give under half the torque the clamps realize, the free ones above it");
            Check (Math.Abs (noFlag.Alpha [0] - localAlpha) < 1e-12,
                "non-finite BAuthority: alpha from B");
        }

        // ============================================================================ 
        // The translation by the engines (lateral force, attitude_translation_v1 mode 3) and the
        // law's authority, on a MAPPED module at saturation. An ideal AR-1 Cluster: six pivots
        // 60 deg apart about the thrust axis (body y), one nozzle per pivot on its radial Y_i at
        // 1.33 m, nozzle forward -y (thrust +y), the plugin's own per-nozzle derivative
        // (GimbalPivots.AddNozzleDerivative), 4372 kN per module as in a recorded flight.

        const double ClusterThrustN = 4372.47314453125e3 / 6.0;

        sealed class Cluster
        {
            internal const double Box = 2.0;
            internal readonly double[] Axes = new double[GimbalPivots.AxesStride * 6];
            internal readonly double[] Map = new double[GimbalPivots.MapStride * 6];
            internal readonly double[] Torque = new double[GimbalPivots.ColumnStride * 6];
            internal readonly double[] Force = new double[GimbalPivots.ColumnStride * 6];

            internal Cluster (double cx, double cy, double cz, double phi0, double thrustN)
            {
                for (int i = 0; i < 6; i++)
                    PutTurned (Axes, i, phi0 + 60.0 * i);
                GimbalPivots.Build (Axes, 6, Map);
                var d = new double[] { 0.0, -1.0, 0.0 };
                var offset = new double[3];
                for (int i = 0; i < 6; i++) {
                    int a = GimbalPivots.AxesStride * i, c = GimbalPivots.ColumnStride * i;
                    var ax = new double[] { Axes [a], Axes [a + 1], Axes [a + 2] };
                    var ay = new double[] { Axes [a + 3], Axes [a + 4], Axes [a + 5] };
                    var lever = new double[] { cx + 1.33 * ay [0], cy, cz + 1.33 * ay [2] };
                    var tX = new double[3];
                    var tY = new double[3];
                    var fX = new double[3];
                    var fY = new double[3];
                    GimbalPivots.AddNozzleDerivative (ax, d, lever, offset, thrustN, tX, 0, fX);
                    GimbalPivots.AddNozzleDerivative (ay, d, lever, offset, thrustN, tY, 0, fY);
                    GimbalPivots.PerDegree (tX);
                    GimbalPivots.PerDegree (tY);
                    GimbalPivots.PerDegree (fX);
                    GimbalPivots.PerDegree (fY);
                    for (int r = 0; r < 3; r++) {
                        Torque [c + r] = tX [r];
                        Torque [c + 3 + r] = tY [r];
                        Force [c + r] = fX [r];
                        Force [c + 3 + r] = fY [r];
                    }
                }
            }

            /// Module columns at the command (x, y): local (a clamped component counts nothing,
            /// what the DLL publishes) or free (every component free).
            internal void Columns (double[] cols, double x, double y, bool free, double[] colX,
                double[] colY)
            {
                for (int r = 0; r < 3; r++)
                    colX [r] = colY [r] = 0.0;
                for (int i = 0; i < 6; i++) {
                    double xi, yi;
                    bool fx, fy;
                    GimbalPivots.PivotCommand (Map, i, x, y, -Box, Box, -Box, Box, out xi, out yi,
                        out fx, out fy);
                    int c = GimbalPivots.ColumnStride * i;
                    var cX = new double[] { cols [c], cols [c + 1], cols [c + 2] };
                    var cY = new double[] { cols [c + 3], cols [c + 4], cols [c + 5] };
                    GimbalPivots.AddPivotColumns (Map, i, free || fx, free || fy, cX, cY, colX, colY);
                }
            }

            internal void Realized (double[] cols, double x, double y, double[] result)
            {
                GimbalPivots.Realized (Map, 6, cols, -Box, Box, -Box, Box, x, y, result);
            }

            /// The module command of a body rotation w = (wx, 0, wz): its components on X0, Y0.
            internal void Command (double wx, double wz, out double x, out double y)
            {
                x = wx * Axes [0] + wz * Axes [2];
                y = wx * Axes [3] + wz * Axes [5];
            }

            internal int ExactAuthority (double[] cols, double[] best)
            {
                return GimbalPivots.ExactAuthority (Map, 6, cols, -Box, Box, -Box, Box, -Box, Box,
                    -Box, Box, best, new double[3]);
            }
        }

        static double Dot (double[] a, double[] b)
        {
            return a [0] * b [0] + a [1] * b [1] + a [2] * b [2];
        }

        static double Lin (double[] colX, double[] colY, double x, double y, int r)
        {
            return colX [r] * x + colY [r] * y;
        }

        static void TestD65ColumnsAndRealized ()
        {
            var m = new Cluster (0.0, -10.83, 0.0, 0.0, ClusterThrustN);
            var fX = new double[3];
            var fY = new double[3];
            var lX = new double[3];
            var lY = new double[3];
            var real = new double[3];
            double perDeg = ClusterThrustN * Math.PI / 180.0;
            m.Columns (m.Force, 0.0, 0.0, true, fX, fY);
            Check (Math.Abs (Plane (fX) / (6.0 * perDeg) - 1.0) < 1e-12 &&
                Math.Abs (Plane (fY) / (6.0 * perDeg) - 1.0) < 1e-12,
                "force columns of a coherent tilt: 6 T pi/180 per degree of the module");
            // an earlier change's premise: the FORCE columns the translation read (805c8a3..1cf5f24) fall like the
            // torque ones: sqrt(31)/6 with 2 components clamped, sqrt(19)/6 at the corner.
            m.Columns (m.Force, 2.0 - 1e-9, 0.8, false, lX, lY);
            double fa = Math.Max (Plane (lX), Plane (lY)) / (6.0 * perDeg);
            m.Columns (m.Force, 2.0 - 1e-9, 2.0 - 1e-9, false, lX, lY);
            double fb = Math.Max (Plane (lX), Plane (lY)) / (6.0 * perDeg);
            Console.WriteLine ("  local FORCE columns in the clamps: " + fa.ToString ("F4") + " (2, 0.8), " +
                fb.ToString ("F4") + " corner, of the free (torque: sqrt(31)/6 = 0.9280, sqrt(19)/6 = 0.7265)");
            Check (Math.Abs (fa - Math.Sqrt (31.0) / 6.0) < 1e-9 && Math.Abs (fb - Math.Sqrt (19.0) / 6.0) < 1e-9,
                "the local force columns fall like the torque columns (sqrt(31)/6, sqrt(19)/6)");
            // Realized: inside the disc it IS the free linear model (torque and force).
            m.Columns (m.Torque, 0.0, 0.0, true, lX, lY);
            m.Realized (m.Torque, 1.2, -1.5, real);
            bool inside = true;
            for (int r = 0; r < 3; r++)
                inside &= Math.Abs (real [r] - Lin (lX, lY, 1.2, -1.5, r)) <= 1e-9 * Plane (lX);
            m.Realized (m.Force, 1.2, -1.5, real);
            for (int r = 0; r < 3; r++)
                inside &= Math.Abs (real [r] - Lin (fX, fY, 1.2, -1.5, r)) <= 1e-9 * Plane (fX);
            Check (inside, "Realized inside the disc = the free columns times (x, y), torque and force");
            // At the corner it lies between the local and the free linear models (the ground
            // check's bracket, 0.57 and 0.89 measured).
            double e = 2.0 - 1e-9;
            var tX = new double[3];
            var tY = new double[3];
            m.Columns (m.Torque, e, e, false, tX, tY);
            m.Realized (m.Torque, e, e, real);
            double loc = Math.Sqrt (Math.Pow (Lin (tX, tY, e, e, 0), 2) + Math.Pow (Lin (tX, tY, e, e, 2), 2));
            double fre = Math.Sqrt (Math.Pow (Lin (lX, lY, e, e, 0), 2) + Math.Pow (Lin (lX, lY, e, e, 2), 2));
            Check (loc < Plane (real) && Plane (real) < fre,
                "corner: local B.u < realized torque < free B.u (the clamps hold their bound)");
            // Beyond the box the lease clamps its target: Realized too.
            var beyond = new double[3];
            m.Realized (m.Torque, 5.0, -7.0, beyond);
            m.Realized (m.Torque, 2.0, -2.0, real);
            Check (Math.Abs (beyond [0] - real [0]) + Math.Abs (beyond [2] - real [2]) < 1e-9 * Plane (lX),
                "Realized clamps the module command to its box first (as the lease)");
        }

        /// K = sum fX aX^T + fY aY^T (x, z rows and columns) of one module whose axes are X0, Y0:
        /// the tilt matrix of the translation (AttitudeV1Translation.ApplyTranslationColumns).
        static double[] TiltMatrix (Cluster m, double[] fX, double[] fY)
        {
            double aXx = m.Axes [0], aXz = m.Axes [2], aYx = m.Axes [3], aYz = m.Axes [5];
            return new[] {
                fX [0] * aXx + fY [0] * aYx, fX [0] * aXz + fY [0] * aYz,
                fX [2] * aXx + fY [2] * aYx, fX [2] * aXz + fY [2] * aYz };
        }

        static void TestD65TranslationModel ()
        {
            // One module, X0 along body x (the pivot axes every 30 deg from body x). Pure
            // translation (no attitude part): the realized deflection is the common one.
            var m = new Cluster (3.0, -10.83, 0.0, 0.0, ClusterThrustN);
            var fX = new double[3];
            var fY = new double[3];
            var lX = new double[3];
            var lY = new double[3];
            m.Columns (m.Force, 0.0, 0.0, true, fX, fY);
            double[] kFree = TiltMatrix (m, fX, fY);
            // (a) Along pivot 1's x axis (60 deg from X0: the module command (0.5 r, -0.87 r) stays
            // in its box): the local tilt matrix is the free one inside the disc and drops to 4/6
            // as soon as |w| passes the range (X_1 and X_4 clamp).
            double ux1 = m.Axes [GimbalPivots.AxesStride], uz1 = m.Axes [GimbalPivots.AxesStride + 2];
            double[] radii = { 1.9, 2.0 - 1e-9, 2.0 + 1e-6, 2.1 };
            var gain = new double[radii.Length];
            for (int k = 0; k < radii.Length; k++) {
                double x, y;
                m.Command (radii [k] * ux1, radii [k] * uz1, out x, out y);
                m.Columns (m.Force, x, y, false, lX, lY);
                double[] kLoc = TiltMatrix (m, lX, lY);
                gain [k] = Math.Sqrt (Math.Pow (kLoc [0] * ux1 + kLoc [1] * uz1, 2) + Math.Pow (kLoc [2] * ux1 + kLoc [3] * uz1, 2)) /
                    Math.Sqrt (Math.Pow (kFree [0] * ux1 + kFree [1] * uz1, 2) + Math.Pow (kFree [2] * ux1 + kFree [3] * uz1, 2));
            }
            Console.WriteLine ("  tilt matrix read on the LOCAL force columns, along a pivot axis, |w| = 1.9 / 2- / 2+ / 2.1 deg: " +
                gain [0].ToString ("F3") + " / " + gain [1].ToString ("F3") + " / " + gain [2].ToString ("F3") +
                " / " + gain [3].ToString ("F3") + " of the free one");
            Check (Math.Abs (gain [0] - 1.0) < 1e-9 && Math.Abs (gain [1] - 1.0) < 1e-9,
                "inside the disc: local tilt matrix = free");
            Check (Math.Abs (gain [2] - 4.0 / 6.0) < 1e-6 && Math.Abs (gain [3] - 4.0 / 6.0) < 1e-6,
                "past |w| = range along a pivot axis the local tilt matrix jumps to 4/6");
            // The solved tilt for the SAME wanted force then jumps by 1.5 (positive feedback: a
            // larger tilt clamps more). With the free matrix it does not move.
            double px, pz, qx, qz;
            {
                double x, y;
                m.Command ((2.0 + 1e-6) * ux1, (2.0 + 1e-6) * uz1, out x, out y);
                m.Columns (m.Force, x, y, false, lX, lY);
            }
            // The force of the 2 deg tilt along that axis.
            double fx0 = (kFree [0] * ux1 + kFree [1] * uz1) * 2.0, fz0 = (kFree [2] * ux1 + kFree [3] * uz1) * 2.0;
            AttitudeTranslation.SolveTilt (TiltMatrix (m, lX, lY), fx0, fz0, out px, out pz);
            AttitudeTranslation.SolveTilt (kFree, fx0, fz0, out qx, out qz);
            double jump = Math.Sqrt (px * px + pz * pz) / Math.Sqrt (qx * qx + qz * qz);
            Check (Math.Abs (Math.Sqrt (qx * qx + qz * qz) - 2.0) < 1e-9 && Math.Abs (jump - 1.5) < 1e-6,
                "the same force asked at |w| = 2+ deg: the local matrix solves 3 deg, the free one 2");
            // (b) At the 0.8 box's diagonal (module command (1.6, 1.6), |w| = 2.26 deg: 4
            // components clamp), the translation's cap on this module.
            double c = 0.8 * Cluster.Box;
            var tX = new double[3];
            var tY = new double[3];
            var tFX = new double[3];
            var tFY = new double[3];
            var real = new double[3];
            var realF = new double[3];
            m.Columns (m.Torque, c, c, false, tX, tY);
            m.Columns (m.Torque, c, c, true, tFX, tFY);
            m.Columns (m.Force, c, c, false, lX, lY);
            m.Realized (m.Torque, c, c, real);
            m.Realized (m.Force, c, c, realF);
            double wHat = Math.Sqrt (0.5);
            // Tilt direction = w / |w|, in body x z: w = c X0 + c Y0.
            double ux = (m.Axes [0] + m.Axes [3]) * wHat, uz = (m.Axes [2] + m.Axes [5]) * wHat;
            double[] kLocD = TiltMatrix (m, lX, lY);
            double gDiag = Math.Sqrt (Math.Pow (kLocD [0] * ux + kLocD [1] * uz, 2) + Math.Pow (kLocD [2] * ux + kLocD [3] * uz, 2)) /
                Math.Sqrt (Math.Pow (kFree [0] * ux + kFree [1] * uz, 2) + Math.Pow (kFree [2] * ux + kFree [3] * uz, 2));
            // Feed-forward of the common deflection's torque: the local columns times c (805c8a3..
            // 1cf5f24) against the torque the module realizes (pitch/yaw plane).
            double ffLocal = Math.Sqrt (Math.Pow (Lin (tX, tY, c, c, 0), 2) + Math.Pow (Lin (tX, tY, c, c, 2), 2));
            double ffFree = Math.Sqrt (Math.Pow (Lin (tFX, tFY, c, c, 0), 2) + Math.Pow (Lin (tFX, tFY, c, c, 2), 2));
            double tReal = Plane (real);
            double fFree = Math.Sqrt (Math.Pow (Lin (fX, fY, c, c, 0), 2) + Math.Pow (Lin (fX, fY, c, c, 2), 2));
            Console.WriteLine ("  at the 0.8 box diagonal (|w| 2.26 deg): local tilt matrix " + gDiag.ToString ("F3") +
                " of the free along w; feed-forward local B.c / realized torque " + (ffLocal / tReal).ToString ("F3") +
                ", free B.c / realized " + (ffFree / tReal).ToString ("F3") + "; realized force / free model " +
                (Plane (realF) / fFree).ToString ("F3"));
            Check (Math.Abs (gDiag - (6.0 - 4.0 * Math.Pow (Math.Cos (Math.PI / 12.0), 2)) / 6.0) < 1e-6,
                "at the 0.8 box diagonal the local tilt matrix is (6 - 4 cos^2 15)/6 = 0.378 of the free");
            Check (ffLocal / tReal < 0.45,
                "the feed-forward read on the local columns cancels under 45 % of the common deflection's torque");
            Check (ffFree / tReal > 1.0 && ffFree / tReal < 1.06,
                "the free model overestimates that torque by under 6 %");
            Check (Plane (realF) / fFree > 0.94 && Plane (realF) / fFree < 1.0,
                "the fix: the free tilt matrix gets at least 94 % of the wanted force at the cap (undershoot, no jump)");
            // (c) The realized torque and force are continuous across |w| = range, the local
            // model is not: a sweep along pivot 1's axis.
            double prevReal = double.NaN, maxRealStep = 0.0, maxLocalStep = 0.0, prevLocal = double.NaN;
            for (int k = 0; k <= 400; k++) {
                double r = 1.8 + 0.001 * k, x, y;
                m.Command (r * ux1, r * uz1, out x, out y);
                m.Realized (m.Torque, x, y, real);
                m.Columns (m.Torque, x, y, false, tX, tY);
                double rt = Plane (real);
                double lt = Math.Sqrt (Math.Pow (Lin (tX, tY, x, y, 0), 2) + Math.Pow (Lin (tX, tY, x, y, 2), 2));
                if (k > 0) {
                    maxRealStep = Math.Max (maxRealStep, Math.Abs (rt - prevReal));
                    maxLocalStep = Math.Max (maxLocalStep, Math.Abs (lt - prevLocal));
                }
                prevReal = rt;
                prevLocal = lt;
            }
            double perStep = 6.0 * 10.83 * ClusterThrustN * Math.PI / 180.0 * 0.001;
            Check (maxRealStep < 1.01 * perStep && maxLocalStep > 100.0 * perStep,
                "sweep 1.8-2.2 deg: the realized torque moves by at most one free step, B_local.c jumps by a third");
            // (d) The announced authority (LateralAuthority) does not read the force columns; its
            // differential cap reads the full common torque range sigma(Kt): at the corner the local
            // Kt falls, so the announced authority RISES there (never collapses).
            double e = 2.0 - 1e-9;
            m.Columns (m.Torque, e, e, false, tX, tY);
            m.Columns (m.Torque, 0.0, 0.0, true, tFX, tFY);
            double[] ktLoc = TiltMatrix (m, tX, tY), ktFree = TiltMatrix (m, tFX, tFY);
            double fullLoc = 2.0 * AttitudeTranslation.MaxSingular2 (ktLoc [0], ktLoc [1], ktLoc [2], ktLoc [3]);
            double fullFree = 2.0 * AttitudeTranslation.MaxSingular2 (ktFree [0], ktFree [1], ktFree [2], ktFree [3]);
            double diffAuth = 0.5 * fullFree;
            double aLoc = AttitudeTranslation.LateralAuthority (9.81, 2.0, 0.8, 0.8, diffAuth, fullLoc);
            double aFree = AttitudeTranslation.LateralAuthority (9.81, 2.0, 0.8, 0.8, diffAuth, fullFree);
            Console.WriteLine ("  announced lateral authority, differential-limited: " + aFree.ToString ("F4") +
                " m/s2 with the free Kt, " + aLoc.ToString ("F4") + " with the corner's local Kt (full common torque " +
                (fullLoc / fullFree).ToString ("F3") + " of the free)");
            Check (fullLoc < fullFree && aLoc > aFree,
                "is not an authority collapse: the local Kt makes the announced authority optimistic");
            // At |w| = range (the full common torque's tilt) nothing clamps: the free Kt is exact.
            double wx = 2.0 * m.Axes [0], wz = 2.0 * m.Axes [2], xx, yy;
            m.Command (wx, wz, out xx, out yy);
            m.Realized (m.Torque, xx, yy, real);
            Check (Math.Abs (Plane (real) - Math.Sqrt (Math.Pow (Lin (tFX, tFY, xx, yy, 0), 2) +
                Math.Pow (Lin (tFX, tFY, xx, yy, 2), 2))) < 1e-9 * Plane (real),
                "the full common torque's tilt (|w| = range) is inside the disc: free = realized");
        }

        // The heavy booster (a recorded flight, dynamics_v3 snapshot at UT 19782.3): the 8
        // AR-1 Cluster parts' nozzle centroids, body x and z from the CoM (6 nozzles each on a
        // 1.329 m circle, the 8 parts turned 45 deg apart ABOUT THE SAME AXIS).
        internal static readonly double[,] HeavyCentroids = {
            { 0.1171, -0.0633 }, { 0.0305, -0.1209 }, { -0.0715, -0.1004 }, { -0.1291, -0.0139 },
            { -0.1086, 0.0881 }, { -0.0220, 0.1458 }, { 0.0800, 0.1253 }, { 0.1376, 0.0387 } };

        static void TestD65HeavyBoosterRefused ()
        {
            const int n = 8;
            var slope = new double[n];
            var x = new double[n];
            var z = new double[n];
            var mp = new double[n];
            var my = new double[n];
            double maxLever = 0.0, xm = 0.0, zm = 0.0;
            for (int i = 0; i < n; i++) {
                slope [i] = 4500e3;
                x [i] = HeavyCentroids [i, 0];
                z [i] = HeavyCentroids [i, 1];
                xm += x [i] / n;
                zm += z [i] / n;
            }
            for (int i = 0; i < n; i++)
                maxLever = Math.Max (maxLever, Math.Max (Math.Abs (x [i] - xm), Math.Abs (z [i] - zm)));
            var p = new TranslationParams ();
            bool motifs = AttitudeTranslation.Motifs (n, slope, x, z, p.MinLeverArmM, mp, my);
            Console.WriteLine ("  heavy booster: largest motif lever " + maxLever.ToString ("F3") + " m < " +
                p.MinLeverArmM + " m: no differential, the translation is refused (state 4, bit 6) before any gimbal geometry");
            Check (!motifs && maxLever < p.MinLeverArmM,
                "cannot reach the heavy booster: its 8 clusters share one axis, no differential lever, translation refused");
            // Computed levers (attitude_translation_v1 bit 4, modes 5/7): the criterion is the
            // levers' numerical resolution (1e-5 |r|), not 0.25 m: the same centroids are accepted.
            var y = new double[n];
            for (int i = 0; i < n; i++)
                y [i] = -10.83;
            double crit = AttitudeTranslation.LeverResolution (n, x, y, z);
            Check (AttitudeTranslation.Motifs (n, slope, x, z, crit, mp, my) && crit < 1e-3 * maxLever,
                "bit 4: the heavy booster's motifs accepted (criterion " + crit.ToString ("E2") + " m against " +
                maxLever.ToString ("F3") + " m)");
            // The refusal is geometric: the same engines on a 1 m ring would be accepted.
            for (int i = 0; i < n; i++) {
                double a = Math.PI / 4.0 * i;
                x [i] = Math.Cos (a);
                z [i] = Math.Sin (a);
            }
            Check (AttitudeTranslation.Motifs (n, slope, x, z, p.MinLeverArmM, mp, my),
                "the same eight engines on a 1 m ring: motifs accepted");
        }

        static void TestD65ExactAuthority ()
        {
            var best = new double[3];
            var real = new double[3];
            var fX = new double[3];
            var fY = new double[3];
            // (a) Clusters at several orientations: the exact maximum is never below a 161 x 161
            // grid of the box, and the corners alone can miss it.
            double[] phis = { 0.0, 7.0, 15.0, 22.5, 30.0, 45.0 };
            bool neverBelowGrid = true, freeAbove = true;
            double cornerAtZero = double.NaN, freeOverExactAt30 = double.NaN;
            int pointsMax = 0;
            foreach (double phi0 in phis) {
                var m = new Cluster (0.0, -10.83, 0.0, phi0, ClusterThrustN);
                pointsMax = Math.Max (pointsMax, m.ExactAuthority (m.Torque, best));
                m.Columns (m.Torque, 0.0, 0.0, true, fX, fY);
                for (int a = 0; a < 3; a += 2) {
                    double grid = 0.0, corners = 0.0;
                    for (int ix = 0; ix <= 160; ix++)
                        for (int iy = 0; iy <= 160; iy++) {
                            m.Realized (m.Torque, -2.0 + 0.025 * ix, -2.0 + 0.025 * iy, real);
                            grid = Math.Max (grid, Math.Abs (real [a]));
                            if ((ix == 0 || ix == 160) && (iy == 0 || iy == 160))
                                corners = Math.Max (corners, Math.Abs (real [a]));
                        }
                    double free = (Math.Abs (fX [a]) + Math.Abs (fY [a])) * 2.0;
                    neverBelowGrid &= best [a] >= grid * (1.0 - 1e-12);
                    freeAbove &= free >= best [a] * (1.0 - 1e-12);
                    if (phi0 == 0.0 && a == 0)
                        cornerAtZero = corners / best [a];
                    if (phi0 == 30.0 && a == 0)
                        freeOverExactAt30 = free / best [a];
                }
            }
            Console.WriteLine ("  exact authority (<= " + pointsMax + " points per module): X0 on pitch, corners give " +
                cornerAtZero.ToString ("F3") + " of it; X0 at 30 deg, free columns x span give " +
                freeOverExactAt30.ToString ("F3") + " of it");
            Check (neverBelowGrid, "exact authority never below a 161 x 161 grid of the box (6 orientations)");
            Check (freeAbove, "the free authority (1cf5f24) is never below the exact one on the cluster");
            Check (Math.Abs (cornerAtZero - 5.0 / 6.0) < 1e-6,
                "X0 on pitch: the corners give 5/6 of the maximum (reached mid-edge, |w| = range): corners alone are not exact");
            Check (Math.Abs (freeOverExactAt30 - 1.2) < 1e-4, "X0 at 30 deg: the free authority is 1.2 x the exact one");
            // (b) Arbitrary pivot sets (2 to 6 pivots, random azimuths): never below the grid.
            var rng = new Random (65);
            bool randomOk = true;
            for (int trial = 0; trial < 12; trial++) {
                int count = 2 + trial % 5;
                var axes = new double[GimbalPivots.AxesStride * count];
                var map = new double[GimbalPivots.MapStride * count];
                var cols = new double[GimbalPivots.ColumnStride * count];
                for (int i = 0; i < count; i++)
                    PutTurned (axes, i, rng.NextDouble () * 360.0);
                if (!GimbalPivots.Build (axes, count, map))
                    continue;
                for (int i = 0; i < count; i++) {
                    int a = GimbalPivots.AxesStride * i, c = GimbalPivots.ColumnStride * i;
                    double w = 0.5 + rng.NextDouble ();
                    for (int r = 0; r < 3; r++) {
                        cols [c + r] = w * axes [a + r] + 0.1 * (rng.NextDouble () - 0.5);
                        cols [c + 3 + r] = w * axes [a + 3 + r] + 0.1 * (rng.NextDouble () - 0.5);
                    }
                }
                double x0 = -2.0, x1 = 2.0, y0 = -1.5, y1 = 2.0;      // an asymmetric box
                GimbalPivots.ExactAuthority (map, count, cols, x0, x1, y0, y1, x0, x1, y0, y1, best, new double[3]);
                for (int a = 0; a < 3; a++) {
                    double grid = 0.0;
                    for (int ix = 0; ix <= 120; ix++)
                        for (int iy = 0; iy <= 120; iy++) {
                            GimbalPivots.Realized (map, count, cols, x0, x1, y0, y1,
                                x0 + (x1 - x0) * ix / 120.0, y0 + (y1 - y0) * iy / 120.0, real);
                            grid = Math.Max (grid, Math.Abs (real [a]));
                        }
                    randomOk &= best [a] >= grid * (1.0 - 1e-12);
                }
            }
            Check (randomOk, "exact authority never below the grid on 12 random pivot sets (asymmetric box)");
            // (c) The heavy booster: 8 parts turned 45 deg apart, pivot 0 at azimuth -170.64 deg
            // (a recorded flight). The free authority of its engines is 1.16 x the exact one.
            double freeSum0 = 0.0, freeSum2 = 0.0, exact0 = 0.0, exact2 = 0.0, corner0 = 0.0;
            for (int k = 0; k < 8; k++) {
                var m = new Cluster (0.0, -10.83, 0.0, -170.64 + 45.0 * k, ClusterThrustN);
                m.ExactAuthority (m.Torque, best);
                m.Columns (m.Torque, 0.0, 0.0, true, fX, fY);
                freeSum0 += (Math.Abs (fX [0]) + Math.Abs (fY [0])) * 2.0;
                freeSum2 += (Math.Abs (fX [2]) + Math.Abs (fY [2])) * 2.0;
                exact0 += best [0];
                exact2 += best [2];
                double cmax = 0.0;
                for (int s = 0; s < 4; s++) {
                    m.Realized (m.Torque, (s & 1) == 0 ? -2.0 : 2.0, (s & 2) == 0 ? -2.0 : 2.0, real);
                    cmax = Math.Max (cmax, Math.Abs (real [0]));
                }
                corner0 += cmax;
            }
            Console.WriteLine ("  heavy booster engines: free / exact authority " + (freeSum0 / exact0).ToString ("F4") +
                " pitch, " + (freeSum2 / exact2).ToString ("F4") + " yaw; corners only " + (corner0 / exact0).ToString ("F4") +
                " of the exact; exact " + (exact0 / 1e6).ToString ("F2") + " MN.m in pitch at 4372 kN per module");
            Check (Math.Abs (freeSum0 / exact0 - 1.159) < 0.005 && Math.Abs (freeSum2 / exact2 - 1.159) < 0.005,
                "heavy booster: the free authority overestimates its engines' exact authority by 16 %");
            Check (corner0 / exact0 > 0.95 && corner0 / exact0 < 0.99,
                "heavy booster: the corners alone would underestimate it by 3 %");
        }

        static void TestD65AuthorityColumns ()
        {
            // Mode 2 feeds the law free columns scaled so that sum |B| span = exact: alpha reads
            // the exact authority, through the unchanged AttitudeMath.
            var m = new Cluster (0.0, -10.83, 0.0, 22.5, ClusterThrustN);
            var fX = new double[3];
            var fY = new double[3];
            var best = new double[3];
            var scale = new double[3];
            m.Columns (m.Torque, 0.0, 0.0, true, fX, fY);
            m.ExactAuthority (m.Torque, best);
            GimbalPivots.AuthorityScale (fX, fY, 2.0, 2.0, best, scale);
            var inp = new AttitudeInput ();
            inp.Resize (2);
            inp.Dt = 0.02;
            inp.OmegaDotValid = true;
            inp.Inertia [0] = inp.Inertia [1] = inp.Inertia [2] = 1e6;
            for (int r = 0; r < 3; r++) {
                inp.B [r * 2] = fX [r];
                inp.B [r * 2 + 1] = fY [r];
                inp.BAuthority [r * 2] = fX [r] * scale [r];
                inp.BAuthority [r * 2 + 1] = fY [r] * scale [r];
            }
            for (int j = 0; j < 2; j++) {
                inp.U0 [j] = 0.0;
                inp.UMin [j] = -2.0;
                inp.UMax [j] = 2.0;
                inp.Priority [j] = 1.0;
            }
            inp.UseBAuthority = true;
            var o = new AttitudeOutput ();
            AttitudeMath.Step (new AttitudeGains (), inp, new AttitudeLawState (), new AttitudeWork (), o);
            Check (o.Filtered && Math.Abs (o.Alpha [0] * 1e6 - best [0]) < 1e-6 * best [0] &&
                Math.Abs (o.Alpha [2] * 1e6 - best [2]) < 1e-6 * best [2],
                "mode 2: alpha = the exact authority / I through the unchanged law");
            Check (scale [0] < 1.0 && scale [0] > 0.8, "mode 2 at 22.5 deg: the free authority scaled down (" +
                scale [0].ToString ("F3") + ")");
            var zero = new double[3];
            var nan = new double[] { double.NaN, 1.0, 1.0 };
            GimbalPivots.AuthorityScale (zero, zero, 2.0, 2.0, best, scale);
            bool unit = scale [0] == 1.0 && scale [1] == 1.0 && scale [2] == 1.0;
            GimbalPivots.AuthorityScale (fX, fY, 2.0, 2.0, nan, scale);
            Check (unit && scale [0] == 1.0, "null free authority or non-finite exact: factor 1 (free kept)");
            // An empty domain (a locked module with no span): no point, zero authority.
            Check (GimbalPivots.ExactAuthority (m.Map, 6, m.Torque, -2, 2, -2, 2, 0.5, 0.4, -2, 2, best,
                new double[3]) == 0 && best [0] == 0.0, "empty domain: zero points, zero authority");
        }
    }
}
