using System;

namespace KRPC.Bridge.Model
{
    /// <summary>
    /// Gimbal modules with SEVERAL pivots
    /// (ModuleGimbal.gimbalTransforms) whose frames are not parallel. Pure maths (System only):
    /// compiled as is by build/tests (PivotTests.cs).
    ///
    /// The case met in flight: Near Future Launch Vehicles' AR-1 Cluster (nflv-engine-ar1c-1,
    /// a heavy booster: 8 such parts), one ModuleGimbal over SIX pivots named
    /// B_AR1C_Gimbal, one per nozzle, each turned about the thrust axis to its nozzle's azimuth.
    ///
    /// Stock (ModuleGimbal.FixedUpdate and GimbalRotation, Assembly-CSharp decompiled)
    /// computes the deflection PER PIVOT: the vessel-frame command is brought into
    /// the pivot's own frame, t.InverseTransformDirection(ReferenceTransform.TransformDirection(
    /// PYR)), and its x and y components (clamped to +-1, times the ranges and the limiter) are
    /// the rotations of THAT pivot. Every nozzle tilts the same way in the world: pitch/yaw torque.
    ///
    /// The bridge lease (Actuation.ApplyGimbal) and the AttitudeV1 columns
    /// (AttitudeGeometry.GimbalColumns) apply ONE (x, y) to every pivot in the pivot's OWN frame.
    /// On the AR-1 Cluster the six nozzles then tilt in six directions 60 deg apart: a swirl,
    /// roll torque (sum r T per degree), no pitch/yaw torque. Measured on a recorded flight:
    /// published columns 350-600 N.m/deg in pitch/yaw and 1.014e5 N.m/deg in roll per part at
    /// 4372 kN (swirl model 1.016-1.018e5), against 8.3e5 N.m/deg of pitch/yaw a coherent tilt
    /// gives; realized engine torque under the lease ~250 N.m against 1.4 MN.m of RCS.
    ///
    /// The mapping (AttitudeGimbalPivotsV1 mode 1, per armed vessel, born closed): the module's
    /// command (x, y) is a rotation in the frame of pivot 0 at rest, w = x X0 + y Y0 (X_i, Y_i:
    /// world unit axes of pivot i's lease rotations, xMult/yMult signs included); pivot i turns by
    /// the projection of w on its own axes, (x_i, y_i) = (w . X_i, w . Y_i) -- the stock rule
    /// with pivot 0's frame in place of the vessel's. Each component is clamped to the module's
    /// box, as stock clamps each one. A module with ONE pivot, or whose pivots are all parallel
    /// to pivot 0, is NOT mapped: the V1 code runs, unchanged (the barge, medium and small
    /// boosters: one pivot per module).
    /// </summary>
    internal static class GimbalPivots
    {
        internal const int Version = 1;
        internal const int ModeV1 = 0;
        internal const int ModeMapped = 1;
        /// <summary>
        /// (born closed): mode 1, and the law's authority of a mapped module is its EXACT
        /// largest realized torque over the law's box (ExactAuthority) instead of the free
        /// columns times the span (1cf5f24, +16 % on the heavy booster's engines).
        /// </summary>
        internal const int ModeExactAuthority = 2;
        /// <summary>Pivot i is parallel to pivot 0 when both its unit axes are within this
        /// (1 - cos) of pivot 0's.</summary>
        internal const double ParallelTolerance = 1e-6;
        internal const int MapStride = 4;
        internal const int AxesStride = 6;
        /// <summary>per pivot i, its column per unit of x_i at [6i, 6i+2], of y_i at [6i+3, 6i+5].</summary>
        internal const int ColumnStride = 6;
        const double DegToRad = Math.PI / 180.0;

        internal static bool ValidMode (int mode)
        {
            return mode == ModeV1 || mode == ModeMapped || mode == ModeExactAuthority;
        }

        /// <summary>Modes that map a module with non-parallel pivots per pivot (1 and 2).</summary>
        internal static bool Mapped (int mode)
        {
            return mode == ModeMapped || mode == ModeExactAuthority;
        }

        static double Norm3 (double[] v, int o)
        {
            return Math.Sqrt (v [o] * v [o] + v [o + 1] * v [o + 1] + v [o + 2] * v [o + 2]);
        }

        static double DotUnit (double[] a, int oa, double na, double[] b, int ob, double nb)
        {
            return (a [oa] * b [ob] + a [oa + 1] * b [ob + 1] + a [oa + 2] * b [ob + 2]) / (na * nb);
        }

        /// <summary>
        /// axes: per pivot i, X_i at [6i, 6i+2] and Y_i at [6i+3, 6i+5] (world, any length; a
        /// zero axis = a missing pivot or a zero xMult/yMult). Fills map[4i, 4i+3] = (xx, xy, yx,
        /// yy): pivot i turns by (xx x + xy y, yx x + yy y) for the module command (x, y).
        /// Pivot 0's row is exactly (1, 0, 0, 1). A missing pivot gets a zero row. Returns true
        /// when the module is MAPPED: at least two pivots, pivot 0 usable, and one pivot not
        /// parallel to pivot 0. Otherwise false, and the caller keeps the V1 path.
        /// </summary>
        internal static bool Build (double[] axes, int count, double[] map)
        {
            if (count < 2 || axes == null || map == null || axes.Length < AxesStride * count ||
                    map.Length < MapStride * count)
                return false;
            double nx0 = Norm3 (axes, 0), ny0 = Norm3 (axes, 3);
            if (!(nx0 > 1e-12) || !(ny0 > 1e-12))
                return false;
            bool parallel = true;
            map [0] = 1.0;
            map [1] = 0.0;
            map [2] = 0.0;
            map [3] = 1.0;
            for (int i = 1; i < count; i++) {
                int a = AxesStride * i, m = MapStride * i;
                double nxi = Norm3 (axes, a), nyi = Norm3 (axes, a + 3);
                if (!(nxi > 1e-12) || !(nyi > 1e-12)) {
                    // Missing pivot: never moved by the mapping.
                    map [m] = map [m + 1] = map [m + 2] = map [m + 3] = 0.0;
                    continue;
                }
                map [m] = DotUnit (axes, 0, nx0, axes, a, nxi);          // X0 . Xi
                map [m + 1] = DotUnit (axes, 3, ny0, axes, a, nxi);      // Y0 . Xi
                map [m + 2] = DotUnit (axes, 0, nx0, axes, a + 3, nyi);  // X0 . Yi
                map [m + 3] = DotUnit (axes, 3, ny0, axes, a + 3, nyi);  // Y0 . Yi
                if (map [m] < 1.0 - ParallelTolerance || map [m + 3] < 1.0 - ParallelTolerance)
                    parallel = false;
            }
            return !parallel;
        }

        /// <summary>
        /// Pivot i's rotations (degrees) for the module command (x, y), each clamped to the
        /// module box like stock clamps each component; xFree / yFree = not clamped (the
        /// component follows the command, its derivative counts).
        /// </summary>
        internal static void PivotCommand (double[] map, int i, double x, double y,
            double xMin, double xMax, double yMin, double yMax,
            out double xi, out double yi, out bool xFree, out bool yFree)
        {
            int m = MapStride * i;
            double ux = map [m] * x + map [m + 1] * y;
            double uy = map [m + 2] * x + map [m + 3] * y;
            xFree = ux >= xMin && ux <= xMax;
            yFree = uy >= yMin && uy <= yMax;
            xi = ux < xMin ? xMin : (ux > xMax ? xMax : ux);
            yi = uy < yMin ? yMin : (uy > yMax ? yMax : uy);
        }

        /// <summary>
        /// The module command whose pivot i rotations are (xi, yi) (inverse of pivot i's row;
        /// used to start a lease from stock when only one pivot's deflection is known). False
        /// when the row is singular.
        /// </summary>
        internal static bool ModuleFromPivot (double[] map, int i, double xi, double yi,
            out double x, out double y)
        {
            int m = MapStride * i;
            double det = map [m] * map [m + 3] - map [m + 1] * map [m + 2];
            if (!(Math.Abs (det) > 1e-6)) {
                x = 0.0;
                y = 0.0;
                return false;
            }
            x = (map [m + 3] * xi - map [m + 1] * yi) / det;
            y = (-map [m + 2] * xi + map [m] * yi) / det;
            return true;
        }

        static void Cross (double ax, double ay, double az, double bx, double by, double bz,
            out double cx, out double cy, out double cz)
        {
            cx = ay * bz - az * by;
            cy = az * bx - ax * bz;
            cz = ax * by - ay * bx;
        }

        /// <summary>
        /// d(r x F)/d(theta) of ONE nozzle for a rotation of its pivot about the unit axis a, per
        /// RADIAN (the formula of AttitudeGeometry.TorqueDerivative, in double): F = -thrustN d,
        /// dF = -thrustN (a x d), dLever = a x offset (offset = nozzle - pivot), result = dLever
        /// x F + lever x dF. Added into acc[o, o+2]. The force derivative a x d x -thrustN is
        /// added into force[o, o+2] when force is not null.
        /// </summary>
        internal static void AddNozzleDerivative (double[] a, double[] d, double[] lever,
            double[] offset, double thrustN, double[] acc, int o, double[] force)
        {
            double fx = -thrustN * d [0], fy = -thrustN * d [1], fz = -thrustN * d [2];
            double cx, cy, cz;
            Cross (a [0], a [1], a [2], d [0], d [1], d [2], out cx, out cy, out cz);
            double dfx = -thrustN * cx, dfy = -thrustN * cy, dfz = -thrustN * cz;
            double lx, ly, lz;
            Cross (a [0], a [1], a [2], offset [0], offset [1], offset [2], out lx, out ly, out lz);
            double t1x, t1y, t1z, t2x, t2y, t2z;
            Cross (lx, ly, lz, fx, fy, fz, out t1x, out t1y, out t1z);
            Cross (lever [0], lever [1], lever [2], dfx, dfy, dfz, out t2x, out t2y, out t2z);
            acc [o] += t1x + t2x;
            acc [o + 1] += t1y + t2y;
            acc [o + 2] += t1z + t2z;
            if (force != null) {
                force [o] += dfx;
                force [o + 1] += dfy;
                force [o + 2] += dfz;
            }
        }

        /// <summary>
        /// Chain rule: the module columns d/dx and d/dy (colX, colY) gain pivot i's columns
        /// cXi = d/dx_i and cYi = d/dy_i through its row: d/dx = xx d/dx_i + yx d/dy_i,
        /// d/dy = xy d/dx_i + yy d/dy_i; a clamped component contributes nothing.
        /// </summary>
        internal static void AddPivotColumns (double[] map, int i, bool xFree, bool yFree,
            double[] cXi, double[] cYi, double[] colX, double[] colY)
        {
            int m = MapStride * i;
            double xx = xFree ? map [m] : 0.0, xy = xFree ? map [m + 1] : 0.0;
            double yx = yFree ? map [m + 2] : 0.0, yy = yFree ? map [m + 3] : 0.0;
            for (int r = 0; r < 3; r++) {
                colX [r] += xx * cXi [r] + yx * cYi [r];
                colY [r] += xy * cXi [r] + yy * cYi [r];
            }
        }

        /// <summary>Per radian to per degree, in place (the unit of the published columns).</summary>
        internal static void PerDegree (double[] v)
        {
            v [0] *= DegToRad;
            v [1] *= DegToRad;
            v [2] *= DegToRad;
        }

        // ------------------------------------------------------ realized and exact

        static double Clamp (double v, double lo, double hi)
        {
            return v < lo ? lo : (v > hi ? hi : v);
        }

        /// <summary>
        /// the wrench (torque or force, 3 values) a mapped module REALIZES for the module
        /// command (x, y): the command clamped to the box (the lease clamps its target), each
        /// pivot component clamped (PivotCommand, as stock), times that pivot's columns (cols,
        /// ColumnStride per pivot, per unit of x_i and y_i). Inside the disc where no component
        /// clamps it is the free columns times (x, y); beyond, a clamped component holds its
        /// bound -- neither zero (the local columns) nor growing (the free ones). Writes result[0..2].
        /// </summary>
        internal static void Realized (double[] map, int count, double[] cols, double xMin,
            double xMax, double yMin, double yMax, double x, double y, double[] result)
        {
            result [0] = 0.0;
            result [1] = 0.0;
            result [2] = 0.0;
            double cx = Clamp (x, xMin, xMax), cy = Clamp (y, yMin, yMax);
            for (int i = 0; i < count; i++) {
                double xi, yi;
                bool fx, fy;
                PivotCommand (map, i, cx, cy, xMin, xMax, yMin, yMax, out xi, out yi, out fx, out fy);
                int c = ColumnStride * i;
                result [0] += cols [c] * xi + cols [c + 3] * yi;
                result [1] += cols [c + 1] * xi + cols [c + 4] * yi;
                result [2] += cols [c + 2] * xi + cols [c + 5] * yi;
            }
        }

        // Evaluates one candidate point and keeps the largest |component| per axis.
        static void Keep (double[] map, int count, double[] cols, double xMin, double xMax,
            double yMin, double yMax, double x, double y, double[] best, double[] work)
        {
            Realized (map, count, cols, xMin, xMax, yMin, yMax, x, y, work);
            for (int r = 0; r < 3; r++) {
                double v = Math.Abs (work [r]);
                if (v > best [r])
                    best [r] = v;
            }
        }

        // Clamp line l of the module: a x + b y = v (pivot l / 4, component x or y, bound low
        // or high). False for a missing pivot (zero row).
        static bool Line (double[] map, int l, double xMin, double xMax, double yMin, double yMax,
            out double a, out double b, out double v)
        {
            int i = l >> 2, k = l & 3, m = MapStride * i;
            bool yComponent = k >= 2;
            a = yComponent ? map [m + 2] : map [m];
            b = yComponent ? map [m + 3] : map [m + 1];
            v = yComponent ? ((k & 1) == 0 ? yMin : yMax) : ((k & 1) == 0 ? xMin : xMax);
            return Math.Abs (a) + Math.Abs (b) > 1e-12;
        }

        /// <summary>
        /// the EXACT authority of a mapped module: per body axis r, the largest |Realized_r|
        /// over the domain [x0, x1] x [y0, y1] (the law's span, within the module box). Realized is
        /// piecewise linear, linear in each cell of the arrangement of the clamp lines (a pivot
        /// component at a bound: 4 lines per pivot), so its extremes over the domain lie at the
        /// vertices of that arrangement inside the domain: the four corners, the crossings of the
        /// edges with the clamp lines, and the crossings of two clamp lines. All are evaluated
        /// (6 pivots: at most 4 + 96 + 276 points). The corners alone are NOT enough: a 6-pivot
        /// cluster with X0 along the axis peaks at the middle of an edge, where |w| = range and
        /// nothing clamps (corners 0.83 of the maximum). Writes best[0..2]; work >= 3 values.
        /// Returns the number of points evaluated.
        /// </summary>
        internal static int ExactAuthority (double[] map, int count, double[] cols, double xMin,
            double xMax, double yMin, double yMax, double x0, double x1, double y0, double y1,
            double[] best, double[] work)
        {
            best [0] = 0.0;
            best [1] = 0.0;
            best [2] = 0.0;
            x0 = Math.Max (x0, xMin);
            x1 = Math.Min (x1, xMax);
            y0 = Math.Max (y0, yMin);
            y1 = Math.Min (y1, yMax);
            if (!(x1 >= x0) || !(y1 >= y0) || count <= 0)
                return 0;
            int points = 0;
            // Corners.
            for (int c = 0; c < 4; c++) {
                Keep (map, count, cols, xMin, xMax, yMin, yMax, (c & 1) == 0 ? x0 : x1,
                    (c & 2) == 0 ? y0 : y1, best, work);
                points++;
            }
            int lines = 4 * count;
            double tolX = 1e-9 * Math.Max (1.0, x1 - x0), tolY = 1e-9 * Math.Max (1.0, y1 - y0);
            for (int l = 0; l < lines; l++) {
                double a, b, v;
                if (!Line (map, l, xMin, xMax, yMin, yMax, out a, out b, out v))
                    continue;
                // Crossings with the edges.
                if (Math.Abs (b) > 1e-12) {
                    for (int e = 0; e < 2; e++) {
                        double xe = e == 0 ? x0 : x1;
                        double y = (v - a * xe) / b;
                        if (y >= y0 - tolY && y <= y1 + tolY) {
                            Keep (map, count, cols, xMin, xMax, yMin, yMax, xe, Clamp (y, y0, y1),
                                best, work);
                            points++;
                        }
                    }
                }
                if (Math.Abs (a) > 1e-12) {
                    for (int e = 0; e < 2; e++) {
                        double ye = e == 0 ? y0 : y1;
                        double x = (v - b * ye) / a;
                        if (x >= x0 - tolX && x <= x1 + tolX) {
                            Keep (map, count, cols, xMin, xMax, yMin, yMax, Clamp (x, x0, x1), ye,
                                best, work);
                            points++;
                        }
                    }
                }
                // Crossings with the other clamp lines.
                for (int l2 = l + 1; l2 < lines; l2++) {
                    double a2, b2, v2;
                    if (!Line (map, l2, xMin, xMax, yMin, yMax, out a2, out b2, out v2))
                        continue;
                    double det = a * b2 - a2 * b;
                    if (!(Math.Abs (det) > 1e-9 * (Math.Abs (a * b2) + Math.Abs (a2 * b))))
                        continue;       // parallel (the two bounds of one component, or opposite pivots)
                    double x = (v * b2 - v2 * b) / det;
                    double y = (a * v2 - a2 * v) / det;
                    if (x >= x0 - tolX && x <= x1 + tolX && y >= y0 - tolY && y <= y1 + tolY) {
                        Keep (map, count, cols, xMin, xMax, yMin, yMax, Clamp (x, x0, x1),
                            Clamp (y, y0, y1), best, work);
                        points++;
                    }
                }
            }
            return points;
        }

        /// <summary>
        /// per-axis factor k = exact / free that brings the module's free authority columns
        /// (freeX, freeY: sum_a |free| span = the free linear model's largest torque over the
        /// symmetric span box) to its exact authority: k free = exact on each axis. 1 on an axis
        /// where the free authority is null or a value is not finite.
        /// </summary>
        internal static void AuthorityScale (double[] freeX, double[] freeY, double spanX,
            double spanY, double[] exact, double[] scale)
        {
            for (int r = 0; r < 3; r++) {
                double free = Math.Abs (freeX [r]) * spanX + Math.Abs (freeY [r]) * spanY;
                double k = free > 1e-12 ? exact [r] / free : 1.0;
                scale [r] = double.IsNaN (k) || double.IsInfinity (k) || k < 0.0 ? 1.0 : k;
            }
        }
    }

    /// <summary>
    /// the per-pivot mapping of ONE gimbal module for one step, filled by AttitudeV1 and
    /// carried by the lease (Actuation.GimbalLease.Pivots) so that the lease applies what
    /// the columns modelled. Null anywhere = the V1 path, unchanged.
    /// </summary>
    internal sealed class GimbalPivotCommand
    {
        internal int Count;
        internal double[] Axes = new double[0];
        internal double[] Map = new double[0];
        /// <summary>The module box (degrees): -rangeXN, rangeXP, -rangeYN, rangeYP, times the limiter.</summary>
        internal double XMin, XMax, YMin, YMax;
        /// <summary>Where a NEW lease starts (module command, degrees): pivot 0's stock deflection.</summary>
        internal double StartX, StartY;
        /// <summary>
        /// this step's per-pivot columns, body frame, per degree of the pivot's own rotation
        /// (GimbalPivots.ColumnStride per pivot): torque about the CoM (filled with the module's
        /// attitude columns) and force (filled with its lateral-force geometry). They give the
        /// wrench the module REALIZES for any command (GimbalPivots.Realized).
        /// </summary>
        internal double[] TorqueCols = new double[0];
        internal double[] ForceCols = new double[0];
        internal bool TorqueColsValid, ForceColsValid;

        internal void Resize (int count)
        {
            Count = count;
            if (Axes.Length < GimbalPivots.AxesStride * count)
                Axes = new double[GimbalPivots.AxesStride * count];
            if (Map.Length < GimbalPivots.MapStride * count)
                Map = new double[GimbalPivots.MapStride * count];
            if (TorqueCols.Length < GimbalPivots.ColumnStride * count) {
                TorqueCols = new double[GimbalPivots.ColumnStride * count];
                ForceCols = new double[GimbalPivots.ColumnStride * count];
            }
            TorqueColsValid = false;
            ForceColsValid = false;
        }

        /// <summary>torque (body, N.m) the module realizes for the command (x, y), degrees.</summary>
        internal void RealizedTorque (double x, double y, double[] result)
        {
            GimbalPivots.Realized (Map, Count, TorqueCols, XMin, XMax, YMin, YMax, x, y, result);
        }

        /// <summary>force (body, N) the module realizes for the command (x, y), degrees.</summary>
        internal void RealizedForce (double x, double y, double[] result)
        {
            GimbalPivots.Realized (Map, Count, ForceCols, XMin, XMax, YMin, YMax, x, y, result);
        }

        /// <summary>Pivot i's rotations for the module command (x, y), clamped to the box.</summary>
        internal void Pivot (int i, double x, double y, out double xi, out double yi)
        {
            bool xFree, yFree;
            if (i >= Count) {
                xi = 0.0;
                yi = 0.0;
                return;
            }
            GimbalPivots.PivotCommand (Map, i, x, y, XMin, XMax, YMin, YMax, out xi, out yi,
                out xFree, out yFree);
        }
    }
}
