using System;

namespace KRPC.Bridge.Model
{
    /// <summary>
    /// the authority of the gimbals per kN of thrust, without thrust, pivots included -- the
    /// pure part of conn.control.gimbal_authority_v1 (Control/GimbalAuthorityRead.cs). System only:
    /// compiled as is by build/tests (GimbalAuthorityTests.cs).
    ///
    /// WHY. Every gimbal column is proportional to the engine's finalThrust, so kRPC and the law's columns
    /// read nothing without thrust (ModuleGimbal.GetPotentialTorque, Engine.AvailableTorque): at the flip
    /// the autopilot regresses 24 frames of the V3 ring (25 RPCs) for the authority of its throttle of
    /// calage, and refuses a gimbal module with several pivots, so the heavy booster serves a profile
    /// under an earlier change. The geometry is already computed per pivot; taken at the REST pose of
    /// every pivot and at one kN, it is the authority per kN, read in one call.
    ///
    /// THE REST POSE. ModuleGimbal turns pivot i by localRotation = initRots[i] x (its two rotations); at
    /// rest its world rotation is parent.rotation x initRots[i] (the q0 of AttitudeGeometry). A nozzle under
    /// the pivot is carried rigidly: from the pivot's world rotation now R to its rest rotation Q, the
    /// nozzle's direction is (Q R^-1) d and its position p + (Q R^-1) (n - p), p the pivot's position.
    /// </summary>
    internal static class GimbalAuthority
    {
        /// <summary>Hamilton product a b of unit quaternions stored (x, y, z, w), Unity's convention.</summary>
        internal static void Multiply (double[] a, double[] b, double[] result)
        {
            double x = a [3] * b [0] + a [0] * b [3] + a [1] * b [2] - a [2] * b [1];
            double y = a [3] * b [1] - a [0] * b [2] + a [1] * b [3] + a [2] * b [0];
            double z = a [3] * b [2] + a [0] * b [1] - a [1] * b [0] + a [2] * b [3];
            double w = a [3] * b [3] - a [0] * b [0] - a [1] * b [1] - a [2] * b [2];
            result [0] = x;
            result [1] = y;
            result [2] = z;
            result [3] = w;
        }

        /// <summary>The conjugate (the inverse of a unit quaternion), (x, y, z, w).</summary>
        internal static void Conjugate (double[] q, double[] result)
        {
            result [0] = -q [0];
            result [1] = -q [1];
            result [2] = -q [2];
            result [3] = q [3];
        }

        /// <summary>q v q^-1: the vector v rotated by the unit quaternion q (Unity's q * v).</summary>
        internal static void Rotate (double[] q, double[] v, double[] result)
        {
            // t = 2 (q_v x v); v' = v + w t + q_v x t
            double tx = 2.0 * (q [1] * v [2] - q [2] * v [1]);
            double ty = 2.0 * (q [2] * v [0] - q [0] * v [2]);
            double tz = 2.0 * (q [0] * v [1] - q [1] * v [0]);
            double x = v [0] + q [3] * tx + (q [1] * tz - q [2] * ty);
            double y = v [1] + q [3] * ty + (q [2] * tx - q [0] * tz);
            double z = v [2] + q [3] * tz + (q [0] * ty - q [1] * tx);
            result [0] = x;
            result [1] = y;
            result [2] = z;
        }

        /// <summary>
        /// A nozzle carried from its pivot's rotation now (now, world) to the pivot's rest rotation (rest,
        /// world): restPosition = pivot + (rest now^-1) (position - pivot), restDirection = (rest now^-1)
        /// direction. Quaternions (x, y, z, w), unit. Allocates (a read, not a physics step).
        /// </summary>
        internal static void RestNozzle (double[] pivot, double[] now, double[] rest, double[] position,
            double[] direction, double[] restPosition, double[] restDirection)
        {
            var inverse = new double[4];
            var delta = new double[4];
            Conjugate (now, inverse);
            Multiply (rest, inverse, delta);
            var offset = new double[3];
            for (int r = 0; r < 3; r++)
                offset [r] = position [r] - pivot [r];
            var carried = new double[3];
            Rotate (delta, offset, carried);
            for (int r = 0; r < 3; r++)
                restPosition [r] = pivot [r] + carried [r];
            Rotate (delta, direction, restDirection);
        }

        /// <summary>
        /// A module's authority on each body axis, the law's alpha rule: |colX_a| spanX + |colY_a| spanY
        /// (columns per degree, spans in degrees: the larger bound of each local axis).
        /// </summary>
        internal static void ModuleAuthority (double[] colX, double[] colY, double spanX, double spanY,
            double[] result)
        {
            for (int a = 0; a < 3; a++)
                result [a] = Math.Abs (colX [a]) * spanX + Math.Abs (colY [a]) * spanY;
        }

        /// <summary>The larger of |min| and |max|: the span the law's alpha reads (AttitudeMath).</summary>
        internal static double Span (double min, double max)
        {
            return Math.Abs (max) > Math.Abs (min) ? Math.Abs (max) : Math.Abs (min);
        }
    }
}
