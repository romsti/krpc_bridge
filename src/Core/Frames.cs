using System;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// Reference frames and units of the mod, written once.
    ///
    /// PURE: System only, doubles only (no Unity type), so the pure model files and the tests
    /// can use it.
    ///
    /// FRAMES. Three are in use, and every field of every layout says which (LayoutV1):
    ///   * BODY: the vessel's ReferenceTransform. x = right (pitch axis), y = forward, the nose
    ///     (roll axis), z = bottom (yaw axis). Vessel.angularVelocity and Vessel.MOI are in it.
    ///     A positive ctrlState command gives a NEGATIVE body torque on its axis (every stock
    ///     effector shares that sign: wheels, RCS, gimbals, surfaces).
    ///   * WORLD: Unity world space (left-handed), floating origin: positions move when KSP
    ///     shifts the origin, directions do not.
    ///   * KRPC: a kRPC ReferenceFrame chosen by the client (directions are converted with
    ///     ReferenceFrame.DirectionToWorldSpace, then to BODY).
    ///
    /// UNITS. The rule for every RPC added from refactor R1 on: SI in, SI out (m, s, kg, N,
    /// N.m, kg.m2, rad, rad/s, Pa, kg/m3). KSP's native units stop at the bridge's door:
    /// tonnes, kN, kN.m and tonne.m2 are converted with the constants below, and a field that
    /// keeps a native unit for wire compatibility says so in its name (mass_tonnes,
    /// final_thrust_kn, moi_native_xyz). Degrees appear only where KSP's own command is in
    /// degrees (gimbal deflection, control-surface angle) and the field name says _deg.
    /// </summary>
    public static class Frames
    {
        /// <summary>Body axis index of pitch (x, right).</summary>
        public const int BodyPitch = 0;

        /// <summary>Body axis index of roll (y, forward).</summary>
        public const int BodyRoll = 1;

        /// <summary>Body axis index of yaw (z, bottom).</summary>
        public const int BodyYaw = 2;

        /// <summary>Frame tags used by LayoutV1 field descriptors.</summary>
        public const string Body = "body";
        public const string World = "world";
        public const string Krpc = "krpc";

        /// <summary>kN to N (KSP forces), also kN.m to N.m.</summary>
        public const double KilonewtonToNewton = 1000.0;

        /// <summary>tonne to kg (KSP masses), also tonne.m2 to kg.m2 (Vessel.MOI).</summary>
        public const double TonneToKilogram = 1000.0;

        /// <summary>Degrees to radians.</summary>
        public const double DegToRad = Math.PI / 180.0;

        /// <summary>Radians to degrees.</summary>
        public const double RadToDeg = 180.0 / Math.PI;

        /// <summary>a x b into result (result may not alias a or b).</summary>
        public static void Cross (double[] a, double[] b, double[] result)
        {
            result [0] = a [1] * b [2] - a [2] * b [1];
            result [1] = a [2] * b [0] - a [0] * b [2];
            result [2] = a [0] * b [1] - a [1] * b [0];
        }

        /// <summary>a . b over three components starting at the given offsets.</summary>
        public static double Dot (double[] a, int ia, double[] b, int ib)
        {
            return a [ia] * b [ib] + a [ia + 1] * b [ib + 1] + a [ia + 2] * b [ib + 2];
        }

        /// <summary>
        /// World to body with the body images of the world basis (x, y, z of world expressed in
        /// body, the columns a caller gets from ReferenceTransform.InverseTransformDirection of
        /// right, up, forward): body = wx * v.x + wy * v.y + wz * v.z.
        /// </summary>
        public static void WorldToBody (double[] worldXInBody, double[] worldYInBody,
            double[] worldZInBody, double[] world, double[] body)
        {
            for (int r = 0; r < 3; r++)
                body [r] = worldXInBody [r] * world [0] + worldYInBody [r] * world [1] +
                    worldZInBody [r] * world [2];
        }

        /// <summary>
        /// Body to world with the same basis (a rotation: the transpose): world_i = image_i . body.
        /// </summary>
        public static void BodyToWorld (double[] worldXInBody, double[] worldYInBody,
            double[] worldZInBody, double[] body, double[] world)
        {
            world [0] = Dot (worldXInBody, 0, body, 0);
            world [1] = Dot (worldYInBody, 0, body, 0);
            world [2] = Dot (worldZInBody, 0, body, 0);
        }

        /// <summary>True when the three values from offset are finite.</summary>
        public static bool Finite3 (double[] v, int offset = 0)
        {
            for (int i = offset; i < offset + 3; i++)
                if (double.IsNaN (v [i]) || double.IsInfinity (v [i]))
                    return false;
            return true;
        }
    }
}
