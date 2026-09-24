using System;

namespace KRPC.Bridge.Actuators
{
    // -----------------------------------------------------------------------------------
    // AttitudeV1 armed mode (plan GNC v3, step M3): whether the loop WRITES an actuator in
    // this physics step or falls back to the kRPC AutoPilot, the roll command it writes,
    // and the check of the vessel's fly-by-wire chain.
    //
    // PURE, like AttitudeMath.cs: System only, no Unity, no KSP, no kRPC type. Compiled
    // three times (the plugin, build/verify, build/tests) and mirrored operation by
    // operation by the autopilot (guidance/attitude_c_loi.py), which replays the golden
    // vectors of build/tests. Keep the arithmetic ORDER when editing.
    //
    // What M3 writes, and why only that:
    //   * ctrlState.roll, from a callback appended AFTER the kRPC AutoPilot's in
    //     Vessel.OnFlyByWire. The AutoPilot stays engaged and keeps pitch and yaw.
    //   * The command is the roll allocation target alone on the ctrlState roll channel,
    //     u = clamp(u0 + (M_d - M)_y / B_yy, -1, 1): the only actuator the loop writes.
    //     B_yy is the pure-roll RCS column, an UPPER bound of the stock ModuleRCS roll
    //     effectiveness (flight B: the realized roll torque is 2 to 7 % of it once pitch
    //     or yaw is commanded, and |u| < 0.05 is a stock dead zone): INDI then converges
    //     monotonically (effectiveness ratio <= 1), slowly rather than overshooting.
    //   * Not under thrust: stock ModuleGimbal mixes ctrlState.roll into the deflections
    //     (GimbalRotation), a roll effectiveness the ctrlState column does not contain.
    //     rho = gimbal roll authority / ctrlState roll effectiveness; above
    //     RollCouplingMax the loop does not write (flight B: rho = 0 in flip and coast,
    //     median 1.2-1.5 and up to 7 in boostback and descent).
    // -----------------------------------------------------------------------------------
    internal static class AttitudeWrite
    {
        // Modes and arm bits (attitude_arm_v1).
        internal const int ModeShadow = 0;
        internal const int ModeArmed = 1;
        internal const int ArmRoll = 1;          // body y
        internal const int ArmPitchYaw = 2;      // body x and z: refused armed by this build

        // Stage codes (frame header offset 7).
        internal const int StageEarlyish = 1;
        internal const int StagePumpFallback = 2;

        // Motif bits (frame header offset 12). Bits 0-15 are those of M2, unchanged.
        internal const long MotifLeaseExpired = 1L << 0;
        internal const long MotifNoReference = 1L << 1;
        internal const long MotifReferenceStale = 1L << 2;
        internal const long MotifWarp = 1L << 3;
        internal const long MotifTickGap = 1L << 4;
        internal const long MotifUnloaded = 1L << 5;
        internal const long MotifTopologyChanged = 1L << 6;
        internal const long MotifResidualGuard = 1L << 7;        // M2, vessel-wide: published only
        internal const long MotifOmegaGuard = 1L << 8;           // M2, vessel-wide: published only
        internal const long MotifSaturationGuard = 1L << 9;      // M2, vessel-wide: published only
        internal const long MotifException = 1L << 10;
        internal const long MotifEarlyishDead = 1L << 11;
        internal const long MotifFrameConversion = 1L << 12;
        internal const long MotifRcsModelApprox = 1L << 13;
        internal const long MotifSurfaceApprox = 1L << 14;
        internal const long MotifNotConverged = 1L << 15;
        // M3. 16-18: the per-axis guards of AttitudeMath, on the axes of the arm mask (in
        // shadow too: what WOULD make an armed loop fall back, as 7-9 in M2).
        internal const long MotifAxisResidualGuard = 1L << 16;
        internal const long MotifAxisOmegaGuard = 1L << 17;
        internal const long MotifAxisSaturationGuard = 1L << 18;
        // 19-25: mode 1 only.
        internal const long MotifRollChannelUnmodeled = 1L << 19;  // rho > max, or no roll column
        internal const long MotifFlyByWireOrder = 1L << 20;        // kRPC absent, or not before us
        internal const long MotifFlyByWireReordered = 1L << 21;    // our callback moved back to the end
        internal const long MotifClientWriteOff = 1L << 22;        // the reference does not allow it
        internal const long MotifGuardHold = 1L << 23;             // fallback held after a guard
        internal const long MotifWriteFallback = 1L << 24;         // armed, not written this step
        internal const long MotifFlyByWireMissed = 1L << 25;       // last decision never reached fly-by-wire

        /// <summary>
        /// Every motif that forbids a write in this step. Not in it: the vessel-wide
        /// guards of M2 (7-9, kept for comparison), and the informational bits 21, 24, 25.
        /// </summary>
        internal const long BlockingMotifs =
            MotifLeaseExpired | MotifNoReference | MotifReferenceStale | MotifWarp |
            MotifTickGap | MotifUnloaded | MotifTopologyChanged | MotifException |
            MotifEarlyishDead | MotifFrameConversion | MotifRcsModelApprox |
            MotifSurfaceApprox | MotifNotConverged | MotifAxisResidualGuard |
            MotifAxisOmegaGuard | MotifAxisSaturationGuard | MotifRollChannelUnmodeled |
            MotifFlyByWireOrder | MotifClientWriteOff | MotifGuardHold;

        // Write state (state offset 87).
        internal const int WriteShadow = 0;
        internal const int WriteWriting = 1;
        internal const int WriteFallback = 2;

        /// <summary>
        /// Largest gimbal-roll coupling rho for which the loop writes the roll: an INDI
        /// effectiveness ratio 1 + rho up to 1.25 (unstable from 2, poorly damped well
        /// before), with margin for the stock gimbal response lag.
        /// </summary>
        internal const double RollCouplingMax = 0.25;

        /// <summary>After a guard of an armed axis, the fallback lasts at least this (game s).</summary>
        internal const double GuardHoldSeconds = 2.0;

        internal const string KrpcPilotType = "KRPC.SpaceCenter.PilotAddon";

        /// <summary>Body axes (bit a = body axis a: 0 x pitch, 1 y roll, 2 z yaw) of an arm mask.</summary>
        internal static int ArmedBodyAxes (int axesMask)
        {
            int axes = 0;
            if ((axesMask & ArmRoll) != 0)
                axes |= 1 << 1;
            if ((axesMask & ArmPitchYaw) != 0)
                axes |= (1 << 0) | (1 << 2);
            return axes;
        }

        /// <summary>
        /// Motif bits 16-18 raised by the per-axis guards (AttitudeOutput.AxisGuardMask: bit
        /// 3a residual, 3a+1 omega, 3a+2 saturation) on the body axes of the arm mask.
        /// </summary>
        internal static long ArmedAxisGuardMotifs (int axisGuardMask, int axesMask)
        {
            int axes = ArmedBodyAxes (axesMask);
            long motif = 0;
            for (int a = 0; a < 3; a++) {
                if ((axes & (1 << a)) == 0)
                    continue;
                if ((axisGuardMask & (1 << (3 * a))) != 0)
                    motif |= MotifAxisResidualGuard;
                if ((axisGuardMask & (1 << (3 * a + 1))) != 0)
                    motif |= MotifAxisOmegaGuard;
                if ((axisGuardMask & (1 << (3 * a + 2))) != 0)
                    motif |= MotifAxisSaturationGuard;
            }
            return motif;
        }

        /// <summary>
        /// rho = sum over the gimbal columns of |B_yj| * span_j, over |B_y| of the ctrlState
        /// roll column (column nGimbalColumns + 1, span 1). B as AttitudeInput.B (axis a,
        /// column j at [a * n + j]). +infinity when the roll column has no roll effect.
        /// </summary>
        internal static double RollCoupling (double[] b, double[] uMin, double[] uMax, int n,
            int nGimbalColumns)
        {
            double gimbal = 0.0;
            for (int j = 0; j < nGimbalColumns; j++) {
                double span = Math.Abs (uMax [j]) > Math.Abs (uMin [j])
                    ? Math.Abs (uMax [j]) : Math.Abs (uMin [j]);
                gimbal = gimbal + Math.Abs (b [1 * n + j]) * span;
            }
            double channel = Math.Abs (b [1 * n + nGimbalColumns + 1]);
            if (!(channel > 0.0) || double.IsInfinity (channel))
                return double.PositiveInfinity;
            return gimbal / channel;
        }

        /// <summary>
        /// The roll command to write: u0 + target / column, clamped to [-1, 1]. target is
        /// the roll allocation target (M_d - M)_y (N.m), column the roll torque per unit of
        /// the ctrlState roll command (N.m). False (NaN) on a null or non-finite column or
        /// input.
        /// </summary>
        internal static bool RollCommand (double u0, double target, double column,
            out double command)
        {
            command = double.NaN;
            if (!(Math.Abs (column) > 0.0) || double.IsInfinity (column) ||
                    double.IsNaN (u0) || double.IsInfinity (u0) ||
                    double.IsNaN (target) || double.IsInfinity (target))
                return false;
            double u = u0 + target / column;
            if (u > 1.0)
                u = 1.0;
            if (u < -1.0)
                u = -1.0;
            command = u;
            return true;
        }

        /// <summary>The write rule of M3: armed on roll, at Earlyish, law computed, no blocking motif.</summary>
        internal static bool MayWrite (int mode, int axesMask, int stage, bool computed, long motif)
        {
            return mode == ModeArmed && (axesMask & ArmRoll) != 0 && stage == StageEarlyish &&
                computed && (motif & BlockingMotifs) == 0;
        }

        /// <summary>
        /// Positions in a fly-by-wire invocation list: of our callback (last occurrence, -1
        /// absent) and of the kRPC PilotAddon callback (last occurrence, -1 absent).
        /// </summary>
        internal static void ClassifyChain (Delegate chain, Delegate own, out int ownIndex,
            out int krpcIndex, out int length)
        {
            ownIndex = -1;
            krpcIndex = -1;
            length = 0;
            if (chain == null)
                return;
            Delegate[] list = chain.GetInvocationList ();
            length = list.Length;
            for (int i = 0; i < list.Length; i++) {
                if (own != null && list [i].Equals (own))
                    ownIndex = i;
                else if (IsKrpcPilot (list [i]))
                    krpcIndex = i;
            }
        }

        /// <summary>
        /// kRPC 0.6 PilotAddon.Fly registers new FlightInputCallback(action.Invoke), action a
        /// closure compiled inside PilotAddon: the entry's Target is that Action, whose
        /// method belongs to a class nested in KRPC.SpaceCenter.PilotAddon.
        /// </summary>
        internal static bool IsKrpcPilot (Delegate d)
        {
            if (d == null)
                return false;
            var inner = d.Target as Delegate;
            var method = inner != null ? inner.Method : d.Method;
            var type = method == null ? null : method.DeclaringType;
            while (type != null && type.DeclaringType != null)
                type = type.DeclaringType;
            return type != null && type.FullName == KrpcPilotType;
        }

        /// <summary>Our callback is the LAST of the chain, and the kRPC one runs before it.</summary>
        internal static bool OrderVerified (int ownIndex, int krpcIndex, int length)
        {
            return krpcIndex >= 0 && ownIndex > krpcIndex && ownIndex == length - 1;
        }
    }
}
