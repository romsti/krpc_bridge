using System.Collections.Generic;
using KRPC.Service.Attributes;
using SCReferenceFrame = KRPC.SpaceCenter.Services.ReferenceFrame;

namespace KRPC.Bridge.Actuators
{
    /// <summary>
    /// AttitudeV1 RPCs: the physics-rate attitude loop, in SHADOW. See AttitudeV1.cs and
    /// docs/API.md. Nothing here writes an actuator.
    /// </summary>
    public static partial class ActuatorsService
    {
        /// <summary>Protocol version of the AttitudeV1 frames and status.</summary>
        [KRPCProperty]
        public static int AttitudeProtocolVersion {
            get { return AttitudeV1.ProtocolVersion; }
        }

        /// <summary>
        /// Arms the AttitudeV1 shadow loop for the loaded vessel persistentId (decimal
        /// string), active or not, for leaseSeconds of real time (0.2-10 s). mode 0 = shadow,
        /// the only mode of this build: the loop computes and publishes, it writes nothing.
        /// axesMask: bit 1 roll, bit 2 pitch/yaw (recorded only in shadow). At most two
        /// vessels. Re-arming a vessel returns a new token and invalidates the old one.
        /// </summary>
        [KRPCProcedure]
        public static string AttitudeArmV1 (string persistentId, string owner,
            float leaseSeconds = 1f, int mode = 0, int axesMask = 3)
        {
            return AttitudeV1.Arm (persistentId, owner, leaseSeconds, mode, axesMask);
        }

        /// <summary>
        /// One reference per GNC tick, the one given to the kRPC AutoPilot: frame and
        /// direction as AutoPilot.ReferenceFrame / TargetDirection, rollMode 0 = roll-rate
        /// damping (target_roll NaN), omegaFf = rate of the wanted direction (rad/s, same
        /// frame), utStamp = UT of the state the guidance used, valid until utValidUntil,
        /// omegaMax = AutoPilot max angular velocity (rad/s, 0 = default). An accepted
        /// reference renews the lease. Returns 1 accepted, 0 ignored (sequence not newer).
        /// </summary>
        [KRPCProcedure]
        public static int AttitudeReferenceV1 (string token, int sequence, SCReferenceFrame frame,
            double dirX, double dirY, double dirZ, int rollMode, double omegaFfX,
            double omegaFfY, double omegaFfZ, double utStamp, double utValidUntil,
            double omegaMax)
        {
            return AttitudeV1.Reference (token, sequence, frame, dirX, dirY, dirZ, rollMode,
                omegaFfX, omegaFfY, omegaFfZ, utStamp, utValidUntil, omegaMax);
        }

        /// <summary>Renews the real-time lease of an armed vessel (0.2-10 s).</summary>
        [KRPCProcedure]
        public static bool AttitudeRenewV1 (string token, float leaseSeconds = 1f)
        {
            return AttitudeV1.Renew (token, leaseSeconds);
        }

        /// <summary>Disarms a vessel. Its ring is dropped.</summary>
        [KRPCProcedure]
        public static bool AttitudeReleaseV1 (string token)
        {
            return AttitudeV1.Release (token);
        }

        /// <summary>
        /// Header: protocol, schema, row count, row stride (16), Earlyish hook registered,
        /// Earlyish calls, pump-fallback ticks, attitude tick, UT. Then one row per armed
        /// vessel, see docs/API.md. Streamable.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeStatusV1 ()
        {
            return AttitudeV1.ReadStatus ();
        }

        /// <summary>
        /// AttitudeV1 frames of one armed vessel newer than sinceTick (attitude tick), in the
        /// six-value history header + length-prefixed layout of DynamicsFramesV3. Read by the
        /// observer process, not by the flight loop. Empty when the vessel is not armed.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeFramesV1 (string persistentId, int sinceTick = 0,
            int maxFrames = 64)
        {
            return AttitudeV1.ReadFrames (persistentId, sinceTick, maxFrames);
        }

        /// <summary>
        /// T-sol-0 order probe, read-only: stamps every TimingManager stage, the Actuators
        /// watcher and the active vessel's fly-by-wire chain for the next frames (1-300)
        /// physics steps. Read the result with attitude_order_probe_read_v1.
        /// </summary>
        [KRPCProcedure]
        public static bool AttitudeOrderProbeV1 (int frames = 50)
        {
            return AttitudeProbe.Start (frames);
        }

        /// <summary>
        /// Order-probe rows: header version (1), still armed, row count, stride (13), then
        /// frame index, call index in the frame, stage, fixedTime, UT, pump physics tick,
        /// render frame, served ctrlState pitch, first gimbal x, first engine thrust, omega x,
        /// first RCS thrust sum, real time.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeOrderProbeReadV1 ()
        {
            return AttitudeProbe.Read ();
        }
    }
}
