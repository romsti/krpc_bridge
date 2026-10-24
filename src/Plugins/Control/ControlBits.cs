using System;
using KRPC.Bridge.Core;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// The masks the Control layouts publish (AttitudeV1 motifs, capabilities and options, the
    /// lateral force's reasons), declared once into the Core v2 bit registry
    /// (conn.bridge.bit_sets_v1 / bits_v1), FROM THE CONSTANTS THAT DEFINE THEM: each entry names
    /// its constant, and Declare checks that the constant is exactly that one bit, so the table
    /// cannot drift from the code. The kind of a motif is computed from the two blocking masks
    /// of AttitudeWrite. Names are those of the client decoders (the client)
    /// where they exist; the rest are the C# names in snake_case. Telemetry declares its own
    /// sets (TelemetryBits) with <see cref="Bit"/>.
    ///
    /// Read-only description: declaring changes nothing that flies.
    /// </summary>
    internal static class ControlBits
    {
        internal const string Owner = "Control";
        internal const string AttitudeMotifs = "attitude_v1.motifs";
        internal const string AttitudeCapabilities = "attitude_v1.capabilities";
        internal const string AttitudeOptions = "attitude_v1.options";
        internal const string TranslationReasons = "attitude_translation.reasons";
        internal const string AllocationReasons = "attitude_allocation.reasons";
        internal const string AllocationFallbacks = "attitude_allocation.fallbacks";

        static bool declared;

        /// <summary>Declares every set once (Actuation.Awake). Never throws into the game.</summary>
        internal static void Declare ()
        {
            if (declared)
                return;
            declared = true;
            try {
                DeclareMotifs ();
                DeclareAttitudeCapabilities ();
                DeclareOptions ();
                DeclareReasons ();
                DeclareAllocationReasons ();
                DeclareAllocationFallbacks ();
            } catch (Exception exc) {
                BridgeLog.Error ("Control", "bit registry: " + exc.Message);
            }
        }

        /// <summary>Index of the single bit of a mask; throws if the mask is not one bit.</summary>
        internal static int Bit (long mask)
        {
            if (mask == 0 || (mask & (mask - 1)) != 0)
                throw new ArgumentException ("not a single bit: " + mask);
            int i = 0;
            while ((mask & 1L) == 0) {
                mask >>= 1;
                i++;
            }
            return i;
        }

        static void D (string set, long mask, string name, string kind, string note = "")
        {
            BitRegistry.Declare (set, Owner, Bit (mask), name, kind, note);
        }

        static string MotifKind (long mask)
        {
            bool roll = (AttitudeWrite.BlockingMotifs & mask) != 0;
            bool pitchYaw = (AttitudeWrite.BlockingMotifsPitchYaw & mask) != 0;
            if (roll && pitchYaw)
                return BitRegistry.KindBlockingAll;
            if (roll)
                return BitRegistry.KindBlockingRoll;
            if (pitchYaw)
                return BitRegistry.KindBlockingPitchYaw;
            return BitRegistry.KindInformational;
        }

        static void M (long mask, string name, string note = "")
        {
            D (AttitudeMotifs, mask, name, MotifKind (mask), note);
        }

        static void DeclareMotifs ()
        {
            const string family = "informational with option 1 (family blocking)";
            M (AttitudeWrite.MotifLeaseExpired, "lease_expired");
            M (AttitudeWrite.MotifNoReference, "no_reference");
            M (AttitudeWrite.MotifReferenceStale, "reference_stale");
            M (AttitudeWrite.MotifWarp, "warp");
            M (AttitudeWrite.MotifTickGap, "tick_gap");
            M (AttitudeWrite.MotifUnloaded, "unloaded");
            M (AttitudeWrite.MotifTopologyChanged, "topology_changed");
            M (AttitudeWrite.MotifResidualGuard, "residual_guard", "M2, vessel-wide, published only");
            M (AttitudeWrite.MotifOmegaGuard, "omega_guard", "M2, vessel-wide, published only");
            M (AttitudeWrite.MotifSaturationGuard, "saturation_guard", "M2, vessel-wide, published only");
            M (AttitudeWrite.MotifException, "exception");
            M (AttitudeWrite.MotifEarlyishDead, "earlyish_dead");
            M (AttitudeWrite.MotifFrameConversion, "frame_conversion");
            M (AttitudeWrite.MotifRcsModelApprox, "rcs_model_approx", family);
            M (AttitudeWrite.MotifSurfaceApprox, "surface_approx", family);
            M (AttitudeWrite.MotifNotConverged, "not_converged", "informational with option 4");
            M (AttitudeWrite.MotifAxisResidualGuard, "axis_residual_guard");
            M (AttitudeWrite.MotifAxisOmegaGuard, "axis_omega_guard");
            M (AttitudeWrite.MotifAxisSaturationGuard, "axis_saturation_guard");
            M (AttitudeWrite.MotifRollChannelUnmodeled, "roll_channel_unmodeled");
            M (AttitudeWrite.MotifFlyByWireOrder, "fly_by_wire_order", family);
            M (AttitudeWrite.MotifFlyByWireReordered, "fly_by_wire_reordered");
            M (AttitudeWrite.MotifClientWriteOff, "client_write_off");
            M (AttitudeWrite.MotifGuardHold, "guard_hold");
            M (AttitudeWrite.MotifWriteFallback, "write_fallback");
            M (AttitudeWrite.MotifFlyByWireMissed, "fly_by_wire_missed");
            M (AttitudeWrite.MotifPitchYawReferenceOld, "pitch_yaw_reference_old");
            M (AttitudeWrite.MotifContact, "contact");
            M (AttitudeWrite.MotifGimbalAuthority, "gimbal_authority");
            M (AttitudeWrite.MotifPitchYawFallback, "pitch_yaw_fallback");
            M (AttitudeWrite.MotifPitchYawWriteOff, "pitch_yaw_write_off");
            M (AttitudeWrite.MotifPitchYawGuardHold, "pitch_yaw_guard_hold");
            M (AttitudeWrite.MotifCtrlFrozenPitch, "ctrl_frozen_pitch", "option 1 only");
            M (AttitudeWrite.MotifCtrlFrozenRoll, "ctrl_frozen_roll", "option 1 only");
            M (AttitudeWrite.MotifCtrlFrozenYaw, "ctrl_frozen_yaw", "option 1 only");
            M (AttitudeWrite.MotifSurfacesNeutralized, "surfaces_neutralized", "option 1 only");
            M (AttitudeWrite.MotifRollSurfaceGate, "roll_surface_gate", "option 1 only");
            M (AttitudeV1.MotifSurfacesEngaged, "surfaces_engaged", "attitude_surfaces_v1 mode 2 only");
            M (AttitudeV1.MotifSurfaceSanction, "surface_sanction", "attitude_surfaces_v1 only");
            M (AttitudeWrite.MotifPitchYawContactHold, "pitch_yaw_contact_hold", "option 8 only (D181)");
            M (AttitudeWrite.MotifPitchYawReferenceHeld, "pitch_yaw_reference_held", "option 16, write-mask bit 8 (D183)");
            M (AttitudeWrite.MotifSurfacesHeldOut, "surfaces_held_out", "option 16, write-mask bit 4 (D183)");
            M (AttitudeWrite.MotifGimbalsNotLeading, "gimbals_not_leading", "option 64, write-mask bit 16 (D189)");
            M (AttitudeWrite.MotifGimbalsLeadLost, "gimbals_lead_lost", "option 64, write-mask bit 16 (D189)");
        }

        static void DeclareAttitudeCapabilities ()
        {
            string k = BitRegistry.KindCapability;
            D (AttitudeCapabilities, AttitudeV1.CapEngineTorque, "engine_torque", k);
            D (AttitudeCapabilities, AttitudeV1.CapRcsTorque, "rcs_torque", k);
            D (AttitudeCapabilities, AttitudeV1.CapWheelTorque, "wheel_torque", k);
            D (AttitudeCapabilities, AttitudeV1.CapGimbalColumns, "gimbal_columns", k);
            D (AttitudeCapabilities, AttitudeV1.CapRcsColumns, "rcs_columns", k);
            D (AttitudeCapabilities, AttitudeV1.CapWheelColumns, "wheel_columns", k);
            D (AttitudeCapabilities, AttitudeV1.CapSurfaceColumns, "surface_columns", k);
            D (AttitudeCapabilities, AttitudeV1.CapReference, "reference", k);
            D (AttitudeCapabilities, AttitudeV1.CapLaw, "law", k);
            D (AttitudeCapabilities, AttitudeV1.CapGimbalPivots, "gimbal_pivots", k);
            D (AttitudeCapabilities, AttitudeV1.CapGimbalExactAuthority, "gimbal_exact_authority", k);
            D (AttitudeCapabilities, AttitudeV1.CapFamilyBlocking, "family_blocking", k);
            D (AttitudeCapabilities, AttitudeV1.CapServeNotConverged, "serve_not_converged", k);
            D (AttitudeCapabilities, AttitudeV1.CapEvidenceAging, "evidence_aging", k);
            D (AttitudeCapabilities, AttitudeV1.CapEffectorRegistry, "effector_registry", k,
                "attitude_effectors_v1 mode 1 (implies family_blocking)");
            D (AttitudeCapabilities, AttitudeV1.CapSurfaceModel, "surface_model", k,
                "attitude_surfaces_v1 mode 1 or 2 (mode 2 implies family_blocking)");
            D (AttitudeCapabilities, AttitudeV1.CapRcsInFlight, "rcs_in_flight", k,
                "option 32 (D188): the RCS torque in flight taken off the allocation target this step");
            D (AttitudeCapabilities, AttitudeV1.CapGimbalTrimNull, "gimbal_trim_null", k,
                "option 128 (D237 K2): a gimbal column without effect regularized toward neutral this step");
            D (AttitudeCapabilities, AttitudeV1.CapGimbalSpool, "gimbal_spool", k,
                "option 128 (D237 K2): gimbal columns at the thrust of the spool's horizon this step");
            D (AttitudeCapabilities, AttitudeV1.CapTranslationFree, "translation_free", k,
                "option 256 (D246): the free authority of the lateral force computed this step");
            D (AttitudeCapabilities, AttitudeV1.CapMainThrottle, "main_throttle", k,
                "option 512 (D252 K5): the main throttle decided this step, written at its fly-by-wire");
            D (AttitudeCapabilities, AttitudeV1.CapRollTarget, "roll_target", k,
                "option 1024 (D252 K6): the roll error of the reference's roll target in the frame this step (roll mode 1: " +
                "in the law; 2: the shadow, published only)");
            D (AttitudeCapabilities, AttitudeV1.CapReferenceRing, "reference_ring", k,
                "option 2048 (D252 K7): the reference ring served a channel this step");
            D (AttitudeCapabilities, AttitudeV1.CapEngineAllocation, "engine_allocation", k,
                "option 4096 (D264): the engine-by-engine allocation solved in shadow this step");
            D (AttitudeCapabilities, AttitudeV1.CapEngineAllocationWrite, "engine_allocation_write", k,
                "option 8192 (D264 step 2): the engine-by-engine allocation wrote this step in place of the law's " +
                "allocation and the lateral-force layer");
            D (AttitudeCapabilities, AttitudeV1.CapRcsCombined, "rcs_combined", k,
                "option 16384 (D267): the RCS nozzles modelled this step by ModuleRCS's combined rule (the in-flight torque " +
                "of option 32, the command the RCS answers)");
            D (AttitudeCapabilities, AttitudeV1.CapRcsRollHeld, "rcs_roll_held", k,
                "option 16384 (D267): the law's roll held this step (written under the RCS dead zone), nearer what the law " +
                "expects");
            D (AttitudeCapabilities, AttitudeV1.CapRcsPitchYawHeld, "rcs_pitch_yaw_held", k,
                "option 16384 (D267): the law's pitch/yaw held this step (written under the RCS dead zone), nearer what the law " +
                "expects");
        }

        static void DeclareOptions ()
        {
            string k = BitRegistry.KindOption;
            D (AttitudeOptions, AttitudeWrite.OptionFamilyBlocking, "family_blocking", k, "D124");
            D (AttitudeOptions, AttitudeWrite.OptionEvidenceAging, "evidence_aging", k, "D125");
            D (AttitudeOptions, AttitudeWrite.OptionServeNotConverged, "serve_not_converged", k, "motif 15");
            D (AttitudeOptions, AttitudeWrite.OptionContactHold, "contact_hold", k, "D181");
            D (AttitudeOptions, AttitudeWrite.OptionCoastWrite, "coast_write", k,
                "D183: write-mask bits 4 surfaces out of the law, 8 pitch/yaw holds its reference");
            D (AttitudeOptions, AttitudeWrite.OptionRcsInFlight, "rcs_in_flight", k,
                "D188: the RCS torque already commanded, fired one render frame later, off the allocation target");
            D (AttitudeOptions, AttitudeWrite.OptionBoostbackWrite, "boostback_write", k,
                "D189: write-mask bit 16, pitch/yaw only while the gimbals lead the ctrlState channel, latched once lost");
            D (AttitudeOptions, AttitudeWrite.OptionGimbalTrim, "gimbal_trim", k,
                "D237 K2: gimbals without thrust to neutral, gimbal columns at the coming thrust, the spool's torque off the target");
            D (AttitudeOptions, AttitudeWrite.OptionFreeAuthority, "free_authority", k,
                "D246: the free authority of the lateral force (what the attitude leaves) in the translation return");
            D (AttitudeOptions, AttitudeWrite.OptionMainThrottle, "main_throttle", k,
                "D252 K5: the main throttle decided at the physics step from the orders of attitude_throttle_v1");
            D (AttitudeOptions, AttitudeWrite.OptionRollTarget, "roll_target", k,
                "D252 K6: roll mode 1 of attitude_reference_v3, the roof toward an up vector in the law");
            D (AttitudeOptions, AttitudeWrite.OptionReferenceRing, "reference_ring", k,
                "D252 K7: the reference ring of attitude_reference_ring_v1, dated samples served at the physics step");
            D (AttitudeOptions, AttitudeWrite.OptionEngineAllocation, "engine_allocation", k,
                "D264: the engine-by-engine allocation in shadow (throttle of each engine, gimbals, RCS; force and torque " +
                "rows), published by attitude_allocation_v1 and attitude_allocation_frames_v1, no actuator written");
            D (AttitudeOptions, AttitudeWrite.OptionEngineAllocationWrite, "engine_allocation_write", k,
                "D264 step 2 (implies 4096): the allocation WRITES the throttle of each engine, the gimbals and the " +
                "ctrlState in place of the law's allocation and the lateral-force layer while its solve is sound; " +
                "otherwise they write, without a jump");
            D (AttitudeOptions, AttitudeWrite.OptionRcsCombined, "rcs_combined", k,
                "D267: the RCS as ModuleRCS fires it (one rotation vector per module, dead zones, each nozzle bounded at 1): " +
                "the in-flight torque of option 32 by that rule; the ctrlState command written is the one of the law's " +
                "command, its pitch/yaw alone and its roll alone whose torque is nearest what the law expects (its own WLS " +
                "metric)");
        }

        static void DeclareReasons ()
        {
            string k = BitRegistry.KindReason;
            string set = TranslationReasons;
            D (set, AttitudeTranslation.ReasonNoRequest, "no_request", k);
            D (set, AttitudeTranslation.ReasonNotArmedPitchYaw, "not_armed_pitch_yaw", k);
            D (set, AttitudeTranslation.ReasonPitchYawNotPermitted, "pitch_yaw_not_permitted", k);
            D (set, AttitudeTranslation.ReasonNotWriting, "not_writing", k);
            D (set, AttitudeTranslation.ReasonStale, "stale", k);
            D (set, AttitudeTranslation.ReasonStopRequested, "stop_requested", k);
            D (set, AttitudeTranslation.ReasonNoDiffAuthority, "no_diff_authority", k);
            D (set, AttitudeTranslation.ReasonNoThrottleMargin, "no_throttle_margin", k);
            D (set, AttitudeTranslation.ReasonNoTranslationGeometry, "no_translation_geometry", k);
            D (set, AttitudeTranslation.ReasonTiltTooLarge, "tilt_too_large", k);
            D (set, AttitudeTranslation.ReasonLimitedByGimbal, "limited_by_gimbal", k);
            D (set, AttitudeTranslation.ReasonLimitedByDiff, "limited_by_diff", k);
            D (set, AttitudeTranslation.ReasonFeedForwardClamped, "feed_forward_clamped", k);
            D (set, AttitudeTranslation.ReasonThrottleWriteMissed, "throttle_write_missed", k);
            D (set, AttitudeTranslation.ReasonWindDownTimeout, "wind_down_timeout", k);
            D (set, AttitudeTranslation.ReasonNotActiveVessel, "not_active_vessel", k);
            D (set, AttitudeTranslation.ReasonNoLateralAuthority, "no_lateral_authority", k, "bit 4 only");
            D (set, AttitudeTranslation.ReasonLimitedByRcs, "limited_by_rcs", k, "bit 4 only");
            D (set, AttitudeTranslation.ReasonRcsModelApprox, "rcs_model_approx", k, "bit 4 only");
            D (set, AttitudeTranslation.ReasonRcsShared, "rcs_shared", k, "bit 16 only");
            D (set, AttitudeTranslation.ReasonServedByAllocation, "served_by_allocation", k,
                "option 8192 only: the engine-by-engine allocation wrote this step, the layer did not");
        }

        static void DeclareAllocationReasons ()
        {
            string k = BitRegistry.KindReason;
            string set = AllocationReasons;
            D (set, EngineAllocation.ReasonNoLawTarget, "no_law_target", k);
            D (set, EngineAllocation.ReasonNoActuator, "no_actuator", k);
            D (set, EngineAllocation.ReasonTiltTooLarge, "tilt_too_large", k);
            D (set, EngineAllocation.ReasonNotConverged, "not_converged", k);
            D (set, EngineAllocation.ReasonRcsModelApprox, "rcs_model_approx", k);
            D (set, EngineAllocation.ReasonLimitedAxial, "limited_axial", k);
            D (set, EngineAllocation.ReasonLimitedPitchYaw, "limited_pitch_yaw", k);
            D (set, EngineAllocation.ReasonLimitedLateral, "limited_lateral", k);
            D (set, EngineAllocation.ReasonLimitedRoll, "limited_roll", k);
            D (set, EngineAllocation.ReasonRcsDeadZone, "rcs_dead_zone", k);
            D (set, EngineAllocation.ReasonJitter, "jitter", k);
            D (set, EngineAllocation.ReasonNoLateralDemand, "no_lateral_demand", k);
            D (set, EngineAllocation.ReasonDeadZoneSnapped, "dead_zone_snapped", k, "option 8192 only");
        }

        static void DeclareAllocationFallbacks ()
        {
            string k = BitRegistry.KindReason;
            string set = AllocationFallbacks;
            D (set, AllocationWriteRule.FallbackNotSolved, "not_solved", k);
            D (set, AllocationWriteRule.FallbackNotConverged, "not_converged", k);
            D (set, AllocationWriteRule.FallbackSaturated, "saturated", k);
            D (set, AllocationWriteRule.FallbackStale, "stale", k);
            D (set, AllocationWriteRule.FallbackLease, "lease", k);
            D (set, AllocationWriteRule.FallbackGimbalMissing, "gimbal_missing", k);
            D (set, AllocationWriteRule.FallbackException, "exception", k);
            D (set, AllocationWriteRule.FallbackResuming, "resuming", k);
        }
    }
}
