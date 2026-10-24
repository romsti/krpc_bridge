using System;

namespace KRPC.Bridge.Model
{
    // -----------------------------------------------------------------------------------
    // AttitudeV1 armed mode (step M3): whether the loop WRITES an actuator in
    // this physics step or falls back to the kRPC AutoPilot, the roll command it writes,
    // and the check of the vessel's fly-by-wire chain.
    //
    // PURE, like AttitudeMath.cs: System only, no Unity, no KSP, no kRPC type. Compiled
    // three times (the plugin, build/verify, build/tests) and mirrored operation by
    // operation by the autopilot (the client), which replays the golden
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
    //
    // What M4 adds (step E4), pitch/yaw under the descent:
    //   * the gimbals are LEASED by the loop itself (the Actuators authority taken inside
    //     the plugin, refused if another owner holds it): a leased gimbal leaves the stock
    //     calculation, so ctrlState no longer moves it and the gimbal columns of the model
    //     (validated in shadow: engine torque = columns x deflection, r = 1.000, slope
    //     0.996-1.008 on three flights) are exactly what is commanded;
    //   * ctrlState.pitch/yaw, now RCS only (columns validated, slope 0.95-1.03), written
    //     after the AutoPilot like the roll; when the loop owns the gimbals the roll is
    //     the allocation's too (no stock mixing left);
    //   * each channel falls back on its own: pitch/yaw needs a direction younger than
    //     0.3 s, stops at contact (Landed/Splashed or an impossible demand), and hands the
    //     gimbals back WITHOUT a jump (the stock Lerp resumes from the leased deflection).
    // -----------------------------------------------------------------------------------
    internal static class AttitudeWrite
    {
        // Modes and arm bits (attitude_arm_v1).
        internal const int ModeShadow = 0;
        internal const int ModeArmed = 1;
        internal const int ArmRoll = 1;          // body y
        internal const int ArmPitchYaw = 2;      // body x and z (M4)

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
        // M4 (chantier 11): 26-31, mode 1 pitch/yaw (leased gimbals + RCS through ctrlState).
        internal const long MotifPitchYawReferenceOld = 1L << 26;  // reference older than 0.3 s of game time
        internal const long MotifContact = 1L << 27;               // Landed/Splashed, or an impossible demand
        internal const long MotifGimbalAuthority = 1L << 28;       // actuator authority unavailable (owner, vessel)
        internal const long MotifPitchYawFallback = 1L << 29;      // pitch/yaw armed, not written this step
        internal const long MotifPitchYawWriteOff = 1L << 30;      // the reference does not allow pitch/yaw
        internal const long MotifPitchYawGuardHold = 1L << 31;     // pitch/yaw fallback held after a guard

        /// <summary>
        /// Every motif that forbids the ROLL write in this step. Not in it: the vessel-wide
        /// guards of M2 (7-9, kept for comparison), and the informational bits 21, 24, 25.
        /// M4 adds the contact (27): at touchdown nothing is written.
        /// </summary>
        internal const long BlockingMotifs =
            MotifLeaseExpired | MotifNoReference | MotifReferenceStale | MotifWarp |
            MotifTickGap | MotifUnloaded | MotifTopologyChanged | MotifException |
            MotifEarlyishDead | MotifFrameConversion | MotifRcsModelApprox |
            MotifSurfaceApprox | MotifNotConverged | MotifAxisResidualGuard |
            MotifAxisOmegaGuard | MotifAxisSaturationGuard | MotifRollChannelUnmodeled |
            MotifFlyByWireOrder | MotifClientWriteOff | MotifGuardHold | MotifContact;

        /// <summary>
        /// Every motif that forbids the PITCH/YAW write (M4): the common ones, the guards of
        /// the pitch and yaw axes (16-18 computed on x and z only), and 26-28, 30, 31. Not
        /// the roll-only 19, 22, 23 : each channel falls back on its own.
        /// </summary>
        internal const long BlockingMotifsPitchYaw =
            MotifLeaseExpired | MotifNoReference | MotifReferenceStale | MotifWarp |
            MotifTickGap | MotifUnloaded | MotifTopologyChanged | MotifException |
            MotifEarlyishDead | MotifFrameConversion | MotifRcsModelApprox |
            MotifSurfaceApprox | MotifNotConverged | MotifAxisResidualGuard |
            MotifAxisOmegaGuard | MotifAxisSaturationGuard | MotifFlyByWireOrder |
            MotifPitchYawReferenceOld | MotifContact | MotifGimbalAuthority |
            MotifPitchYawWriteOff | MotifPitchYawGuardHold;

        /// <summary>
        /// Pitch/yaw needs a FRESH direction: past this age (game s since the state the
        /// guidance used) the loop stops holding it and the AutoPilot resumes. The roll,
        /// rate damping only, keeps the 1 s validity of M3. Measured transport delay under D:
        /// median 40 ms, p95 80 ms, max 0.32 s (flights B, M3 RTLS and barge).
        /// </summary>
        internal const double PitchYawMaxReferenceAgeS = 0.3;

        /// <summary>
        /// Touchdown: an unrealizable share above this on an armed axis cannot be flight (max
        /// 2.6 in flight B's coast, 1.5 in its flip) - it is contact (1e4-1e5 at the barge
        /// touchdown). The loop hands everything back at once.
        /// </summary>
        internal const double ContactUnrealizedRatio = 5.0;

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
        /// The write rule of M4: armed on pitch/yaw, at Earlyish, law computed, no motif of
        /// BlockingMotifsPitchYaw in the pitch/yaw motif set (its guards on x and z only).
        /// </summary>
        internal static bool MayWritePitchYaw (int mode, int axesMask, int stage, bool computed,
            long motif)
        {
            return mode == ModeArmed && (axesMask & ArmPitchYaw) != 0 && stage == StageEarlyish &&
                computed && (motif & BlockingMotifsPitchYaw) == 0;
        }

        /// <summary>
        /// Contact read from the law: an unrealizable share above ContactUnrealizedRatio on a
        /// body axis of the arm mask (NaN, the law did not run, is not contact).
        /// </summary>
        internal static bool ImpossibleDemand (double[] unrealized, int axesMask)
        {
            int axes = ArmedBodyAxes (axesMask);
            for (int a = 0; a < 3; a++)
                if ((axes & (1 << a)) != 0 && unrealized [a] > ContactUnrealizedRatio)
                    return true;
            return false;
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

        // -----------------------------------------------------------------------------------
        // the NON-ACTIVE vessel of a double flight. The actuator
        // authority of M4 (Actuation.TryHoldForAttitude) belongs to FlightGlobals.
        // ActiveVessel: on the other booster the pitch/yaw channel fell back at every step
        // (motif 28, a recorded flight: 1052 steps, 0 pitch/yaw written) and the lateral
        // force said reason 15. AttitudeArmV2 options, BORN CLOSED (0 = exactly the V1 arming):
        //   bit 1: the pitch/yaw channel of a vessel that is NOT the active one takes its own
        //          per-vessel authority (Actuation.TryHoldForAttitudeInactive) instead of
        //          the active vessel's; the active vessel keeps the unchanged path;
        //   bit 2: the lateral-force layer may also run there (requires bit 1).
        // -----------------------------------------------------------------------------------
        internal const int InactivePitchYaw = 1;
        internal const int InactiveTranslation = 2;

        /// <summary>0, 1 or 3 (translation without the pitch/yaw channel means nothing).</summary>
        internal static bool ValidInactiveOptions (int options)
        {
            return options == 0 || options == InactivePitchYaw ||
                options == (InactivePitchYaw | InactiveTranslation);
        }

        /// <summary>
        /// The options need the armed pitch/yaw channel (mode 1, axes bit 2); 0 goes with any
        /// arming.
        /// </summary>
        internal static bool InactiveOptionsFit (int options, int mode, int axesMask)
        {
            return options == 0 || (mode == ModeArmed && (axesMask & ArmPitchYaw) != 0);
        }

        /// <summary>
        /// This step's pitch/yaw write takes the per-vessel authority of a non-active vessel
        /// (true) or the unchanged active-vessel path (false: TryHoldForAttitude, which refuses
        /// a non-active vessel, as before).
        /// </summary>
        internal static bool UsesInactiveAuthority (bool isActiveVessel, int options)
        {
            return !isActiveVessel && (options & InactivePitchYaw) != 0;
        }

        /// <summary>The lateral-force layer may run on this vessel (the active one always).</summary>
        internal static bool TranslationAllowed (bool isActiveVessel, int options)
        {
            return isActiveVessel ||
                (options & (InactivePitchYaw | InactiveTranslation)) ==
                (InactivePitchYaw | InactiveTranslation);
        }

        /// <summary>
        /// Arming pitch/yaw is refused while another owner of the ACTIVE vessel's actuators is
        /// present (v2 control, legacy lease, the other armed vessel's loop). A vessel armed
        /// with option 1 that is loaded and NOT active at arming never takes that authority:
        /// the check does not apply to it (a booster re-armed during the descent while the
        /// active one writes). Option 0: always, as before.
        /// </summary>
        internal static bool ArmNeedsOwnerCheck (int mode, int axesMask, int options,
            bool loadedInactiveVessel)
        {
            if (mode != ModeArmed || (axesMask & ArmPitchYaw) == 0)
                return false;
            return !(loadedInactiveVessel && (options & InactivePitchYaw) != 0);
        }

        // -----------------------------------------------------------------------------------
        // Refactor lot 1: per-arming options of AttitudeOptionsV1. BORN CLOSED:
        // every arming resets them to 0, and with 0 nothing below is reached (same motif sets,
        // same columns, same writes, same frames, same returns).
        //
        //   bit 1, FAMILY BLOCKING. A model motif no longer blocks a whole channel; it
        //          sanctions the family of actuators it is about:
        //            * 13 (RCS model approximate: precision mode, fullThrust) and 20 (fly-by-wire
        //              order not verified: the kRPC AutoPilot callback is not before ours) FREEZE
        //              the ctrlState columns (B = 0, bounds = the served u0): the loop allocates
        //              the leased gimbals only and does not write ctrlState pitch/yaw/roll; the
        //              gimbals do not go through fly-by-wire and stay allocatable;
        //            * 14 (control surfaces with authority, unmodelled): the surfaces leave B
        //              (their GetPotentialTorque column is wrong by a lever factor) and, in every
        //              step where the loop writes ctrlState pitch/yaw (or wrote, at the last
        //              fly-by-wire, pitch/yaw or a roll the surfaces answer), each surface that
        //              answers a ctrlState axis is NEUTRALIZED for
        //              that physics step only (ModuleControlSurface.deflectionDirection = 0 from
        //              Earlyish, restored at BetterLateThanNever of the same step: never saved,
        //              never shown, never shared with a client write of ignorePitch/Yaw/Roll); a
        //              roll written WITHOUT that neutralization while a surface answers the roll
        //              is not written (motif 19 + 36); a surface that cannot be neutralized
        //              (deployed, field missing) freezes the ctrlState axes it answers;
        //            * 13, 14 and 20 stay in the frame, informational; bits 32-36 below say what
        //              was frozen or neutralized; the per-axis guards (16-18), fed by the
        //              measurement, remain the net.
        //   bit 2, EVIDENCE AGING: the measured share of the common deflection
        //          (translation bit 32) ages in game time when no sample comes, and the
        //          AttitudeTranslationV1 return carries a named evidence block (see
        //          AttitudeTranslation.EvidenceFields).
        //   bit 4, SERVE NOT CONVERGED (motif 15): the WLS iterate left when the 20 iterations
        //          ran out is feasible (box respected by construction) and no worse than the
        //          warm start (primal active set on a convex QP: every step lowers the cost); the
        //          hedge and the guards already read it. With this bit motif 15 is informational
        //          and that iterate is written, instead of a one-step fallback that hands the
        //          gimbals to the stock Lerp and back.
        //   bit 8, CONTACT HOLD. At the MEASURED contact -- Landed outside PRELAUNCH, or
        //          Splashed, the contact of motif 27 without its impossible-demand half, which a
        //          flight can produce -- while the client's write mask still allows pitch/yaw, the
        //          gimbals the loop leased at its last write stay LEASED at that deflection instead
        //          of going back to the stock Lerp toward the AutoPilot's ctrlState (motif 39).
        //          Why: the client keeps the permission until it has cut the thrust (the
        //          AutoPilot, whose integrators wound up while the loop wrote over it, slammed the
        //          gimbals to their stops in the last metres of 6 flights); between the contact step
        //          and that cut, thrust is still full. Held at the last value, not recentred: that
        //          deflection is the one that balanced the torque about the centre of mass at the
        //          contact step (0.2-0.3 deg measured), so holding it changes nothing in the torque
        //          while the legs take the load; recentring would step the torque by that trim at
        //          the moment of touchdown, and a bounce would resume the law from its own last
        //          output (INDI starts from the realized deflection). The hold is no law: it writes no
        //          ctrlState (the AutoPilot keeps pitch/yaw RCS and the roll), the differential
        //          throttles of the lateral force go back to the main throttle at its first step
        //          (the client's cut stays immediate), and it only CONTINUES a lease: any blocking
        //          motif other than the contact and what the contact does to the law (its axis guards
        //          and their hold, a step not converged or approximate, the fly-by-wire order of a
        //          ctrlState it does not write) -- the client's write-off first of all, then a stale
        //          reference, the lease, a topology change, an unloaded vessel -- hands the gimbals
        //          back as before.
        // -----------------------------------------------------------------------------------
        internal const int OptionFamilyBlocking = 1;
        internal const int OptionEvidenceAging = 2;
        internal const int OptionServeNotConverged = 4;
        internal const int OptionContactHold = 8;
        internal const int OptionCoastWrite = 16;
        internal const int OptionRcsInFlight = 32;
        internal const int OptionBoostbackWrite = 64;
        internal const int OptionGimbalTrim = 128;
        internal const int OptionFreeAuthority = 256;
        internal const int OptionMainThrottle = 512;
        internal const int OptionRollTarget = 1024;
        internal const int OptionReferenceRing = 2048;
        // (born closed): the engine-by-engine allocation IN SHADOW (EngineAllocation.cs,
        // AttitudeV1Allocation.cs): computed and published every step, no actuator written.
        internal const int OptionEngineAllocation = 4096;
        // (born closed): the engine-by-engine allocation WRITES (implies 4096): the throttle of every engine
        // it drives (independent throttles under the loop's lease), the gimbals and the ctrlState (rotation and RCS
        // translation) in place of the law's allocation and of the lateral-force layer, on every step the pitch/yaw
        // channel writes and its solve is sound; otherwise the law and the layer write, without a jump
        // (EngineAllocation.AllocationWriteRule).
        internal const int OptionEngineAllocationWrite = 8192;
        // (born closed): the RCS of the ctrlState channel as ModuleRCS fires it (RcsCombined.cs,
        // AttitudeV1RcsCombined.cs): the in-flight torque of option 32 by the combined rule, and the ctrlState command
        // written the one the RCS answers (the law's, its pitch/yaw alone or its roll alone, by the law's own metric).
        internal const int OptionRcsCombined = 16384;
        internal const int OptionsSupported = OptionFamilyBlocking | OptionEvidenceAging |
            OptionServeNotConverged | OptionContactHold | OptionCoastWrite | OptionRcsInFlight |
            OptionBoostbackWrite | OptionGimbalTrim | OptionFreeAuthority | OptionMainThrottle | OptionRollTarget |
            OptionReferenceRing | OptionEngineAllocation | OptionEngineAllocationWrite | OptionRcsCombined;

        /// <summary>the allocation runs (in shadow with 4096, writing with 8192, which implies it).</summary>
        internal static bool AllocationOn (int options)
        {
            return (options & (OptionEngineAllocation | OptionEngineAllocationWrite)) != 0;
        }
        internal const int OptionsVersion = 1;

        // -----------------------------------------------------------------------------------
        // (M5: the loop owns pitch/yaw in the COAST), option 16 of AttitudeOptionsV1, BORN
        // CLOSED. Without it a write mask above 3 is refused exactly as before. With it, the write
        // mask of AttitudeReferenceV2 may carry two more bits, per reference:
        //   bit 4, SURFACES OUT: no control surface is engaged into the law for this reference --
        //          SurfaceModelV1 sees none of the loop's axes as owned, so every surface is
        //          stock and, while the loop writes pitch/yaw, option 1 holds it neutral. The
        //          loop flies the coast on RCS and wheels alone; the surfaces join the law in the
        //          descent, where the client stops setting the bit (one written step later).
        //          Why: the fins' authority grows with q (6 MN.m on pitch above 60 kPa on the super
        //          hope); the first coast the loop flies must not also be the first one where the
        //          fins serve it.
        //   bit 8, HOLD THE REFERENCE: the pitch/yaw channel keeps this reference up to its own
        //          validity (utValidUntil) instead of falling back to the AutoPilot past
        //          PitchYawMaxReferenceAgeS (motif 26). Why: in the coast the Python loop is heavy
        //          (p95 243 RPCs a tick); the reference's age passed 0.3 s in 10-34 % of the coast
        //          steps of five flights of the super hope (08/10), in about 110 runs a coast. Each
        //          fallback hands the gimbals and ctrlState to a kRPC AutoPilot whose integrators
        //          wound up while the loop wrote over it -- and the AutoPilot holds the same
        //          stale target anyway. The loop holds it instead; motif 40 says when.
        // Both bits are masked out of the stored write mask if the option is withdrawn later
        // (EffectiveWriteMask), so nothing below them runs without the option.
        // -----------------------------------------------------------------------------------
        internal const int WriteSurfacesOut = 4;

        // -----------------------------------------------------------------------------------
        // option 32 of AttitudeOptionsV1, BORN CLOSED: the RCS torque IN FLIGHT. Stock KSP reads
        // ctrlState for its RCS in ModuleRCS.Update (the render frame) and fires it at the next physics
        // step: the RCS torque measured at Earlyish is that of the command BEFORE the served one, which
        // the INDI increment takes it for. Under the loop at the coast of the super hope (08/10 22h54)
        // that one step made a six-step limit cycle at full RCS power (-1, -1, 0, +1, +1, 0, 65 % of the
        // steps at +-1 for a demand of 6-8 % of the authority). With the option, each step, per RCS
        // module, the torque its columns give the command its Update will make of the served ctrlState
        // (enable flags, its EPSILON dead zone) minus their torque at the command it fired when the
        // measured torque was realized (its inputRot, read at that step's Earlyish) is taken off the
        // allocation target (AttitudeInput.TorqueFree; RcsInFlight.cs). Nothing for a module whose
        // model is approximate (precision mode, fullThrust: motif 13), at the first step of an arming,
        // after a topology change or a tick gap. Capability bit 16 says the step used it. Without the
        // option: exactly as before (the target M_d - M, the surfaces' own free torque unchanged).
        // -----------------------------------------------------------------------------------
        internal const int WriteHoldReference = 8;
        internal const int WriteMaskBase = ArmRoll | ArmPitchYaw;
        internal const int WriteMaskCoast = WriteMaskBase | WriteSurfacesOut | WriteHoldReference;

        // -----------------------------------------------------------------------------------
        // (M6: the loop owns pitch/yaw in the BOOSTBACK), option 64 of AttitudeOptionsV1, BORN
        // CLOSED. Without it a write mask carrying bit 16 is refused exactly as before. With it, the write
        // mask of AttitudeReferenceV2 may carry one more bit, per reference:
        //   bit 16, GIMBALS LEAD: the pitch/yaw channel writes only while its gimbals LEAD -- their
        //          pitch/yaw authority, sum_j |B_aj| span_j over the gimbal columns (the law's own alpha
        //          rule, an earlier change's unclamped columns for a mapped module), is not zero and at least the
        //          ctrlState channel's (its three columns), on body x AND z (GimbalsLead) -- and, once it
        //          has written and they no longer lead, not again under this bit (LeadLatch: the latch
        //          lasts while the references keep bit 16; one without it, or a new arming, clears it).
        //          Why (offline shadow of the law flying four boostbacks of the super hope, 08-09/10,
        //          the client): at full thrust the loop holds the boostback as the tuned
        //          AutoPilot does and points better, on gimbals alone; in the throttle taper that
        //          ends it, the gimbals' authority falls with the thrust below the RCS's (about 5 % of
        //          full thrust on that booster) while the law keeps allocating them first (priority 1
        //          against 3): they run to their stops (up to 39 % of the gimbal-samples) and their
        //          torque comes with its lateral force, torque / lever (15-35 kN, 4 times what the RCS
        //          would make for the same torque), in the last seconds before the cut, where the
        //          guidance can no longer steer it out. Before the thrust arrives (0.3-0.5 s after the
        //          throttle) the gimbals do not lead either: the channel waits for them instead of
        //          flying the ignition on RCS alone. The AutoPilot (an earlier change's tuning) keeps both ends.
        // Masked out of the stored write mask if the option is withdrawn later (EffectiveWriteMask).
        // -----------------------------------------------------------------------------------
        internal const int WriteGimbalsLead = 16;

        // -----------------------------------------------------------------------------------
        // option 128 of AttitudeOptionsV1 (OptionGimbalTrim), BORN CLOSED: the gimbals without
        // thrust go to neutral, and their columns are taken at the thrust that comes. Two parts:
        //   * the law (AttitudeMath, AttitudeInput.TrimNullColumns = the gimbal columns): a gimbal column
        //     whose effect is below the floor (no thrust) is regularized for real toward u_pref = 0 instead
        //     of being left by the active set at the bound it reached under thrust (measured: the super
        //     hope's gimbals at a stop 33 % of its coast, the heavy booster's at 78-100 % of their travel
        //     at the descent ignition, then a 1.0-3.7 MN.m kick for 0.4-0.9 s, |omega| times 2 to 4);
        //   * the columns (AttitudeV1GimbalTrim.cs, GimbalSpool): each engine's thrust is taken at the
        //     throttle its spool reaches over the gimbal's own response time (the time constant of the stock
        //     Lerp), never below the thrust now, and the torque that spool adds with the gimbals where they
        //     are is taken off the allocation target (AttitudeInput.TorqueFree): the gimbals move to the trim
        //     of the coming thrust instead of running to their stops on the small columns of a spool start.
        // Frames carry capability bits 17 (a null gimbal column regularized this step) and 18 (gimbal
        // columns at the coming thrust). Without the option: the columns of finalThrust, the regularization
        // of before, nothing added to the free torque.
        // -----------------------------------------------------------------------------------

        // -----------------------------------------------------------------------------------
        // option 256 of AttitudeOptionsV1 (OptionFreeAuthority), BORN CLOSED: the FREE authority of the
        // lateral-force layer, published beside the honest one. Measured on eight pad landings of the heavy
        // booster: the authority of bit 32 (the model's times the measured share) was 0 on 24-33 % of the
        // translating steps of the last six seconds, in three or four holes of 0.5-1 s, while the gimbals
        // realized the commanded common (realized / commanded 0.96-1.02). The share projects the TOTAL gimbal
        // deflection on the common; the law's attitude part (its largest column at 74-104 % of a 2-degree
        // span on that booster, against a common of about a tenth of a degree) turns it negative whenever the
        // nose turns toward the demand, and the client's final phase switched law at every hole. With the option, every planned step with the
        // computed levers (bit 4) also solves the joint allocation ALONG the gap the nose does not give, twice:
        // with the model's bounds, then with what the attitude leaves -- each gimbal's directional range for
        // the common is the smaller of frac_gimbal of its range and the room its realized attitude part leaves
        // in its box, the motifs' bounds keep the realized attitude part of the differential within the
        // symmetric half width, the RCS is the joint's own (bit 16: its headroom after the rotation). The free
        // authority is the model's (offset 25) times the ratio of the two, in [0, 1]: the model's when the
        // attitude leaves the room, never a share that the attitude's own deflection turns negative. No gap
        // this step (no request, or the nose gives it all): the ratio is 1. The layer itself, the law, every
        // frame and every other return value are unchanged; the return of attitude_translation_v1 carries the
        // block after the evidence block (AttitudeTranslation.FreeFields), and law frames carry capability bit
        // 19 on the steps that computed it. Without the option nothing of it runs.
        // -----------------------------------------------------------------------------------

        // -----------------------------------------------------------------------------------
        // option 512 of AttitudeOptionsV1 (OptionMainThrottle), BORN CLOSED: the MAIN THROTTLE decided
        // at the physics step. Without it attitude_throttle_v1 is refused and nothing below runs; the loop's
        // fly-by-wire link never touches ctrlState.mainThrottle. With it, an order of attitude_throttle_v1 (a dated
        // ramp, a pulse of N steps, a cut when s(t) = 0, an ignition when h <= h_ign(v), or the flip's calage of
        // an earlier change) is decided at every Earlyish and written at this step's fly-by-wire, after kRPC's control
        // inputs and AutoPilot; an earlier change's thrust guard, the order's validity, the client's throttle as the fallback,
        // the hold of a step without decision and the zero at the lease's expiry are MainThrottle.cs's. Law frames
        // carry capability bit 20 on the steps that decided a throttle. The law, every other write and every other
        // return are unchanged.
        // -----------------------------------------------------------------------------------

        // -----------------------------------------------------------------------------------
        // option 1024 of AttitudeOptionsV1 (OptionRollTarget), BORN CLOSED: the ROLL POINTED. Without it a
        // reference with a roll mode other than 0 is refused exactly as before. With it, attitude_reference_v3 may carry
        // roll mode 1 and an up vector (in the reference's frame): the roof of the vessel (-z) toward the up vector's part
        // across the nose. At every step the loop converts it to the body frame, its roll error (AttitudeMath.RollError)
        // goes into the law's Error[1] with AttitudeInput.RollTargetValid, and the roll follows the square-root reference
        // of the other axes on its own authority and AutoPilot limit; law frames carry capability bit 21. The roll is
        // written by the existing roll channel (M3: write-mask bit 1, its guards, its coupling rules) -- the client gives
        // the AutoPilot the direction only (roll damped) and the loop the roll target, instead of the AutoPilot's
        // set_direction_and_up at RCS +-1. Without the option, or with roll mode 0 (every V1 and V2 reference), the roll
        // is rate damping, bit for bit.
        // -----------------------------------------------------------------------------------

        // -----------------------------------------------------------------------------------
        // option 2048 of AttitudeOptionsV1 (OptionReferenceRing), BORN CLOSED: the REFERENCE RING. Without it
        // attitude_reference_ring_v1 is refused and nothing of it runs. With it, a ring of dated samples (the nose, the
        // lateral acceleration, the main throttle) is interpolated at every physics step inside its span
        // (ReferenceRing.cs) and served in place of the standing reference's direction (no omega_ff then), the
        // live translation request's acceleration and a live ramp order's throttle; outside its span, the loop is
        // the loop without a ring. The ring renews nothing (lease, validity, the reference's age). Law frames carry
        // capability bit 22 on the steps it served a channel.
        // -----------------------------------------------------------------------------------

        /// <summary>pitch/yaw not written this step under bit 16: the gimbals do not lead.</summary>
        internal const long MotifGimbalsNotLeading = 1L << 42;

        /// <summary>pitch/yaw not written under bit 16 since the gimbals stopped leading after a write
        /// (the latch, until a reference without the bit).</summary>
        internal const long MotifGimbalsLeadLost = 1L << 43;

        /// <summary>The write-mask bits the options allow: 0-3, with option 16 bits 4 and 8, with option 64 bit 16.</summary>
        internal static int AllowedWriteMask (int options)
        {
            int allowed = (options & OptionCoastWrite) != 0 ? WriteMaskCoast : WriteMaskBase;
            if ((options & OptionBoostbackWrite) != 0)
                allowed |= WriteGimbalsLead;
            return allowed;
        }

        /// <summary>
        /// the gimbals LEAD the pitch/yaw channel -- on body x and z, the authority of the gimbal
        /// columns (sum over j &lt; nGimbalColumns of |b[a n + j]| times the larger of |uMin_j|, |uMax_j|) is
        /// above zero and at least that of the three ctrlState columns after them. b as AttitudeInput.B (or
        /// BAuthority, as the law's alpha reads it). False on fewer columns, or a NaN.
        /// </summary>
        internal static bool GimbalsLead (double[] b, double[] uMin, double[] uMax, int n, int nGimbalColumns)
        {
            if (nGimbalColumns <= 0 || n < nGimbalColumns + 3)
                return false;
            for (int a = 0; a < 3; a += 2) {
                double gimbal = 0.0;
                for (int j = 0; j < nGimbalColumns; j++) {
                    double span = Math.Abs (uMax [j]) > Math.Abs (uMin [j]) ? Math.Abs (uMax [j]) : Math.Abs (uMin [j]);
                    gimbal = gimbal + Math.Abs (b [a * n + j]) * span;
                }
                double channel = 0.0;
                for (int j = nGimbalColumns; j < nGimbalColumns + 3; j++) {
                    double span = Math.Abs (uMax [j]) > Math.Abs (uMin [j]) ? Math.Abs (uMax [j]) : Math.Abs (uMin [j]);
                    channel = channel + Math.Abs (b [a * n + j]) * span;
                }
                if (!(gimbal > 0.0) || !(gimbal >= channel))
                    return false;
            }
            return true;
        }

        /// <summary>
        /// the latch after this step -- cleared without bit 16 in effect; set once the channel wrote
        /// pitch/yaw at the previous physics step UNDER the bit (`wroteLastStep`: a write without it, a coast's,
        /// does not count) and the gimbals do not lead now; kept otherwise.
        /// </summary>
        internal static bool LeadLatch (bool latched, bool bitInEffect, bool wroteLastStep, bool lead)
        {
            if (!bitInEffect)
                return false;
            return latched || (wroteLastStep && !lead);
        }

        /// <summary>
        /// the motifs bit 16 adds to the pitch/yaw channel this step (0 without the bit in effect): 42
        /// while the gimbals do not lead, 43 while latched. Not zero: pitch/yaw is not written.
        /// </summary>
        internal static long LeadMotifs (bool bitInEffect, bool lead, bool latched)
        {
            if (!bitInEffect)
                return 0;
            long m = 0;
            if (!lead)
                m |= MotifGimbalsNotLeading;
            if (latched)
                m |= MotifGimbalsLeadLost;
            return m;
        }

        /// <summary>the pitch/yaw channel wrote (or may write) through a reference older than
        /// PitchYawMaxReferenceAgeS, held by bit 8 of the write mask (informational).</summary>
        internal const long MotifPitchYawReferenceHeld = 1L << 40;

        /// <summary>the client's write mask held the control surfaces out of the law (bit 4,
        /// informational; SurfaceModelV1's not-owned sanction bits say it per surface).</summary>
        internal const long MotifSurfacesHeldOut = 1L << 41;

        /// <summary>a write mask the reference may carry -- 0 to 3, and with option 16 also bits 4 and 8
        /// (with option 64 also bit 16).</summary>
        internal static bool ValidWriteMask (int writeMask, int options)
        {
            int allowed = AllowedWriteMask (options);
            return writeMask >= 0 && (writeMask & ~allowed) == 0;
        }

        /// <summary>the stored write mask as the tick reads it -- bits 4 and 8 only with option 16 now (
        /// bit 16 only with option 64 now).</summary>
        internal static int EffectiveWriteMask (int writeMask, int options)
        {
            return writeMask & AllowedWriteMask (options);
        }

        /// <summary>
        /// the axes the loop owns for the ENGAGEMENT of the control surfaces (an earlier change's LoopOwnedAxes),
        /// none while the client's write mask holds them out (bit 4, option 16). Without the option or the
        /// bit: exactly LoopOwnedAxes.
        /// </summary>
        internal static int SurfaceOwnedAxes (int mode, int axesMask, int writeMask, int options, int stage,
            int lastWrittenAxes)
        {
            if ((EffectiveWriteMask (writeMask, options) & WriteSurfacesOut) != 0)
                return 0;
            return LoopOwnedAxes (mode, axesMask, writeMask, stage, lastWrittenAxes);
        }

        /// <summary>
        /// The pitch/yaw reference is too old for the channel (motif 26): older than
        /// PitchYawMaxReferenceAgeS (or none, age +infinity) -- unless the client's write mask holds it
        /// (bit 8, option 16; its own validity, utValidUntil, still ends it through motif 2). Without the
        /// option or the bit: exactly the rule of M4.
        /// </summary>
        internal static bool PitchYawReferenceOld (double age, int writeMask, int options)
        {
            if (age <= PitchYawMaxReferenceAgeS)
                return false;
            if (double.IsInfinity (age) || double.IsNaN (age))
                return true;
            return (EffectiveWriteMask (writeMask, options) & WriteHoldReference) == 0;
        }

        /// <summary>the hold of bit 8 is what keeps the channel on this reference (motif 40).</summary>
        internal static bool PitchYawReferenceHeld (double age, int writeMask, int options)
        {
            return !(age <= PitchYawMaxReferenceAgeS) && !PitchYawReferenceOld (age, writeMask, options);
        }

        /// <summary>the pitch/yaw channel holds its leased gimbals at the measured contact (option 8).</summary>
        internal const long MotifPitchYawContactHold = 1L << 39;

        /// <summary>
        /// the pitch/yaw motifs a contact hold tolerates -- the contact itself and what the contact
        /// does to the law (its axis guards and the hold after them, a step not converged or approximate),
        /// and the fly-by-wire order (the hold writes no ctrlState). Every other blocking motif ends it.
        /// </summary>
        internal const long ContactHoldTolerated = MotifContact | MotifAxisResidualGuard | MotifAxisOmegaGuard |
            MotifAxisSaturationGuard | MotifPitchYawGuardHold | MotifNotConverged | MotifRcsModelApprox |
            MotifSurfaceApprox | MotifFlyByWireOrder;

        /// <summary>
        /// the pitch/yaw channel, which does not write this step, keeps its gimbals leased at their
        /// last deflection: option 8, armed on pitch/yaw, at Earlyish, the contact MEASURED (Landed outside
        /// PRELAUNCH, or Splashed), a lease to continue (gimbals leased at the last write, never released
        /// since), and no blocking motif of the channel but those of ContactHoldTolerated (the contact among
        /// them). False without the option: exactly the fallback of before.
        /// </summary>
        internal static bool ContactHold (int options, int mode, int axesMask, int stage, bool measuredContact,
            bool leaseToContinue, long pitchYawMotif)
        {
            return (options & OptionContactHold) != 0 && mode == ModeArmed && (axesMask & ArmPitchYaw) != 0 &&
                stage == StageEarlyish && measuredContact && leaseToContinue &&
                (pitchYawMotif & MotifContact) != 0 &&
                (pitchYawMotif & BlockingMotifsPitchYaw & ~ContactHoldTolerated) == 0;
        }

        // Informational motif bits, set only with OptionFamilyBlocking (frame header offset 12;
        // a double holds them exactly up to bit 52).
        internal const long MotifCtrlFrozenPitch = 1L << 32;   // ctrlState column of body x frozen
        internal const long MotifCtrlFrozenRoll = 1L << 33;    // body y
        internal const long MotifCtrlFrozenYaw = 1L << 34;     // body z
        internal const long MotifSurfacesNeutralized = 1L << 35;
        internal const long MotifRollSurfaceGate = 1L << 36;   // roll not written: a surface answers it

        /// <summary>The model motifs that, with OptionFamilyBlocking, sanction a family, not a channel.</summary>
        internal const long FamilyModelMotifs = MotifRcsModelApprox | MotifSurfaceApprox |
            MotifFlyByWireOrder;

        internal static bool ValidOptions (int options)
        {
            return options >= 0 && (options & ~OptionsSupported) == 0;
        }

        /// <summary>The roll blocking set under the options (0: BlockingMotifs, unchanged).</summary>
        internal static long RollBlockingSet (int options)
        {
            long set = BlockingMotifs;
            if ((options & OptionFamilyBlocking) != 0)
                set &= ~FamilyModelMotifs;
            if ((options & OptionServeNotConverged) != 0)
                set &= ~MotifNotConverged;
            return set;
        }

        /// <summary>The pitch/yaw blocking set under the options (0: BlockingMotifsPitchYaw).</summary>
        internal static long PitchYawBlockingSet (int options)
        {
            long set = BlockingMotifsPitchYaw;
            if ((options & OptionFamilyBlocking) != 0)
                set &= ~FamilyModelMotifs;
            if ((options & OptionServeNotConverged) != 0)
                set &= ~MotifNotConverged;
            return set;
        }

        /// <summary>MayWrite under the options (0: exactly MayWrite).</summary>
        internal static bool MayWrite (int mode, int axesMask, int stage, bool computed, long motif,
            int options)
        {
            return mode == ModeArmed && (axesMask & ArmRoll) != 0 && stage == StageEarlyish &&
                computed && (motif & RollBlockingSet (options)) == 0;
        }

        /// <summary>MayWritePitchYaw under the options (0: exactly MayWritePitchYaw).</summary>
        internal static bool MayWritePitchYaw (int mode, int axesMask, int stage, bool computed,
            long motif, int options)
        {
            return mode == ModeArmed && (axesMask & ArmPitchYaw) != 0 && stage == StageEarlyish &&
                computed && (motif & PitchYawBlockingSet (options)) == 0;
        }

        /// <summary>
        /// body axes (bit a: 0 pitch x, 1 roll y, 2 yaw z) whose ctrlState column is frozen
        /// this step: all three when the RCS model is approximate (13) or the fly-by-wire order is
        /// not verified (20, armed only), plus the axes a surface that cannot be neutralized
        /// answers. 0 without OptionFamilyBlocking.
        /// </summary>
        internal static int FrozenCtrlAxes (int options, long motif, int unneutralizableAxes)
        {
            if ((options & OptionFamilyBlocking) == 0)
                return 0;
            int axes = unneutralizableAxes & 7;
            if ((motif & (MotifRcsModelApprox | MotifFlyByWireOrder)) != 0)
                axes = 7;
            return axes;
        }

        /// <summary>Motif bits 32-34 of a frozen-axes mask.</summary>
        internal static long FrozenMotifs (int frozenAxes)
        {
            long m = 0;
            if ((frozenAxes & 1) != 0)
                m |= MotifCtrlFrozenPitch;
            if ((frozenAxes & 2) != 0)
                m |= MotifCtrlFrozenRoll;
            if ((frozenAxes & 4) != 0)
                m |= MotifCtrlFrozenYaw;
            return m;
        }

        /// <summary>
        /// the surfaces are neutralized this step (at Earlyish, armed, with the option, some
        /// surface answering an axis) when the loop writes ctrlState pitch/yaw now (nowAxes, bit 0
        /// pitch, bit 2 yaw), or wrote pitch/yaw at the last fly-by-wire, or wrote there a roll the
        /// surfaces answer (lastAxes, bits 0-2): whether a surface's FixedUpdate reads the ctrlState
        /// of this step's fly-by-wire or of the previous one is not measured, both are covered. A
        /// roll alone never starts it (the roll gate holds it back while the surfaces answer the
        /// roll un-neutralized); once the pitch/yaw channel has, a roll the loop keeps writing keeps
        /// them neutral -- the configuration the witness flights flew with their fins ignored.
        /// </summary>
        internal static bool NeutralizeSurfaces (int options, int mode, int stage, int nowAxes,
            int lastAxes, int answeringAxes)
        {
            return (options & OptionFamilyBlocking) != 0 && mode == ModeArmed &&
                stage == StageEarlyish && (answeringAxes & 7) != 0 &&
                ((nowAxes & 5) != 0 || (lastAxes & 5) != 0 || (lastAxes & answeringAxes & 2) != 0);
        }

        /// <summary>
        /// the ctrlState axes (bit 0 pitch, 1 roll, 2 yaw; written values, NaN when not)
        /// the loop's own fly-by-wire link wrote at the previous physics step; 0 when the
        /// callback did not run at that step (`ranLastStep`: its attitude tick is this one minus 1).
        /// </summary>
        internal static int WrittenAxes (bool ranLastStep, double pitch, double roll, double yaw)
        {
            if (!ranLastStep)
                return 0;
            return (double.IsNaN (pitch) ? 0 : 1) | (double.IsNaN (roll) ? 0 : 2) | (double.IsNaN (yaw) ? 0 : 4);
        }

        /// <summary>
        /// the axes the loop OWNS for the engagement of the control surfaces at this
        /// Earlyish: armed, at Earlyish, written by its own fly-by-wire link at the previous
        /// physics step (lastWrittenAxes, WrittenAxes) AND still in both its arm mask and the
        /// client's write mask now. This step's own write decision cannot be used: it comes after
        /// the law, and the law needs the engaged surfaces as its input (their columns, their
        /// torque in M, M_free). So a surface is never engaged before the loop has actually
        /// written every axis it needs -- one step of proof at an engagement -- and a write the
        /// loop refuses at this same step (law not computed, an axis guard, contact, authority)
        /// costs one physics step at most, the surfaces disengaged from the next.
        /// </summary>
        internal static int LoopOwnedAxes (int mode, int axesMask, int writeMask, int stage, int lastWrittenAxes)
        {
            if (mode != ModeArmed || stage != StageEarlyish)
                return 0;
            return lastWrittenAxes & ArmedBodyAxes (axesMask & writeMask) & 7;
        }

        /// <summary>
        /// a roll write is held back when a surface answers the roll and is not neutralized
        /// this step (its roll effectiveness is not in the column). False without the option.
        /// </summary>
        internal static bool RollSurfaceGate (int options, int answeringAxes, bool neutralized)
        {
            return (options & OptionFamilyBlocking) != 0 && (answeringAxes & 2) != 0 && !neutralized;
        }
    }
}
