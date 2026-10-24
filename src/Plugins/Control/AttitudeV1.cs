using System;
using System.Collections.Generic;
using System.Diagnostics;
using KRPC.Bridge.Core;
using UnityEngine;
using SCReferenceFrame = KRPC.SpaceCenter.Services.ReferenceFrame;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// AttitudeV1: the attitude loop at the physics rate (steps M2 and M3).
    ///
    /// Every FixedUpdate (TimingManager stage Earlyish), for each vessel a client armed by
    /// persistentId, active or not: measures the rotational state and the REALIZED torques,
    /// builds the control-effectiveness columns (gimbals, ctrlState channel = RCS + reaction
    /// wheels + control surfaces), runs the law of AttitudeMath (square-root reference + PCH,
    /// torque-form INDI, bounded WLS allocation) and publishes the wanted torque M_d, the
    /// allocation and the residuals in a 1000-frame ring.
    ///
    /// Mode 0 (shadow) writes no actuator. Mode 1 (armed) has two channels, each falling
    /// back on its own:
    ///   * ROLL (M3, axes bit 1): the command decided at Earlyish (AttitudeWrite.RollCommand)
    ///     is written into ctrlState.roll by a callback kept LAST in Vessel.OnFlyByWire,
    ///     after the kRPC AutoPilot's;
    ///   * PITCH/YAW (M4, axes bit 2): the gimbals are leased under the loop's own actuator
    ///     authority (taken inside the plugin, refused if another owner holds it) at the
    ///     allocation's deflections, and ctrlState.pitch/yaw (RCS only, once the gimbals
    ///     are leased) is written by the same callback. Handed back WITHOUT a jump.
    /// The AutoPilot stays engaged; any blocking motif of a channel (stale or old
    /// reference, guard of its axes, contact, authority, chain order, ...) means no write
    /// of that channel in that step, and the AutoPilot's own command, computed every step
    /// anyway, goes through. Born closed: nothing runs until AttitudeArmV1, and a vessel
    /// whose real-time lease runs out is disarmed, unhooked and its gimbals handed back.
    ///
    /// Lateral force (chantier 19 suite, AttitudeV1Translation.cs + AttitudeTranslation.cs):
    /// once the client calls AttitudeTranslationV1, and only while the pitch/yaw channel may
    /// write, two differential-thrust motif columns join the allocation and a common gimbal
    /// deflection (translation) is pre-allocated, its torque cancelled by the motifs.
    ///
    /// (the non-active booster of a double flight): the actuator authority of the
    /// pitch/yaw channel is the ACTIVE vessel's, so the other armed booster fell back at every
    /// step (motif 28) and flew under the AutoPilot. Armed with AttitudeArmV2 option 1, a
    /// vessel that is not the active one takes its own per-vessel authority instead
    /// (Actuation.TryHoldForAttitudeInactive, leases apart from the active vessel's);
    /// option 3 lets the lateral force run there too. Option 0 (AttitudeArmV1): unchanged.
    /// </summary>
    internal static partial class AttitudeV1
    {
        internal const int ProtocolVersion = 1;
        // Schema 4 (refactor R3): the frame ends with the clock section (ClockStride values).
        internal const int SchemaVersion = 4;
        // attitude_status_v1 did not change: it keeps the schema it had (3).
        internal const int StatusSchemaVersion = 3;
        // The trailing clock section: the GameClock master tick of the step the frame is of.
        internal const int ClockStride = 1;
        internal const int MaxVessels = 2;
        internal const int HistoryCapacity = 1000;
        internal const int ModeShadow = AttitudeWrite.ModeShadow;
        internal const int ModeArmed = AttitudeWrite.ModeArmed;
        internal const int HeaderStride = 24;
        internal const int StateStride = 110;
        internal const int RowStride = 16;
        internal const int StatusHeaderStride = 9;
        internal const int StatusRowStride = 25;
        const float MinLeaseSeconds = 0.2f;
        const float MaxLeaseSeconds = 10f;
        const int PumpFallbackAfter = 3;

        // Actuator row kinds.
        internal const int KindGimbal = 1;
        internal const int KindCtrlState = 3;

        // Stage codes (header offset 7).
        internal const int StageEarlyish = AttitudeWrite.StageEarlyish;
        internal const int StagePumpFallback = AttitudeWrite.StagePumpFallback;

        // Motif bits: why the law did not run or did not write, or what WOULD make an armed
        // loop fall back. Defined in the pure AttitudeWrite (tested, mirrored in Python).
        internal const long MotifLeaseExpired = AttitudeWrite.MotifLeaseExpired;
        internal const long MotifNoReference = AttitudeWrite.MotifNoReference;
        internal const long MotifReferenceStale = AttitudeWrite.MotifReferenceStale;
        internal const long MotifWarp = AttitudeWrite.MotifWarp;
        internal const long MotifTickGap = AttitudeWrite.MotifTickGap;
        internal const long MotifUnloaded = AttitudeWrite.MotifUnloaded;
        internal const long MotifTopologyChanged = AttitudeWrite.MotifTopologyChanged;
        internal const long MotifResidualGuard = AttitudeWrite.MotifResidualGuard;
        internal const long MotifOmegaGuard = AttitudeWrite.MotifOmegaGuard;
        internal const long MotifSaturationGuard = AttitudeWrite.MotifSaturationGuard;
        internal const long MotifException = AttitudeWrite.MotifException;
        internal const long MotifEarlyishDead = AttitudeWrite.MotifEarlyishDead;
        internal const long MotifFrameConversion = AttitudeWrite.MotifFrameConversion;
        internal const long MotifRcsModelApprox = AttitudeWrite.MotifRcsModelApprox;
        internal const long MotifSurfaceApprox = AttitudeWrite.MotifSurfaceApprox;
        internal const long MotifNotConverged = AttitudeWrite.MotifNotConverged;
        internal const long MotifRollChannelUnmodeled = AttitudeWrite.MotifRollChannelUnmodeled;
        internal const long MotifFlyByWireOrder = AttitudeWrite.MotifFlyByWireOrder;
        internal const long MotifFlyByWireReordered = AttitudeWrite.MotifFlyByWireReordered;
        internal const long MotifClientWriteOff = AttitudeWrite.MotifClientWriteOff;
        internal const long MotifGuardHold = AttitudeWrite.MotifGuardHold;
        internal const long MotifWriteFallback = AttitudeWrite.MotifWriteFallback;
        internal const long MotifFlyByWireMissed = AttitudeWrite.MotifFlyByWireMissed;
        internal const long MotifPitchYawReferenceOld = AttitudeWrite.MotifPitchYawReferenceOld;
        internal const long MotifContact = AttitudeWrite.MotifContact;
        internal const long MotifGimbalAuthority = AttitudeWrite.MotifGimbalAuthority;
        internal const long MotifPitchYawFallback = AttitudeWrite.MotifPitchYawFallback;
        internal const long MotifPitchYawWriteOff = AttitudeWrite.MotifPitchYawWriteOff;
        internal const long MotifPitchYawGuardHold = AttitudeWrite.MotifPitchYawGuardHold;
        const long AxisGuardBits = AttitudeWrite.MotifAxisResidualGuard |
            AttitudeWrite.MotifAxisOmegaGuard | AttitudeWrite.MotifAxisSaturationGuard;

        // Capability bits (header offset 23).
        internal const long CapEngineTorque = 1L << 0;
        internal const long CapRcsTorque = 1L << 1;
        internal const long CapWheelTorque = 1L << 2;
        internal const long CapGimbalColumns = 1L << 3;
        internal const long CapRcsColumns = 1L << 4;
        internal const long CapWheelColumns = 1L << 5;
        internal const long CapSurfaceColumns = 1L << 6;
        internal const long CapReference = 1L << 7;
        internal const long CapLaw = 1L << 8;
        // at least one gimbal module was mapped per pivot this step (columns, lease).
        internal const long CapGimbalPivots = 1L << 9;
        // the authority of at least one mapped module was its exact realized maximum
        // (attitude_gimbal_pivots_v1 mode 2) this step.
        internal const long CapGimbalExactAuthority = 1L << 10;

        internal sealed class GimbalRef
        {
            internal ModuleGimbal Gimbal;
            internal Part Part;
            internal int Ordinal;
            internal List<ModuleEngines> PartEngines;
            // the per-pivot mapping of this module for this step (AttitudeGimbalPivotsV1
            // mode 1 and a module whose pivots are not parallel), else the V1 path.
            internal readonly GimbalPivotCommand Pivots = new GimbalPivotCommand ();
            internal bool PivotsMapped;
            // this step's columns with no pivot at its bound (the law's authority).
            internal Vector3 FreeX, FreeY;
            // (option 8): the deflection this module was leased at by the last pitch/yaw write
            // (HeldTick = that write's own tick, -1 never): what a contact hold commands again.
            internal float HeldX, HeldY;
            internal long HeldTick = -1;
            // (option 128): each engine's thrust (kN, aligned with PartEngines) at this gimbal's
            // horizon, the thrust its columns are evaluated at (AttitudeV1GimbalTrim.cs).
            internal double[] SpoolThrust;
            // (option 512): each engine's thrust (kN, aligned with PartEngines) at full main throttle, the
            // thrust the calage's gimbal authority is taken at (AttitudeV1Throttle.cs).
            internal double[] FullThrust;
        }

        internal sealed class Entry
        {
            internal uint PersistentId;
            // Core v2: owner, token and wall guard of the arming are one Lease.
            internal readonly Lease Arming = new Lease ();
            internal string Owner {
                get { return Arming.Owner; }
                set { Arming.Owner = value; }
            }
            internal string Token {
                get { return Arming.Token; }
                set { Arming.Token = value; }
            }
            internal int Mode;
            internal int AxesMask;
            internal float LeaseSeconds;

            // Reference, as the client last sent it (same meaning as the AutoPilot's).
            internal bool HaveReference;
            internal int Sequence;
            internal SCReferenceFrame Frame;
            internal double DirX, DirY, DirZ;
            internal int RollMode;
            internal double FfX, FfY, FfZ;
            internal double UtStamp, UtValidUntil, OmegaMax;
            internal int AcceptedCount;
            // (option 1024): the roll target of the last reference (roll mode 1 of attitude_reference_v3: the
            // roof toward this up vector, in Frame); false for every V1/V2 reference and at every arming.
            internal bool RollTargetOn;
            // Roll mode 2 (shadow): the roll error published in the frame's error y, never in the law.
            internal bool RollTargetShadow;
            internal double UpX, UpY, UpZ;

            // Vessel cache, rebuilt when the topology hash moves.
            internal Vessel Vessel;
            internal Guid VesselId = Guid.Empty;
            internal ulong TopologyHash;
            internal long TopologyGeneration;
            internal readonly List<ModuleEngines> Engines = new List<ModuleEngines> ();
            internal readonly List<GimbalRef> Gimbals = new List<GimbalRef> ();
            internal readonly List<ModuleRCS> Rcs = new List<ModuleRCS> ();
            internal readonly List<ModuleReactionWheel> Wheels = new List<ModuleReactionWheel> ();
            internal readonly List<ModuleControlSurface> Surfaces = new List<ModuleControlSurface> ();

            // Law.
            internal readonly AttitudeGains Gains = new AttitudeGains ();
            internal readonly AttitudeLawState Law = new AttitudeLawState ();
            internal readonly AttitudeInput Input = new AttitudeInput ();
            internal readonly AttitudeOutput Output = new AttitudeOutput ();
            internal readonly AttitudeWork Work = new AttitudeWork ();
            internal readonly double[] BPos = new double[9];
            internal readonly double[] BNeg = new double[9];
            internal readonly double[] Scratch = new double[3];
            internal readonly double[] RefBody = new double[3];
            internal readonly double[] FfBody = new double[3];
            // ctrlState pitch, roll, yaw as served this step (was an array allocated per tick).
            internal readonly double[] Served = new double[3];

            // Measurement memory.
            internal bool HavePrevious;
            internal long PreviousTick;
            internal Vector3 PreviousOmega;

            // Ring and bookkeeping (Core v2: the preallocated frame ring).
            internal readonly Recorder History = new Recorder (HistoryCapacity);
            internal long LatestTick;
            internal long LastMotif;
            internal int ConsumedSequence;
            internal double ConsumedUtStamp = double.NaN;
            internal long Computed;
            internal long Exceptions;
            internal double CostMicrosLast;
            internal double CostMicrosMax;
            internal bool Loaded;

            // M3: per-axis AutoPilot limit and write permission of the last reference.
            internal double OmegaMaxX, OmegaMaxY, OmegaMaxZ;
            internal int WriteMask;
            internal bool Dropped;
            // M3: fly-by-wire hook (mode 1 only).
            internal FlightInputCallback FlyByWire;
            internal Vessel HookedVessel;
            internal Delegate ChainSeen;
            internal int ChainOwnIndex = -1;
            internal int ChainKrpcIndex = -1;
            internal int ChainLength;
            internal long ChainReorders;
            // M3: the decision of this step (Earlyish), consumed by the fly-by-wire callback.
            internal bool WriteRoll;
            internal double WriteRollValue = double.NaN;
            internal long WriteTick = -1;
            internal double GuardHoldUntil = double.NegativeInfinity;
            // M3: what the fly-by-wire callback saw and wrote, published in the next frame.
            internal long FbwTick = -1;
            internal double FbwApPitch = double.NaN, FbwApRoll = double.NaN, FbwApYaw = double.NaN;
            internal double FbwWrittenRoll = double.NaN;
            internal long FbwCalls;
            internal long FbwExceptions;
            internal long Writes;
            internal long FallbackSteps;

            // M4 : pitch/yaw (gimbals leased under the loop's own authority + RCS by ctrlState).
            internal string AuthorityToken;
            internal bool WritePitchYaw;
            internal double WritePitchValue = double.NaN, WriteYawValue = double.NaN;
            internal long WritePitchYawTick = -1;
            internal double PitchYawHoldUntil = double.NegativeInfinity;
            internal double FbwWrittenPitch = double.NaN, FbwWrittenYaw = double.NaN;
            internal long PitchYawWrites;
            internal long PitchYawFallbackSteps;
            internal int GimbalsLeased;
            // (option 8): physics steps the pitch/yaw channel held its gimbals at the contact.
            internal long ContactHoldSteps;
            internal int LastWriteStateRoll;
            internal int LastWriteStatePitchYaw;

            // options of AttitudeArmV2 (0 = V1 arming, born closed), and whether
            // AuthorityToken is the per-vessel authority of a NON-ACTIVE vessel.
            internal int InactiveOptions;
            internal bool AuthorityInactive;

            // AttitudeGimbalPivotsV1 (0 = V1 at every arming, born closed ; 1 = modules
            // with several non-parallel pivots mapped per pivot, GimbalPivots.cs).
            internal int PivotMode;

            // Lateral force (chantier 19 suite): AttitudeV1Translation.cs. Idle until the
            // client calls AttitudeTranslationV1.
            internal readonly TranslationEntry T = new TranslationEntry ();

            // Refactor lot 1 (AttitudeV1Families.cs): options of AttitudeOptionsV1 (0 at every
            // arming, born closed); the ctrlState axes frozen this step, the axes the surfaces
            // answer, and the surfaces held neutral until BetterLateThanNever.
            internal int Options;
            // Refactor R1 (EffectorRegistry.cs, AttitudeEffectorsV1): the effector registry of
            // this arming (false at every arming: born closed) and its table of the last step.
            internal bool EffectorRegistry;
            internal readonly EffectorTable Effectors = new EffectorTable ();
            internal int FrozenAxes;
            internal int SurfaceAxes;
            internal int SurfaceAxesUnneutralizable;
            internal readonly List<ModuleControlSurface> Neutralized = new List<ModuleControlSurface> ();
            internal readonly List<object> NeutralizedOriginal = new List<object> ();
            // Lot 3, 3.1 (AttitudeV1Surfaces.cs, AttitudeSurfacesV1): SurfaceModelV1 of this arming
            // (off at every arming: born closed; its scale guard starts at k = 1: mono-flight).
            internal readonly SurfaceState Sm = new SurfaceState ();
            // (AttitudeV1RcsInFlight.cs, option 32): the RCS commands fired at the last step and the
            // torque in flight of this one (reset at every arming).
            internal readonly RcsInFlightState RcsFlight = new RcsInFlightState ();
            // (option 64): the latch of bit 16 -- the gimbals stopped leading after a pitch/yaw write made
            // under the bit (false at every arming and on every reference without the bit) -- and whether the last
            // pitch/yaw decision wrote under the bit.
            internal bool LeadLatched;
            internal bool LeadWritten;
            // (option 128): the torque the engines' spool adds this step with the nozzles where they
            // are (body, N.m), taken off the allocation target (AttitudeV1GimbalTrim.cs).
            internal readonly double[] SpoolFree = new double[3];
            // (option 512): the order of the main throttle, the step's decision and what was written
            // (AttitudeV1Throttle.cs; reset at every arming).
            internal readonly ThrottleState Th = new ThrottleState ();
            // (option 2048): the client's reference ring and its value at this step (AttitudeV1Ring.cs; reset
            // at every arming).
            internal readonly RingState Ring = new RingState ();
            // (option 4096): the engine-by-engine allocation in shadow (AttitudeV1Allocation.cs; reset
            // at every arming).
            internal readonly AllocationState Al = new AllocationState ();
            // (option 16384): the RCS nozzles of this step as ModuleRCS fires them and the decision of the step
            // (AttitudeV1RcsCombined.cs; reset at every arming and when the option changes).
            internal readonly RcsCombinedState Rc = new RcsCombinedState ();
        }

        static readonly Dictionary<uint, Entry> entries = new Dictionary<uint, Entry> ();
        static readonly TimingManager.UpdateAction earlyishAction = OnEarlyish;
        static bool registered;
        // +1 per attitude tick (Earlyish or fallback). Core v2: a counter hosted by the game
        // clock (clock_v1 maps it onto the master tick), same increments, same value.
        static readonly ClockCounter attitudeTicks = GameClock.Counter ("attitude.tick");
        static long ownTick {
            get { return attitudeTicks.Value; }
        }
        // The read of a vessel that is not armed: an empty ring.
        static readonly Recorder emptyRing = new Recorder (1);
        static long earlyishCalls;
        static long pumpFallbackTicks;
        static long lastEarlyishSeenByPump = -1;
        static int pumpCallsWithoutEarlyish;

        // ---------------------------------------------------------------- API (RPC bodies)

        internal static string Arm (string persistentId, string owner, float leaseSeconds,
            int mode, int axesMask)
        {
            return Arm (persistentId, owner, leaseSeconds, mode, axesMask, 0);
        }

        /// <summary>
        /// Arming. inactiveOptions (AttitudeArmV2; 0 = exactly the V1 arming): bit 1 the
        /// pitch/yaw channel holds its own authority while the vessel is NOT the active one,
        /// bit 2 the lateral force may run there too (with bit 1). See AttitudeWrite.
        /// </summary>
        internal static string Arm (string persistentId, string owner, float leaseSeconds,
            int mode, int axesMask, int inactiveOptions)
        {
            uint pid = VesselLookup.ParsePersistentId (persistentId);
            if (string.IsNullOrWhiteSpace (owner))
                throw new ArgumentException ("owner vide", nameof (owner));
            if (owner.Length > 80)
                throw new ArgumentOutOfRangeException (nameof (owner));
            ValidateLease (leaseSeconds);
            if (mode != ModeShadow && mode != ModeArmed)
                throw new ArgumentOutOfRangeException (nameof (mode), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode, "0 ombre, 1 arme"));
            if (axesMask < 0 || axesMask > 3)
                throw new ArgumentOutOfRangeException (nameof (axesMask), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode, "bits : 1 roulis, 2 tangage/lacet"));
            if (mode == ModeArmed && axesMask == 0)
                throw new ArgumentOutOfRangeException (nameof (axesMask), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode, "mode arme (1) : au moins un axe (1 roulis, 2 tangage/lacet)"));
            if (!AttitudeWrite.ValidInactiveOptions (inactiveOptions))
                throw new ArgumentOutOfRangeException (nameof (inactiveOptions), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode, "options vaisseau non actif : 0, 1 (tangage/lacet) ou 3 (+ translation)"));
            if (!AttitudeWrite.InactiveOptionsFit (inactiveOptions, mode, axesMask))
                throw new ArgumentOutOfRangeException (nameof (inactiveOptions), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode,
                    "options vaisseau non actif : mode arme (1) avec tangage/lacet (axes 2) requis"));
            Entry entry;
            bool existing = entries.TryGetValue (pid, out entry);
            if (existing) {
                // Re-arming: the previous hook and authority go, the next tick installs them.
                Unhook (entry);
                ReleaseAuthority (entry);
                RestoreSurfaces (entry);
            }
            // M4 : the gimbals are leased under the loop's own authority ; another owner
            // (the v2 control of S7, a legacy lease, the other armed vessel) forbids it.
            // not for a vessel armed with option 1 that is loaded and not the active
            // one (it never takes that authority). Option 0 : exactly as before.
            if (mode == ModeArmed && (axesMask & AttitudeWrite.ArmPitchYaw) != 0 &&
                    Actuation.OtherOwnerPresent () &&
                    AttitudeWrite.ArmNeedsOwnerCheck (mode, axesMask, inactiveOptions,
                        inactiveOptions != 0 && LoadedInactive (pid)))
                throw new InvalidOperationException (BridgeErrors.Tag (BridgeErrors.Busy,
                    "AttitudeV1 : un proprietaire des actionneurs (controle v2 ou bail " +
                    "historique) est present ; tangage/lacet arme refuse"));
            if (!existing) {
                if (entries.Count >= MaxVessels)
                    throw new InvalidOperationException (BridgeErrors.Tag (BridgeErrors.Busy,
                        "deja " + MaxVessels + " vaisseaux armes : en liberer un avant"));
                entry = new Entry { PersistentId = pid };
                entries.Add (pid, entry);
            }
            entry.Dropped = false;
            entry.Owner = owner.Trim ();
            droppedTokens.Remember (entry.Token, BridgeErrors.TokenInvalid);
            entry.Token = Guid.NewGuid ().ToString ("N");
            entry.Mode = mode;
            entry.AxesMask = axesMask;
            entry.InactiveOptions = inactiveOptions;
            entry.AuthorityInactive = false;
            entry.PivotMode = GimbalPivots.ModeV1;
            // Refactor lot 1: the options are of one arming (born closed).
            entry.Options = 0;
            entry.FrozenAxes = 0;
            // Refactor R1: so is the effector registry.
            entry.EffectorRegistry = false;
            entry.Effectors.Enabled = false;
            // Lot 3, 3.1: so is SurfaceModelV1, and its guard forgets the previous arming.
            entry.Sm.Reset ();
            // and the RCS commands of the previous arming.
            entry.RcsFlight.Reset ();
            // and the latch of bit 16.
            entry.LeadLatched = false;
            entry.LeadWritten = false;
            // and the order of the main throttle, and the zero left by a previous arming's expiry.
            entry.Th.Reset ();
            RemoveThrottleReturn (pid);
            // and the reference ring.
            entry.Ring.Reset ();
            // and the shadow allocation.
            entry.Al.Reset ();
            // and the combined RCS's state.
            entry.Rc.Reset ();
            entry.LeaseSeconds = leaseSeconds;
            entry.Arming.RenewReal (Time.realtimeSinceStartup, leaseSeconds);
            entry.HaveReference = false;
            entry.Sequence = 0;
            entry.Frame = null;
            entry.RollTargetOn = false;
            entry.WriteMask = 0;
            entry.OmegaMaxX = 0.0;
            entry.OmegaMaxY = 0.0;
            entry.OmegaMaxZ = 0.0;
            entry.GuardHoldUntil = double.NegativeInfinity;
            entry.PitchYawHoldUntil = double.NegativeInfinity;
            entry.WritePitchYaw = false;
            entry.LastWriteStateRoll = AttitudeWrite.WriteShadow;
            entry.LastWriteStatePitchYaw = AttitudeWrite.WriteShadow;
            ClearFlyByWireRecord (entry);
            ResetTranslation (entry);
            entry.Law.Reset ();
            entry.HavePrevious = false;
            Register ();
            return entry.Token;
        }

        internal static bool Renew (string token, float leaseSeconds)
        {
            ValidateLease (leaseSeconds);
            var entry = RequireToken (token);
            entry.LeaseSeconds = leaseSeconds;
            entry.Arming.RenewReal (Time.realtimeSinceStartup, leaseSeconds);
            return true;
        }

        internal static bool Release (string token)
        {
            var entry = RequireToken (token);
            Drop (entry.PersistentId);
            return true;
        }

        internal const int PivotsReplyLength = 8;

        // scratch (Unity main thread only).
        static readonly double[] exactBest = new double[3], exactWork = new double[3];
        static readonly double[] exactScale = new double[3];
        static readonly double[] exactFreeX = new double[3], exactFreeY = new double[3];

        /// <summary>
        /// (mode 2): the authority columns of mapped module g become its free columns times
        /// k_a = exact_a / free_a per body axis, exact_a = the largest |torque_a| the module
        /// REALIZES over the law's span box [-sx, sx] x [-sy, sy] within its own box
        /// (GimbalPivots.ExactAuthority), free_a = |free_x,a| sx + |free_y,a| sy. The law's alpha
        /// (sum |BAuthority| span / I) then reads exact_a for this module. False (free columns
        /// kept) when a value is not finite.
        /// </summary>
        static bool ExactAuthorityColumns (AttitudeInput inp, int n, int g, GimbalRef gr)
        {
            int jx = 2 * g, jy = 2 * g + 1;
            var p = gr.Pivots;
            double sx = Math.Max (Math.Abs (inp.UMax [jx]), Math.Abs (inp.UMin [jx]));
            double sy = Math.Max (Math.Abs (inp.UMax [jy]), Math.Abs (inp.UMin [jy]));
            GimbalPivots.ExactAuthority (p.Map, p.Count, p.TorqueCols, p.XMin, p.XMax, p.YMin,
                p.YMax, -sx, sx, -sy, sy, exactBest, exactWork);
            exactFreeX [0] = gr.FreeX.x;
            exactFreeX [1] = gr.FreeX.y;
            exactFreeX [2] = gr.FreeX.z;
            exactFreeY [0] = gr.FreeY.x;
            exactFreeY [1] = gr.FreeY.y;
            exactFreeY [2] = gr.FreeY.z;
            GimbalPivots.AuthorityScale (exactFreeX, exactFreeY, sx, sy, exactBest, exactScale);
            for (int r = 0; r < 3; r++) {
                double bx = exactFreeX [r] * exactScale [r], by = exactFreeY [r] * exactScale [r];
                if (double.IsNaN (bx) || double.IsInfinity (bx) || double.IsNaN (by) ||
                        double.IsInfinity (by))
                    return false;
            }
            for (int r = 0; r < 3; r++) {
                inp.BAuthority [r * n + jx] = exactFreeX [r] * exactScale [r];
                inp.BAuthority [r * n + jy] = exactFreeY [r] * exactScale [r];
            }
            return true;
        }

        /// <summary>
        /// (born closed, reset to 0 at every arming): mode 0 = V1, one (x, y) applied to every
        /// pivot of a gimbal module in the pivot's own frame; mode 1 = a module whose pivots are
        /// not parallel (AR-1 Cluster) is mapped per pivot, as stock does (GimbalPivots.cs), in its
        /// columns, its lease and its lateral-force geometry; mode 2 (born closed) = mode 1,
        /// and the law's authority of a mapped module is its exact realized maximum over the box
        /// (GimbalPivots.ExactAuthority) instead of its free columns times the span. Modules with
        /// one pivot, or parallel pivots, keep the V1 path in every mode. Returns [accepted (1),
        /// version (1), mode, gimbal modules, modules with two pivots or more, modules mapped in
        /// mode 1 or 2 (pivots not parallel), most pivots in one module, attitude tick]; the
        /// counts are NaN while the vessel is not loaded.
        /// </summary>
        internal static IList<double> GimbalPivotsConfig (string token, int mode)
        {
            if (!GimbalPivots.ValidMode (mode))
                throw new ArgumentOutOfRangeException (nameof (mode), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode,
                    "0 V1 (meme deflexion locale sur chaque pivot), 1 par pivot (D42), " +
                    "2 par pivot et autorite exacte (D65)"));
            var entry = RequireToken (token);
            entry.PivotMode = mode;
            double modules = double.NaN, multi = double.NaN, mapped = double.NaN,
                most = double.NaN;
            var vessel = Resolve (entry);
            if (vessel != null) {
                modules = multi = mapped = most = 0.0;
                var scratch = new GimbalPivotCommand ();
                for (int p = 0; p < vessel.parts.Count; p++) {
                    var part = vessel.parts [p];
                    if (part == null)
                        continue;
                    for (int m = 0; m < part.Modules.Count; m++) {
                        var gimbal = part.Modules [m] as ModuleGimbal;
                        if (gimbal == null)
                            continue;
                        modules++;
                        int pivots = gimbal.gimbalTransforms == null ? 0 : gimbal.gimbalTransforms.Count;
                        if (pivots >= 2)
                            multi++;
                        if (pivots > most)
                            most = pivots;
                        if (AttitudeGeometry.RefreshPivots (gimbal, scratch))
                            mapped++;
                    }
                }
            }
            return new List<double> (PivotsReplyLength) {
                1.0, GimbalPivots.Version, entry.PivotMode, modules, multi, mapped, most, ownTick
            };
        }

        /// <summary>
        /// Stores the reference; an accepted reference also renews the real-time lease (a
        /// client that sends references is alive). Returns 1 accepted, 0 ignored (older or
        /// equal sequence).
        /// </summary>
        internal static int Reference (string token, int sequence, SCReferenceFrame frame,
            double dirX, double dirY, double dirZ, int rollMode,
            double omegaFfX, double omegaFfY, double omegaFfZ,
            double utStamp, double utValidUntil, double omegaMax)
        {
            // V1: one scalar limit (the client's max of pitch and yaw) for every axis, and
            // no write permission: a V1 client never makes an armed loop write.
            return Accept (token, sequence, frame, dirX, dirY, dirZ, rollMode, omegaFfX,
                omegaFfY, omegaFfZ, utStamp, utValidUntil, omegaMax, omegaMax, omegaMax,
                omegaMax, 0);
        }

        /// <summary>
        /// V2 (M3): the AutoPilot's limit per body axis (pitch x, roll y, yaw z; 0 = its
        /// default), for the per-axis omega guard, and the write permission of this
        /// reference (bit 1 roll, bit 2 pitch/yaw - M4): the client grants the roll only while
        /// the AutoPilot damps the roll rate (target_roll NaN), pitch/yaw only under the
        /// descent. The law's own reference keeps the scalar max(|x|, |z|) of V1, so the
        /// computed law is the one of M2. Returns 0 ignored, else 1 + 2 if the loop wrote
        /// pitch/yaw at the last physics step + 4 if it wrote the roll (D3 reads it).
        /// </summary>
        internal static int ReferenceV2 (string token, int sequence, SCReferenceFrame frame,
            double dirX, double dirY, double dirZ, int rollMode,
            double omegaFfX, double omegaFfY, double omegaFfZ,
            double utStamp, double utValidUntil, double omegaMaxX, double omegaMaxY,
            double omegaMaxZ, int writeMask)
        {
            if (!Finite (omegaMaxX) || !Finite (omegaMaxY) || !Finite (omegaMaxZ) ||
                    omegaMaxX < 0.0 || omegaMaxY < 0.0 || omegaMaxZ < 0.0)
                throw new ArgumentOutOfRangeException ("omegaMax", "limites par axe finies et >= 0 attendues");
            // bits 4 (surfaces out of the law) and 8 (pitch/yaw holds its reference) only under
            // option 16 of the token; without it, exactly the refusal of before (same message).
            if (writeMask < 0 || (writeMask > AttitudeWrite.WriteMaskBase &&
                    !AttitudeWrite.ValidWriteMask (writeMask, OptionsOf (token))))
                throw new ArgumentOutOfRangeException (nameof (writeMask), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode, "bits : 1 roulis, 2 tangage/lacet"));
            double scalar = Math.Max (omegaMaxX, omegaMaxZ);
            int accepted = Accept (token, sequence, frame, dirX, dirY, dirZ, rollMode, omegaFfX,
                omegaFfY, omegaFfZ, utStamp, utValidUntil, scalar, omegaMaxX, omegaMaxY,
                omegaMaxZ, writeMask);
            if (accepted == 0)
                return 0;
            var entry = RequireToken (token);
            return 1 + (entry.LastWriteStatePitchYaw == AttitudeWrite.WriteWriting ? 2 : 0) +
                (entry.LastWriteStateRoll == AttitudeWrite.WriteWriting ? 4 : 0);
        }

        static int Accept (string token, int sequence, SCReferenceFrame frame,
            double dirX, double dirY, double dirZ, int rollMode,
            double omegaFfX, double omegaFfY, double omegaFfZ,
            double utStamp, double utValidUntil, double omegaMax, double omegaMaxX,
            double omegaMaxY, double omegaMaxZ, int writeMask)
        {
            var entry = RequireToken (token);
            if (frame == null)
                throw new ArgumentNullException (nameof (frame));
            if (!Finite (dirX) || !Finite (dirY) || !Finite (dirZ) ||
                    dirX * dirX + dirY * dirY + dirZ * dirZ < 1e-18)
                throw new ArgumentOutOfRangeException ("direction", "direction nulle ou non finie");
            if (rollMode != 0)
                throw new ArgumentOutOfRangeException (nameof (rollMode), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode, "seul le roulis en amortissement de taux (0) existe"));
            if (!Finite (omegaFfX) || !Finite (omegaFfY) || !Finite (omegaFfZ) ||
                    !Finite (utStamp) || !Finite (utValidUntil) || !Finite (omegaMax) ||
                    utValidUntil < utStamp)
                throw new ArgumentOutOfRangeException ("reference", "valeurs non finies ou validite anterieure");
            if (sequence <= entry.Sequence)
                return 0;
            entry.Sequence = sequence;
            entry.Frame = frame;
            entry.DirX = dirX;
            entry.DirY = dirY;
            entry.DirZ = dirZ;
            entry.RollMode = rollMode;
            entry.FfX = omegaFfX;
            entry.FfY = omegaFfY;
            entry.FfZ = omegaFfZ;
            entry.UtStamp = utStamp;
            entry.UtValidUntil = utValidUntil;
            entry.OmegaMax = omegaMax;
            entry.OmegaMaxX = omegaMaxX;
            entry.OmegaMaxY = omegaMaxY;
            entry.OmegaMaxZ = omegaMaxZ;
            entry.WriteMask = writeMask;
            // a V1/V2 reference carries no roll target (ReferenceV3 sets it after this).
            entry.RollTargetOn = false;
            entry.HaveReference = true;
            entry.AcceptedCount++;
            entry.Arming.RenewReal (Time.realtimeSinceStartup, entry.LeaseSeconds);
            return 1;
        }

        /// <summary>
        /// (option 1024): the V2 reference plus a roll mode and an up vector. rollMode 0 is exactly V2 (the up
        /// vector ignored, the roll rate damping); rollMode 1, refused without option 1024, keeps the up vector (finite,
        /// with a part across any nose) as the roll target: the roof (-z) toward it. Returns as V2.
        /// </summary>
        internal static int ReferenceV3 (string token, int sequence, SCReferenceFrame frame,
            double dirX, double dirY, double dirZ, int rollMode, double upX, double upY, double upZ,
            double omegaFfX, double omegaFfY, double omegaFfZ,
            double utStamp, double utValidUntil, double omegaMaxX, double omegaMaxY,
            double omegaMaxZ, int writeMask)
        {
            if (rollMode != 0 && rollMode != 1 && rollMode != 2)
                throw new ArgumentOutOfRangeException (nameof (rollMode), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode, "0 roulis amorti, 1 roulis pointe, 2 roulis pointe en ombre (D252 K6)"));
            if (rollMode != 0) {
                if ((OptionsOf (token) & AttitudeWrite.OptionRollTarget) == 0)
                    throw new ArgumentOutOfRangeException (nameof (rollMode), BridgeErrors.Tag (
                        BridgeErrors.UnsupportedMode, "roulis pointe : option 1024 d'attitude_options_v1 requise (D252 K6)"));
                if (!Finite (upX) || !Finite (upY) || !Finite (upZ) || upX * upX + upY * upY + upZ * upZ < 1e-18)
                    throw new ArgumentOutOfRangeException ("up", "vecteur haut nul ou non fini");
            }
            int r = ReferenceV2 (token, sequence, frame, dirX, dirY, dirZ, 0, omegaFfX, omegaFfY, omegaFfZ, utStamp,
                utValidUntil, omegaMaxX, omegaMaxY, omegaMaxZ, writeMask);
            if (r == 0 || rollMode == 0)
                return r;
            var entry = RequireToken (token);
            entry.RollTargetOn = true;
            entry.RollTargetShadow = rollMode == 2;
            entry.UpX = upX;
            entry.UpY = upY;
            entry.UpZ = upZ;
            return r;
        }

        internal static IList<double> ReadStatus ()
        {
            float now = Time.realtimeSinceStartup;
            var output = new List<double> (StatusHeaderStride + entries.Count * StatusRowStride) {
                ProtocolVersion,
                StatusSchemaVersion,
                entries.Count,
                StatusRowStride,
                registered ? 1.0 : 0.0,
                earlyishCalls,
                pumpFallbackTicks,
                ownTick,
                Planetarium.GetUniversalTime ()
            };
            double utNow = Planetarium.GetUniversalTime ();
            foreach (var pair in entries) {
                var e = pair.Value;
                output.Add (e.PersistentId);
                output.Add (e.Mode);
                output.Add (e.AxesMask);
                output.Add (e.Arming.RemainingReal (now));
                output.Add (e.Loaded ? 1.0 : 0.0);
                output.Add (e.LastMotif);
                output.Add (e.Sequence);
                output.Add (e.ConsumedSequence);
                output.Add (e.HaveReference ? utNow - e.UtStamp : double.NaN);
                output.Add (e.LatestTick);
                output.Add (e.History.Count);
                output.Add (e.Computed);
                output.Add (e.Exceptions);
                output.Add (e.CostMicrosLast);
                output.Add (e.CostMicrosMax);
                output.Add (e.AcceptedCount);
                // Schema 2 (M3).
                output.Add (e.WriteMask);
                output.Add (e.Mode == ModeArmed &&
                    AttitudeWrite.OrderVerified (e.ChainOwnIndex, e.ChainKrpcIndex, e.ChainLength)
                    ? 1.0 : 0.0);
                output.Add (e.Writes);
                output.Add (e.FbwCalls);
                output.Add (e.ChainReorders);
                output.Add (e.FallbackSteps);
                // Schema 3 (M4).
                output.Add (e.PitchYawWrites);
                output.Add (e.AuthorityToken != null &&
                    Actuation.HoldsForAttitude (e.AuthorityToken) ? 1.0 : 0.0);
                output.Add (e.PitchYawFallbackSteps);
            }
            return output;
        }

        internal static IList<double> ReadFrames (string persistentId, int sinceTick, int maxFrames)
        {
            uint pid = VesselLookup.ParsePersistentId (persistentId);
            Entry entry;
            if (!entries.TryGetValue (pid, out entry))
                return emptyRing.ReadSince (ProtocolVersion, SchemaVersion, sinceTick, maxFrames);
            return entry.History.ReadSince (ProtocolVersion, SchemaVersion, sinceTick, maxFrames);
        }

        /// <summary>Leaving the flight scene: every armed vessel is gone.</summary>
        internal static void ResetForScene ()
        {
            foreach (var pair in entries) {
                pair.Value.Dropped = true;
                droppedTokens.Remember (pair.Value.Token, BridgeErrors.TokenInvalid);
                Unhook (pair.Value);
                ReleaseAuthority (pair.Value);
                RestoreSurfaces (pair.Value);
            }
            entries.Clear ();
            Unregister ();
            AttitudeProbe.Stop ();
            // no zero outlives the flight scene.
            RemoveAllThrottleReturns ();
            lastEarlyishSeenByPump = -1;
            pumpCallsWithoutEarlyish = 0;
        }

        /// <summary>
        /// Called from Actuation.Pump (watcher FixedUpdate). Witness of the Earlyish
        /// hook: when it has not ticked for PumpFallbackAfter pumps, the Pump runs the
        /// attitude tick itself and every frame says so (stage 2, motif EarlyishDead).
        /// </summary>
        internal static void PumpWatch ()
        {
            AttitudeProbe.Stamp (AttitudeProbe.StagePump);
            if (entries.Count == 0)
                return;
            if (earlyishCalls != lastEarlyishSeenByPump) {
                lastEarlyishSeenByPump = earlyishCalls;
                pumpCallsWithoutEarlyish = 0;
                return;
            }
            pumpCallsWithoutEarlyish++;
            if (pumpCallsWithoutEarlyish >= PumpFallbackAfter) {
                pumpFallbackTicks++;
                TickAll (StagePumpFallback);
            }
        }

        // ---------------------------------------------------------------- registration

        static void Register ()
        {
            if (registered)
                return;
            TimingManager.FixedUpdateAdd (TimingManager.TimingStage.Earlyish, earlyishAction);
            registered = true;
            lastEarlyishSeenByPump = earlyishCalls;
            pumpCallsWithoutEarlyish = 0;
        }

        static void Unregister ()
        {
            if (!registered)
                return;
            try {
                TimingManager.FixedUpdateRemove (TimingManager.TimingStage.Earlyish, earlyishAction);
            } catch (Exception exc) {
                BridgeLog.Error ("AttitudeV1: FixedUpdateRemove: " + exc.Message);
            }
            registered = false;
            UnregisterLate ();
        }

        // Refactor R1: why the last tokens stopped being valid (TOKEN_EXPIRED or TOKEN_INVALID).
        static readonly TokenMemory droppedTokens = new TokenMemory (16);

        static void Drop (uint persistentId, string why = BridgeErrors.TokenInvalid)
        {
            Entry entry;
            if (entries.TryGetValue (persistentId, out entry)) {
                droppedTokens.Remember (entry.Token, why);
                entry.Dropped = true;
                // (option 512): a lease that expires while the loop wrote the main throttle gives it back at
                // zero, before the link goes.
                if (why == BridgeErrors.TokenExpired && ThrottleLive (entry))
                    InstallThrottleReturn (entry, entry.HookedVessel ?? entry.Vessel);
                Unhook (entry);
                ReleaseAuthority (entry);
                RestoreSurfaces (entry);
            }
            entries.Remove (persistentId);
            if (entries.Count == 0)
                Unregister ();
        }

        static void OnEarlyish ()
        {
            // Timing2 wraps the whole multicast in one try/catch: an exception here would
            // skip every later delegate of the stage. Nothing may escape.
            try {
                earlyishCalls++;
                TickAll (StageEarlyish);
            } catch (Exception exc) {
                BridgeLog.Error ("AttitudeV1 Earlyish: " + exc);
            }
        }

        static readonly List<uint> expiredScratch = new List<uint> ();

        static void TickAll (int stage)
        {
            if (entries.Count == 0)
                return;
            attitudeTicks.Increment ();
            // Core v2: one window of the vessel registry (each armed vessel's fingerprint is
            // walked once in this tick, then served from the memo: no game code runs between).
            int window = VesselRegistry.OpenWindow ();
            try {
                float now = Time.realtimeSinceStartup;
                expiredScratch.Clear ();
                foreach (var pair in entries) {
                    if (pair.Value.Arming.ExpiredReal (now)) {
                        expiredScratch.Add (pair.Key);
                        continue;
                    }
                    TickEntry (pair.Value, stage);
                }
                for (int i = 0; i < expiredScratch.Count; i++) {
                    BridgeLog.Info ("AttitudeV1: bail expire, vaisseau " + expiredScratch [i] + " desarme");
                    Drop (expiredScratch [i], BridgeErrors.TokenExpired);
                }
            } finally {
                VesselRegistry.CloseWindow (window);
            }
        }

        // ------------------------------------------------------------------ one tick

        static void TickEntry (Entry e, int stage)
        {
            long t0 = Stopwatch.GetTimestamp ();
            long motif = stage == StagePumpFallback ? MotifEarlyishDead : 0L;
            long caps = 0;
            // Refactor lot 1 (born closed: false, 0 and nothing restored without options).
            // Refactor R1: the effector registry implies the family mechanics of option 1;
            // without it, rules == e.Options and nothing below changes.
            bool registry = e.EffectorRegistry;
            int rules = RuleOptions (e);
            bool families = (rules & AttitudeWrite.OptionFamilyBlocking) != 0;
            caps |= OptionCaps (e);
            if (registry) {
                caps |= CapFamilyBlocking | CapEffectorRegistry;
                e.Effectors.Begin (ownTick);
            }
            // a hold BetterLateThanNever did not put back is put back first; and the
            // ctrlState axes the last fly-by-wire wrote (published, then cleared, below).
            RestoreSurfaces (e);
            int ctrlLastAxes = !families ? 0 : (!double.IsNaN (e.FbwWrittenPitch) ? 1 : 0) |
                (!double.IsNaN (e.FbwWrittenRoll) ? 2 : 0) | (!double.IsNaN (e.FbwWrittenYaw) ? 4 : 0);
            // the axes our own fly-by-wire link wrote at the previous physics step (its
            // callback ran then: FbwTick is that step's attitude tick), for the engagement of the
            // control surfaces below, which must precede the law and so this step's own decision.
            int lastWrittenAxes = AttitudeWrite.WrittenAxes (e.FbwTick == ownTick - 1, e.FbwWrittenPitch,
                e.FbwWrittenRoll, e.FbwWrittenYaw);
            // M3/M4: a decision of the previous step that the fly-by-wire callback never
            // consumed (hook lost, vessel not fed): dropped, and said.
            if (e.WriteRoll || e.WritePitchYaw) {
                e.WriteRoll = false;
                e.WritePitchYaw = false;
                motif |= MotifFlyByWireMissed;
            }
            if (e.T.WriteThrottle) {
                // Lateral force: throttles decided but never written. Back to the main
                // throttle at once rather than held at a stale offset.
                e.T.WriteThrottle = false;
                e.T.MissedThrottle = true;
                ReleaseDiffThrottles (e);
            }
            // Computed levers: an X/Y write decided but never done (same step as the
            // throttles, same reason bit) is dropped.
            e.T.WriteRcs = false;
            // (option 512): a main throttle decided but never written is dropped, and counted.
            if (e.Th.Write) {
                e.Th.Write = false;
                e.Th.Missed++;
            }
            // (option 8192): an allocation write decided but never written is dropped, its throttles back to
            // the main throttle at once (as the layer's).
            if (e.Al.Write) {
                e.Al.Write = false;
                e.Al.Missed++;
                AllocationReleaseThrottles (e);
            }
            e.LastWriteStateRoll = AttitudeWrite.WriteShadow;
            e.LastWriteStatePitchYaw = AttitudeWrite.WriteShadow;
            try {
                var vessel = Resolve (e);
                if (vessel == null) {
                    e.Loaded = false;
                    e.HavePrevious = false;
                    e.Law.Reset ();
                    ReleaseAuthority (e);
                    e.LastMotif = motif | MotifUnloaded |
                        (e.Mode == ModeArmed ? MotifWriteFallback : 0L);
                    return;
                }
                e.Loaded = true;
                var reference = vessel.ReferenceTransform;
                if (reference == null) {
                    e.HavePrevious = false;
                    e.Law.Reset ();
                    ReleaseAuthority (e);
                    e.LastMotif = motif | MotifUnloaded |
                        (e.Mode == ModeArmed ? MotifWriteFallback : 0L);
                    return;
                }

                // Topology: the actuator vector and every cached module belong to one layout.
                ulong hash = Actuation.ComputeTopologyHash (vessel);
                if (e.VesselId != vessel.id || e.TopologyHash != hash) {
                    Rebuild (e, vessel, hash);
                    motif |= MotifTopologyChanged;
                }

                double dt = TimeWarp.fixedDeltaTime;
                bool warp = TimeWarp.CurrentRate != 1f || Math.Abs (dt - 0.02) > 1e-5;
                if (warp)
                    motif |= MotifWarp;

                // Lot 3, 3.1 (born closed): SurfaceModelV1's records, the Mach the surfaces will
                // read, and the surfaces engaged in this step (from the last capture). Only on
                // axes the loop owns -- written by its own link at the previous step and still
                // permitted now -- and the law's M changing definition without a step.
                bool surfaceModel = e.Sm.Mode != SurfaceModel.ModeOff;
                // (born closed: option 16): the client's write mask may hold the surfaces out of the
                // law for this reference -- none of the loop's axes is owned for their engagement.
                bool surfacesHeldOut = e.Mode == ModeArmed && (AttitudeWrite.EffectiveWriteMask (e.WriteMask,
                    e.Options) & AttitudeWrite.WriteSurfacesOut) != 0;
                if (surfacesHeldOut)
                    motif |= AttitudeWrite.MotifSurfacesHeldOut;
                if (surfaceModel) {
                    SurfacesBegin (e, stage, AttitudeWrite.SurfaceOwnedAxes (e.Mode, e.AxesMask, e.WriteMask,
                        e.Options, stage, lastWrittenAxes));
                    SurfacesShift (e);
                    caps |= CapSurfaceModel;
                }

                var inp = e.Input;
                int nGimbalColumns = 2 * e.Gimbals.Count;
                inp.Dt = dt;

                // ------------------------------------------------ rotational state
                Vector3 omega = vessel.angularVelocity;
                Vector3 moi = vessel.MOI;
                inp.Omega [0] = omega.x;
                inp.Omega [1] = omega.y;
                inp.Omega [2] = omega.z;
                inp.Inertia [0] = moi.x * Frames.TonneToKilogram;
                inp.Inertia [1] = moi.y * Frames.TonneToKilogram;
                inp.Inertia [2] = moi.z * Frames.TonneToKilogram;
                bool consecutive = e.HavePrevious && ownTick == e.PreviousTick + 1 && !warp &&
                    (motif & MotifTopologyChanged) == 0;
                if (consecutive) {
                    inp.OmegaDotMeasured [0] = (omega.x - e.PreviousOmega.x) / dt;
                    inp.OmegaDotMeasured [1] = (omega.y - e.PreviousOmega.y) / dt;
                    inp.OmegaDotMeasured [2] = (omega.z - e.PreviousOmega.z) / dt;
                } else {
                    inp.OmegaDotMeasured [0] = 0.0;
                    inp.OmegaDotMeasured [1] = 0.0;
                    inp.OmegaDotMeasured [2] = 0.0;
                    motif |= MotifTickGap;
                }
                inp.OmegaDotValid = consecutive;
                e.HavePrevious = true;
                e.PreviousTick = ownTick;
                e.PreviousOmega = omega;

                // ------------------------------------------------ realized torques
                Vector3 com = vessel.CurrentCoM;
                var engineWorld = new Vector3 (0f, 0f, 0f);
                for (int i = 0; i < e.Engines.Count; i++) {
                    var engine = e.Engines [i];
                    if (engine != null)
                        engineWorld = AttitudeGeometry.Add (engineWorld,
                            AttitudeGeometry.EngineTorqueWorld (engine, com));
                }
                caps |= CapEngineTorque;
                var rcsWorld = new Vector3 (0f, 0f, 0f);
                for (int i = 0; i < e.Rcs.Count; i++) {
                    var rcs = e.Rcs [i];
                    if (rcs != null)
                        rcsWorld = AttitudeGeometry.Add (rcsWorld,
                            AttitudeGeometry.RcsTorqueWorld (rcs, com));
                }
                caps |= CapRcsTorque;
                Vector3 engineBody = reference.InverseTransformDirection (engineWorld);
                Vector3 rcsBody = reference.InverseTransformDirection (rcsWorld);

                // ctrlState channel columns (RCS + wheels + surfaces), plus wheel torque.
                for (int k = 0; k < 9; k++) {
                    e.BPos [k] = 0.0;
                    e.BNeg [k] = 0.0;
                }
                var wheelBody = e.Scratch;
                wheelBody [0] = 0.0;
                wheelBody [1] = 0.0;
                wheelBody [2] = 0.0;
                bool precision = FlightInputHandler.fetch != null &&
                    FlightInputHandler.fetch.precisionMode;
                for (int i = 0; i < e.Rcs.Count; i++) {
                    var rcs = e.Rcs [i];
                    if (rcs == null)
                        continue;
                    if (registry) {
                        // R1: the module's columns, scaled when its status is exact-scaled.
                        if (!AddRcsRegistry (e, rcs, com, reference, precision))
                            motif |= MotifRcsModelApprox;
                    } else if (!AttitudeGeometry.AddRcsColumns (rcs, com, reference, precision, e.BPos, e.BNeg))
                        motif |= MotifRcsModelApprox;
                    caps |= CapRcsColumns;
                }
                for (int i = 0; i < e.Wheels.Count; i++) {
                    var wheel = e.Wheels [i];
                    if (wheel != null && AttitudeGeometry.AddWheel (wheel, wheelBody,
                            registry ? e.Effectors.WheelPos : e.BPos, registry ? e.Effectors.WheelNeg : e.BNeg)) {
                        caps |= CapWheelTorque | CapWheelColumns;
                        if (registry)
                            e.Effectors.Family (EffectorRegistry.FamilyWheel, EffectorRegistry.StatusExact, 1);
                    }
                }
                if (registry) {
                    // R1: the ctrlState columns are the sum of the families' sub-columns.
                    for (int k = 0; k < 9; k++) {
                        e.BPos [k] = e.Effectors.RcsPos [k] + e.Effectors.WheelPos [k];
                        e.BNeg [k] = e.Effectors.RcsNeg [k] + e.Effectors.WheelNeg [k];
                    }
                }
                if (families) {
                    // the surfaces leave B (unmodelled); motif 14 stays, informational.
                    motif |= ClassifySurfaces (e);
                } else {
                    for (int i = 0; i < e.Surfaces.Count; i++) {
                        var surface = e.Surfaces [i];
                        if (surface != null && AttitudeGeometry.AddSurface (surface, e.BPos, e.BNeg)) {
                            motif |= MotifSurfaceApprox;
                            caps |= CapSurfaceColumns;
                        }
                    }
                }
                // (born closed: option 16384): the nozzles of every RCS module as ModuleRCS fires them, and the
                // rest of the ctrlState columns. Without the option nothing is read.
                bool rcsCombined = false;
                if (RcsCombinedOn (e)) {
                    rcsCombined = RcsCombinedPrepare (e, com, reference, precision);
                    if (rcsCombined)
                        caps |= CapRcsCombined;
                } else {
                    e.Rc.Modelled = false;
                }
                inp.Torque [0] = engineBody.x + rcsBody.x + wheelBody [0];
                inp.Torque [1] = engineBody.y + rcsBody.y + wheelBody [1];
                inp.Torque [2] = engineBody.z + rcsBody.z + wheelBody [2];
                if (surfaceModel) {
                    // (born closed): the non-surface torques, before the engaged surfaces join M.
                    SurfacesKappaNonFin (e, inp.Torque, engineBody);
                    // (born closed): the inertia Vessel.MOI leaves out, for kappa's measured side.
                    SurfacesKappaInertia (e, vessel, reference, com);
                    SurfacesTorque (e, inp);
                }

                // (born closed: option 2048): the reference ring's value at this step's UT, read by the
                // lateral force, the reference and the main throttle below.
                caps |= RingStep (e);

                // Lateral force: two motif columns after the ctrlState ones, only when the
                // client asked for them (born closed: n = 2 x gimbals + 3 otherwise).
                bool planned = PlanTranslation (e, vessel, reference, com, vessel.ctrlState,
                    stage, warp);
                int n = nGimbalColumns + 3 + (planned ? 2 : 0);
                inp.Resize (n);

                // (born closed: option 128): each gimbal's engines at the thrust of its horizon, and
                // the torque that spool adds (AttitudeV1GimbalTrim.cs). Without it: finalThrust, as before.
                bool gimbalTrim = (e.Options & AttitudeWrite.OptionGimbalTrim) != 0;
                if (gimbalTrim)
                    caps |= GimbalTrimPrepare (e, com, reference, dt);

                // ------------------------------------------------------- B columns
                for (int g = 0; g < e.Gimbals.Count; g++) {
                    var gr = e.Gimbals [g];
                    var gimbal = gr.Gimbal;
                    int jx = 2 * g, jy = 2 * g + 1;
                    bool leased = gimbal != null && Actuation.GimbalIsLeased (gimbal);
                    bool usable = gimbal != null && gimbal.moduleIsEnabled &&
                        (leased || (gimbal.gimbalActive && !gimbal.gimbalLock));
                    float dx = gimbal == null || (!leased && gimbal.gimbalLock) ? 0f : gimbal.actuationLocal.x;
                    float dy = gimbal == null || (!leased && gimbal.gimbalLock) ? 0f : gimbal.actuationLocal.y;
                    // (born closed): a module with several non-parallel pivots, mapped per
                    // pivot; its deflection is the module's, in the frame of pivot 0.
                    gr.PivotsMapped = GimbalPivots.Mapped (e.PivotMode) && gimbal != null &&
                        AttitudeGeometry.RefreshPivots (gimbal, gr.Pivots);
                    if (gr.PivotsMapped) {
                        if (!leased && !gimbal.gimbalLock)
                            AttitudeGeometry.PivotZeroDeflection (gimbal, gr.Pivots, out dx, out dy);
                        gr.Pivots.StartX = dx;
                        gr.Pivots.StartY = dy;
                        caps |= CapGimbalPivots;
                    }
                    Vector3 colX = new Vector3 (0f, 0f, 0f), colY = new Vector3 (0f, 0f, 0f);
                    double[] spool = gimbalTrim ? gr.SpoolThrust : null;
                    if (gimbal != null && gr.PivotsMapped)
                        AttitudeGeometry.GimbalColumnsPivots (gimbal, gr.PartEngines, com,
                            reference, gr.Pivots, dx, dy, spool, out colX, out colY, out gr.FreeX,
                            out gr.FreeY);
                    else if (gimbal != null)
                        AttitudeGeometry.GimbalColumns (gimbal, gr.PartEngines, com, reference,
                            dx, dy, spool, out colX, out colY);
                    SetColumn (inp, jx, colX);
                    SetColumn (inp, jy, colY);
                    inp.U0 [jx] = dx;
                    inp.U0 [jy] = dy;
                    float limiter = gimbal == null ? 0f
                        : Math.Min (100f, Math.Max (0f, gimbal.gimbalLimiter)) * 0.01f;
                    if (usable) {
                        inp.UMin [jx] = -Math.Abs (gimbal.gimbalRangeXN) * limiter;
                        inp.UMax [jx] = Math.Abs (gimbal.gimbalRangeXP) * limiter;
                        inp.UMin [jy] = -Math.Abs (gimbal.gimbalRangeYN) * limiter;
                        inp.UMax [jy] = Math.Abs (gimbal.gimbalRangeYP) * limiter;
                    } else {
                        inp.UMin [jx] = dx;
                        inp.UMax [jx] = dx;
                        inp.UMin [jy] = dy;
                        inp.UMax [jy] = dy;
                    }
                    inp.UPref [jx] = 0.0;
                    inp.UPref [jy] = 0.0;
                    inp.Priority [jx] = 1.0;
                    inp.Priority [jy] = 1.0;
                    if (planned)
                        RecordGimbalGeometry (e, g, gimbal, gr.PartEngines, reference, dx,
                            usable, dt, gr.PivotsMapped ? gr.Pivots : null, dy);
                }
                if (e.Gimbals.Count > 0)
                    caps |= CapGimbalColumns;
                var ctrl = vessel.ctrlState;
                double[] served = e.Served;
                served [0] = ctrl == null ? 0.0 : ctrl.pitch;
                served [1] = ctrl == null ? 0.0 : ctrl.roll;
                served [2] = ctrl == null ? 0.0 : ctrl.yaw;
                // (born closed: option 32): the RCS torque in flight -- the served command's, which the
                // RCS fires one render frame later, minus that of the command the measured torque came from.
                if ((e.Options & AttitudeWrite.OptionRcsInFlight) != 0)
                    RcsInFlightStep (e, com, reference, precision, served, hash, stage);
                else
                    e.RcsFlight.Reset ();
                for (int a = 0; a < 3; a++) {
                    int j = nGimbalColumns + a;
                    // The QP sees the mean of the two half-columns; both are published.
                    for (int r = 0; r < 3; r++)
                        inp.B [r * n + j] = 0.5 * (e.BPos [3 * a + r] + e.BNeg [3 * a + r]);
                    inp.U0 [j] = served [a];
                    inp.UMin [j] = -1.0;
                    inp.UMax [j] = 1.0;
                    inp.UPref [j] = 0.0;
                    inp.Priority [j] = 3.0;
                }
                // Lot 3, 3.1: the engaged surfaces' sub-columns and their free torque (armed only).
                if (surfaceModel)
                    SurfacesColumns (e, inp, n, nGimbalColumns, served, dt);
                else
                    inp.UseTorqueFree = false;
                // (born closed: option 32): what the RCS will fire anyway is not asked again.
                if ((e.Options & AttitudeWrite.OptionRcsInFlight) != 0 && RcsInFlightApply (e, inp))
                    caps |= CapRcsInFlight;
                // (born closed: option 128): the spool's torque is not asked again, and a gimbal
                // column without effect is regularized for real toward neutral (the law's TrimNullColumns).
                if (gimbalTrim)
                    GimbalTrimApply (e, inp);
                inp.TrimNullColumns = gimbalTrim ? nGimbalColumns : 0;
                // (born closed: option 4096): the shadow allocation's snapshot, before the lateral-force layer
                // shifts the gimbal bounds. Reads only; its own failure never reaches the loop.
                bool allocation = AttitudeWrite.AllocationOn (e.Options);
                if (allocation) {
                    try {
                        AllocationPrepare (e, vessel, reference, com, inp, n, nGimbalColumns, dt, served, precision,
                            !surfaceModel && (caps & (CapWheelColumns | CapSurfaceColumns)) == 0);
                    } catch (Exception exc) {
                        AllocationFailed (e, exc);
                    }
                }
                if (planned) {
                    // Honest translation (bits 16/32) reads the step's dt and the served rotation;
                    // nothing else does.
                    e.T.StepDt = dt;
                    e.T.Served [0] = served [0];
                    e.T.Served [1] = served [1];
                    e.T.Served [2] = served [2];
                    ApplyTranslationColumns (e, vessel, reference, com, inp, n, nGimbalColumns);
                    // (born closed: option 256): the free authority was computed this step.
                    if (e.T.FreeComputed)
                        caps |= CapTranslationFree;
                }
                // (born closed): a doubtful family freezes its ctrlState columns BEFORE the
                // law -- no authority (B = 0), bounds at the served u0, not written: the leased
                // gimbals are allocated alone. The fly-by-wire order (motif 20) is read here, armed.
                long hookMotif = 0;
                bool hooked = false;
                e.FrozenAxes = 0;
                if (families) {
                    long pre = motif;
                    if (e.Mode == ModeArmed) {
                        hookMotif = EnsureHook (e, vessel);
                        hooked = true;
                        if (!AttitudeWrite.OrderVerified (e.ChainOwnIndex, e.ChainKrpcIndex, e.ChainLength))
                            pre |= MotifFlyByWireOrder;
                    }
                    e.FrozenAxes = registry
                        ? EffectorRegistry.FrozenAxes ((pre & MotifFlyByWireOrder) != 0,
                            e.Effectors.RcsUnmodeledAxes, e.SurfaceAxesUnneutralizable)
                        : AttitudeWrite.FrozenCtrlAxes (e.Options, pre, e.SurfaceAxesUnneutralizable);
                    for (int a = 0; a < 3; a++) {
                        if ((e.FrozenAxes & (1 << a)) == 0)
                            continue;
                        int j = nGimbalColumns + a;
                        for (int r = 0; r < 3; r++)
                            inp.B [r * n + j] = 0.0;
                        inp.UMin [j] = inp.U0 [j];
                        inp.UMax [j] = inp.U0 [j];
                    }
                    motif |= AttitudeWrite.FrozenMotifs (e.FrozenAxes);
                }
                // R1: statuses, actions and computed authority of every column (published).
                if (registry)
                    FillEffectorTable (e, inp, n, nGimbalColumns, planned);
                // the authority of a mapped module = its unclamped columns (false, and alpha
                // from B, unless a module is mapped this step: born closed).
                inp.UseBAuthority = false;
                for (int g = 0; g < e.Gimbals.Count; g++) {
                    var gr = e.Gimbals [g];
                    if (!gr.PivotsMapped || gr.Gimbal == null)
                        continue;
                    if (!inp.UseBAuthority) {
                        Array.Copy (inp.B, inp.BAuthority, 3 * n);
                        inp.UseBAuthority = true;
                    }
                    SetColumn (inp.BAuthority, n, 2 * g, gr.FreeX);
                    SetColumn (inp.BAuthority, n, 2 * g + 1, gr.FreeY);
                    // (mode 2, born closed): the free columns scaled per axis so that
                    // sum |B| span gives the module's EXACT largest realized torque over the
                    // law's span (+16 % on the heavy booster's engines with the free columns).
                    if (e.PivotMode == GimbalPivots.ModeExactAuthority && gr.Pivots.TorqueColsValid &&
                            ExactAuthorityColumns (inp, n, g, gr))
                        caps |= CapGimbalExactAuthority;
                }

                // --------------------------------------------------------- reference
                inp.ReferenceValid = false;
                double ut = Planetarium.GetUniversalTime ();
                e.RefBody [0] = double.NaN;
                e.RefBody [1] = double.NaN;
                e.RefBody [2] = double.NaN;
                e.FfBody [0] = 0.0;
                e.FfBody [1] = 0.0;
                e.FfBody [2] = 0.0;
                if (!e.HaveReference) {
                    motif |= MotifNoReference;
                } else if (ut > e.UtValidUntil) {
                    motif |= MotifReferenceStale;
                } else {
                    try {
                        // (option 2048): the ring's direction of this step when it serves one, never with
                        // the standing reference's omega_ff; else the standing reference, as before.
                        var dFrame = e.Frame;
                        double dX = e.DirX, dY = e.DirY, dZ = e.DirZ;
                        bool ringDir = RingDirection (e, ref dFrame, ref dX, ref dY, ref dZ);
                        Vector3d dWorld = dFrame.DirectionToWorldSpace (new Vector3d (dX, dY, dZ));
                        Vector3 dBody = reference.InverseTransformDirection (
                            new Vector3 ((float)dWorld.x, (float)dWorld.y, (float)dWorld.z));
                        Vector3d ffWorld = e.Frame.DirectionToWorldSpace (new Vector3d (e.FfX, e.FfY, e.FfZ));
                        Vector3 ffBody = reference.InverseTransformDirection (
                            new Vector3 ((float)ffWorld.x, (float)ffWorld.y, (float)ffWorld.z));
                        if (ringDir)
                            ffBody = new Vector3 (0f, 0f, 0f);
                        e.RefBody [0] = dBody.x;
                        e.RefBody [1] = dBody.y;
                        e.RefBody [2] = dBody.z;
                        e.FfBody [0] = ffBody.x;
                        e.FfBody [1] = ffBody.y;
                        e.FfBody [2] = ffBody.z;
                        if (AttitudeMath.PointingError (dBody.x, dBody.y, dBody.z, inp.Error)) {
                            inp.ReferenceValid = true;
                            caps |= CapReference;
                        }
                    } catch {
                        motif |= MotifFrameConversion;
                    }
                }
                // (born closed: option 1024 and a roll mode 1 reference): the roll error into the law. Roll
                // mode 2 (shadow): the same error into Error[1] for the frame only, RollTargetValid left false: the
                // law ignores Error[1] then (AttitudeMath, RollTargetTests' born-closed draws), bit for bit mode 0.
                inp.RollTargetValid = false;
                if (inp.ReferenceValid && e.RollTargetOn && (e.Options & AttitudeWrite.OptionRollTarget) != 0 &&
                        RollTargetStep (e, reference, inp)) {
                    caps |= CapRollTarget;
                    if (e.RollTargetShadow)
                        inp.RollTargetValid = false;
                }
                inp.OmegaFf [0] = e.FfBody [0];
                inp.OmegaFf [1] = e.FfBody [1];
                inp.OmegaFf [2] = e.FfBody [2];
                inp.OmegaDotFf [0] = 0.0;
                inp.OmegaDotFf [1] = 0.0;
                inp.OmegaDotFf [2] = 0.0;
                inp.OmegaMax = e.HaveReference ? e.OmegaMax : 0.0;
                inp.OmegaMaxAxis [0] = e.HaveReference ? e.OmegaMaxX : 0.0;
                inp.OmegaMaxAxis [1] = e.HaveReference ? e.OmegaMaxY : 0.0;
                inp.OmegaMaxAxis [2] = e.HaveReference ? e.OmegaMaxZ : 0.0;
                if (warp) {
                    // Physics warp: a real loop would fall back; the shadow restarts.
                    inp.OmegaDotValid = false;
                }

                // --------------------------------------------------------------- law
                var o = e.Output;
                AttitudeMath.Step (e.Gains, inp, e.Law, e.Work, o);
                if (o.NullColumnsTrimmed > 0)
                    caps |= CapGimbalTrimNull;
                if (o.Computed) {
                    caps |= CapLaw;
                    e.Computed++;
                    e.ConsumedSequence = e.Sequence;
                    e.ConsumedUtStamp = e.UtStamp;
                    if (!o.Converged)
                        motif |= MotifNotConverged;
                }
                if (o.ResidualGuard)
                    motif |= MotifResidualGuard;
                if (o.OmegaGuard)
                    motif |= MotifOmegaGuard;
                if (o.SaturationGuard)
                    motif |= MotifSaturationGuard;
                if (surfaceModel) {
                    SurfacesResidual (e, inp, o);
                    // (born closed): one step of the physical slope kappa.
                    SurfacesKappaStep (e, inp, o);
                    motif |= SurfaceMotifs (e);
                }
                // (born closed: option 4096): the shadow allocation of this step, after the law. Writes nothing.
                if (allocation) {
                    try {
                        caps |= AllocationSolve (e, o, ut);
                    } catch (Exception exc) {
                        AllocationFailed (e, exc);
                    }
                }

                // ------------------------------------------------ M3/M4: write or not
                // Guards are published on the axes of the arm mask (in shadow, what WOULD
                // fall back); each armed CHANNEL decides on its own axes: roll = body y,
                // pitch/yaw = body x and z.
                long common = motif;
                long armedGuards = AttitudeWrite.ArmedAxisGuardMotifs (o.AxisGuardMask, e.AxesMask);
                long rollGuards = AttitudeWrite.ArmedAxisGuardMotifs (o.AxisGuardMask,
                    e.AxesMask & AttitudeWrite.ArmRoll);
                long pitchYawGuards = AttitudeWrite.ArmedAxisGuardMotifs (o.AxisGuardMask,
                    e.AxesMask & AttitudeWrite.ArmPitchYaw);
                motif |= armedGuards;
                double rho = AttitudeWrite.RollCoupling (inp.B, inp.UMin, inp.UMax, n,
                    nGimbalColumns);
                int jPitch = nGimbalColumns, jRoll = nGimbalColumns + 1, jYaw = nGimbalColumns + 2;
                double rollCommand = double.NaN;
                bool scalarOk = o.Computed && AttitudeWrite.RollCommand (inp.U0 [jRoll],
                    o.TorqueAllocTarget [1], inp.B [1 * n + jRoll], out rollCommand);
                int writeState = AttitudeWrite.WriteShadow;
                int writeStatePitchYaw = AttitudeWrite.WriteShadow;
                bool contact = false;
                int ctrlNowAxes = 0;
                bool allocationWrote = false;
                if (e.Mode == ModeArmed) {
                    common |= hooked ? hookMotif : EnsureHook (e, vessel);
                    if (!AttitudeWrite.OrderVerified (e.ChainOwnIndex, e.ChainKrpcIndex, e.ChainLength))
                        common |= MotifFlyByWireOrder;
                    // Contact: on the ground or in the water (PRELAUNCH, the pad before any
                    // flight, is not a touchdown), or a demand no actuator can meet. The first
                    // half alone is the MEASURED contact a hold may follow (option 8).
                    bool measuredContact = (vessel.Landed && vessel.situation != Vessel.Situations.PRELAUNCH) ||
                        vessel.Splashed;
                    contact = measuredContact || AttitudeWrite.ImpossibleDemand (o.Unrealized, e.AxesMask);
                    if (contact)
                        common |= MotifContact;

                    // ---- pitch/yaw (M4): gimbals leased under the loop's own authority,
                    // RCS by ctrlState after the AutoPilot (fly-by-wire callback).
                    bool pitchYawWritten = false;
                    if ((e.AxesMask & AttitudeWrite.ArmPitchYaw) != 0) {
                        long py = common | pitchYawGuards;
                        double age = e.HaveReference ? ut - e.UtStamp : double.PositiveInfinity;
                        // (born closed: option 16): bit 8 of the client's write mask holds the
                        // reference past 0.3 s, up to its own validity (motif 2 still ends it).
                        if (AttitudeWrite.PitchYawReferenceOld (age, e.WriteMask, e.Options))
                            py |= MotifPitchYawReferenceOld;
                        else if (AttitudeWrite.PitchYawReferenceHeld (age, e.WriteMask, e.Options))
                            motif |= AttitudeWrite.MotifPitchYawReferenceHeld;
                        if ((e.WriteMask & AttitudeWrite.ArmPitchYaw) == 0)
                            py |= MotifPitchYawWriteOff;
                        if (pitchYawGuards != 0)
                            e.PitchYawHoldUntil = ut + AttitudeWrite.GuardHoldSeconds;
                        else if (ut < e.PitchYawHoldUntil)
                            py |= MotifPitchYawGuardHold;
                        // (born closed: option 64): bit 16 of the client's write mask -- pitch/yaw only while
                        // the gimbals lead the ctrlState channel (the law's alpha rule on body x and z), never again
                        // under the bit once they stopped leading after a write (motifs 42, 43). Without the bit in
                        // effect the latch clears and nothing else changes.
                        bool leadBit = (AttitudeWrite.EffectiveWriteMask (e.WriteMask, e.Options) &
                            AttitudeWrite.WriteGimbalsLead) != 0;
                        long leadMotifs = 0;
                        if (leadBit || e.LeadLatched) {
                            bool lead = leadBit && AttitudeWrite.GimbalsLead (
                                inp.UseBAuthority && inp.BAuthority.Length >= 3 * n ? inp.BAuthority : inp.B,
                                inp.UMin, inp.UMax, n, nGimbalColumns);
                            e.LeadLatched = AttitudeWrite.LeadLatch (e.LeadLatched, leadBit,
                                e.LeadWritten && e.WritePitchYawTick == ownTick - 1, lead);
                            leadMotifs = AttitudeWrite.LeadMotifs (leadBit, lead, e.LeadLatched);
                            py |= leadMotifs;
                        }
                        bool write = AttitudeWrite.MayWritePitchYaw (e.Mode, e.AxesMask, stage,
                            o.Computed, py, rules) && leadMotifs == 0;
                        if (write && !HoldAuthority (e, vessel)) {
                            py |= MotifGimbalAuthority;
                            write = false;
                        }
                        // a write made under bit 16 (the latch's "wrote at the previous step").
                        e.LeadWritten = leadBit && write;
                        if (write) {
                            // (born closed: option 8192): the engine-by-engine allocation writes in place of
                            // the law's allocation and the layer when its solve of this step is sound.
                            allocationWrote = (e.Options & AttitudeWrite.OptionEngineAllocationWrite) != 0 &&
                                AllocationDecide (e, inp);
                            int leased = 0;
                            if (allocationWrote)
                                leased = AllocationCommandGimbals (e);
                            for (int g = 0; g < e.Gimbals.Count && !allocationWrote; g++) {
                                int jx = 2 * g, jy = 2 * g + 1;
                                var gimbal = e.Gimbals [g].Gimbal;
                                var pivots = e.Gimbals [g].PivotsMapped ? e.Gimbals [g].Pivots : null;
                                if (gimbal == null ||
                                        !(inp.UMax [jx] > inp.UMin [jx] || inp.UMax [jy] > inp.UMin [jy]))
                                    continue;
                                float tx, ty;
                                if (planned) {
                                    // Common offset + the allocation's attitude part.
                                    tx = (float)(e.T.Common [jx] + inp.U0 [jx] + o.DeltaU [jx]);
                                    ty = (float)(e.T.Common [jy] + inp.U0 [jy] + o.DeltaU [jy]);
                                } else {
                                    tx = (float)(inp.U0 [jx] + o.DeltaU [jx]);
                                    ty = (float)(inp.U0 [jy] + o.DeltaU [jy]);
                                }
                                try {
                                    // pivots null (every V1 module) = the V1 overload.
                                    if (Actuation.CommandGimbalForAttitude (e.AuthorityToken,
                                            gimbal, tx, ty, pivots)) {
                                        leased++;
                                        // what a contact hold would command again (bookkeeping).
                                        e.Gimbals [g].HeldX = tx;
                                        e.Gimbals [g].HeldY = ty;
                                        e.Gimbals [g].HeldTick = ownTick;
                                    }
                                } catch (Exception exc) {
                                    // Transforms not ready: this gimbal stays stock this step.
                                    if (++e.FbwExceptions <= 5)
                                        BridgeLog.Error ("AttitudeV1 bail cardan: " + exc.Message);
                                }
                            }
                            e.GimbalsLeased = leased;
                            if (allocationWrote)
                                AllocationTakesLayer (e, planned);
                            else if (planned)
                                CommitTranslation (e, inp, o, nGimbalColumns);
                            else if (e.T.CommonComputed)
                                ClearReleasedCommon (e);
                            if (allocationWrote) {
                                e.WritePitchValue = Clamp1 (e.Al.CmdRot [0]);
                                e.WriteYawValue = Clamp1 (e.Al.CmdRot [2]);
                                caps |= CapEngineAllocationWrite;
                            } else {
                                e.WritePitchValue = Clamp1 (inp.U0 [jPitch] + o.DeltaU [jPitch]);
                                e.WriteYawValue = Clamp1 (inp.U0 [jYaw] + o.DeltaU [jYaw]);
                            }
                            if (families) {
                                // a frozen ctrlState axis is not written (NaN), the
                                // gimbals are; the callback writes each axis on its own.
                                if ((e.FrozenAxes & 1) != 0)
                                    e.WritePitchValue = double.NaN;
                                if ((e.FrozenAxes & 4) != 0)
                                    e.WriteYawValue = double.NaN;
                                ctrlNowAxes = (!double.IsNaN (e.WritePitchValue) ? 1 : 0) |
                                    (!double.IsNaN (e.WriteYawValue) ? 4 : 0);
                            }
                            e.WritePitchYaw = true;
                            e.WritePitchYawTick = ownTick;
                            writeStatePitchYaw = AttitudeWrite.WriteWriting;
                            pitchYawWritten = true;
                        } else if (AttitudeWrite.ContactHold (rules, e.Mode, e.AxesMask, stage, measuredContact,
                                e.GimbalsLeased > 0 && e.AuthorityToken != null, py) && HoldAtContact (e, vessel)) {
                            // (option 8): the measured contact while the client still allows
                            // pitch/yaw: the gimbals stay leased at their last deflection, not
                            // handed to the AutoPilot; no ctrlState written, the differential
                            // throttles already back to the main throttle (HoldAtContact).
                            if (planned)
                                e.T.Reasons |= AttitudeTranslation.ReasonNotWriting;
                            py |= MotifPitchYawFallback | AttitudeWrite.MotifPitchYawContactHold;
                            e.PitchYawFallbackSteps++;
                            e.ContactHoldSteps++;
                            writeStatePitchYaw = AttitudeWrite.WriteFallback;
                        } else {
                            // Bumpless: the stock Lerp resumes from where the gimbals are,
                            // toward the AutoPilot's ctrlState.
                            ReleaseAuthority (e);
                            if (planned)
                                e.T.Reasons |= AttitudeTranslation.ReasonNotWriting;
                            py |= MotifPitchYawFallback;
                            e.PitchYawFallbackSteps++;
                            writeStatePitchYaw = AttitudeWrite.WriteFallback;
                        }
                        motif |= py;
                    }

                    // (born closed): the surfaces are held neutral this step when the loop
                    // writes ctrlState pitch/yaw now, or wrote pitch/yaw (or a roll they answer)
                    // at the last fly-by-wire.
                    bool neutralize = AttitudeWrite.NeutralizeSurfaces (rules, e.Mode, stage,
                        ctrlNowAxes, ctrlLastAxes, e.SurfaceAxes & ~e.SurfaceAxesUnneutralizable);

                    // ---- roll (M3). When the loop owns the gimbals this step, no stock
                    // mixing is left: the roll command is the allocation's own.
                    if ((e.AxesMask & AttitudeWrite.ArmRoll) != 0) {
                        long r = common | rollGuards;
                        bool commandOk = scalarOk;
                        if (pitchYawWritten) {
                            // the allocation's roll when it wrote (its roll column free only when the loop
                            // writes the roll).
                            rollCommand = !o.Computed ? double.NaN : allocationWrote ? Clamp1 (e.Al.CmdRot [1])
                                : Clamp1 (inp.U0 [jRoll] + o.DeltaU [jRoll]);
                            commandOk = o.Computed && !double.IsNaN (rollCommand);
                            // a frozen roll column is not written (its u0 is not ours).
                            if (families && (e.FrozenAxes & 2) != 0)
                                commandOk = false;
                        } else if (!(rho <= AttitudeWrite.RollCouplingMax)) {
                            r |= MotifRollChannelUnmodeled;
                        }
                        if (o.Computed && !commandOk)
                            r |= MotifRollChannelUnmodeled;
                        // a surface that answers the roll and is not held neutral this
                        // step would add a roll effectiveness the column does not have.
                        if (AttitudeWrite.RollSurfaceGate (rules, e.SurfaceAxes, neutralize))
                            r |= MotifRollChannelUnmodeled | AttitudeWrite.MotifRollSurfaceGate;
                        if (rollGuards != 0)
                            e.GuardHoldUntil = ut + AttitudeWrite.GuardHoldSeconds;
                        else if (ut < e.GuardHoldUntil)
                            r |= MotifGuardHold;
                        if ((e.WriteMask & AttitudeWrite.ArmRoll) == 0)
                            r |= MotifClientWriteOff;
                        if (commandOk && AttitudeWrite.MayWrite (e.Mode, e.AxesMask, stage,
                                o.Computed, r, rules)) {
                            // (born closed: option 16384): the command the RCS answers -- the law's, its roll held,
                            // or its pitch/yaw held (only when the loop writes both this step): written just under the RCS
                            // dead zone, which the RCS ignores and the wheels still get.
                            if (rcsCombined) {
                                bool pyOwn = pitchYawWritten && e.WritePitchYawTick == ownTick &&
                                    !double.IsNaN (e.WritePitchValue) && !double.IsNaN (e.WriteYawValue);
                                double pw = pyOwn ? e.WritePitchValue : served [0];
                                double yw = pyOwn ? e.WriteYawValue : served [2];
                                int decision = RcsCombinedDecide (e, inp, pw, rollCommand, yw, pyOwn);
                                if (decision == RcsCombined.RollHeld) {
                                    rollCommand = RcsCombined.HeldValue (e.Rc.Set, 1, rollCommand);
                                    caps |= CapRcsRollHeld;
                                } else if (decision == RcsCombined.PitchYawHeld) {
                                    e.WritePitchValue = RcsCombined.HeldValue (e.Rc.Set, 0, e.WritePitchValue);
                                    e.WriteYawValue = RcsCombined.HeldValue (e.Rc.Set, 2, e.WriteYawValue);
                                    caps |= CapRcsPitchYawHeld;
                                }
                            }
                            e.WriteRoll = true;
                            e.WriteRollValue = rollCommand;
                            e.WriteTick = ownTick;
                            writeState = AttitudeWrite.WriteWriting;
                        } else {
                            r |= MotifWriteFallback;
                            e.FallbackSteps++;
                            writeState = AttitudeWrite.WriteFallback;
                        }
                        motif |= r;
                    }
                    motif |= common;

                    // hold the answering surfaces neutral until BetterLateThanNever. If
                    // the field cannot be written, no ctrlState is written this step (the
                    // leased gimbals are).
                    if (neutralize) {
                        if (HoldSurfacesNeutral (e)) {
                            motif |= AttitudeWrite.MotifSurfacesNeutralized;
                            if (registry)
                                e.Effectors.NeutralizedSurfaces = e.Neutralized.Count;
                        } else {
                            e.WritePitchValue = double.NaN;
                            e.WriteYawValue = double.NaN;
                            if (e.WriteRoll) {
                                e.WriteRoll = false;
                                e.FallbackSteps++;
                                writeState = AttitudeWrite.WriteFallback;
                                motif |= MotifWriteFallback;
                            }
                            motif |= AttitudeWrite.MotifCtrlFrozenPitch |
                                AttitudeWrite.MotifCtrlFrozenRoll | AttitudeWrite.MotifCtrlFrozenYaw;
                        }
                    }
                }
                e.LastWriteStateRoll = writeState;
                e.LastWriteStatePitchYaw = writeStatePitchYaw;
                // (born closed: option 512): the main throttle of this step, written at its fly-by-wire.
                if ((e.Options & AttitudeWrite.OptionMainThrottle) != 0)
                    caps |= ThrottleStep (e, vessel, reference, com, ut, dt, stage, warp, nGimbalColumns);
                // (born closed: nothing without the client's window): the roll excitation of
                // this step, added to the AutoPilot's roll by our fly-by-wire link -- never while the
                // loop writes the roll itself or holds the surfaces neutral.
                if (surfaceModel)
                    SurfacesRollExcitation (e, ut, dt, families, (e.WriteRoll && e.WriteTick == ownTick) ||
                        (motif & AttitudeWrite.MotifSurfacesNeutralized) != 0);

                // (option 4096 or 8192): the allocation's block and frame, after the write decision.
                if (allocation) {
                    try {
                        AllocationPublish (e, ut);
                    } catch (Exception exc) {
                        AllocationFailed (e, exc);
                    }
                }

                // ------------------------------------------------------------- frame
                long end = Stopwatch.GetTimestamp ();
                e.CostMicrosLast = (end - t0) * 1e6 / Stopwatch.Frequency;
                if (e.CostMicrosLast > e.CostMicrosMax)
                    e.CostMicrosMax = e.CostMicrosLast;
                WriteFrame (e, vessel, stage, ut, dt, motif, caps, served, engineBody, rcsBody,
                    wheelBody, rho, writeState, rollCommand, writeStatePitchYaw, contact);
                WriteTranslationFrame (e, n, nGimbalColumns,
                    planned && writeStatePitchYaw == AttitudeWrite.WriteWriting && !allocationWrote);
                // (born closed): the named evidence block of this step.
                UpdateEvidenceBlock (e, ut, planned);
                e.LastMotif = motif;
            } catch (Exception exc) {
                e.WriteRoll = false;
                e.WritePitchYaw = false;
                e.Th.Write = false;            // no main throttle from a failed tick
                e.Al.Write = false;            // no allocation write from a failed tick
                e.Sm.Roll.ValueTick = -1;      // no excitation from a failed tick
                e.LastWriteStateRoll = AttitudeWrite.WriteShadow;
                e.LastWriteStatePitchYaw = AttitudeWrite.WriteShadow;
                try {
                    ReleaseAuthority (e);
                } catch {
                    // Nothing may escape the tick.
                }
                RestoreSurfaces (e);
                e.Exceptions++;
                e.Law.Reset ();
                e.HavePrevious = false;
                e.LastMotif = motif | MotifException |
                    (e.Mode == ModeArmed ? MotifWriteFallback : 0L);
                if (e.Exceptions <= 5)
                    BridgeLog.Error ("AttitudeV1 tick: " + exc);
            }
        }

        static double Clamp1 (double u)
        {
            if (double.IsNaN (u) || double.IsInfinity (u))
                return double.NaN;
            return u > 1.0 ? 1.0 : (u < -1.0 ? -1.0 : u);
        }

        /// <summary>
        /// M4 : the actuator authority of this step's pitch/yaw write. The active vessel (and
        /// any vessel armed with option 0) takes the active vessel's authority, unchanged:
        /// TryHoldForAttitude, which refuses a non-active vessel. A NON-ACTIVE vessel armed
        /// with option 1 takes its own per-vessel authority; the kind held before a change of
        /// active vessel is handed back first (its gimbals without a jump).
        /// </summary>
        static bool HoldAuthority (Entry e, Vessel vessel)
        {
            string owner = "AttitudeV1:" + e.PersistentId;
            if (e.InactiveOptions == 0)
                return Actuation.TryHoldForAttitude (owner, vessel, ref e.AuthorityToken);
            bool active = ReferenceEquals (vessel, FlightGlobals.ActiveVessel);
            if (AttitudeWrite.UsesInactiveAuthority (active, e.InactiveOptions)) {
                if (e.AuthorityToken != null && !e.AuthorityInactive)
                    ReleaseAuthority (e);
                bool held = Actuation.TryHoldForAttitudeInactive (owner, vessel,
                    ref e.AuthorityToken);
                e.AuthorityInactive = held && e.AuthorityToken != null;
                return held;
            }
            if (e.AuthorityInactive)
                ReleaseAuthority (e);
            return Actuation.TryHoldForAttitude (owner, vessel, ref e.AuthorityToken);
        }

        /// <summary>
        /// (option 8): one step of the contact hold. The authority renewed (refused: false, and the
        /// caller hands everything back), the differential throttles of the lateral force given back to
        /// the main throttle (the client's cut must act at once: an independent throttle would outlive it
        /// for its lease), and every module leased at the last pitch/yaw write commanded again at that
        /// deflection, through the same lease and pivots as the write. False when no module could be
        /// held. Never throws.
        /// </summary>
        static bool HoldAtContact (Entry e, Vessel vessel)
        {
            try {
                if (!HoldAuthority (e, vessel))
                    return false;
                ReleaseDiffThrottles (e);
                e.T.WriteThrottle = false;
                e.T.WriteRcs = false;
                // the allocation's throttles back to the main throttle too, no write of its own.
                AllocationReleaseThrottles (e);
                e.Al.Write = false;
                int held = 0;
                for (int g = 0; g < e.Gimbals.Count; g++) {
                    var r = e.Gimbals [g];
                    if (r.Gimbal == null || r.HeldTick < 0 || r.HeldTick != e.WritePitchYawTick)
                        continue;
                    if (Actuation.CommandGimbalForAttitude (e.AuthorityToken, r.Gimbal, r.HeldX, r.HeldY,
                            r.PivotsMapped ? r.Pivots : null))
                        held++;
                }
                e.GimbalsLeased = held;
                return held > 0;
            } catch (Exception exc) {
                if (++e.FbwExceptions <= 5)
                    BridgeLog.Error ("AttitudeV1 tenue au contact: " + exc.Message);
                return false;
            }
        }

        /// <summary>
        /// M4: hands the gimbals back WITHOUT a jump (the stock Lerp resumes from the leased
        /// deflection) and the Actuators authority, if this loop holds them (either kind).
        /// </summary>
        static void ReleaseAuthority (Entry e)
        {
            var token = e.AuthorityToken;
            e.AuthorityToken = null;
            e.AuthorityInactive = false;
            e.GimbalsLeased = 0;
            // Lateral force: every lease (throttles too) goes back with the authority.
            ResetTranslationDynamics (e);
            // the allocation's throttles too (restored with the authority), and it starts again from a
            // streak of zero when the channel writes again.
            e.Al.ResetWrite ();
            if (token == null)
                return;
            try {
                Actuation.ReleaseForAttitude (token);
            } catch (Exception exc) {
                BridgeLog.Error ("AttitudeV1: rendu des cardans: " + exc.Message);
            }
        }

        // ----------------------------------------------------- M3: fly-by-wire hook

        /// <summary>
        /// Keeps our callback LAST in vessel.OnFlyByWire (after the kRPC AutoPilot's, which
        /// PilotAddon.Fly registers once per vessel), and records the chain positions. The
        /// chain is re-read only when the delegate object changes (Combine/Remove make a
        /// new one), so a steady chain costs one reference comparison per tick.
        /// </summary>
        static long EnsureHook (Entry e, Vessel vessel)
        {
            long motif = 0;
            if (e.FlyByWire == null) {
                var entry = e;
                e.FlyByWire = new FlightInputCallback (s => OnFlyByWire (entry, s));
            }
            if (!ReferenceEquals (e.HookedVessel, vessel)) {
                Unhook (e);
                vessel.OnFlyByWire = (FlightInputCallback)Delegate.Combine (vessel.OnFlyByWire,
                    e.FlyByWire);
                e.HookedVessel = vessel;
                e.ChainSeen = null;
            }
            Delegate chain = vessel.OnFlyByWire;
            if (!ReferenceEquals (chain, e.ChainSeen)) {
                int own, krpc, length;
                AttitudeWrite.ClassifyChain (chain, e.FlyByWire, out own, out krpc, out length);
                if (own < 0 || own != length - 1) {
                    // Someone registered after us (or removed us): back to the end.
                    vessel.OnFlyByWire = (FlightInputCallback)Delegate.Remove (vessel.OnFlyByWire,
                        e.FlyByWire);
                    vessel.OnFlyByWire = (FlightInputCallback)Delegate.Combine (vessel.OnFlyByWire,
                        e.FlyByWire);
                    e.ChainReorders++;
                    motif |= MotifFlyByWireReordered;
                    chain = vessel.OnFlyByWire;
                    AttitudeWrite.ClassifyChain (chain, e.FlyByWire, out own, out krpc, out length);
                }
                e.ChainSeen = chain;
                e.ChainOwnIndex = own;
                e.ChainKrpcIndex = krpc;
                e.ChainLength = length;
            }
            return motif;
        }

        static void Unhook (Entry e)
        {
            var hooked = e.HookedVessel;
            if (hooked != null && e.FlyByWire != null) {
                try {
                    // Computed levers: X/Y still holding our last write go back to 0 (a
                    // non-active vessel's ctrlState is not reset by FlightInputHandler).
                    ReleaseRcs (e.T, hooked.ctrlState);
                    AllocationReleaseLinear (e.Al, hooked.ctrlState);
                } catch (Exception exc) {
                    BridgeLog.Error ("AttitudeV1: rendu de X/Y: " + exc.Message);
                }
                try {
                    hooked.OnFlyByWire = (FlightInputCallback)Delegate.Remove (hooked.OnFlyByWire,
                        e.FlyByWire);
                } catch (Exception exc) {
                    BridgeLog.Error ("AttitudeV1: retrait du fly-by-wire: " + exc.Message);
                }
            }
            e.HookedVessel = null;
            e.ChainSeen = null;
            e.ChainOwnIndex = -1;
            e.ChainKrpcIndex = -1;
            e.ChainLength = 0;
            e.WriteRoll = false;
        }

        /// <summary>
        /// Our link of the vessel's fly-by-wire chain, after the kRPC AutoPilot's: records
        /// the ctrlState it served (published in the next frame), then writes the roll
        /// decided at this step's Earlyish, once. Nothing may escape: an exception would
        /// stop the chain and FeedInputFeed before propagateControlUpdate.
        /// </summary>
        static void OnFlyByWire (Entry e, FlightCtrlState state)
        {
            try {
                if (state == null)
                    return;
                e.FbwCalls++;
                e.FbwTick = ownTick;
                e.FbwApPitch = state.pitch;
                e.FbwApRoll = state.roll;
                e.FbwApYaw = state.yaw;
                e.FbwWrittenRoll = double.NaN;
                e.FbwWrittenPitch = double.NaN;
                e.FbwWrittenYaw = double.NaN;
                // Lot 3, 3.1 (born closed): where the surfaces' FixedUpdate is against this callback.
                if (e.Sm.Mode != SurfaceModel.ModeOff)
                    SurfacesFlyByWire (e);
                if (e.WriteRoll && e.WriteTick == ownTick && e.Mode == ModeArmed && !e.Dropped &&
                        !double.IsNaN (e.WriteRollValue)) {
                    float value = (float)e.WriteRollValue;
                    state.roll = value;
                    e.FbwWrittenRoll = value;
                    e.Writes++;
                }
                // (born closed): the roll excitation decided at this step's Earlyish, added to
                // the AutoPilot's roll (never over the loop's own roll write).
                if (e.Sm.Roll.ValueTick == ownTick)
                    SurfacesRollFlyByWire (e, state);
                // M4: the RCS pitch/yaw decided at this step's Earlyish (the gimbals, leased,
                // no longer follow ctrlState).
                bool pitchYawWrote = false;
                if ((RuleOptions (e) & AttitudeWrite.OptionFamilyBlocking) != 0) {
                    // (born closed): the channel wrote its gimbals at Earlyish; each
                    // ctrlState axis that is not frozen (NaN) is written on its own.
                    if (e.WritePitchYaw && e.WritePitchYawTick == ownTick && e.Mode == ModeArmed &&
                            !e.Dropped) {
                        if (!double.IsNaN (e.WritePitchValue)) {
                            float pitch = (float)e.WritePitchValue;
                            state.pitch = pitch;
                            e.FbwWrittenPitch = pitch;
                        }
                        if (!double.IsNaN (e.WriteYawValue)) {
                            float yaw = (float)e.WriteYawValue;
                            state.yaw = yaw;
                            e.FbwWrittenYaw = yaw;
                        }
                        e.PitchYawWrites++;
                        pitchYawWrote = true;
                    }
                } else if (e.WritePitchYaw && e.WritePitchYawTick == ownTick && e.Mode == ModeArmed &&
                        !e.Dropped && !double.IsNaN (e.WritePitchValue) &&
                        !double.IsNaN (e.WriteYawValue)) {
                    float pitch = (float)e.WritePitchValue;
                    float yaw = (float)e.WriteYawValue;
                    state.pitch = pitch;
                    state.yaw = yaw;
                    e.FbwWrittenPitch = pitch;
                    e.FbwWrittenYaw = yaw;
                    e.PitchYawWrites++;
                    pitchYawWrote = true;
                }
                // (born closed: option 512): the main throttle decided at this step's Earlyish, before the
                // differential throttles that take it.
                if ((e.Options & AttitudeWrite.OptionMainThrottle) != 0)
                    WriteMainThrottle (e, state);
                // (born closed: option 8192): the allocation's throttles and X / Y around THIS step's main
                // throttle, or every lease of its own given back, before the layer writes.
                if ((e.Options & AttitudeWrite.OptionEngineAllocationWrite) != 0 || e.Al.Leased.Count > 0 ||
                        e.Al.LinWritten || e.Al.Write)
                    AllocationFlyByWire (e, state, pitchYawWrote);
                // Lateral force, computed levers (bit 4): the RCS translation (ctrlState.X/Y),
                // or the release of the previous write. Nothing at all in modes 1 and 3.
                WriteRcsTranslation (e, state, pitchYawWrote);
                // Lateral force: the differential throttles around THIS step's main throttle.
                WriteDiffThrottles (e, state);
                e.WriteRoll = false;
                e.WritePitchYaw = false;
                e.T.WriteThrottle = false;
            } catch (Exception exc) {
                e.WriteRoll = false;
                e.WritePitchYaw = false;
                e.T.WriteThrottle = false;
                e.T.WriteRcs = false;
                e.Th.Write = false;
                e.Al.Write = false;
                e.FbwExceptions++;
                if (e.FbwExceptions <= 5)
                    BridgeLog.Error ("AttitudeV1 fly-by-wire: " + exc);
            }
        }

        static void ClearFlyByWireRecord (Entry e)
        {
            e.FbwTick = -1;
            e.FbwApPitch = double.NaN;
            e.FbwApRoll = double.NaN;
            e.FbwApYaw = double.NaN;
            e.FbwWrittenRoll = double.NaN;
            e.FbwWrittenPitch = double.NaN;
            e.FbwWrittenYaw = double.NaN;
        }

        static void SetColumn (AttitudeInput inp, int j, Vector3 column)
        {
            SetColumn (inp.B, inp.N, j, column);
        }

        static void SetColumn (double[] b, int n, int j, Vector3 column)
        {
            b [0 * n + j] = column.x;
            b [1 * n + j] = column.y;
            b [2 * n + j] = column.z;
        }

        static void WriteFrame (Entry e, Vessel vessel, int stage, double ut, double dt,
            long motif, long caps, double[] served, Vector3 engineBody, Vector3 rcsBody,
            double[] wheelBody, double rollCoupling, int writeState, double rollCommand,
            int writeStatePitchYaw, bool contact)
        {
            var inp = e.Input;
            var o = e.Output;
            int n = inp.N;
            // Gimbal columns, then the 3 ctrlState ones, then (lateral force only) the 2
            // differential motifs: n - 3 gimbal columns exactly as before when none is planned.
            int nGimbalColumns = 2 * e.Gimbals.Count;
            int firstMotif = nGimbalColumns + 3;
            // Core v2: the ring's next buffer, zeroed like the new array it replaces.
            int frameLength = HeaderStride + StateStride + n * RowStride + ClockStride;
            var frame = e.History.Reserve (frameLength);
            int h = 0;
            frame [h++] = ProtocolVersion;
            frame [h++] = SchemaVersion;
            frame [h++] = ownTick;
            frame [h++] = ut;
            frame [h++] = dt;
            frame [h++] = e.PersistentId;
            frame [h++] = e.TopologyGeneration;
            frame [h++] = stage;
            frame [h++] = Actuation.CurrentPhysicsTick;
            frame [h++] = Time.fixedTime;
            frame [h++] = e.Mode;
            frame [h++] = e.AxesMask;
            frame [h++] = motif;
            frame [h++] = o.Computed ? e.ConsumedSequence : 0;
            frame [h++] = o.Computed ? e.ConsumedUtStamp : double.NaN;
            frame [h++] = o.Computed ? ut - e.ConsumedUtStamp : double.NaN;
            frame [h++] = e.CostMicrosLast;
            frame [h++] = o.Computed ? o.Iterations : 0;
            frame [h++] = o.Computed && o.Converged ? 1.0 : 0.0;
            frame [h++] = n;
            frame [h++] = StateStride;
            frame [h++] = n;
            frame [h++] = RowStride;
            frame [h++] = caps;

            int s = HeaderStride;
            Put3 (frame, s + 0, inp.Error, inp.ReferenceValid);
            Put3 (frame, s + 3, inp.Omega, true);
            Put3 (frame, s + 6, inp.OmegaDotMeasured, inp.OmegaDotValid);
            // Filtered quantities exist without a reference (the filters run whenever the
            // tick is consecutive); the law's quantities only when the law ran.
            Put3 (frame, s + 9, o.OmegaDotF, o.Filtered);
            Put3 (frame, s + 12, o.Alpha, o.Filtered);
            Put3 (frame, s + 15, o.OmegaSqrt, o.Computed);
            Put3 (frame, s + 18, o.OmegaRef, o.Computed);
            Put3 (frame, s + 21, o.OmegaPch, o.Computed);
            Put3 (frame, s + 24, o.Nu, o.Computed);
            Put3 (frame, s + 27, inp.Inertia, true);
            Put3 (frame, s + 30, inp.Torque, true);
            Put3 (frame, s + 33, o.TorqueF, o.Filtered);
            Put3 (frame, s + 36, o.DeltaTorqueDesired, o.Computed);
            Put3 (frame, s + 39, o.TorqueDesired, o.Computed);
            Put3 (frame, s + 42, o.TorqueAllocTarget, o.Computed);
            Put3 (frame, s + 45, o.TorqueAllocated, o.Computed);
            Put3 (frame, s + 48, o.NuHedge, o.Computed);
            Put3 (frame, s + 51, o.Residual, o.Filtered);
            Put3 (frame, s + 54, o.ResidualF, o.Filtered);
            Put3 (frame, s + 57, e.RefBody, true);
            Put3 (frame, s + 60, served, true);
            Put3 (frame, s + 63, e.FfBody, true);
            frame [s + 66] = inp.OmegaMax > 0.0 ? inp.OmegaMax : e.Gains.OmegaMaxDefault;
            frame [s + 67] = o.Computed ? o.SaturationFraction : double.NaN;
            frame [s + 68] = o.Computed ? o.ActiveCount : double.NaN;
            frame [s + 69] = (o.ResidualGuard ? 1.0 : 0.0) + (o.OmegaGuard ? 2.0 : 0.0) +
                (o.SaturationGuard ? 4.0 : 0.0);
            frame [s + 70] = rcsBody.x;
            frame [s + 71] = rcsBody.y;
            frame [s + 72] = rcsBody.z;
            frame [s + 73] = wheelBody [0];
            frame [s + 74] = wheelBody [1];
            frame [s + 75] = wheelBody [2];
            frame [s + 76] = engineBody.x;
            frame [s + 77] = engineBody.y;
            frame [s + 78] = engineBody.z;
            // Schema 2 (M3).
            Put3 (frame, s + 79, o.Unrealized, o.Computed);
            Put3 (frame, s + 82, o.OmegaMaxAxis, o.Filtered);
            frame [s + 85] = o.Filtered ? o.AxisGuardMask : double.NaN;
            frame [s + 86] = rollCoupling;
            frame [s + 87] = writeState;
            frame [s + 88] = rollCommand;
            // What the fly-by-wire callback saw and wrote in the PREVIOUS physics step
            // (it runs after Earlyish); frame [s + 89] is that step's attitude tick.
            frame [s + 89] = e.FbwTick >= 0 ? e.FbwTick : double.NaN;
            frame [s + 90] = e.FbwApPitch;
            frame [s + 91] = e.FbwApRoll;
            frame [s + 92] = e.FbwApYaw;
            frame [s + 93] = e.FbwWrittenRoll;
            frame [s + 94] = e.ChainOwnIndex;
            frame [s + 95] = e.ChainKrpcIndex;
            frame [s + 96] = e.ChainLength;
            frame [s + 97] = e.ChainReorders;
            frame [s + 98] = e.Writes;
            frame [s + 99] = e.WriteMask;
            // Schema 3 (M4).
            frame [s + 100] = writeStatePitchYaw;
            frame [s + 101] = e.FbwWrittenPitch;
            frame [s + 102] = e.FbwWrittenYaw;
            ClearFlyByWireRecord (e);
            frame [s + 103] = e.GimbalsLeased;
            frame [s + 104] = e.AuthorityToken != null &&
                Actuation.HoldsForAttitude (e.AuthorityToken) ? 1.0 : 0.0;
            frame [s + 105] = contact ? 1.0 : 0.0;
            frame [s + 106] = e.HaveReference ? ut - e.UtStamp : double.NaN;
            Put3 (frame, s + 107, o.OmegaGuardLimit, o.Filtered);

            int r = HeaderStride + StateStride;
            for (int j = 0; j < n; j++) {
                bool gimbal = j < nGimbalColumns;
                bool diffMotif = j >= firstMotif;
                var gr = gimbal ? e.Gimbals [j / 2] : null;
                frame [r++] = gimbal ? KindGimbal : (diffMotif ? KindDiffMotif : KindCtrlState);
                frame [r++] = gimbal && gr.Part != null ? gr.Part.flightID : 0.0;
                frame [r++] = gimbal ? gr.Ordinal : 0.0;
                // Motif rows: axis 0 = pitch motif, 2 = yaw motif (body axis of their torque).
                frame [r++] = gimbal ? j % 2 : (diffMotif ? (j == firstMotif ? 0 : 2) : j - nGimbalColumns);
                frame [r++] = inp.U0 [j];
                frame [r++] = o.Computed ? o.DeltaU [j] : double.NaN;
                frame [r++] = inp.UMin [j];
                frame [r++] = inp.UMax [j];
                frame [r++] = o.Computed ? o.ActiveSet [j] : double.NaN;
                frame [r++] = inp.B [0 * n + j];
                frame [r++] = inp.B [1 * n + j];
                frame [r++] = inp.B [2 * n + j];
                if (gimbal || diffMotif) {
                    frame [r++] = inp.B [0 * n + j];
                    frame [r++] = inp.B [1 * n + j];
                    frame [r++] = inp.B [2 * n + j];
                } else {
                    int a = j - nGimbalColumns;
                    // Asymmetric ctrlState columns: the row carries B+ then B-; the QP used
                    // their mean.
                    frame [r - 3] = e.BPos [3 * a];
                    frame [r - 2] = e.BPos [3 * a + 1];
                    frame [r - 1] = e.BPos [3 * a + 2];
                    frame [r++] = e.BNeg [3 * a];
                    frame [r++] = e.BNeg [3 * a + 1];
                    frame [r++] = e.BNeg [3 * a + 2];
                }
                frame [r++] = inp.Priority [j];
            }
            // Schema 4 (refactor R3): the clock section, after every row.
            frame [HeaderStride + StateStride + n * RowStride] = GameClock.StepTick;
            e.History.Commit (frameLength);
            e.LatestTick = ownTick;
        }

        static void Put3 (double[] target, int offset, double[] v, bool valid)
        {
            target [offset] = valid ? v [0] : double.NaN;
            target [offset + 1] = valid ? v [1] : double.NaN;
            target [offset + 2] = valid ? v [2] : double.NaN;
        }

        // ------------------------------------------------------------------ vessel cache

        /// <summary>the vessel persistentId is loaded, unpacked and NOT the active one.</summary>
        static bool LoadedInactive (uint persistentId)
        {
            var loaded = FlightGlobals.VesselsLoaded;
            for (int i = 0; loaded != null && i < loaded.Count; i++) {
                var v = loaded [i];
                if (v != null && v.loaded && !v.packed && v.persistentId == persistentId)
                    return !ReferenceEquals (v, FlightGlobals.ActiveVessel);
            }
            return false;
        }

        static Vessel Resolve (Entry e)
        {
            var v = e.Vessel;
            if (v != null && v.loaded && !v.packed && v.persistentId == e.PersistentId)
                return v;
            e.Vessel = null;
            var loaded = FlightGlobals.VesselsLoaded;
            for (int i = 0; loaded != null && i < loaded.Count; i++) {
                var candidate = loaded [i];
                if (candidate != null && candidate.loaded && !candidate.packed &&
                        candidate.persistentId == e.PersistentId) {
                    e.Vessel = candidate;
                    return candidate;
                }
            }
            return null;
        }

        static void Rebuild (Entry e, Vessel vessel, ulong hash)
        {
            // Lateral force: the columns belonged to the old layout (the request stays).
            DisengageTranslation (e);
            e.VesselId = vessel.id;
            e.TopologyHash = hash;
            e.TopologyGeneration++;
            e.Engines.Clear ();
            e.Gimbals.Clear ();
            e.Rcs.Clear ();
            e.Wheels.Clear ();
            e.Surfaces.Clear ();
            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                if (part == null)
                    continue;
                var partEngines = new List<ModuleEngines> ();
                int gimbalOrdinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var module = part.Modules [m];
                    var engine = module as ModuleEngines;
                    if (engine != null) {
                        partEngines.Add (engine);
                        e.Engines.Add (engine);
                    }
                    var gimbal = module as ModuleGimbal;
                    if (gimbal != null)
                        e.Gimbals.Add (new GimbalRef {
                            Gimbal = gimbal, Part = part, Ordinal = gimbalOrdinal++,
                            PartEngines = partEngines
                        });
                    var rcs = module as ModuleRCS;
                    if (rcs != null)
                        e.Rcs.Add (rcs);
                    var wheel = module as ModuleReactionWheel;
                    if (wheel != null)
                        e.Wheels.Add (wheel);
                    var surface = module as ModuleControlSurface;
                    if (surface != null && !(surface is ModuleAeroSurface))
                        e.Surfaces.Add (surface);
                }
            }
            e.Law.Reset ();
            e.HavePrevious = false;
        }

        // ------------------------------------------------------------------ helpers

        static Entry RequireToken (string token)
        {
            if (!string.IsNullOrEmpty (token))
                foreach (var pair in entries)
                    if (pair.Value.Token == token) {
                        if (pair.Value.Arming.ExpiredReal (Time.realtimeSinceStartup)) {
                            Drop (pair.Key, BridgeErrors.TokenExpired);
                            break;
                        }
                        return pair.Value;
                    }
            throw new InvalidOperationException (BridgeErrors.Tag (droppedTokens.CodeOf (token),
                "token d'attitude absent, expire ou invalide"));
        }

        /// <summary>
        /// the options of the token's entry, 0 for an unknown token -- read without dropping or
        /// throwing (the write-mask check of AttitudeReferenceV2 runs before the token is required, and
        /// keeps its error first, as before).
        /// </summary>
        static int OptionsOf (string token)
        {
            if (!string.IsNullOrEmpty (token))
                foreach (var pair in entries)
                    if (pair.Value.Token == token)
                        return pair.Value.Options;
            return 0;
        }

        static void ValidateLease (float leaseSeconds)
        {
            if (float.IsNaN (leaseSeconds) || float.IsInfinity (leaseSeconds) ||
                    leaseSeconds < MinLeaseSeconds || leaseSeconds > MaxLeaseSeconds)
                throw new ArgumentOutOfRangeException (
                    nameof (leaseSeconds), "bail d'attitude attendu entre 0.2 et 10 s");
        }

        static bool Finite (double v)
        {
            return !double.IsNaN (v) && !double.IsInfinity (v);
        }
    }

    /// <summary>
    /// T-sol-0 order probe: which of the TimingManager stages, the Actuators watcher and
    /// the vessel's fly-by-wire chain run first inside one physics step, and what each of
    /// them sees (tick counters, UT, served ctrlState, first gimbal, first engine thrust,
    /// angular velocity). Read-only: the fly-by-wire hook only reads ctrlState.
    /// </summary>
    internal static class AttitudeProbe
    {
        internal const int StagePrecalc = 1;
        internal const int StageEarlyish = 2;
        internal const int StageNormal = 3;
        internal const int StageFlightIntegrator = 4;
        internal const int StageLate = 5;
        internal const int StageBetterLateThanNever = 6;
        internal const int StagePump = 7;
        internal const int StageFlyByWire = 8;
        internal const int RowStride = 13;
        const int MaxRows = 4000;

        static readonly TimingManager.UpdateAction precalc = () => Stamp (StagePrecalc);
        static readonly TimingManager.UpdateAction earlyish = () => Stamp (StageEarlyish);
        static readonly TimingManager.UpdateAction normal = () => Stamp (StageNormal);
        static readonly TimingManager.UpdateAction integrator = () => Stamp (StageFlightIntegrator);
        static readonly TimingManager.UpdateAction late = () => Stamp (StageLate);
        static readonly TimingManager.UpdateAction lateNever = () => Stamp (StageBetterLateThanNever);
        static readonly FlightInputCallback flyByWire = OnFlyByWire;

        static bool armed;
        static int framesLeft;
        static float lastFixedTime = float.NaN;
        static int frameIndex;
        static int sequenceInFrame;
        static Vessel hooked;
        static readonly List<double> rows = new List<double> ();

        internal static bool Start (int frames)
        {
            if (frames < 1 || frames > 300)
                throw new ArgumentOutOfRangeException (nameof (frames), "1 a 300 pas physiques");
            Stop ();
            rows.Clear ();
            framesLeft = frames;
            frameIndex = 0;
            sequenceInFrame = 0;
            lastFixedTime = float.NaN;
            TimingManager.FixedUpdateAdd (TimingManager.TimingStage.Precalc, precalc);
            TimingManager.FixedUpdateAdd (TimingManager.TimingStage.Earlyish, earlyish);
            TimingManager.FixedUpdateAdd (TimingManager.TimingStage.Normal, normal);
            TimingManager.FixedUpdateAdd (TimingManager.TimingStage.FlightIntegrator, integrator);
            TimingManager.FixedUpdateAdd (TimingManager.TimingStage.Late, late);
            TimingManager.FixedUpdateAdd (TimingManager.TimingStage.BetterLateThanNever, lateNever);
            var vessel = FlightGlobals.ActiveVessel;
            if (vessel != null) {
                vessel.OnFlyByWire += flyByWire;
                hooked = vessel;
            }
            armed = true;
            return true;
        }

        internal static void Stop ()
        {
            if (!armed)
                return;
            armed = false;
            try {
                TimingManager.FixedUpdateRemove (TimingManager.TimingStage.Precalc, precalc);
                TimingManager.FixedUpdateRemove (TimingManager.TimingStage.Earlyish, earlyish);
                TimingManager.FixedUpdateRemove (TimingManager.TimingStage.Normal, normal);
                TimingManager.FixedUpdateRemove (TimingManager.TimingStage.FlightIntegrator, integrator);
                TimingManager.FixedUpdateRemove (TimingManager.TimingStage.Late, late);
                TimingManager.FixedUpdateRemove (TimingManager.TimingStage.BetterLateThanNever, lateNever);
            } catch { }
            try {
                if (hooked != null)
                    hooked.OnFlyByWire -= flyByWire;
            } catch { }
            hooked = null;
        }

        static void OnFlyByWire (FlightCtrlState state)
        {
            Stamp (StageFlyByWire);
        }

        /// <summary>One row per stage call; a new Time.fixedTime opens a new physics step.</summary>
        internal static void Stamp (int stage)
        {
            if (!armed)
                return;
            try {
                float fixedTime = Time.fixedTime;
                if (!(fixedTime == lastFixedTime)) {
                    if (!float.IsNaN (lastFixedTime)) {
                        framesLeft--;
                        if (framesLeft <= 0) {
                            Stop ();
                            return;
                        }
                        frameIndex++;
                    }
                    lastFixedTime = fixedTime;
                    sequenceInFrame = 0;
                }
                if (rows.Count >= MaxRows * RowStride)
                    return;
                var vessel = FlightGlobals.ActiveVessel;
                double pitch = double.NaN, gimbalX = double.NaN, thrust = double.NaN,
                    omegaX = double.NaN, rcsSum = double.NaN;
                if (vessel != null && vessel.loaded) {
                    if (vessel.ctrlState != null)
                        pitch = vessel.ctrlState.pitch;
                    omegaX = vessel.angularVelocity.x;
                    FirstActuators (vessel, out gimbalX, out thrust, out rcsSum);
                }
                rows.Add (frameIndex);
                rows.Add (sequenceInFrame++);
                rows.Add (stage);
                rows.Add (fixedTime);
                rows.Add (Planetarium.GetUniversalTime ());
                rows.Add (Actuation.CurrentPhysicsTick);
                rows.Add (Time.frameCount);
                rows.Add (pitch);
                rows.Add (gimbalX);
                rows.Add (thrust);
                rows.Add (omegaX);
                rows.Add (rcsSum);
                rows.Add (Time.realtimeSinceStartup);
            } catch {
                // Pure observability.
            }
        }

        static void FirstActuators (Vessel vessel, out double gimbalX, out double thrust,
            out double rcsSum)
        {
            gimbalX = double.NaN;
            thrust = double.NaN;
            rcsSum = double.NaN;
            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                if (part == null)
                    continue;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var module = part.Modules [m];
                    if (double.IsNaN (gimbalX)) {
                        var gimbal = module as ModuleGimbal;
                        if (gimbal != null)
                            gimbalX = gimbal.actuationLocal.x;
                    }
                    if (double.IsNaN (thrust)) {
                        var engine = module as ModuleEngines;
                        if (engine != null)
                            thrust = engine.finalThrust;
                    }
                    if (double.IsNaN (rcsSum)) {
                        var rcs = module as ModuleRCS;
                        if (rcs != null && rcs.thrustForces != null) {
                            double sum = 0.0;
                            for (int i = 0; i < rcs.thrustForces.Length; i++)
                                sum += rcs.thrustForces [i];
                            rcsSum = sum;
                        }
                    }
                }
                if (!double.IsNaN (gimbalX) && !double.IsNaN (thrust) && !double.IsNaN (rcsSum))
                    return;
            }
        }

        internal static IList<double> Read ()
        {
            var output = new List<double> (4 + rows.Count) {
                1.0,
                armed ? 1.0 : 0.0,
                rows.Count / RowStride,
                RowStride
            };
            output.AddRange (rows);
            return output;
        }
    }
}
