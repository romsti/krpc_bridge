using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using UnityEngine;
using SCReferenceFrame = KRPC.SpaceCenter.Services.ReferenceFrame;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// AttitudeV1, lateral force (chantier 19 suite): differential thrust as two motif
    /// columns of the attitude allocation, and translation by a common gimbal deflection
    /// whose torque the differential cancels in advance. The maths is AttitudeTranslation.cs
    /// (pure); this file reads the vessel, feeds the law's input and commands the actuators.
    ///
    /// BORN CLOSED. Nothing here runs until the client calls AttitudeTranslationV1 for an
    /// armed vessel; a vessel not armed on pitch/yaw, a pitch/yaw channel without permission
    /// or with an old reference, or a request past its validity (after the wind-down) leave
    /// TickEntry exactly as in c03d5de: same columns, same bounds, same writes.
    /// </summary>
    internal static partial class AttitudeV1
    {
        internal const int KindDiffMotif = 4;
        internal const int TranslationProtocolVersion = 1;
        // Schema 2 (refactor R3): the frame ends with the clock section (TranslationClockStride).
        internal const int TranslationSchemaVersion = 2;
        // The trailing clock section: the GameClock master tick of the step the frame is of.
        internal const int TranslationClockStride = 1;
        internal const int TranslationHeaderStride = 56;
        internal const int TranslationGimbalRowStride = 6;
        internal const int TranslationEngineRowStride = 10;
        // Capability bit of the law frame header (offset 23), option 256 only: the free authority was
        // computed this step.
        internal const long CapTranslationFree = 1L << 19;

        internal sealed class TranslationEntry
        {
            // Request, as the client last sent it.
            internal bool HaveRequest;
            internal int Sequence;
            internal int Mode;
            internal SCReferenceFrame Frame;
            internal double Ax, Ay, Az;
            internal double UtStamp, UtValidUntil;
            internal int AcceptedCount;
            internal readonly TranslationParams Params = new TranslationParams ();

            // Layer state.
            internal bool Engaged;
            internal bool WindDown;
            internal double WindDownStart;
            internal int WindDownSteps;
            internal bool Planned;          // motif columns in THIS tick's allocation
            internal bool Translate;        // a common deflection is computed this tick
            internal bool AttitudeShare;    // the motifs carry an attitude share this tick
            internal bool MissedThrottle;

            // Engines of the motifs (this tick), reused buffers.
            internal readonly List<ModuleEngines> Engines = new List<ModuleEngines> ();
            internal int Count;
            internal double[] Slope = new double[0];
            internal double[] LeverX = new double[0];
            internal double[] LeverZ = new double[0];
            internal double[] MotifP = new double[0];
            internal double[] MotifY = new double[0];
            internal double[] TorquePerNewton = new double[0];
            internal double[] Direction = new double[0];     // thrust direction per engine (body), computed levers
            internal double[] TauReal = new double[0];
            internal double[] Delta = new double[0];
            internal double[] Margin = new double[0];
            internal uint[] FlightIds = new uint[0];
            internal readonly double[] ColumnP = new double[3];
            internal readonly double[] ColumnY = new double[3];
            internal double MainThrottle, MarginAll;
            internal double DRealP, DRealY, DffP, DffY, DffPrevP, DffPrevY;
            internal double HalfP, HalfY, Share, FfScale = 1.0, HalfSymmetric;
            internal double DCmdP = double.NaN, DCmdY = double.NaN;

            // Per gimbal column (2 per gimbal module, the order of the law's columns).
            internal double[] Common = new double[0];
            internal double[] CommonRealized = new double[0];
            internal double[] CommonRealizedNext = new double[0];
            // Per gimbal module: usable, ramp factor, body axes and force columns (3 each).
            internal bool[] GimbalUsable = new bool[0];
            internal double[] RampFactor = new double[0];
            internal double[] AxisX = new double[0];
            internal double[] AxisY = new double[0];
            internal double[] ForceX = new double[0];
            internal double[] ForceY = new double[0];

            // Outputs (last tick), for the RPC return and the ring.
            internal int State;
            internal long Reasons;
            internal double LateralAuthority, DiffAuthority, FullCommonTorque, RangeDeg;
            internal double FractionCommanded, FractionRealized;
            internal double ThrustN, MassKg;
            internal readonly double[] DemandBody = new double[3];
            internal readonly double[] UpBody = new double[3];
            internal readonly double[] Gap = new double[3];
            internal double WantX, WantZ, CmdX, CmdZ, RealX, RealZ, PhiX, PhiZ;
            internal double LimitScale = 1.0;
            internal readonly double[] FfTorque = new double[3];
            internal readonly double[] TiltMatrix = new double[4];
            internal long LastTick = -1;
            internal double LastUt = double.NaN;

            // Throttle write, decided at Earlyish, done in the fly-by-wire callback.
            internal bool WriteThrottle;
            internal long WriteThrottleTick = -1;
            internal long FbwThrottleTick = -1;
            internal double FbwMainThrottle = double.NaN, FbwKappa = double.NaN;
            internal double[] WrittenPct = new double[0];
            internal long ThrottleWrites;
            internal readonly HashSet<ModuleEngines> Leased = new HashSet<ModuleEngines> ();

            // Core v2: the preallocated frame ring.
            internal readonly Recorder History = new Recorder (HistoryCapacity);

            // Computed levers (bit 4, modes 5 and 7). Never set by modes 0, 1 and 3.
            internal bool Levers;               // this tick: computed lever criterion, RCS lever, joint allocation
            internal bool EngagedComputed;      // the live request that engaged the layer had bit 4 (kept through the wind-down)
            internal double[] LeverY = new double[0];
            internal double MotifLeverP, MotifLeverY, LeverCriterion;
            internal readonly JointTranslation Joint = new JointTranslation ();
            // RCS translation columns (body): [0..2] per unit of ctrlState.X, [3..5] of ctrlState.Y.
            internal readonly double[] RcsFPos = new double[6], RcsFNeg = new double[6];
            internal readonly double[] RcsTPos = new double[6], RcsTNeg = new double[6];
            internal double RcsCapacity;        // N, smallest along-axis force of the four commands +-1
            internal double AuthorityDiffGimbal;// m/s^2, the formula of modes 1/3 (differential + common tilt)
            internal double RcsCmdX, RcsCmdY;   // the joint solution's ctrlState.X / .Y
            internal double RcsWriteX, RcsWriteY, RcsKappa;   // what this step writes: r x realized share
            internal double FfResidual;         // N.m, torque left after the feed-forward
            // X/Y write, decided at Earlyish, done in the fly-by-wire callback.
            internal bool WriteRcs;
            internal long WriteRcsTick = -1;
            internal bool RcsWritten;           // our callback wrote X/Y at its last call
            internal float RcsWrittenX, RcsWrittenY;
            internal long RcsWrites;
            // A bit-4 step committed a common deflection: while the law does not hold the gimbals
            // (fallback: stock Lerp), CommonRealized follows them (ReleasedCommonFollows).
            internal bool CommonComputed;
            internal long RealizedFollowTick = -1;

            // Honest translation (bits 16 and 32, on top of bit 4). Never set by modes 0-7.
            internal bool Headroom;             // this tick: RCS headroom (bit 16)
            internal bool Measured;             // this tick: measured authority (bit 32)
            internal int EngagedHonest;         // bits 16/32 of the live request that engaged the layer
            internal double StepDt;             // this step's physics dt (set before the columns)
            internal readonly double[] Served = new double[3];   // ctrlState pitch, roll, yaw as found
            internal readonly Dictionary<ModuleRCS, double[]> RcsHold = new Dictionary<ModuleRCS, double[]> ();
            internal double RcsShareX = double.NaN, RcsShareY = double.NaN;   // weaker / stronger sign kept
            internal double RcsHeldMax = double.NaN;     // largest held rotation fraction this step
            internal double RcsCapacityFull = double.NaN;// the capacity without the headroom (N)
            internal double ShareNum, ShareDen;          // forgetting means of the measured share
            internal double MeasuredShare = double.NaN;  // num / den once evidence, else NaN
            internal double ModelAuthority = double.NaN; // the model's authority before the factor
            internal long CommitTick = -1;               // last step whose translation the law wrote
            // (AttitudeWrite.OptionEvidenceAging, born closed): game time of the last update
            // of the means and of the last sample, and the named evidence block of the last step.
            internal double ShareUt = double.NaN;
            internal double LastSampleUt = double.NaN;
            internal double EvModel = double.NaN, EvShare = double.NaN, EvEvidence = double.NaN;
            internal double EvEvidenceMin = double.NaN, EvAge = double.NaN, EvFactorRaw = double.NaN;
            internal double EvFactor = double.NaN, EvServable = 0.0, EvUt = double.NaN;
            internal long EvTick = -1;
            // Bit 16: the law's RCS pitch / yaw left out this step (ModuleRCS's combined rotation
            // nonlinear at the served point), the roll rows of the gimbals and motifs dropped (the
            // law does not write the roll), and the per-module nozzles of the RCS (scratch).
            internal bool RcsPitchNonlinear, RcsYawNonlinear, RollRowsDropped;
            internal double RollHeld;           // |ctrlState.roll| served, held RcsHoldSeconds
            internal int NzModules;
            internal int[] NzCounts = new int[0];
            internal double[][] NzLever = new double[0][], NzDir = new double[0][], NzPower = new double[0][];
            internal double[] NzEnable = new double[0];
            internal double[] NzRho = new double[0];
            internal readonly double[] NzScratch = new double[3];

            // (AttitudeWrite.OptionFreeAuthority, born closed): the free authority of the last step, its
            // scratch joint allocation and the realized attitude parts it read (2 per gimbal), with their boxes.
            internal readonly JointTranslation JointFree = new JointTranslation ();
            internal double[] FreeAttitude = new double[0];
            internal double[] FreeUMin = new double[0];
            internal double[] FreeUMax = new double[0];
            internal bool FreeComputed;
            internal int FreeSource;
            internal double FreeAuthority = double.NaN, FreeRatio = double.NaN, FreeModel = double.NaN;
            internal double FreeAlongModel = double.NaN, FreeAlongFree = double.NaN;
            internal double FreeDirX = double.NaN, FreeDirZ = double.NaN;
            internal double FreeGimbalAttitude = double.NaN, FreeDiffAttitude = double.NaN;
        }

        // ---------------------------------------------------------------- RPC bodies

        /// <summary>
        /// Stores the lateral-force request; returns the state of the LAST physics step (see
        /// docs/API.md). mode 0 stop, 1 differential thrust in the attitude allocation, 3
        /// differential + translation by common gimbal deflection (2 alone is refused: the
        /// translation needs the differential to cancel its torque). Bit 4 (born closed),
        /// "computed levers": 5 = 1 with the computed lever criterion and the honest authority,
        /// 7 = 3 with the RCS translation and the joint allocation (return layout 2).
        /// </summary>
        internal static IList<double> Translation (string token, int sequence,
            SCReferenceFrame frame, double ax, double ay, double az, double utStamp,
            double utValidUntil, int mode)
        {
            Entry entry;
            int accepted = StoreRequest (token, sequence, frame, ax, ay, az, utStamp, utValidUntil,
                mode, out entry);
            return TranslationReturnWithEvidence (entry, accepted);
        }

        /// <summary>Protocol of the fixed return of AttitudeTranslationV2 (refactor R1).</summary>
        internal const int TranslationReturnV2Protocol = 1;

        /// <summary>
        /// Refactor R1 (plan 2.5): the same request as Translation, and the return in ONE layout
        /// whatever the mode and the options (attitude_translation_v2): the 32 fields of layout 3
        /// (a field the mode does not fill is NaN; offset 4 still says which part it fills), then
        /// the evidence block [protocol, schema, n, n values] (NaN values without option 2).
        /// </summary>
        internal static IList<double> TranslationV2 (string token, int sequence,
            SCReferenceFrame frame, double ax, double ay, double az, double utStamp,
            double utValidUntil, int mode)
        {
            Entry entry;
            int accepted = StoreRequest (token, sequence, frame, ax, ay, az, utStamp, utValidUntil,
                mode, out entry);
            return TranslationReturnFixed (entry, accepted);
        }

        /// <summary>The validation and storage of a lateral-force request (1 accepted, 0 older).</summary>
        static int StoreRequest (string token, int sequence, SCReferenceFrame frame, double ax,
            double ay, double az, double utStamp, double utValidUntil, int mode, out Entry entry)
        {
            entry = RequireToken (token);
            if (frame == null)
                throw new ArgumentNullException (nameof (frame));
            if (!AttitudeTranslation.ValidMode (mode))
                throw new ArgumentOutOfRangeException (nameof (mode), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode,
                    "0 arret, 1 poussee differentielle, 3 differentielle + translation " +
                    "(2 seul refuse : la translation exige la differentielle), 5 et 7 : 1 et 3 " +
                    "avec leviers calcules (bit 4) ; bits 16 / 32 (translation honnete) sur 5 ou 7 " +
                    "seulement"));
            if (!Finite (ax) || !Finite (ay) || !Finite (az) || !Finite (utStamp) ||
                    !Finite (utValidUntil) || utValidUntil < utStamp)
                throw new ArgumentOutOfRangeException ("demande",
                    "valeurs non finies ou validite anterieure");
            var t = entry.T;
            int accepted = 0;
            if (sequence > t.Sequence) {
                t.Sequence = sequence;
                t.Frame = frame;
                t.Ax = ax;
                t.Ay = ay;
                t.Az = az;
                t.UtStamp = utStamp;
                t.UtValidUntil = utValidUntil;
                t.Mode = mode;
                t.HaveRequest = true;
                t.AcceptedCount++;
                accepted = 1;
            }
            return accepted;
        }

        /// <summary>The fixed return of AttitudeTranslationV2 (see TranslationV2).</summary>
        static IList<double> TranslationReturnFixed (Entry e, int accepted)
        {
            var reply = TranslationReturn (e, accepted);
            while (reply.Count < AttitudeTranslation.ReturnLengthHonest)
                reply.Add (double.NaN);
            var t = e.T;
            bool evidence = (e.Options & AttitudeWrite.OptionEvidenceAging) != 0;
            reply.Add (AttitudeTranslation.EvidenceBlockProtocol);
            reply.Add (AttitudeTranslation.EvidenceBlockSchema);
            reply.Add (AttitudeTranslation.EvidenceFields.Length);
            reply.Add (evidence ? t.EvModel : double.NaN);
            reply.Add (evidence ? t.EvShare : double.NaN);
            reply.Add (evidence ? t.EvEvidence : double.NaN);
            reply.Add (evidence ? t.EvEvidenceMin : double.NaN);
            reply.Add (evidence ? t.EvAge : double.NaN);
            reply.Add (evidence ? t.EvFactorRaw : double.NaN);
            reply.Add (evidence ? t.EvFactor : double.NaN);
            reply.Add (evidence ? t.EvServable : double.NaN);
            reply.Add (evidence ? t.Params.ShareTauSeconds : double.NaN);
            reply.Add (evidence ? t.EvTick : double.NaN);
            reply.Add (evidence ? t.EvUt : double.NaN);
            return reply;
        }

        /// <summary>Sets the tunables of the lateral-force layer of one armed vessel.</summary>
        internal static IList<double> TranslationConfig (string token, double fracGimbal,
            double fracDiff, double diffPriority)
        {
            var entry = RequireToken (token);
            if (!Finite (fracGimbal) || fracGimbal < 0.0 || fracGimbal > 0.95)
                throw new ArgumentOutOfRangeException (nameof (fracGimbal), "0 a 0.95");
            if (!Finite (fracDiff) || fracDiff < 0.0 || fracDiff > 0.95)
                throw new ArgumentOutOfRangeException (nameof (fracDiff), "0 a 0.95");
            if (!Finite (diffPriority) || diffPriority < 0.1 || diffPriority > 10.0)
                throw new ArgumentOutOfRangeException (nameof (diffPriority), "0.1 a 10");
            var p = entry.T.Params;
            p.FracGimbal = fracGimbal;
            p.FracDiff = fracDiff;
            p.DiffPriority = diffPriority;
            return new List<double> {
                p.FracGimbal, p.FracDiff, p.DiffPriority, p.MinLeverArmM, p.MinThrottleMargin,
                p.MinProjectionDet, p.WindDownDoneDegrees, p.WindDownMaxSeconds,
                Actuation.AttitudeThrottleLeaseSeconds
            };
        }

        internal static IList<double> ReadTranslationFrames (string persistentId, int sinceTick,
            int maxFrames)
        {
            uint pid = VesselLookup.ParsePersistentId (persistentId);
            Entry entry;
            var history = entries.TryGetValue (pid, out entry) ? entry.T.History : emptyRing;
            return history.ReadSince (TranslationProtocolVersion, TranslationSchemaVersion,
                sinceTick, maxFrames);
        }

        static IList<double> TranslationReturn (Entry e, int accepted)
        {
            var t = e.T;
            // Available = the motifs were in the allocation AND the pitch/yaw channel wrote.
            bool on = (t.State == AttitudeTranslation.StateDiff ||
                t.State == AttitudeTranslation.StateTranslate) &&
                (t.Reasons & AttitudeTranslation.ReasonNotWriting) == 0;
            bool computed = AttitudeTranslation.Computed (t.Mode);
            var reply = new List<double> {
                accepted,
                on ? t.LateralAuthority : 0.0,
                on ? t.DiffAuthority : 0.0,
                t.FractionRealized,
                computed ? AttitudeTranslation.ReturnVersionComputed : AttitudeTranslation.ReturnVersion,
                t.State,
                t.Reasons,
                Math.Sqrt (t.WantX * t.WantX + t.WantZ * t.WantZ),
                Math.Sqrt (t.CmdX * t.CmdX + t.CmdZ * t.CmdZ),
                Math.Sqrt (t.RealX * t.RealX + t.RealZ * t.RealZ),
                t.FullCommonTorque,
                Math.Sqrt (t.FfTorque [0] * t.FfTorque [0] + t.FfTorque [2] * t.FfTorque [2]),
                t.Count,
                t.MainThrottle,
                t.LastTick,
                t.LastUt,
                t.ThrottleWrites
            };
            if (!computed)
                return reply;
            // Layout 2 (stored mode with bit 4): 8 values appended, the 17 above unchanged.
            reply.Add (on ? t.AuthorityDiffGimbal : 0.0);         // 17
            reply.Add (on ? t.LateralAuthority : 0.0);            // 18 (= offset 1)
            reply.Add (t.RcsWriteX);                              // 19
            reply.Add (t.RcsWriteY);                              // 20
            reply.Add (t.RcsCapacity);                            // 21
            reply.Add (Math.Min (t.MotifLeverP, t.MotifLeverY));  // 22
            reply.Add (t.FfResidual);                             // 23
            reply.Add (t.Params.FracRcs);                         // 24
            if (!AttitudeTranslation.Honest (t.Mode))
                return reply;
            // Layout 3 (stored mode with bit 16 or 32): 7 values appended, the 25 above unchanged
            // (offset 4 says 3).
            reply [4] = AttitudeTranslation.ReturnVersionHonest;
            reply.Add (on ? t.ModelAuthority : 0.0);              // 25 the model's authority (bit 32: before the factor)
            reply.Add (t.MeasuredShare);                          // 26 measured share of the common (NaN: no evidence)
            reply.Add (t.ShareDen);                               // 27 its evidence (deg^2, forgetting mean of sum c^2)
            reply.Add (t.RcsCapacityFull);                        // 28 RCS capacity without the headroom (N)
            reply.Add (Math.Min (t.RcsShareX, t.RcsShareY));      // 29 weaker / stronger sign kept (bit 16)
            reply.Add (t.RcsHeldMax);                             // 30 largest held rotation fraction (bit 16)
            reply.Add ((t.RcsPitchNonlinear ? 1 : 0) + (t.RcsYawNonlinear ? 2 : 0) +
                (t.RollRowsDropped ? 4 : 0));                      // 31 bit 16: law's RCS pitch / yaw left out, roll rows dropped
            return reply;
        }

        /// <summary>
        /// (AttitudeWrite.OptionEvidenceAging, born closed): the return of the mode's layout,
        /// padded with NaN to ReturnLengthHonest, then the named evidence block [protocol, schema,
        /// n, n values] (AttitudeTranslation.EvidenceFields), of the last step. an earlier change
        /// (AttitudeWrite.OptionFreeAuthority, born closed): the evidence block is then always there (its
        /// values NaN without option 2), followed by the named free block (AttitudeTranslation.FreeFields).
        /// Without either option, TranslationReturn unchanged.
        /// </summary>
        static IList<double> TranslationReturnWithEvidence (Entry e, int accepted)
        {
            var reply = TranslationReturn (e, accepted);
            bool evidence = (e.Options & AttitudeWrite.OptionEvidenceAging) != 0;
            bool free = (e.Options & AttitudeWrite.OptionFreeAuthority) != 0;
            if (!evidence && !free)
                return reply;
            var t = e.T;
            while (reply.Count < AttitudeTranslation.ReturnLengthHonest)
                reply.Add (double.NaN);
            reply.Add (AttitudeTranslation.EvidenceBlockProtocol);
            reply.Add (AttitudeTranslation.EvidenceBlockSchema);
            reply.Add (AttitudeTranslation.EvidenceFields.Length);
            reply.Add (evidence ? t.EvModel : double.NaN);             // model_authority_ms2
            reply.Add (evidence ? t.EvShare : double.NaN);             // measured_share
            reply.Add (evidence ? t.EvEvidence : double.NaN);          // evidence_deg2
            reply.Add (evidence ? t.EvEvidenceMin : double.NaN);       // evidence_min_deg2
            reply.Add (evidence ? t.EvAge : double.NaN);               // evidence_age_s
            reply.Add (evidence ? t.EvFactorRaw : double.NaN);         // factor_unaged
            reply.Add (evidence ? t.EvFactor : double.NaN);            // factor
            reply.Add (evidence ? t.EvServable : double.NaN);          // servable_authority_ms2
            reply.Add (evidence ? t.Params.ShareTauSeconds : double.NaN);   // aging_tau_s
            reply.Add (evidence ? t.EvTick : double.NaN);              // attitude_tick
            reply.Add (evidence ? t.EvUt : double.NaN);                // ut
            if (!free)
                return reply;
            bool on = (t.State == AttitudeTranslation.StateDiff ||
                t.State == AttitudeTranslation.StateTranslate) &&
                (t.Reasons & AttitudeTranslation.ReasonNotWriting) == 0;
            reply.Add (AttitudeTranslation.FreeBlockProtocol);
            reply.Add (AttitudeTranslation.FreeBlockSchema);
            reply.Add (AttitudeTranslation.FreeFields.Length);
            reply.Add (t.FreeComputed ? (on ? t.FreeAuthority : 0.0) : double.NaN);   // free_authority_ms2
            reply.Add (t.FreeComputed ? t.FreeRatio : double.NaN);   // ratio
            reply.Add (t.FreeModel);           // model_authority_ms2
            reply.Add (t.FreeAlongModel);      // along_model_ms2
            reply.Add (t.FreeAlongFree);       // along_free_ms2
            reply.Add (t.FreeDirX);            // direction_x
            reply.Add (t.FreeDirZ);            // direction_z
            reply.Add (t.FreeSource);          // source
            reply.Add (t.FreeGimbalAttitude);  // gimbal_attitude_max
            reply.Add (t.FreeDiffAttitude);    // diff_attitude_max
            reply.Add (t.LastTick);            // attitude_tick
            reply.Add (t.LastUt);              // ut
            return reply;
        }

        /// <summary>
        /// the evidence block of this step, every tick of a vessel armed with the option
        /// (the layer planned or not): the means aged to now (read only; they are updated on the
        /// planned ticks), the age of the last sample, both factors, and the authority offset 1
        /// publishes (the model's times the factor while the layer may act, else 0).
        /// </summary>
        static void UpdateEvidenceBlock (Entry e, double ut, bool planned)
        {
            if ((e.Options & AttitudeWrite.OptionEvidenceAging) == 0)
                return;
            var t = e.T;
            var p = t.Params;
            double tau = p.ShareTauSeconds;
            double denMin = 2 * e.Gimbals.Count * p.ShareMinDegrees * p.ShareMinDegrees;
            double num = t.ShareNum, den = t.ShareDen;
            if (AttitudeTranslation.Finite (t.ShareUt))
                AttitudeTranslation.AgeShare (ref num, ref den, ut - t.ShareUt, tau);
            t.EvEvidence = den;
            t.EvEvidenceMin = denMin;
            t.EvShare = den >= denMin && den > 0.0 ? num / den : double.NaN;
            t.EvAge = AttitudeTranslation.Finite (t.LastSampleUt) ? ut - t.LastSampleUt : double.NaN;
            t.EvFactorRaw = AttitudeTranslation.MeasuredFactor (num, den, denMin);
            t.EvFactor = AttitudeTranslation.AgedFactor (t.EvFactorRaw,
                double.IsNaN (t.EvAge) ? double.PositiveInfinity : t.EvAge, tau);
            if (!planned)
                t.EvModel = double.NaN;
            bool on = (t.State == AttitudeTranslation.StateDiff ||
                t.State == AttitudeTranslation.StateTranslate) &&
                (t.Reasons & AttitudeTranslation.ReasonNotWriting) == 0;
            t.EvServable = on ? t.LateralAuthority : 0.0;
            t.EvTick = ownTick;
            t.EvUt = ut;
        }

        // ------------------------------------------------------------ state keeping

        /// <summary>Re-arming: the request, the tunables and the dynamics go.</summary>
        static void ResetTranslation (Entry e)
        {
            var t = e.T;
            t.CommonComputed = false;       // before the reset: every realized common back to 0
            DisengageTranslation (e);
            t.HaveRequest = false;
            t.Sequence = 0;
            t.Mode = 0;
            t.Frame = null;
            t.Params.Reset ();
            t.State = AttitudeTranslation.StateIdle;
            t.Reasons = 0;
            t.History.Clear ();
            t.RealizedFollowTick = -1;
            // Honest translation: the held rotation and the measured share are of this arming only.
            t.RcsHold.Clear ();
            t.ShareNum = 0.0;
            t.ShareDen = 0.0;
            t.MeasuredShare = double.NaN;
            t.CommitTick = -1;
            t.RollHeld = 0.0;
            // the evidence clock is of this arming only.
            t.ShareUt = double.NaN;
            t.LastSampleUt = double.NaN;
            t.EvModel = t.EvShare = t.EvEvidence = t.EvEvidenceMin = t.EvAge = double.NaN;
            t.EvFactorRaw = t.EvFactor = t.EvUt = double.NaN;
            t.EvServable = 0.0;
            t.EvTick = -1;
            // nothing of the last arming's free authority.
            ClearFreeAuthority (t);
        }

        /// <summary>the free authority of this step not computed (yet).</summary>
        static void ClearFreeAuthority (TranslationEntry t)
        {
            t.FreeComputed = false;
            t.FreeSource = AttitudeTranslation.FreeNotComputed;
            t.FreeAuthority = t.FreeRatio = t.FreeModel = double.NaN;
            t.FreeAlongModel = t.FreeAlongFree = double.NaN;
            t.FreeDirX = t.FreeDirZ = double.NaN;
            t.FreeGimbalAttitude = t.FreeDiffAttitude = double.NaN;
        }

        /// <summary>
        /// Offsets, feed-forward and write decision back to zero. Called whenever the actuator
        /// authority is handed back (every lease, throttles included, is then restored).
        /// Computed levers (a bit-4 step committed a common deflection): the gimbals handed to
        /// the stock Lerp still hold that deflection and leave it one Lerp step per physics step,
        /// so CommonRealized follows them (AttitudeTranslation.ReleasedCommonFollows, once per
        /// step) instead of dropping to 0: otherwise the next writing step counts it as the law's
        /// ATTITUDE part (u0 = realized - realized common) and, in the launch clamps, keeps it.
        /// </summary>
        static void ResetTranslationDynamics (Entry e)
        {
            var t = e.T;
            if (t.CommonComputed) {
                if (t.RealizedFollowTick != ownTick) {
                    t.RealizedFollowTick = ownTick;
                    AttitudeTranslation.ReleasedCommonFollows (t.CommonRealized, t.RampFactor);
                }
                for (int j = 0; j < t.Common.Length; j++) {
                    t.Common [j] = 0.0;
                    t.CommonRealizedNext [j] = 0.0;
                }
            } else {
                for (int j = 0; j < t.Common.Length; j++) {
                    t.Common [j] = 0.0;
                    t.CommonRealized [j] = 0.0;
                    t.CommonRealizedNext [j] = 0.0;
                }
            }
            t.DffP = 0.0;
            t.DffY = 0.0;
            t.DffPrevP = 0.0;
            t.DffPrevY = 0.0;
            t.DCmdP = double.NaN;
            t.DCmdY = double.NaN;
            t.WriteThrottle = false;
            t.Leased.Clear ();
            // Computed levers: no X/Y write any more (the callback releases what it wrote).
            t.RcsCmdX = t.RcsCmdY = 0.0;
            t.RcsWriteX = t.RcsWriteY = 0.0;
            t.WriteRcs = false;
        }

        /// <summary>
        /// Computed levers: the law writes the gimbals with the layer off, its u0 is their whole
        /// realized deflection (its own attitude part): the followed common ends at 0, as without
        /// bit 4.
        /// </summary>
        static void ClearReleasedCommon (Entry e)
        {
            var t = e.T;
            t.CommonComputed = false;
            for (int j = 0; j < t.CommonRealized.Length; j++)
                t.CommonRealized [j] = 0.0;
        }

        /// <summary>Gives the throttles back (original independent-throttle fields) and resets.</summary>
        static void DisengageTranslation (Entry e)
        {
            ReleaseDiffThrottles (e);
            ResetTranslationDynamics (e);
            var t = e.T;
            t.Engaged = false;
            t.WindDown = false;
            t.WindDownSteps = 0;
            t.Planned = false;
            t.Translate = false;
            t.AttitudeShare = false;
            t.EngagedComputed = false;
            t.Levers = false;
            t.Headroom = false;
            t.Measured = false;
            t.EngagedHonest = 0;
        }

        static void ReleaseDiffThrottles (Entry e)
        {
            var t = e.T;
            if (t.Leased.Count == 0)
                return;
            // Without the authority every lease was already restored when it was lost
            // (ClearController restores), and another owner may lease these engines now.
            if (!Actuation.HoldsForAttitude (e.AuthorityToken)) {
                t.Leased.Clear ();
                return;
            }
            foreach (var engine in t.Leased) {
                try {
                    // The active vessel's authority : ReleaseThrottle, as before ; the
                    // leases of a non-active vessel's own authority.
                    if (engine != null)
                        Actuation.ReleaseThrottleForAttitude (e.AuthorityToken, engine);
                } catch (Exception exc) {
                    BridgeLog.Error ("AttitudeV1: rendu des manettes: " + exc.Message);
                }
            }
            t.Leased.Clear ();
        }

        static void EnsureTranslationSizes (Entry e)
        {
            var t = e.T;
            int g = e.Gimbals.Count;
            if (t.GimbalUsable.Length != g) {
                t.GimbalUsable = new bool[g];
                t.RampFactor = new double[g];
                t.AxisX = new double[3 * g];
                t.AxisY = new double[3 * g];
                t.ForceX = new double[3 * g];
                t.ForceY = new double[3 * g];
            }
            t.Joint.Resize (g);
            if (t.Common.Length != 2 * g) {
                t.Common = new double[2 * g];
                t.CommonRealized = new double[2 * g];
                t.CommonRealizedNext = new double[2 * g];
            }
            if (t.FreeAttitude.Length != 2 * g) {
                t.FreeAttitude = new double[2 * g];
                t.FreeUMin = new double[2 * g];
                t.FreeUMax = new double[2 * g];
            }
            int n = e.Engines.Count;
            if (t.Slope.Length < n) {
                t.Slope = new double[n];
                t.LeverX = new double[n];
                t.LeverY = new double[n];
                t.LeverZ = new double[n];
                t.MotifP = new double[n];
                t.MotifY = new double[n];
                t.TorquePerNewton = new double[3 * n];
                t.Direction = new double[3 * n];
                t.TauReal = new double[n];
                t.Delta = new double[n];
                t.Margin = new double[n];
                t.FlightIds = new uint[n];
                t.WrittenPct = new double[n];
                for (int i = 0; i < n; i++)
                    t.WrittenPct [i] = double.NaN;
            }
        }

        // ------------------------------------------------------------------ the tick

        /// <summary>
        /// Before the law's input is sized: decides whether the two motif columns enter this
        /// tick's allocation, and prepares the engines' motifs, columns and realized
        /// amplitudes. False (nothing added) in every case c03d5de must see.
        /// </summary>
        static bool PlanTranslation (Entry e, Vessel vessel, Transform reference, Vector3 com,
            FlightCtrlState ctrl, int stage, bool warp)
        {
            var t = e.T;
            t.Planned = false;
            t.Translate = false;
            t.AttitudeShare = false;
            t.Levers = false;
            t.Headroom = false;
            t.Measured = false;
            if (!t.HaveRequest && !t.Engaged) {
                // Born closed: never requested since the arming.
                t.State = AttitudeTranslation.StateIdle;
                t.Reasons = AttitudeTranslation.ReasonNoRequest;
                return false;
            }
            double ut = Planetarium.GetUniversalTime ();
            long reasons = t.MissedThrottle ? AttitudeTranslation.ReasonThrottleWriteMissed : 0L;
            t.MissedThrottle = false;
            ClearTranslationOutputs (t);
            t.LastTick = ownTick;
            t.LastUt = ut;

            bool armedPitchYaw = e.Mode == ModeArmed && (e.AxesMask & AttitudeWrite.ArmPitchYaw) != 0;
            double age = e.HaveReference ? ut - e.UtStamp : double.PositiveInfinity;
            bool permitted = armedPitchYaw && (e.WriteMask & AttitudeWrite.ArmPitchYaw) != 0 &&
                age <= AttitudeWrite.PitchYawMaxReferenceAgeS && stage == StageEarlyish && !warp;
            bool live = AttitudeTranslation.RequestLive (t.HaveRequest, t.Mode, ut, t.UtValidUntil);
            if (!armedPitchYaw)
                reasons |= AttitudeTranslation.ReasonNotArmedPitchYaw;
            else if (!permitted)
                reasons |= AttitudeTranslation.ReasonPitchYawNotPermitted;
            if (t.HaveRequest && t.Mode == 0)
                reasons |= AttitudeTranslation.ReasonStopRequested;
            else if (t.HaveRequest && !live)
                reasons |= AttitudeTranslation.ReasonStale;
            // a non-active vessel only with AttitudeArmV2 option 3 (option 0 : the active
            // vessel only, as before).
            if (!permitted || !AttitudeWrite.TranslationAllowed (
                    ReferenceEquals (vessel, FlightGlobals.ActiveVessel), e.InactiveOptions)) {
                if (permitted)
                    reasons |= AttitudeTranslation.ReasonNotActiveVessel;
                DisengageTranslation (e);
                t.State = AttitudeTranslation.StateIdle;
                t.Reasons = reasons;
                return false;
            }
            if (live) {
                t.Engaged = true;
                t.WindDown = false;
                t.WindDownSteps = 0;
                t.EngagedComputed = AttitudeTranslation.Computed (t.Mode);
                t.EngagedHonest = t.Mode & AttitudeTranslation.BitsHonest;
            } else if (t.Engaged) {
                if (!t.WindDown) {
                    t.WindDown = true;
                    t.WindDownStart = ut;
                    t.WindDownSteps = 0;
                } else {
                    double maxCommon = 0.0;
                    for (int j = 0; j < t.CommonRealized.Length; j++)
                        maxCommon = Math.Max (maxCommon, Math.Abs (t.CommonRealized [j]));
                    bool timeout;
                    if (AttitudeTranslation.WindDownDone (maxCommon, ut - t.WindDownStart,
                            t.WindDownSteps, t.Params, out timeout)) {
                        if (timeout)
                            reasons |= AttitudeTranslation.ReasonWindDownTimeout;
                        DisengageTranslation (e);
                        t.State = AttitudeTranslation.StateIdle;
                        t.Reasons = reasons;
                        return false;
                    }
                }
            }
            if (!t.Engaged) {
                t.State = AttitudeTranslation.StateIdle;
                t.Reasons = reasons;
                return false;
            }

            EnsureTranslationSizes (e);
            CollectMotifEngines (e, com, reference);
            t.MainThrottle = ctrl == null ? 0.0 : ctrl.mainThrottle;
            var p = t.Params;
            // Computed levers (bit 4): the live request's mode, or, winding down, the mode that
            // engaged the layer (a stop after mode 7 winds down with the same criterion).
            t.Levers = live ? AttitudeTranslation.Computed (t.Mode) : t.EngagedComputed;
            // Honest translation (bits 16/32, born closed): with the computed levers only, from the
            // live request or, winding down, the request that engaged the layer.
            int honest = live ? (t.Mode & AttitudeTranslation.BitsHonest) : t.EngagedHonest;
            t.Headroom = t.Levers && (honest & AttitudeTranslation.BitRcsHeadroom) != 0;
            t.Measured = t.Levers && (honest & AttitudeTranslation.BitMeasured) != 0;
            double minLever = p.MinLeverArmM;
            if (t.Levers) {
                minLever = AttitudeTranslation.LeverResolution (t.Count, t.LeverX, t.LeverY, t.LeverZ);
                // The motif columns along the engines' COMMON thrust direction, not each one's
                // tilted nozzles (third ground check: 0.41 m per degree of gimbal against the
                // heavy booster's 0.133 m motif lever, the throttles chattered period 2).
                AttitudeTranslation.CommonDirectionTorques (t.Count, t.Slope, t.LeverX, t.LeverY,
                    t.LeverZ, t.Direction, t.TorquePerNewton);
            }
            t.LeverCriterion = minLever;
            bool motifs = AttitudeTranslation.Motifs (t.Count, t.Slope, t.LeverX, t.LeverZ,
                minLever, t.MotifP, t.MotifY, out t.MotifLeverP, out t.MotifLeverY);
            t.MarginAll = AttitudeTranslation.ThrottleMargin (t.MainThrottle, 0.0, 1.0);
            if (motifs) {
                AttitudeTranslation.MotifColumns (t.Count, t.Slope, t.MotifP, t.MotifY,
                    t.TorquePerNewton, t.ColumnP, t.ColumnY);
                for (int i = 0; i < t.Count; i++)
                    t.Margin [i] = t.MarginAll;
                t.HalfSymmetric = AttitudeTranslation.SymmetricHalfWidth (t.Count, t.MotifP,
                    t.MotifY, t.Margin);
                t.DiffAuthority = AttitudeTranslation.Inradius (t.ColumnP, t.ColumnY,
                    t.HalfSymmetric);
            }
            if (!motifs || !(AttitudeTranslation.Inradius (t.ColumnP, t.ColumnY, 1.0) > 0.0)) {
                reasons |= AttitudeTranslation.ReasonNoDiffAuthority;
            } else if (t.MarginAll < p.MinThrottleMargin) {
                reasons |= AttitudeTranslation.ReasonNoThrottleMargin;
            }
            if ((reasons & (AttitudeTranslation.ReasonNoDiffAuthority |
                    AttitudeTranslation.ReasonNoThrottleMargin)) != 0) {
                // Refused cleanly this step: throttles back to the main throttle, no column.
                DisengageTranslation (e);
                t.DiffAuthority = 0.0;
                t.State = AttitudeTranslation.StateUnavailable;
                t.Reasons = reasons;
                return false;
            }
            double dp, dy;
            if (AttitudeTranslation.RealizedAmplitudes (t.Count, t.Slope, t.MotifP, t.MotifY,
                    t.TauReal, out dp, out dy)) {
                t.DRealP = dp;
                t.DRealY = dy;
            } else {
                t.DRealP = 0.0;
                t.DRealY = 0.0;
            }
            t.Planned = true;
            t.Translate = live && (t.Mode & AttitudeTranslation.BitTranslate) != 0;
            t.AttitudeShare = live;
            t.Reasons = reasons;
            t.State = t.WindDown ? AttitudeTranslation.StateWindDown
                : (t.Translate ? AttitudeTranslation.StateTranslate : AttitudeTranslation.StateDiff);
            return true;
        }

        static void ClearTranslationOutputs (TranslationEntry t)
        {
            t.Count = 0;
            t.LateralAuthority = 0.0;
            t.DiffAuthority = 0.0;
            t.FullCommonTorque = 0.0;
            t.RangeDeg = 0.0;
            t.FractionCommanded = 0.0;
            t.FractionRealized = 0.0;
            t.ThrustN = 0.0;
            t.MassKg = 0.0;
            t.WantX = t.WantZ = t.CmdX = t.CmdZ = t.RealX = t.RealZ = 0.0;
            t.PhiX = t.PhiZ = 0.0;
            t.LimitScale = 1.0;
            t.FfScale = 1.0;
            t.HalfP = t.HalfY = t.Share = t.HalfSymmetric = 0.0;
            t.MainThrottle = double.NaN;
            t.MarginAll = 0.0;
            t.MotifLeverP = t.MotifLeverY = t.LeverCriterion = 0.0;
            t.RcsCapacity = 0.0;
            t.AuthorityDiffGimbal = 0.0;
            t.RcsCmdX = t.RcsCmdY = 0.0;
            t.RcsWriteX = t.RcsWriteY = 0.0;
            t.RcsKappa = 0.0;
            t.FfResidual = 0.0;
            t.RcsShareX = t.RcsShareY = double.NaN;
            t.RcsHeldMax = double.NaN;
            t.RcsCapacityFull = double.NaN;
            t.ModelAuthority = double.NaN;
            t.RcsPitchNonlinear = t.RcsYawNonlinear = t.RollRowsDropped = false;
            ClearFreeAuthority (t);
            for (int r = 0; r < 3; r++) {
                t.DemandBody [r] = double.NaN;
                t.UpBody [r] = double.NaN;
                t.Gap [r] = double.NaN;
                t.FfTorque [r] = 0.0;
                t.ColumnP [r] = 0.0;
                t.ColumnY [r] = 0.0;
            }
        }

        /// <summary>
        /// Engines the differential may drive: ignited, operational, not flamed out, throttle
        /// not locked (solids), thrust limiter above 0, and not in an independent throttle
        /// someone else set. Slope dT/d(throttle command) from ModuleEngines.UpdateThrottle
        /// and RequiredPropellantMass: thrust = K currentThrottle, currentThrottle =
        /// Lerp(throttleMin, 1, command x thrustPercentage%) - so slope = K (1 - throttleMin)
        /// tp%, K = finalThrust / currentThrottle (maxThrust when not thrusting).
        /// </summary>
        static void CollectMotifEngines (Entry e, Vector3 com, Transform reference)
        {
            var t = e.T;
            t.Engines.Clear ();
            int count = 0;
            for (int k = 0; k < e.Engines.Count; k++) {
                var engine = e.Engines [k];
                if (engine == null || !engine.EngineIgnited || engine.throttleLocked ||
                        !engine.isOperational || engine.getFlameoutState)
                    continue;
                if (engine.independentThrottle && !Actuation.ThrottleIsLeased (engine))
                    continue;
                double tp = engine.thrustPercentage * 0.01;
                if (!(tp > 0.0))
                    continue;
                Vector3 centroid, torquePerNewton;
                if (!AttitudeGeometry.EngineLever (engine, com, reference, out centroid,
                        out torquePerNewton))
                    continue;
                double tmin = engine.throttleMin;
                if (!(tmin >= 0.0) || !(tmin < 1.0))
                    tmin = 0.0;
                double cur = engine.currentThrottle;
                double perThrottle = cur > 1e-3 && engine.finalThrust > 0f
                    ? engine.finalThrust * 1000.0 / cur : engine.maxThrust * 1000.0;
                double slope = perThrottle * (1.0 - tmin) * tp;
                if (!(slope > 0.0) || double.IsInfinity (slope))
                    continue;
                t.Engines.Add (engine);
                t.Slope [count] = slope;
                t.LeverX [count] = centroid.x;
                t.LeverY [count] = centroid.y;
                t.LeverZ [count] = centroid.z;
                t.TorquePerNewton [3 * count] = torquePerNewton.x;
                t.TorquePerNewton [3 * count + 1] = torquePerNewton.y;
                t.TorquePerNewton [3 * count + 2] = torquePerNewton.z;
                // Computed levers only read it (CommonDirectionTorques); 0 when undefined.
                Vector3 direction;
                bool haveDirection = AttitudeGeometry.EngineDirection (engine, reference, out direction);
                t.Direction [3 * count] = haveDirection ? direction.x : 0.0;
                t.Direction [3 * count + 1] = haveDirection ? direction.y : 0.0;
                t.Direction [3 * count + 2] = haveDirection ? direction.z : 0.0;
                t.TauReal [count] = (cur - tmin) / ((1.0 - tmin) * tp);
                t.FlightIds [count] = engine.part != null ? engine.part.flightID : 0u;
                t.Delta [count] = 0.0;
                count++;
            }
            t.Count = count;
        }

        /// <summary>
        /// Gimbal loop: body axes and force columns of gimbal g (planned ticks only). 
        /// `pivots` not null = a module mapped per pivot (GimbalAxesAndForcePivots, at the module
        /// deflection (dx, dy)); null = V1, unchanged.
        /// </summary>
        static void RecordGimbalGeometry (Entry e, int g, ModuleGimbal gimbal,
            List<ModuleEngines> partEngines, Transform reference, float dx, bool usable,
            double dt, GimbalPivotCommand pivots, float dy)
        {
            var t = e.T;
            t.GimbalUsable [g] = false;
            t.RampFactor [g] = 1.0;
            for (int r = 0; r < 3; r++) {
                t.AxisX [3 * g + r] = 0.0;
                t.AxisY [3 * g + r] = 0.0;
                t.ForceX [3 * g + r] = 0.0;
                t.ForceY [3 * g + r] = 0.0;
            }
            if (gimbal == null || !usable)
                return;
            Vector3 ax, ay, fx, fy;
            if (pivots != null) {
                if (!AttitudeGeometry.GimbalAxesAndForcePivots (gimbal, partEngines, reference,
                        pivots, dx, dy, out ax, out ay, out fx, out fy))
                    return;
            } else if (!AttitudeGeometry.GimbalAxesAndForce (gimbal, partEngines, reference, dx,
                    out ax, out ay, out fx, out fy))
                return;
            t.GimbalUsable [g] = true;
            t.RampFactor [g] = AttitudeTranslation.RampFactor (gimbal.useGimbalResponseSpeed,
                gimbal.gimbalResponseSpeed, dt);
            t.AxisX [3 * g] = ax.x;
            t.AxisX [3 * g + 1] = ax.y;
            t.AxisX [3 * g + 2] = ax.z;
            t.AxisY [3 * g] = ay.x;
            t.AxisY [3 * g + 1] = ay.y;
            t.AxisY [3 * g + 2] = ay.z;
            t.ForceX [3 * g] = fx.x;
            t.ForceX [3 * g + 1] = fx.y;
            t.ForceX [3 * g + 2] = fx.z;
            t.ForceY [3 * g] = fy.x;
            t.ForceY [3 * g + 1] = fy.y;
            t.ForceY [3 * g + 2] = fy.z;
        }

        // scratch (Unity main thread only).
        static readonly double[] mappedWrench = new double[3];

        static double Component (Vector3 v, int r)
        {
            return r == 0 ? v.x : (r == 1 ? v.y : v.z);
        }

        /// <summary>
        /// gimbal g is a module mapped per pivot this step (an earlier change mode 1 or 2), with its
        /// per-pivot columns. The translation then reads its FREE columns for every linear model
        /// of a TOTAL deflection from rest (tilt matrix, torque of a tilt) and the wrench it
        /// REALIZES for the common deflection (feed-forward, commanded and realized
        /// acceleration): its local columns at (dx, dy) count a clamped pivot component as
        /// nothing, which is the right derivative for the law's increments but not the torque or
        /// the force of a whole deflection. False for every V1 module (one pivot, parallel
        /// pivots, mode 0): their columns are linear in the box, B u is exact, unchanged.
        /// </summary>
        static bool TranslationMapped (Entry e, int g)
        {
            var gr = e.Gimbals [g];
            return gr.PivotsMapped && gr.Gimbal != null && gr.Pivots.TorqueColsValid &&
                gr.Pivots.ForceColsValid;
        }

        /// <summary>
        /// After the gimbal and ctrlState columns: the common deflection (translation), its
        /// realized part after this step's ramp, the feed-forward that cancels its torque,
        /// the motif box, the shifted gimbal bounds, and the two motif columns (at n - 2 and
        /// n - 1). Only on planned ticks.
        /// </summary>
        static void ApplyTranslationColumns (Entry e, Vessel vessel, Transform reference,
            Vector3 com, AttitudeInput inp, int n, int nGimbalColumns)
        {
            var t = e.T;
            var p = t.Params;
            int gimbals = e.Gimbals.Count;

            // ------------------------------------------------ measured share (bit 32)
            // Before anything moves CommonRealized or shifts u0: the realized deflection of every
            // gimbal column (u0 = actuationLocal) against the realized common of the previous step,
            // when the law wrote that step with this layer (CommitTick): the share of the common
            // the gimbals actually hold, whatever the attitude part did with it.
            bool aging = (e.Options & AttitudeWrite.OptionEvidenceAging) != 0;
            if (aging) {
                // (born closed): the means forget in game time, sample or not.
                bool candidate = t.Measured && t.CommitTick == ownTick - 1 &&
                    t.CommonRealized.Length >= nGimbalColumns;
                double elapsed = AttitudeTranslation.Finite (t.ShareUt) ? t.LastUt - t.ShareUt : 0.0;
                bool sampled = false;
                if (candidate) {
                    double alpha = p.ShareTauSeconds > 0.0 && t.StepDt > 0.0 ? t.StepDt / p.ShareTauSeconds : 1.0;
                    sampled = AttitudeTranslation.AccumulateShareAged (inp.U0, t.CommonRealized,
                        nGimbalColumns, alpha, elapsed, t.StepDt, p.ShareTauSeconds, ref t.ShareNum,
                        ref t.ShareDen);
                } else {
                    AttitudeTranslation.AgeShare (ref t.ShareNum, ref t.ShareDen, elapsed,
                        p.ShareTauSeconds);
                }
                t.ShareUt = t.LastUt;
                if (sampled)
                    t.LastSampleUt = t.LastUt;
            } else if (t.Measured && t.CommitTick == ownTick - 1 && t.CommonRealized.Length >= nGimbalColumns) {
                double alpha = p.ShareTauSeconds > 0.0 && t.StepDt > 0.0 ? t.StepDt / p.ShareTauSeconds : 1.0;
                AttitudeTranslation.AccumulateShare (inp.U0, t.CommonRealized, nGimbalColumns, alpha,
                    ref t.ShareNum, ref t.ShareDen);
            }

            // ------------------------------------------------ geometry of a body tilt
            double thrustN = 0.0;
            for (int k = 0; k < e.Engines.Count; k++)
                if (e.Engines [k] != null)
                    thrustN += e.Engines [k].finalThrust * 1000.0;
            double massKg = vessel.totalMass * 1000.0;
            t.ThrustN = thrustN;
            t.MassKg = massKg;
            // K: lateral force (x, z) per degree of body tilt (x, z); Kt: torque, same.
            double kxx = 0.0, kxz = 0.0, kzx = 0.0, kzz = 0.0;
            double txx = 0.0, txz = 0.0, tyx = 0.0, tyz = 0.0, tzx = 0.0, tzz = 0.0;
            double range = double.PositiveInfinity;
            for (int g = 0; g < gimbals; g++) {
                int jx = 2 * g, jy = 2 * g + 1;
                t.Common [jx] = 0.0;
                t.Common [jy] = 0.0;
                if (!t.GimbalUsable [g]) {
                    // A gimbal that left the allocation (locked, disabled) carries no
                    // common part: its column is its realized deflection, as in c03d5de.
                    t.CommonRealized [jx] = 0.0;
                    t.CommonRealized [jy] = 0.0;
                    continue;
                }
                double aXx = t.AxisX [3 * g], aXz = t.AxisX [3 * g + 2];
                double aYx = t.AxisY [3 * g], aYz = t.AxisY [3 * g + 2];
                double fXx = t.ForceX [3 * g], fXz = t.ForceX [3 * g + 2];
                double fYx = t.ForceY [3 * g], fYz = t.ForceY [3 * g + 2];
                kxx += fXx * aXx + fYx * aYx;
                kxz += fXx * aXz + fYx * aYz;
                kzx += fXz * aXx + fYz * aYx;
                kzz += fXz * aXz + fYz * aYz;
                // a mapped module's torque of a tilt from rest = its free columns.
                bool mapped = TranslationMapped (e, g);
                var gr = e.Gimbals [g];
                for (int r = 0; r < 3; r++) {
                    double bx = mapped ? Component (gr.FreeX, r) : inp.B [r * n + jx];
                    double by = mapped ? Component (gr.FreeY, r) : inp.B [r * n + jy];
                    double cx = bx * aXx + by * aYx, cz = bx * aXz + by * aYz;
                    if (r == 0) {
                        txx += cx;
                        txz += cz;
                    } else if (r == 1) {
                        tyx += cx;
                        tyz += cz;
                    } else {
                        tzx += cx;
                        tzz += cz;
                    }
                }
                double lim = Math.Min (Math.Min (-inp.UMin [jx], inp.UMax [jx]),
                    Math.Min (-inp.UMin [jy], inp.UMax [jy]));
                if (lim < range)
                    range = lim;
            }
            if (double.IsInfinity (range) || !(range > 0.0))
                range = 0.0;
            t.RangeDeg = range;
            t.FullCommonTorque = range * AttitudeTranslation.MaxSingular2 (txx, txz, tzx, tzz);
            double thrustAccel = massKg > 0.0 ? thrustN / massKg : 0.0;
            t.LateralAuthority = AttitudeTranslation.LateralAuthority (thrustAccel, range,
                p.FracGimbal, p.FracDiff, t.DiffAuthority, t.FullCommonTorque);

            // ------------------------------------------------ computed levers (bit 4)
            // The RCS lever, the joint allocation and the HONEST authority: the largest
            // torque-free lateral acceleration it reaches (the formula above, published at
            // return offset 17, reads the differential and the common tilt only).
            if (t.Levers) {
                t.AuthorityDiffGimbal = t.LateralAuthority;
                bool ready = PrepareJoint (e, reference, com, inp, massKg, range, kxx, kxz, kzx,
                    kzz, txx, txz, tyx, tyz, tzx, tzz);
                t.LateralAuthority = ready ? t.Joint.Authority () : 0.0;
                if (!(t.LateralAuthority > 0.0))
                    t.Reasons |= AttitudeTranslation.ReasonNoLateralAuthority;
            }

            // ------------------------------------------------ translation target
            if (t.Translate) {
                if (!t.Levers)
                    ComputeCommon (e, vessel, reference, com, inp, n, thrustAccel, massKg, range,
                        kxx, kxz, kzx, kzz, txx, txz, tzx, tzz);
                else if (t.LateralAuthority > 0.0)
                    ComputeCommonJoint (e, vessel, reference, com, thrustAccel, massKg, range);
            }

            // ------------------------------------------------ free authority (option 256)
            // Before the bit-32 factor and before the shift: the model's authority, this step's gap and the
            // realized attitude parts. Reads only; the layer and the law's input are not touched.
            if ((e.Options & AttitudeWrite.OptionFreeAuthority) != 0)
                ComputeFreeAuthority (e, inp, nGimbalColumns);

            // ------------------------------------------------ honest authority (bits 16, 32)
            // The layer above ran on the model's authority (it keeps translating, so the share
            // keeps being measured); bit 32 publishes it times the measured share.
            if (t.Headroom || t.Measured)
                t.ModelAuthority = t.LateralAuthority;
            if (aging)
                t.EvModel = t.LateralAuthority;
            if (t.Measured) {
                double denMin = nGimbalColumns * p.ShareMinDegrees * p.ShareMinDegrees;
                t.MeasuredShare = t.ShareDen >= denMin && t.ShareDen > 0.0 ? t.ShareNum / t.ShareDen : double.NaN;
                double factor = AttitudeTranslation.MeasuredFactor (t.ShareNum, t.ShareDen, denMin);
                if (aging) {
                    // absence of evidence is not evidence of absence.
                    double age = AttitudeTranslation.Finite (t.LastSampleUt) ? t.LastUt - t.LastSampleUt
                        : double.PositiveInfinity;
                    factor = AttitudeTranslation.AgedFactor (factor, age, p.ShareTauSeconds);
                }
                t.LateralAuthority = t.LateralAuthority * factor;
            }

            // ------------------------------------------------ realized common, feed-forward
            double mx = 0.0, my = 0.0, mz = 0.0;
            double fcx = 0.0, fcz = 0.0, frx = 0.0, frz = 0.0;
            double fracCmd = 0.0, fracReal = 0.0;
            for (int g = 0; g < gimbals; g++) {
                int jx = 2 * g, jy = 2 * g + 1;
                if (!t.GimbalUsable [g]) {
                    t.CommonRealizedNext [jx] = 0.0;
                    t.CommonRealizedNext [jy] = 0.0;
                    continue;
                }
                double f = t.RampFactor [g];
                double nx = AttitudeTranslation.Ramp (t.CommonRealized [jx], t.Common [jx], f);
                double ny = AttitudeTranslation.Ramp (t.CommonRealized [jy], t.Common [jy], f);
                t.CommonRealizedNext [jx] = nx;
                t.CommonRealizedNext [jy] = ny;
                if (TranslationMapped (e, g)) {
                    // the torque and force the mapped module REALIZES for the common
                    // deflection alone (clamped pivot components hold their bound).
                    var pc = e.Gimbals [g].Pivots;
                    pc.RealizedTorque (nx, ny, mappedWrench);
                    mx += mappedWrench [0];
                    my += mappedWrench [1];
                    mz += mappedWrench [2];
                    pc.RealizedForce (t.Common [jx], t.Common [jy], mappedWrench);
                    fcx += mappedWrench [0];
                    fcz += mappedWrench [2];
                    pc.RealizedForce (nx, ny, mappedWrench);
                    frx += mappedWrench [0];
                    frz += mappedWrench [2];
                } else {
                    mx += inp.B [0 * n + jx] * nx + inp.B [0 * n + jy] * ny;
                    my += inp.B [1 * n + jx] * nx + inp.B [1 * n + jy] * ny;
                    mz += inp.B [2 * n + jx] * nx + inp.B [2 * n + jy] * ny;
                    fcx += t.ForceX [3 * g] * t.Common [jx] + t.ForceY [3 * g] * t.Common [jy];
                    fcz += t.ForceX [3 * g + 2] * t.Common [jx] + t.ForceY [3 * g + 2] * t.Common [jy];
                    frx += t.ForceX [3 * g] * nx + t.ForceY [3 * g] * ny;
                    frz += t.ForceX [3 * g + 2] * nx + t.ForceY [3 * g + 2] * ny;
                }
                double xn = -inp.UMin [jx], xp = inp.UMax [jx], yn = -inp.UMin [jy], yp = inp.UMax [jy];
                fracCmd = Math.Max (fracCmd, AttitudeTranslation.GimbalFraction (t.Common [jx],
                    t.Common [jy], xn, xp, yn, yp));
                fracReal = Math.Max (fracReal, AttitudeTranslation.GimbalFraction (nx, ny,
                    xn, xp, yn, yp));
            }
            if (t.Levers) {
                // Computed levers: the RCS translation is written times the share of the
                // commanded common deflection the gimbals realize after this step's ramp (the
                // RCS answers at once, the gimbals at their ramp): its torque keeps balancing
                // the realized tilt's, and the feed-forward below cancels the sum.
                double kappa = AttitudeTranslation.RealizedShare (t.Common, t.CommonRealizedNext,
                    2 * gimbals);
                var jt = t.Joint;
                t.RcsKappa = kappa;
                t.RcsWriteX = t.RcsCmdX * kappa;
                t.RcsWriteY = t.RcsCmdY * kappa;
                mx += jt.Rt [0] * t.RcsWriteX + jt.Rt [1] * t.RcsWriteY;
                my += jt.Rt [2] * t.RcsWriteX + jt.Rt [3] * t.RcsWriteY;
                mz += jt.Rt [4] * t.RcsWriteX + jt.Rt [5] * t.RcsWriteY;
                fcx += jt.Rf [0] * t.RcsCmdX + jt.Rf [1] * t.RcsCmdY;
                fcz += jt.Rf [2] * t.RcsCmdX + jt.Rf [3] * t.RcsCmdY;
                frx += jt.Rf [0] * t.RcsWriteX + jt.Rf [1] * t.RcsWriteY;
                frz += jt.Rf [2] * t.RcsWriteX + jt.Rf [3] * t.RcsWriteY;
            }
            t.FractionCommanded = fracCmd;
            t.FractionRealized = fracReal;
            if (massKg > 0.0) {
                t.CmdX = fcx / massKg;
                t.CmdZ = fcz / massKg;
                t.RealX = frx / massKg;
                t.RealZ = frz / massKg;
            }
            t.FfTorque [0] = mx;
            t.FfTorque [1] = my;
            t.FfTorque [2] = mz;
            double dffP = 0.0, dffY = 0.0;
            if ((mx != 0.0 || mz != 0.0) &&
                    !AttitudeTranslation.CancelTorque (t.ColumnP, t.ColumnY, mx, mz, out dffP, out dffY)) {
                dffP = 0.0;
                dffY = 0.0;
                t.Reasons |= AttitudeTranslation.ReasonFeedForwardClamped;
            }
            double halfP, halfY, share;
            t.FfScale = AttitudeTranslation.BoxHalfWidths (t.Count, t.MotifP, t.MotifY, t.Margin,
                ref dffP, ref dffY, out halfP, out halfY, out share);
            if (t.FfScale < 1.0)
                t.Reasons |= AttitudeTranslation.ReasonFeedForwardClamped;
            t.DffP = dffP;
            t.DffY = dffY;
            t.HalfP = halfP;
            t.HalfY = halfY;
            t.Share = share;
            if (t.Levers) {
                // Pitch/yaw torque left after the feed-forward (the rows CancelTorque works on;
                // the roll of the tilt and of the RCS, the law's roll channel's, is in the ring's
                // realized-common torque, offset 33).
                double rx = mx + t.ColumnP [0] * dffP + t.ColumnY [0] * dffY;
                double rz = mz + t.ColumnP [2] * dffP + t.ColumnY [2] * dffY;
                t.FfResidual = Math.Sqrt (rx * rx + rz * rz);
            }

            // ------------------------------------------------ shifted gimbal columns
            for (int j = 0; j < nGimbalColumns; j++) {
                double u0, lo, hi;
                AttitudeTranslation.ShiftGimbal (inp.U0 [j], t.CommonRealized [j], t.Common [j],
                    inp.UMin [j], inp.UMax [j], out u0, out lo, out hi);
                inp.U0 [j] = u0;
                inp.UMin [j] = lo;
                inp.UMax [j] = hi;
            }

            // ------------------------------------------------ the two motif columns
            int jp = nGimbalColumns + 3, jyaw = nGimbalColumns + 4;
            for (int r = 0; r < 3; r++) {
                inp.B [r * n + jp] = t.ColumnP [r];
                inp.B [r * n + jyaw] = t.ColumnY [r];
            }
            double u0p, lop, hip, u0y, loy, hiy;
            AttitudeTranslation.DiffBounds (t.DRealP, t.DffPrevP, dffP, halfP, t.AttitudeShare,
                out u0p, out lop, out hip);
            AttitudeTranslation.DiffBounds (t.DRealY, t.DffPrevY, dffY, halfY, t.AttitudeShare,
                out u0y, out loy, out hiy);
            inp.U0 [jp] = u0p;
            inp.UMin [jp] = lop;
            inp.UMax [jp] = hip;
            inp.U0 [jyaw] = u0y;
            inp.UMin [jyaw] = loy;
            inp.UMax [jyaw] = hiy;
            inp.UPref [jp] = 0.0;
            inp.UPref [jyaw] = 0.0;
            inp.Priority [jp] = p.DiffPriority;
            inp.Priority [jyaw] = p.DiffPriority;

            // ------------------------------------------------ shared RCS (bit 16)
            if (t.Headroom)
                ShareRcsWithAttitude (e, inp, n, nGimbalColumns);
        }

        /// <summary>
        /// (option 256): the free authority of this step (see AttitudeWrite.cs). With the computed levers
        /// and the joint allocation prepared: the realized attitude part of every usable gimbal column (u0 before
        /// the shift minus the realized common of the previous written step, the law's own) and of the two
        /// motifs (realized amplitude minus the previous feed-forward), the gap the nose does not give (WantX,
        /// WantZ, set by this step's mode-7 target), and AttitudeTranslation.FreeAuthorityRatio on the
        /// prepared joint (its outputs untouched: a scratch instance). No gap: ratio 1. Called before the
        /// bit-32 factor, so FreeModel is the model's authority.
        /// </summary>
        static void ComputeFreeAuthority (Entry e, AttitudeInput inp, int nGimbalColumns)
        {
            var t = e.T;
            if (!t.Levers || !t.Joint.Prepared)
                return;
            t.FreeComputed = true;
            t.FreeModel = t.LateralAuthority;
            double gimbalMax = 0.0;
            for (int g = 0; g < e.Gimbals.Count; g++) {
                int jx = 2 * g, jy = 2 * g + 1;
                if (jy >= nGimbalColumns || jy >= t.FreeAttitude.Length)
                    break;
                for (int j = jx; j <= jy; j++) {
                    t.FreeAttitude [j] = inp.U0 [j] - t.CommonRealized [j];
                    t.FreeUMin [j] = inp.UMin [j];
                    t.FreeUMax [j] = inp.UMax [j];
                    if (!t.GimbalUsable [g])
                        continue;
                    double a = t.FreeAttitude [j];
                    double range = a >= 0.0 ? inp.UMax [j] : -inp.UMin [j];
                    if (range > 0.0 && Math.Abs (a) / range > gimbalMax)
                        gimbalMax = Math.Abs (a) / range;
                }
            }
            t.FreeGimbalAttitude = gimbalMax;
            double attP = t.DRealP - t.DffPrevP, attY = t.DRealY - t.DffPrevY;
            t.FreeDiffAttitude = t.HalfSymmetric > 0.0
                ? Math.Max (Math.Abs (attP), Math.Abs (attY)) / t.HalfSymmetric : double.NaN;
            double want = Math.Sqrt (t.WantX * t.WantX + t.WantZ * t.WantZ);
            if (!(want > 0.0) || double.IsInfinity (want)) {
                t.FreeSource = AttitudeTranslation.FreeNoDirection;
                t.FreeRatio = 1.0;
            } else {
                double ux = t.WantX / want, uz = t.WantZ / want;
                t.FreeDirX = ux;
                t.FreeDirZ = uz;
                double alongModel, alongFree;
                int source;
                t.FreeRatio = AttitudeTranslation.FreeAuthorityRatio (t.Joint, t.JointFree, ux, uz, t.FreeAttitude,
                    t.FreeUMin, t.FreeUMax, attP, attY, t.HalfSymmetric, out alongModel, out alongFree, out source);
                t.FreeAlongModel = alongModel;
                t.FreeAlongFree = alongFree;
                t.FreeSource = source;
            }
            t.FreeAuthority = t.FreeModel * t.FreeRatio;
        }

        /// <summary>
        /// Honest translation, bit 16: the law's input where the RCS is shared. (1) Its RCS pitch /
        /// yaw column is left out (bounds [0, 0]: it writes 0) while ModuleRCS's combined rotation
        /// answers a small command with more than twice the column at the served point
        /// (PrepareRcsHeadroom): with the AutoPilot's roll saturated, the heavy booster's ring
        /// fired its whole torque for 0.05-0.3 of pitch/yaw (x 4.8), a bang-bang the law cannot
        /// allocate. (2) While the law does not write the roll (pitch/yaw-only write mask: the
        /// roll is the AutoPilot's) AND that roll is commanded (|ctrlState.roll| held above
        /// RollActiveThreshold), the gimbal and motif columns carry no roll: the allocation no
        /// longer fights, with the gimbals, the roll the AutoPilot commands with the RCS (which
        /// saturated the gimbals and cancelled the common tilt, build/tests
        /// TestHeavySharedRcsHover); without a commanded roll the law keeps damping it with its
        /// gimbals, as with bit 4. Nothing else of the law's input changes.
        /// </summary>
        static void ShareRcsWithAttitude (Entry e, AttitudeInput inp, int n, int nGimbalColumns)
        {
            var t = e.T;
            int jPitch = nGimbalColumns, jYaw = nGimbalColumns + 2;
            if (t.RcsPitchNonlinear) {
                inp.UMin [jPitch] = 0.0;
                inp.UMax [jPitch] = 0.0;
            }
            if (t.RcsYawNonlinear) {
                inp.UMin [jYaw] = 0.0;
                inp.UMax [jYaw] = 0.0;
            }
            if (t.RcsPitchNonlinear || t.RcsYawNonlinear)
                t.Reasons |= AttitudeTranslation.ReasonRcsShared;
            bool lawWritesRoll = e.Mode == ModeArmed && (e.AxesMask & AttitudeWrite.ArmRoll) != 0 &&
                (e.WriteMask & AttitudeWrite.ArmRoll) != 0;
            if (!lawWritesRoll && t.RollHeld > AttitudeTranslation.RollActiveThreshold) {
                for (int j = 0; j < n; j++)
                    if (j < nGimbalColumns || j >= nGimbalColumns + 3)
                        inp.B [1 * n + j] = 0.0;
                t.RollRowsDropped = true;
            }
        }

        /// <summary>
        /// The common deflection of this tick: the lateral acceleration the nose does not
        /// give, as a body tilt, bounded by FracGimbal of every gimbal's range and by what
        /// FracDiff of the throttle margins can cancel, converted into each gimbal's axes.
        /// </summary>
        static void ComputeCommon (Entry e, Vessel vessel, Transform reference, Vector3 com,
            AttitudeInput inp, int n, double thrustAccel, double massKg, double range,
            double kxx, double kxz, double kzx, double kzz, double txx, double txz,
            double tzx, double tzz)
        {
            var t = e.T;
            var p = t.Params;
            double ax, az;
            if (!TranslationGap (e, vessel, reference, com, thrustAccel, massKg, range, out ax,
                    out az))
                return;
            double phiX, phiZ;
            t.TiltMatrix [0] = kxx;
            t.TiltMatrix [1] = kxz;
            t.TiltMatrix [2] = kzx;
            t.TiltMatrix [3] = kzz;
            if (!AttitudeTranslation.SolveTilt (t.TiltMatrix, ax * massKg, az * massKg,
                    out phiX, out phiZ)) {
                t.Reasons |= AttitudeTranslation.ReasonNoTranslationGeometry;
                return;
            }
            // Bound 1: FracGimbal of every usable gimbal's directional range.
            double worst = 0.0;
            for (int g = 0; g < e.Gimbals.Count; g++) {
                if (!t.GimbalUsable [g])
                    continue;
                int jx = 2 * g, jy = 2 * g + 1;
                double cx = phiX * t.AxisX [3 * g] + phiZ * t.AxisX [3 * g + 2];
                double cy = phiX * t.AxisY [3 * g] + phiZ * t.AxisY [3 * g + 2];
                worst = Math.Max (worst, AttitudeTranslation.GimbalFraction (cx, cy,
                    -inp.UMin [jx], inp.UMax [jx], -inp.UMin [jy], inp.UMax [jy]));
            }
            double scaleGimbal = worst > p.FracGimbal ? p.FracGimbal / worst : 1.0;
            // Bound 2: the torque of this deflection must stay cancellable by FracDiff of the
            // throttle margins.
            double mx = txx * phiX + txz * phiZ;
            double mz = tzx * phiX + tzz * phiZ;
            double scaleDiff = 1.0;
            double dp, dy;
            if (mx != 0.0 || mz != 0.0) {
                if (AttitudeTranslation.CancelTorque (t.ColumnP, t.ColumnY, mx, mz, out dp, out dy))
                    scaleDiff = AttitudeTranslation.DiffCapacityScale (t.Count, t.MotifP,
                        t.MotifY, t.Margin, dp, dy, p.FracDiff);
                else
                    scaleDiff = 0.0;
            }
            double scale = 1.0;
            if (scaleGimbal < scale) {
                scale = scaleGimbal;
                t.Reasons |= AttitudeTranslation.ReasonLimitedByGimbal;
            }
            if (scaleDiff < scale) {
                scale = scaleDiff;
                t.Reasons |= AttitudeTranslation.ReasonLimitedByDiff;
            }
            if (!(scale >= 0.0) || double.IsInfinity (scale))
                scale = 0.0;
            t.LimitScale = scale;
            phiX *= scale;
            phiZ *= scale;
            t.PhiX = phiX;
            t.PhiZ = phiZ;
            for (int g = 0; g < e.Gimbals.Count; g++) {
                if (!t.GimbalUsable [g])
                    continue;
                t.Common [2 * g] = phiX * t.AxisX [3 * g] + phiZ * t.AxisX [3 * g + 2];
                t.Common [2 * g + 1] = phiX * t.AxisY [3 * g] + phiZ * t.AxisY [3 * g + 2];
            }
        }

        /// <summary>
        /// The lateral acceleration the nose does not give (WantX, WantZ: body x and z, m/s^2),
        /// read from the request, the local vertical and the thrust. False (with its reason)
        /// when the geometry is missing or the nose is too far from the vertical. Shared by
        /// ComputeCommon (modes 3) and ComputeCommonJoint (mode 7), the same arithmetic.
        /// </summary>
        static bool TranslationGap (Entry e, Vessel vessel, Transform reference, Vector3 com,
            double thrustAccel, double massKg, double range, out double ax, out double az)
        {
            var t = e.T;
            var p = t.Params;
            ax = 0.0;
            az = 0.0;
            if (!(massKg > 0.0) || !(thrustAccel > 0.0) || !(range > 0.0) ||
                    vessel.mainBody == null) {
                t.Reasons |= AttitudeTranslation.ReasonNoTranslationGeometry;
                return false;
            }
            try {
                Vector3d body = vessel.mainBody.position;
                Vector3 upWorld = new Vector3 ((float)(com.x - body.x), (float)(com.y - body.y),
                    (float)(com.z - body.z));
                Vector3 upBody = reference.InverseTransformDirection (AttitudeGeometry.Unit (upWorld));
                // (option 2048): the ring's lateral acceleration of this step when it serves one, else the
                // request's, as before.
                var aFrame = t.Frame;
                double rAx = t.Ax, rAy = t.Ay, rAz = t.Az;
                RingLateral (e, ref aFrame, ref rAx, ref rAy, ref rAz);
                double norm = Math.Sqrt (rAx * rAx + rAy * rAy + rAz * rAz);
                Vector3 demandBody = new Vector3 (0f, 0f, 0f);
                if (norm > 0.0) {
                    Vector3d dWorld = aFrame.DirectionToWorldSpace (
                        new Vector3d (rAx / norm, rAy / norm, rAz / norm));
                    Vector3 unitWorld = AttitudeGeometry.Unit (new Vector3 ((float)dWorld.x,
                        (float)dWorld.y, (float)dWorld.z));
                    demandBody = reference.InverseTransformDirection (
                        AttitudeGeometry.Scale (unitWorld, (float)norm));
                }
                t.UpBody [0] = upBody.x;
                t.UpBody [1] = upBody.y;
                t.UpBody [2] = upBody.z;
                t.DemandBody [0] = demandBody.x;
                t.DemandBody [1] = demandBody.y;
                t.DemandBody [2] = demandBody.z;
            } catch {
                t.Reasons |= AttitudeTranslation.ReasonNoTranslationGeometry;
                return false;
            }
            double det;
            if (!AttitudeTranslation.LateralFromGap (t.DemandBody, t.UpBody, thrustAccel,
                    p.MinProjectionDet, t.Gap, out ax, out az, out det)) {
                t.Reasons |= AttitudeTranslation.ReasonTiltTooLarge;
                return false;
            }
            t.WantX = ax;
            t.WantZ = az;
            return true;
        }

        // Bit 16 scratch (Unity main thread only).
        static readonly double[] fullFPos = new double[6], fullFNeg = new double[6];
        static readonly double[] fullTPos = new double[6], fullTNeg = new double[6];

        /// <summary>
        /// Honest translation, bit 16 (RCS headroom): the RCS translation columns of every module
        /// with each nozzle's share bounded by what the step's ROTATION leaves it
        /// (AttitudeGeometry.AddRcsTranslationColumnsHeadroom: ctrlState pitch/roll/yaw as found,
        /// rotation fractions held RcsHoldSeconds), then each channel kept at its WEAKER sign
        /// (AttitudeTranslation.WeakerSign). Also the capacity without the headroom (the return's
        /// diagnostic) and the largest held fraction. Returns false when a module's columns are
        /// approximate (precision mode, fullThrust), as the bit-4 loop.
        /// </summary>
        static bool PrepareRcsHeadroom (Entry e, Transform reference, Vector3 com, bool precision)
        {
            var t = e.T;
            var p = t.Params;
            bool linear = true;
            double drop = p.RcsHoldSeconds > 0.0 && t.StepDt > 0.0 ? t.StepDt / p.RcsHoldSeconds : 1.0;
            for (int k = 0; k < 6; k++)
                fullFPos [k] = fullFNeg [k] = fullTPos [k] = fullTNeg [k] = 0.0;
            double heldMax = 0.0;
            for (int i = 0; i < e.Rcs.Count; i++) {
                var rcs = e.Rcs [i];
                if (rcs == null)
                    continue;
                AttitudeGeometry.AddRcsTranslationColumns (rcs, com, reference, precision, fullFPos,
                    fullFNeg, fullTPos, fullTNeg);
                double[] hold;
                t.RcsHold.TryGetValue (rcs, out hold);
                if (!AttitudeGeometry.AddRcsTranslationColumnsHeadroom (rcs, com, reference, precision,
                        t.Served, ref hold, drop, p.FracRcs, t.RcsFPos, t.RcsFNeg, t.RcsTPos, t.RcsTNeg))
                    linear = false;
                if (hold != null) {
                    t.RcsHold [rcs] = hold;
                    for (int k = 0; k < hold.Length; k++)
                        if (hold [k] > heldMax)
                            heldMax = hold [k];
                }
            }
            t.RcsCapacityFull = Math.Min (Math.Min (Math.Abs (fullFPos [0]), Math.Abs (fullFNeg [0])),
                Math.Min (Math.Abs (fullFPos [5]), Math.Abs (fullFNeg [5])));
            double sx, sy;
            AttitudeTranslation.WeakerSign (t.RcsFPos, t.RcsFNeg, t.RcsTPos, t.RcsTNeg, out sx, out sy);
            t.RcsShareX = sx;
            t.RcsShareY = sy;
            t.RcsHeldMax = heldMax;
            double roll = Math.Abs (t.Served [1]);
            double rollDecayed = t.RollHeld - drop;
            t.RollHeld = AttitudeTranslation.Finite (roll) && roll > rollDecayed ? roll
                : (AttitudeTranslation.Finite (rollDecayed) ? rollDecayed : 0.0);

            // The law's own RCS pitch / yaw at the served rotation: ModuleRCS's combined rule.
            int modules = 0, largest = 0;
            if (t.NzCounts.Length < e.Rcs.Count) {
                int m = e.Rcs.Count;
                Array.Resize (ref t.NzCounts, m);
                Array.Resize (ref t.NzLever, m);
                Array.Resize (ref t.NzDir, m);
                Array.Resize (ref t.NzPower, m);
                Array.Resize (ref t.NzEnable, 3 * m);
            }
            for (int i = 0; i < e.Rcs.Count; i++) {
                var rcs = e.Rcs [i];
                int count;
                bool ep, er, ey;
                if (rcs == null || !AttitudeGeometry.RcsRotationNozzles (rcs, com, reference, ref t.NzLever [modules],
                        ref t.NzDir [modules], ref t.NzPower [modules], out count, out ep, out er, out ey))
                    continue;
                t.NzCounts [modules] = count;
                t.NzEnable [3 * modules] = ep ? 1.0 : 0.0;
                t.NzEnable [3 * modules + 1] = er ? 1.0 : 0.0;
                t.NzEnable [3 * modules + 2] = ey ? 1.0 : 0.0;
                if (count > largest)
                    largest = count;
                modules++;
            }
            t.NzModules = modules;
            if (t.NzRho.Length < largest)
                t.NzRho = new double[largest];
            t.RcsPitchNonlinear = modules > 0 && AttitudeTranslation.RcsRotationNonlinear (modules, t.NzCounts,
                t.NzLever, t.NzDir, t.NzPower, t.NzEnable, t.Served [0], t.Served [1], t.Served [2], 0, t.NzRho,
                t.NzScratch);
            t.RcsYawNonlinear = modules > 0 && AttitudeTranslation.RcsRotationNonlinear (modules, t.NzCounts,
                t.NzLever, t.NzDir, t.NzPower, t.NzEnable, t.Served [0], t.Served [1], t.Served [2], 2, t.NzRho,
                t.NzScratch);
            return linear;
        }

        /// <summary>
        /// Computed levers (bit 4): fills the joint allocation of this step -- the RCS
        /// translation columns of every RCS module (AttitudeGeometry.AddRcsTranslationColumns,
        /// the mean of the + and - columns as for the rotation columns), the tilt's force and
        /// torque (K, Kt of ApplyTranslationColumns, an earlier change included), the motif columns, the
        /// bounds (FracGimbal x the smallest range, FracRcs, FracDiff x the symmetric half
        /// width) and every usable gimbal's axes and directional ranges -- and prepares it.
        /// </summary>
        static bool PrepareJoint (Entry e, Transform reference, Vector3 com, AttitudeInput inp,
            double massKg, double range, double kxx, double kxz, double kzx, double kzz,
            double txx, double txz, double tyx, double tyz, double tzx, double tzz)
        {
            var t = e.T;
            var p = t.Params;
            var j = t.Joint;
            j.Clear ();
            for (int k = 0; k < 6; k++) {
                t.RcsFPos [k] = 0.0;
                t.RcsFNeg [k] = 0.0;
                t.RcsTPos [k] = 0.0;
                t.RcsTNeg [k] = 0.0;
            }
            bool precision = FlightInputHandler.fetch != null && FlightInputHandler.fetch.precisionMode;
            bool linear = true;
            if (!t.Headroom) {
                for (int i = 0; i < e.Rcs.Count; i++) {
                    var rcs = e.Rcs [i];
                    if (rcs != null && !AttitudeGeometry.AddRcsTranslationColumns (rcs, com, reference,
                            precision, t.RcsFPos, t.RcsFNeg, t.RcsTPos, t.RcsTNeg))
                        linear = false;
                }
            } else {
                linear = PrepareRcsHeadroom (e, reference, com, precision);
            }
            // Capacity: the along-axis force of X = +1, X = -1, Y = +1, Y = -1 (N), the smallest.
            t.RcsCapacity = Math.Min (Math.Min (Math.Abs (t.RcsFPos [0]), Math.Abs (t.RcsFNeg [0])),
                Math.Min (Math.Abs (t.RcsFPos [5]), Math.Abs (t.RcsFNeg [5])));
            bool rcsPresent = false;
            for (int k = 0; k < 6; k++)
                rcsPresent |= t.RcsFPos [k] != 0.0 || t.RcsFNeg [k] != 0.0 ||
                    t.RcsTPos [k] != 0.0 || t.RcsTNeg [k] != 0.0;
            if (rcsPresent && !linear)
                t.Reasons |= AttitudeTranslation.ReasonRcsModelApprox;
            j.Mass = massKg;
            j.K [0] = kxx;
            j.K [1] = kxz;
            j.K [2] = kzx;
            j.K [3] = kzz;
            j.Kt [0] = txx;
            j.Kt [1] = txz;
            j.Kt [2] = tyx;
            j.Kt [3] = tyz;
            j.Kt [4] = tzx;
            j.Kt [5] = tzz;
            // RCS: column X at [0..2], Y at [3..5] of the + and - arrays; their mean.
            j.Rf [0] = 0.5 * (t.RcsFPos [0] + t.RcsFNeg [0]);
            j.Rf [1] = 0.5 * (t.RcsFPos [3] + t.RcsFNeg [3]);
            j.Rf [2] = 0.5 * (t.RcsFPos [2] + t.RcsFNeg [2]);
            j.Rf [3] = 0.5 * (t.RcsFPos [5] + t.RcsFNeg [5]);
            for (int r = 0; r < 3; r++) {
                j.Rt [2 * r] = 0.5 * (t.RcsTPos [r] + t.RcsTNeg [r]);
                j.Rt [2 * r + 1] = 0.5 * (t.RcsTPos [3 + r] + t.RcsTNeg [3 + r]);
                j.Bd [2 * r] = t.ColumnP [r];
                j.Bd [2 * r + 1] = t.ColumnY [r];
            }
            j.PhiMax = p.FracGimbal * range;
            j.RcsMax = p.FracRcs;
            j.DiffMax = p.FracDiff * t.HalfSymmetric;
            j.DiffWeight = 1.0 / p.DiffPriority;
            j.FracGimbal = p.FracGimbal;
            int gimbals = e.Gimbals.Count;
            j.Resize (gimbals);
            for (int g = 0; g < gimbals; g++) {
                int jx = 2 * g, jy = 2 * g + 1;
                j.GimbalUsable [g] = t.GimbalUsable [g];
                j.GimbalAxes [4 * g] = t.AxisX [3 * g];
                j.GimbalAxes [4 * g + 1] = t.AxisX [3 * g + 2];
                j.GimbalAxes [4 * g + 2] = t.AxisY [3 * g];
                j.GimbalAxes [4 * g + 3] = t.AxisY [3 * g + 2];
                j.GimbalRanges [4 * g] = -inp.UMin [jx];
                j.GimbalRanges [4 * g + 1] = inp.UMax [jx];
                j.GimbalRanges [4 * g + 2] = -inp.UMin [jy];
                j.GimbalRanges [4 * g + 3] = inp.UMax [jy];
            }
            return j.Prepare ();
        }

        /// <summary>
        /// Computed levers (bit 4), mode 7: the common tilt AND the RCS translation of this tick,
        /// allocated jointly with the differential (JointTranslation: the lateral force the nose
        /// does not give, zero torque, then the worst gimbal fraction). The differential part of
        /// the solution only sized the others: the feed-forward recomputes it from the REALIZED
        /// common deflection and the RCS actually written. Not converged: nothing this step.
        /// </summary>
        static void ComputeCommonJoint (Entry e, Vessel vessel, Transform reference, Vector3 com,
            double thrustAccel, double massKg, double range)
        {
            var t = e.T;
            double ax, az;
            if (!TranslationGap (e, vessel, reference, com, thrustAccel, massKg, range, out ax,
                    out az))
                return;
            var j = t.Joint;
            double fx = ax * massKg, fz = az * massKg;
            if (!j.Solve (fx, fz) || !j.Converged) {
                t.Reasons |= AttitudeTranslation.ReasonNoTranslationGeometry;
                return;
            }
            double want = Math.Sqrt (fx * fx + fz * fz);
            double missX = j.Force [0] - fx, missZ = j.Force [1] - fz;
            if (Math.Sqrt (missX * missX + missZ * missZ) > JointTranslation.TargetTolerance * want) {
                if (j.AtPhiBound || j.Scale < 1.0)
                    t.Reasons |= AttitudeTranslation.ReasonLimitedByGimbal;
                if (j.AtDiffBound)
                    t.Reasons |= AttitudeTranslation.ReasonLimitedByDiff;
                if (j.AtRcsBound)
                    t.Reasons |= AttitudeTranslation.ReasonLimitedByRcs;
            }
            t.LimitScale = j.Scale;
            double phiX = j.X [0], phiZ = j.X [1];
            t.PhiX = phiX;
            t.PhiZ = phiZ;
            t.RcsCmdX = j.X [2];
            t.RcsCmdY = j.X [3];
            for (int g = 0; g < e.Gimbals.Count; g++) {
                if (!t.GimbalUsable [g])
                    continue;
                t.Common [2 * g] = phiX * t.AxisX [3 * g] + phiZ * t.AxisX [3 * g + 2];
                t.Common [2 * g + 1] = phiX * t.AxisY [3 * g] + phiZ * t.AxisY [3 * g + 2];
            }
        }

        /// <summary>
        /// The pitch/yaw channel wrote this step: the differential command (feed-forward +
        /// the allocation's attitude part), the per-engine offsets the fly-by-wire callback
        /// will write, and the realized common and feed-forward become the previous ones.
        /// </summary>
        static void CommitTranslation (Entry e, AttitudeInput inp, AttitudeOutput o,
            int nGimbalColumns)
        {
            var t = e.T;
            int jp = nGimbalColumns + 3, jy = nGimbalColumns + 4;
            double dP = t.DffP + inp.U0 [jp] + o.DeltaU [jp];
            double dY = t.DffY + inp.U0 [jy] + o.DeltaU [jy];
            t.DCmdP = dP;
            t.DCmdY = dY;
            AttitudeTranslation.ThrottleDeltas (t.Count, t.MotifP, t.MotifY, dP, dY, t.Delta);
            t.WriteThrottle = true;
            t.WriteThrottleTick = ownTick;
            for (int j = 0; j < t.CommonRealized.Length; j++)
                t.CommonRealized [j] = t.CommonRealizedNext [j];
            if (t.Levers)
                t.CommonComputed = true;
            // Bit 32 reads it at the next step (the realized deflection then holds this commit).
            t.CommitTick = ownTick;
            t.DffPrevP = t.DffP;
            t.DffPrevY = t.DffY;
            if (t.WindDown)
                t.WindDownSteps++;
            // Computed levers, mode 7 with a live request (t.Translate) on a vessel with an RCS
            // translation lever: ctrlState.X / .Y are written by the fly-by-wire callback.
            if (t.Levers && t.Translate && t.Joint.Prepared && (t.Joint.Rf [0] != 0.0 ||
                    t.Joint.Rf [1] != 0.0 || t.Joint.Rf [2] != 0.0 || t.Joint.Rf [3] != 0.0)) {
                t.WriteRcs = true;
                t.WriteRcsTick = ownTick;
            }
        }

        /// <summary>
        /// Fly-by-wire (computed levers): ctrlState.X / .Y = this step's RCS translation, only
        /// when it was decided at this step's Earlyish (bit 4, live request, pitch/yaw written)
        /// and the pitch/yaw channel wrote in this same callback. Otherwise nothing is written
        /// (X/Y stay the pilot's or kRPC's); and if the callback wrote X/Y at its previous call,
        /// ReleaseRcs gives them back.
        /// </summary>
        static void WriteRcsTranslation (Entry e, FlightCtrlState state, bool pitchYawWrote)
        {
            var t = e.T;
            bool write = t.WriteRcs && t.WriteRcsTick == ownTick && pitchYawWrote &&
                e.Mode == ModeArmed && !e.Dropped;
            t.WriteRcs = false;
            double x = Clamp1 (t.RcsWriteX), y = Clamp1 (t.RcsWriteY);
            if (write && !double.IsNaN (x) && !double.IsNaN (y)) {
                float fx = (float)x, fy = (float)y;
                state.X = fx;
                state.Y = fy;
                t.RcsWritten = true;
                t.RcsWrittenX = fx;
                t.RcsWrittenY = fy;
                t.RcsWrites++;
                return;
            }
            ReleaseRcs (t, state);
        }

        /// <summary>
        /// The step after our last X/Y write: the active vessel's ctrlState was reset from the
        /// pilot at the start of the step (FlightInputHandler), so nothing of ours is left; a
        /// ctrlState nobody refreshes (a non-active vessel) would keep our last values,
        /// so X or Y still holding EXACTLY what we wrote is set to 0, once. Also at unhooking.
        /// </summary>
        static void ReleaseRcs (TranslationEntry t, FlightCtrlState state)
        {
            if (!t.RcsWritten)
                return;
            t.RcsWritten = false;
            if (state == null)
                return;
            if (state.X == t.RcsWrittenX)
                state.X = 0f;
            if (state.Y == t.RcsWrittenY)
                state.Y = 0f;
        }

        /// <summary>
        /// Fly-by-wire (after the kRPC AutoPilot, which served this step's main throttle):
        /// throttle_i = main + kappa delta_i, kappa scaling the offsets so that every engine
        /// stays within [0, 1] around THIS main throttle (a cut main throttle cuts them all).
        /// Nothing may escape.
        /// </summary>
        static void WriteDiffThrottles (Entry e, FlightCtrlState state)
        {
            var t = e.T;
            if (!t.WriteThrottle || t.WriteThrottleTick != ownTick || e.Mode != ModeArmed ||
                    e.Dropped)
                return;
            t.WriteThrottle = false;
            double main = state.mainThrottle;
            double kappa = AttitudeTranslation.WriteScale (t.Count, t.Delta, main, 0.0, 1.0);
            t.FbwThrottleTick = ownTick;
            t.FbwMainThrottle = main;
            t.FbwKappa = kappa;
            for (int i = 0; i < t.Count && i < t.Engines.Count; i++) {
                double u = main + kappa * t.Delta [i];
                if (u > 1.0)
                    u = 1.0;
                if (!(u > 0.0))
                    u = 0.0;
                float pct = (float)(100.0 * u);
                var engine = t.Engines [i];
                if (Actuation.CommandThrottleForAttitude (e.AuthorityToken, engine, pct)) {
                    t.Leased.Add (engine);
                    t.WrittenPct [i] = pct;
                } else {
                    t.WrittenPct [i] = double.NaN;
                }
            }
            t.ThrottleWrites++;
        }

        // --------------------------------------------------------------- the ring

        static void WriteTranslationFrame (Entry e, int n, int nGimbalColumns, bool wrote)
        {
            var t = e.T;
            // One frame per tick in which PlanTranslation went past its born-closed fast path
            // (a request received since the arming, or the layer still engaged).
            if (t.LastTick != ownTick)
                return;
            int gimbals = e.Gimbals.Count;
            int engines = t.Count;
            // Core v2: the ring's next buffer, zeroed like the new array it replaces.
            int frameLength = TranslationHeaderStride + gimbals * TranslationGimbalRowStride +
                engines * TranslationEngineRowStride + TranslationClockStride;
            var f = t.History.Reserve (frameLength);
            double ut = double.IsNaN (t.LastUt) ? Planetarium.GetUniversalTime () : t.LastUt;
            int h = 0;
            f [h++] = TranslationProtocolVersion;             // 0
            f [h++] = TranslationSchemaVersion;               // 1
            f [h++] = ownTick;                                // 2
            f [h++] = ut;                                     // 3
            f [h++] = e.PersistentId;                         // 4
            f [h++] = t.State;                                // 5
            f [h++] = t.Reasons;                              // 6
            f [h++] = t.Mode;                                 // 7
            f [h++] = t.Sequence;                             // 8
            f [h++] = t.HaveRequest ? ut - t.UtStamp : double.NaN;        // 9
            f [h++] = t.HaveRequest ? t.UtValidUntil - ut : double.NaN;   // 10
            f [h++] = t.ThrustN;                              // 11
            f [h++] = t.MassKg;                               // 12
            f [h++] = t.MainThrottle;                         // 13
            f [h++] = t.DemandBody [0];                       // 14-16
            f [h++] = t.DemandBody [1];
            f [h++] = t.DemandBody [2];
            f [h++] = t.UpBody [0];                           // 17-19
            f [h++] = t.UpBody [1];
            f [h++] = t.UpBody [2];
            f [h++] = t.WantX;                                // 20-21
            f [h++] = t.WantZ;
            f [h++] = t.CmdX;                                 // 22-23
            f [h++] = t.CmdZ;
            f [h++] = t.PhiX;                                 // 24-25
            f [h++] = t.PhiZ;
            f [h++] = t.LimitScale;                           // 26
            f [h++] = t.LateralAuthority;                     // 27
            f [h++] = t.DiffAuthority;                        // 28
            f [h++] = t.FullCommonTorque;                     // 29
            f [h++] = t.FractionCommanded;                    // 30
            f [h++] = t.FractionRealized;                     // 31
            f [h++] = t.FfTorque [0];                         // 32-34
            f [h++] = t.FfTorque [1];
            f [h++] = t.FfTorque [2];
            f [h++] = t.DffP;                                 // 35-36
            f [h++] = t.DffY;
            f [h++] = t.DRealP;                               // 37-38
            f [h++] = t.DRealY;
            f [h++] = t.HalfP;                                // 39-40
            f [h++] = t.HalfY;
            f [h++] = wrote ? t.DCmdP : double.NaN;           // 41-42
            f [h++] = wrote ? t.DCmdY : double.NaN;
            f [h++] = t.Share;                                // 43
            // Fly-by-wire record of the PREVIOUS step (the callback runs after Earlyish).
            f [h++] = t.FbwThrottleTick >= 0 ? t.FbwThrottleTick : double.NaN;   // 44
            f [h++] = t.FbwMainThrottle;                      // 45
            f [h++] = t.FbwKappa;                             // 46
            f [h++] = t.Leased.Count;                         // 47
            f [h++] = t.WindDown ? ut - t.WindDownStart : double.NaN;   // 48
            f [h++] = t.RealX;                                // 49-50
            f [h++] = t.RealZ;
            f [h++] = t.RangeDeg;                             // 51
            f [h++] = t.HalfSymmetric;                        // 52
            f [h++] = gimbals;                                // 53
            f [h++] = engines;                                // 54
            f [h++] = t.ThrottleWrites;                       // 55
            int r = TranslationHeaderStride;
            for (int g = 0; g < gimbals; g++) {
                var gr = e.Gimbals [g];
                int jx = 2 * g, jy = 2 * g + 1;
                bool sized = jy < t.Common.Length;
                f [r++] = gr.Part != null ? gr.Part.flightID : 0.0;
                f [r++] = gr.Ordinal;
                f [r++] = sized ? t.Common [jx] : 0.0;
                f [r++] = sized ? t.Common [jy] : 0.0;
                f [r++] = sized ? t.CommonRealizedNext [jx] : 0.0;
                f [r++] = sized ? t.CommonRealizedNext [jy] : 0.0;
            }
            for (int i = 0; i < engines; i++) {
                f [r++] = t.FlightIds [i];
                f [r++] = t.Slope [i];
                f [r++] = t.MotifP [i];
                f [r++] = t.MotifY [i];
                f [r++] = t.LeverX [i];
                f [r++] = t.LeverZ [i];
                f [r++] = t.TauReal [i];
                f [r++] = wrote ? t.Delta [i] : double.NaN;
                // What the fly-by-wire callback wrote in the PREVIOUS step (same row order
                // while the engine set does not change), then cleared.
                f [r++] = t.WrittenPct [i];
                t.WrittenPct [i] = double.NaN;
                f [r++] = t.Engines [i] != null && Actuation.ThrottleIsLeased (t.Engines [i]) ? 1.0 : 0.0;
            }
            // Schema 2 (refactor R3): the clock section, after every row.
            f [r] = GameClock.StepTick;
            t.History.Commit (frameLength);
        }
    }
}
