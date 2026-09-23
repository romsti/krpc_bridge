using System;
using System.Collections.Generic;
using System.Diagnostics;
using KRPC.Bridge.Core;
using UnityEngine;
using SCReferenceFrame = KRPC.SpaceCenter.Services.ReferenceFrame;

namespace KRPC.Bridge.Actuators
{
    /// <summary>
    /// AttitudeV1: the attitude loop at the physics rate, IN SHADOW (plan GNC v3, step M2).
    ///
    /// Every FixedUpdate (TimingManager stage Earlyish), for each vessel a client armed by
    /// persistentId, active or not: measures the rotational state and the REALIZED torques,
    /// builds the control-effectiveness columns (gimbals, ctrlState channel = RCS + reaction
    /// wheels + control surfaces), runs the law of AttitudeMath (square-root reference + PCH,
    /// torque-form INDI, bounded WLS allocation) and publishes the wanted torque M_d, the
    /// allocation and the residuals in a 1000-frame ring.
    ///
    /// It WRITES NO ACTUATOR. Mode 1 (armed) is refused by this build: the allocation is
    /// published, never applied. Born closed: nothing runs until AttitudeArmV1, and a
    /// vessel whose real-time lease runs out is disarmed.
    /// </summary>
    internal static class AttitudeV1
    {
        internal const int ProtocolVersion = 1;
        internal const int SchemaVersion = 1;
        internal const int MaxVessels = 2;
        internal const int HistoryCapacity = 1000;
        internal const int ModeShadow = 0;
        internal const int HeaderStride = 24;
        internal const int StateStride = 79;
        internal const int RowStride = 16;
        internal const int StatusHeaderStride = 9;
        internal const int StatusRowStride = 16;
        const float MinLeaseSeconds = 0.2f;
        const float MaxLeaseSeconds = 10f;
        const int PumpFallbackAfter = 3;

        // Actuator row kinds.
        internal const int KindGimbal = 1;
        internal const int KindCtrlState = 3;

        // Stage codes (header offset 7).
        internal const int StageEarlyish = 1;
        internal const int StagePumpFallback = 2;

        // Motif bits: why the law did not run, or what WOULD make an armed loop fall back.
        internal const long MotifLeaseExpired = 1L << 0;
        internal const long MotifNoReference = 1L << 1;
        internal const long MotifReferenceStale = 1L << 2;
        internal const long MotifWarp = 1L << 3;
        internal const long MotifTickGap = 1L << 4;
        internal const long MotifUnloaded = 1L << 5;
        internal const long MotifTopologyChanged = 1L << 6;
        internal const long MotifResidualGuard = 1L << 7;
        internal const long MotifOmegaGuard = 1L << 8;
        internal const long MotifSaturationGuard = 1L << 9;
        internal const long MotifException = 1L << 10;
        internal const long MotifEarlyishDead = 1L << 11;
        internal const long MotifFrameConversion = 1L << 12;
        internal const long MotifRcsModelApprox = 1L << 13;
        internal const long MotifSurfaceApprox = 1L << 14;
        internal const long MotifNotConverged = 1L << 15;

        // Capability bits (header offset 23).
        const long CapEngineTorque = 1L << 0;
        const long CapRcsTorque = 1L << 1;
        const long CapWheelTorque = 1L << 2;
        const long CapGimbalColumns = 1L << 3;
        const long CapRcsColumns = 1L << 4;
        const long CapWheelColumns = 1L << 5;
        const long CapSurfaceColumns = 1L << 6;
        const long CapReference = 1L << 7;
        const long CapLaw = 1L << 8;

        internal sealed class GimbalRef
        {
            internal ModuleGimbal Gimbal;
            internal Part Part;
            internal int Ordinal;
            internal List<ModuleEngines> PartEngines;
        }

        internal sealed class Entry
        {
            internal uint PersistentId;
            internal string Owner;
            internal string Token;
            internal int Mode;
            internal int AxesMask;
            internal float LeaseSeconds;
            internal float ExpiresAt;

            // Reference, as the client last sent it (same meaning as the AutoPilot's).
            internal bool HaveReference;
            internal int Sequence;
            internal SCReferenceFrame Frame;
            internal double DirX, DirY, DirZ;
            internal int RollMode;
            internal double FfX, FfY, FfZ;
            internal double UtStamp, UtValidUntil, OmegaMax;
            internal int AcceptedCount;

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

            // Measurement memory.
            internal bool HavePrevious;
            internal long PreviousTick;
            internal Vector3 PreviousOmega;

            // Ring and bookkeeping.
            internal readonly LinkedList<double[]> History = new LinkedList<double[]> ();
            internal long LatestTick;
            internal long LastMotif;
            internal int ConsumedSequence;
            internal double ConsumedUtStamp = double.NaN;
            internal long Computed;
            internal long Exceptions;
            internal double CostMicrosLast;
            internal double CostMicrosMax;
            internal bool Loaded;
        }

        static readonly Dictionary<uint, Entry> entries = new Dictionary<uint, Entry> ();
        static readonly TimingManager.UpdateAction earlyishAction = OnEarlyish;
        static bool registered;
        static long ownTick;                 // +1 per attitude tick (Earlyish or fallback)
        static long earlyishCalls;
        static long pumpFallbackTicks;
        static long lastEarlyishSeenByPump = -1;
        static int pumpCallsWithoutEarlyish;

        // ---------------------------------------------------------------- API (RPC bodies)

        internal static string Arm (string persistentId, string owner, float leaseSeconds,
            int mode, int axesMask)
        {
            uint pid = ParsePersistentId (persistentId);
            if (string.IsNullOrWhiteSpace (owner))
                throw new ArgumentException ("owner vide", nameof (owner));
            if (owner.Length > 80)
                throw new ArgumentOutOfRangeException (nameof (owner));
            ValidateLease (leaseSeconds);
            if (mode != ModeShadow)
                throw new InvalidOperationException (
                    "AttitudeV1 : seul le mode ombre (0) existe dans cette DLL ; aucun actionneur n'est ecrit");
            if (axesMask < 0 || axesMask > 3)
                throw new ArgumentOutOfRangeException (nameof (axesMask), "bits : 1 roulis, 2 tangage/lacet");
            Entry entry;
            if (!entries.TryGetValue (pid, out entry)) {
                if (entries.Count >= MaxVessels)
                    throw new InvalidOperationException (
                        "deja " + MaxVessels + " vaisseaux armes : en liberer un avant");
                entry = new Entry { PersistentId = pid };
                entries.Add (pid, entry);
            }
            entry.Owner = owner.Trim ();
            entry.Token = Guid.NewGuid ().ToString ("N");
            entry.Mode = mode;
            entry.AxesMask = axesMask;
            entry.LeaseSeconds = leaseSeconds;
            entry.ExpiresAt = Time.realtimeSinceStartup + leaseSeconds;
            entry.HaveReference = false;
            entry.Sequence = 0;
            entry.Frame = null;
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
            entry.ExpiresAt = Time.realtimeSinceStartup + leaseSeconds;
            return true;
        }

        internal static bool Release (string token)
        {
            var entry = RequireToken (token);
            Drop (entry.PersistentId);
            return true;
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
            var entry = RequireToken (token);
            if (frame == null)
                throw new ArgumentNullException (nameof (frame));
            if (!Finite (dirX) || !Finite (dirY) || !Finite (dirZ) ||
                    dirX * dirX + dirY * dirY + dirZ * dirZ < 1e-18)
                throw new ArgumentOutOfRangeException ("direction", "direction nulle ou non finie");
            if (rollMode != 0)
                throw new ArgumentOutOfRangeException (nameof (rollMode),
                    "seul le roulis en amortissement de taux (0) existe");
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
            entry.HaveReference = true;
            entry.AcceptedCount++;
            entry.ExpiresAt = Time.realtimeSinceStartup + entry.LeaseSeconds;
            return 1;
        }

        internal static IList<double> ReadStatus ()
        {
            float now = Time.realtimeSinceStartup;
            var output = new List<double> (StatusHeaderStride + entries.Count * StatusRowStride) {
                ProtocolVersion,
                SchemaVersion,
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
                output.Add (Math.Max (0f, e.ExpiresAt - now));
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
            }
            return output;
        }

        internal static IList<double> ReadFrames (string persistentId, int sinceTick, int maxFrames)
        {
            uint pid = ParsePersistentId (persistentId);
            Entry entry;
            if (!entries.TryGetValue (pid, out entry))
                return DynamicsV3.ReadRing (new LinkedList<double[]> (), ProtocolVersion,
                    SchemaVersion, sinceTick, maxFrames);
            return DynamicsV3.ReadRing (entry.History, ProtocolVersion, SchemaVersion,
                sinceTick, maxFrames);
        }

        /// <summary>Leaving the flight scene: every armed vessel is gone.</summary>
        internal static void ResetForScene ()
        {
            entries.Clear ();
            Unregister ();
            AttitudeProbe.Stop ();
            lastEarlyishSeenByPump = -1;
            pumpCallsWithoutEarlyish = 0;
        }

        /// <summary>
        /// Called from ActuatorsAddon.Pump (watcher FixedUpdate). Witness of the Earlyish
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
        }

        static void Drop (uint persistentId)
        {
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
            ownTick++;
            float now = Time.realtimeSinceStartup;
            expiredScratch.Clear ();
            foreach (var pair in entries) {
                if (now >= pair.Value.ExpiresAt) {
                    expiredScratch.Add (pair.Key);
                    continue;
                }
                TickEntry (pair.Value, stage);
            }
            for (int i = 0; i < expiredScratch.Count; i++) {
                BridgeLog.Info ("AttitudeV1: bail expire, vaisseau " + expiredScratch [i] + " desarme");
                Drop (expiredScratch [i]);
            }
        }

        // ------------------------------------------------------------------ one tick

        static void TickEntry (Entry e, int stage)
        {
            long t0 = Stopwatch.GetTimestamp ();
            long motif = stage == StagePumpFallback ? MotifEarlyishDead : 0L;
            long caps = 0;
            try {
                var vessel = Resolve (e);
                if (vessel == null) {
                    e.Loaded = false;
                    e.HavePrevious = false;
                    e.Law.Reset ();
                    e.LastMotif = motif | MotifUnloaded;
                    return;
                }
                e.Loaded = true;
                var reference = vessel.ReferenceTransform;
                if (reference == null) {
                    e.HavePrevious = false;
                    e.Law.Reset ();
                    e.LastMotif = motif | MotifUnloaded;
                    return;
                }

                // Topology: the actuator vector and every cached module belong to one layout.
                ulong hash = ActuatorsAddon.ComputeTopologyHash (vessel);
                if (e.VesselId != vessel.id || e.TopologyHash != hash) {
                    Rebuild (e, vessel, hash);
                    motif |= MotifTopologyChanged;
                }

                double dt = TimeWarp.fixedDeltaTime;
                bool warp = TimeWarp.CurrentRate != 1f || Math.Abs (dt - 0.02) > 1e-5;
                if (warp)
                    motif |= MotifWarp;

                var inp = e.Input;
                int nGimbalColumns = 2 * e.Gimbals.Count;
                int n = nGimbalColumns + 3;
                inp.Resize (n);
                inp.Dt = dt;

                // ------------------------------------------------ rotational state
                Vector3 omega = vessel.angularVelocity;
                Vector3 moi = vessel.MOI;
                inp.Omega [0] = omega.x;
                inp.Omega [1] = omega.y;
                inp.Omega [2] = omega.z;
                inp.Inertia [0] = moi.x * 1000.0;
                inp.Inertia [1] = moi.y * 1000.0;
                inp.Inertia [2] = moi.z * 1000.0;
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
                    if (!AttitudeGeometry.AddRcsColumns (rcs, com, reference, precision, e.BPos, e.BNeg))
                        motif |= MotifRcsModelApprox;
                    caps |= CapRcsColumns;
                }
                for (int i = 0; i < e.Wheels.Count; i++) {
                    var wheel = e.Wheels [i];
                    if (wheel != null && AttitudeGeometry.AddWheel (wheel, wheelBody, e.BPos, e.BNeg))
                        caps |= CapWheelTorque | CapWheelColumns;
                }
                for (int i = 0; i < e.Surfaces.Count; i++) {
                    var surface = e.Surfaces [i];
                    if (surface != null && AttitudeGeometry.AddSurface (surface, e.BPos, e.BNeg)) {
                        motif |= MotifSurfaceApprox;
                        caps |= CapSurfaceColumns;
                    }
                }
                inp.Torque [0] = engineBody.x + rcsBody.x + wheelBody [0];
                inp.Torque [1] = engineBody.y + rcsBody.y + wheelBody [1];
                inp.Torque [2] = engineBody.z + rcsBody.z + wheelBody [2];

                // ------------------------------------------------------- B columns
                for (int g = 0; g < e.Gimbals.Count; g++) {
                    var gr = e.Gimbals [g];
                    var gimbal = gr.Gimbal;
                    int jx = 2 * g, jy = 2 * g + 1;
                    bool leased = gimbal != null && ActuatorsAddon.GimbalIsLeased (gimbal);
                    bool usable = gimbal != null && gimbal.moduleIsEnabled &&
                        (leased || (gimbal.gimbalActive && !gimbal.gimbalLock));
                    float dx = gimbal == null || (!leased && gimbal.gimbalLock) ? 0f : gimbal.actuationLocal.x;
                    float dy = gimbal == null || (!leased && gimbal.gimbalLock) ? 0f : gimbal.actuationLocal.y;
                    Vector3 colX = new Vector3 (0f, 0f, 0f), colY = new Vector3 (0f, 0f, 0f);
                    if (gimbal != null)
                        AttitudeGeometry.GimbalColumns (gimbal, gr.PartEngines, com, reference,
                            dx, dy, out colX, out colY);
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
                }
                if (e.Gimbals.Count > 0)
                    caps |= CapGimbalColumns;
                var ctrl = vessel.ctrlState;
                double[] served = {
                    ctrl == null ? 0.0 : ctrl.pitch,
                    ctrl == null ? 0.0 : ctrl.roll,
                    ctrl == null ? 0.0 : ctrl.yaw
                };
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
                        Vector3d dWorld = e.Frame.DirectionToWorldSpace (new Vector3d (e.DirX, e.DirY, e.DirZ));
                        Vector3 dBody = reference.InverseTransformDirection (
                            new Vector3 ((float)dWorld.x, (float)dWorld.y, (float)dWorld.z));
                        Vector3d ffWorld = e.Frame.DirectionToWorldSpace (new Vector3d (e.FfX, e.FfY, e.FfZ));
                        Vector3 ffBody = reference.InverseTransformDirection (
                            new Vector3 ((float)ffWorld.x, (float)ffWorld.y, (float)ffWorld.z));
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
                inp.OmegaFf [0] = e.FfBody [0];
                inp.OmegaFf [1] = e.FfBody [1];
                inp.OmegaFf [2] = e.FfBody [2];
                inp.OmegaDotFf [0] = 0.0;
                inp.OmegaDotFf [1] = 0.0;
                inp.OmegaDotFf [2] = 0.0;
                inp.OmegaMax = e.HaveReference ? e.OmegaMax : 0.0;
                if (warp) {
                    // Physics warp: a real loop would fall back; the shadow restarts.
                    inp.OmegaDotValid = false;
                }

                // --------------------------------------------------------------- law
                var o = e.Output;
                AttitudeMath.Step (e.Gains, inp, e.Law, e.Work, o);
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

                // ------------------------------------------------------------- frame
                long end = Stopwatch.GetTimestamp ();
                e.CostMicrosLast = (end - t0) * 1e6 / Stopwatch.Frequency;
                if (e.CostMicrosLast > e.CostMicrosMax)
                    e.CostMicrosMax = e.CostMicrosLast;
                WriteFrame (e, vessel, stage, ut, dt, motif, caps, served, engineBody, rcsBody,
                    wheelBody);
                e.LastMotif = motif;
            } catch (Exception exc) {
                e.Exceptions++;
                e.Law.Reset ();
                e.HavePrevious = false;
                e.LastMotif = motif | MotifException;
                if (e.Exceptions <= 5)
                    BridgeLog.Error ("AttitudeV1 tick: " + exc);
            }
        }

        static void SetColumn (AttitudeInput inp, int j, Vector3 column)
        {
            int n = inp.N;
            inp.B [0 * n + j] = column.x;
            inp.B [1 * n + j] = column.y;
            inp.B [2 * n + j] = column.z;
        }

        static void WriteFrame (Entry e, Vessel vessel, int stage, double ut, double dt,
            long motif, long caps, double[] served, Vector3 engineBody, Vector3 rcsBody,
            double[] wheelBody)
        {
            var inp = e.Input;
            var o = e.Output;
            int n = inp.N;
            int nGimbalColumns = n - 3;
            var frame = new double[HeaderStride + StateStride + n * RowStride];
            int h = 0;
            frame [h++] = ProtocolVersion;
            frame [h++] = SchemaVersion;
            frame [h++] = ownTick;
            frame [h++] = ut;
            frame [h++] = dt;
            frame [h++] = e.PersistentId;
            frame [h++] = e.TopologyGeneration;
            frame [h++] = stage;
            frame [h++] = ActuatorsAddon.CurrentPhysicsTick;
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

            int r = HeaderStride + StateStride;
            for (int j = 0; j < n; j++) {
                bool gimbal = j < nGimbalColumns;
                var gr = gimbal ? e.Gimbals [j / 2] : null;
                frame [r++] = gimbal ? KindGimbal : KindCtrlState;
                frame [r++] = gimbal && gr.Part != null ? gr.Part.flightID : 0.0;
                frame [r++] = gimbal ? gr.Ordinal : 0.0;
                frame [r++] = gimbal ? j % 2 : j - nGimbalColumns;
                frame [r++] = inp.U0 [j];
                frame [r++] = o.Computed ? o.DeltaU [j] : double.NaN;
                frame [r++] = inp.UMin [j];
                frame [r++] = inp.UMax [j];
                frame [r++] = o.Computed ? o.ActiveSet [j] : double.NaN;
                frame [r++] = inp.B [0 * n + j];
                frame [r++] = inp.B [1 * n + j];
                frame [r++] = inp.B [2 * n + j];
                if (gimbal) {
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
            e.History.AddLast (frame);
            while (e.History.Count > HistoryCapacity)
                e.History.RemoveFirst ();
            e.LatestTick = ownTick;
        }

        static void Put3 (double[] target, int offset, double[] v, bool valid)
        {
            target [offset] = valid ? v [0] : double.NaN;
            target [offset + 1] = valid ? v [1] : double.NaN;
            target [offset + 2] = valid ? v [2] : double.NaN;
        }

        // ------------------------------------------------------------------ vessel cache

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
                        if (Time.realtimeSinceStartup >= pair.Value.ExpiresAt) {
                            Drop (pair.Key);
                            break;
                        }
                        return pair.Value;
                    }
            throw new InvalidOperationException ("token d'attitude absent, expire ou invalide");
        }

        static void ValidateLease (float leaseSeconds)
        {
            if (float.IsNaN (leaseSeconds) || float.IsInfinity (leaseSeconds) ||
                    leaseSeconds < MinLeaseSeconds || leaseSeconds > MaxLeaseSeconds)
                throw new ArgumentOutOfRangeException (
                    nameof (leaseSeconds), "bail d'attitude attendu entre 0.2 et 10 s");
        }

        static uint ParsePersistentId (string persistentId)
        {
            uint id;
            if (!uint.TryParse (persistentId, System.Globalization.NumberStyles.None,
                    System.Globalization.CultureInfo.InvariantCulture, out id))
                throw new ArgumentException ("persistentId doit etre un uint decimal",
                    nameof (persistentId));
            return id;
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
                rows.Add (ActuatorsAddon.CurrentPhysicsTick);
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
