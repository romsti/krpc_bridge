using System;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Bridge.Core;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// AttitudeV1, refactor lot 1: the per-arming options of AttitudeOptionsV1,
    /// the capability and layout reads, and the per-step neutralization of the control
    /// surfaces. The rules are pure (AttitudeWrite.cs, AttitudeTranslation.cs); this file
    /// touches the game.
    ///
    /// BORN CLOSED: every arming resets the options to 0, and with 0 none of this runs.
    ///
    /// Surface neutralization (OptionFamilyBlocking). A ModuleControlSurface follows
    /// ctrlState pitch/yaw/roll in its own FixedUpdate (FixedCtrlSurfaceUpdate, through
    /// ignorePitch/Yaw/Roll); its column in the law (GetPotentialTorque) is wrong by a lever
    /// factor, so a ctrlState written by the loop must not move it. In every step where the
    /// loop writes ctrlState pitch/yaw (or wrote, at the last fly-by-wire, pitch/yaw or a roll
    /// the surfaces answer: a roll alone never starts it, AttitudeWrite.NeutralizeSurfaces),
    /// each surface that answers a ctrlState axis gets deflectionDirection = 0 at Earlyish (its
    /// target deflection is then 0: it returns to neutral at its actuator speed) and the value
    /// it had back at BetterLateThanNever of the SAME physics step. Why that field and not
    /// ignorePitch/Yaw/Roll or authorityLimiter:
    ///   * deflectionDirection is a protected float, 1 from the constructor, never persisted,
    ///     never shown in the part menu, not exposed by kRPC: nothing else reads or writes it;
    ///   * ignorePitch/Yaw/Roll and authorityLimiter are KSPFields: saved by a quicksave taken
    ///     while held, shown in the menu, and written by the client itself through kRPC
    ///     (ControlSurface.pitch_enabled, the per-phase fin policy) -- kRPC runs its RPCs in its
    ///     FixedUpdate, inside the Earlyish..BetterLateThanNever window, so a restore could
    ///     silently undo a client write;
    ///   * the price: it neutralizes every axis of the surface, the AutoPilot's roll too when
    ///     the loop writes pitch/yaw but not the roll (the loop arms both on every booster).
    /// The hold never outlives the physics step: a frozen client, a fallback, a release, a
    /// scene change or an exception leave every surface as it was at the next restore point
    /// (BetterLateThanNever, the next tick, Drop, ResetForScene, the tick's catch).
    /// </summary>
    internal static partial class AttitudeV1
    {
        internal const int CapabilitiesVersion = 1;
        internal const int OptionsReplyLength = 8;

        // Capability bits of the frame header (offset 23), set only with the options.
        internal const long CapFamilyBlocking = 1L << 11;
        internal const long CapServeNotConverged = 1L << 12;
        internal const long CapEvidenceAging = 1L << 13;

        static readonly TimingManager.UpdateAction lateAction = OnBetterLateThanNever;
        static bool lateRegistered;

        // ModuleControlSurface.deflectionDirection (protected float), resolved once.
        static bool directionResolved;
        static FieldInfo directionField;
        static readonly object boxedZero = 0f;
        static readonly double[] surfaceScratchPos = new double[9], surfaceScratchNeg = new double[9];

        // ---------------------------------------------------------------- RPC bodies

        /// <summary>
        /// Sets the options of an armed vessel (AttitudeWrite.Option*; 0 = exactly the arming).
        /// Returns [accepted (1), version, options now, supported options, surfaces answering a
        /// ctrlState axis, of which neutralizable, axes they answer (bit 0 pitch, 1 roll, 2 yaw),
        /// attitude tick]; the surface counts are NaN while the vessel is not loaded.
        /// </summary>
        internal static IList<double> SetOptions (string token, int options)
        {
            if (!AttitudeWrite.ValidOptions (options))
                throw new ArgumentOutOfRangeException (nameof (options), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode,
                    "bits : 1 blocage par famille (D124), 2 vieillissement de l'evidence (D125), " +
                    "4 iteree WLS non convergee servie (motif 15), 8 cardans tenus au contact (D181), " +
                    "16 bits 4/8 du masque d'ecriture au coast (D183), 32 couple RCS en vol (D188), " +
                    "64 bit 16 du masque d'ecriture au boostback (D189), 128 cardans au neutre sans poussee " +
                    "et colonnes a la poussee qui vient (D237), 256 autorite de translation libre (D246), " +
                    "512 manette principale au pas physique (D252 K5), 1024 roulis pointe (D252 K6), " +
                    "2048 anneau de references datees (D252 K7), 4096 allocation moteur par moteur en ombre (D264), " +
                    "8192 allocation moteur par moteur qui ecrit (D264 etape 2, implique 4096), " +
                    "16384 RCS tire comme ModuleRCS, commande que le RCS sert (roulis ou tangage/lacet retenu, D267)"));
            var entry = RequireToken (token);
            if ((options & AttitudeWrite.OptionFamilyBlocking) == 0)
                RestoreSurfaces (entry);
            // without option 512 no order of the main throttle survives.
            if ((options & AttitudeWrite.OptionMainThrottle) == 0)
                entry.Th.Reset ();
            // nor the reference ring without option 2048.
            if ((options & AttitudeWrite.OptionReferenceRing) == 0)
                entry.Ring.Reset ();
            // nor the shadow allocation's block and ring without option 4096.
            // Step 2: without option 8192 no throttle leased by the allocation survives (they follow the main throttle).
            // Option 8192 set or taken off: its write starts again from nothing (ResumeSteps sound solves before the
            // allocation takes the hand), and without it no throttle leased by the allocation survives (they follow the
            // main throttle).
            if (((options ^ entry.Options) & AttitudeWrite.OptionEngineAllocationWrite) != 0) {
                AllocationReleaseThrottles (entry);
                entry.Al.ResetWrite ();
            }
            if (!AttitudeWrite.AllocationOn (options))
                entry.Al.Reset ();
            else if (!AttitudeWrite.AllocationOn (entry.Options))
                AllocationWarmUp ();
            // option 16384 set or taken off: its state starts again.
            if (((options ^ entry.Options) & AttitudeWrite.OptionRcsCombined) != 0)
                entry.Rc.Reset ();
            entry.Options = options;
            if ((options & AttitudeWrite.OptionFamilyBlocking) != 0)
                RegisterLate ();
            double answering = double.NaN, neutralizable = double.NaN, axes = double.NaN;
            var vessel = Resolve (entry);
            if (vessel != null) {
                answering = neutralizable = 0.0;
                int mask = 0;
                for (int p = 0; p < vessel.parts.Count; p++) {
                    var part = vessel.parts [p];
                    if (part == null)
                        continue;
                    for (int m = 0; m < part.Modules.Count; m++) {
                        var surface = part.Modules [m] as ModuleControlSurface;
                        if (surface == null || surface is ModuleAeroSurface)
                            continue;
                        int a = AttitudeGeometry.SurfaceAnsweredAxes (surface);
                        if (a == 0)
                            continue;
                        answering++;
                        mask |= a;
                        if (CanNeutralize (surface))
                            neutralizable++;
                    }
                }
                axes = mask;
            }
            return new List<double> (OptionsReplyLength) {
                1.0, AttitudeWrite.OptionsVersion, entry.Options, AttitudeWrite.OptionsSupported,
                answering, neutralizable, axes, ownTick
            };
        }

        /// <summary>
        /// What this build can do, without a token: [version (1), supported options
        /// (AttitudeOptionsV1), evidence block protocol, its schema, its field count, surface
        /// neutralizer available (1: ModuleControlSurface.deflectionDirection found)].
        /// </summary>
        internal static IList<double> Capabilities ()
        {
            return new List<double> {
                CapabilitiesVersion, AttitudeWrite.OptionsSupported,
                AttitudeTranslation.EvidenceBlockProtocol, AttitudeTranslation.EvidenceBlockSchema,
                AttitudeTranslation.EvidenceFields.Length, DirectionField () != null ? 1.0 : 0.0
            };
        }

        /// <summary>
        /// The field names of a named layout, in order (append-only; the schema says how many):
        /// "translation_evidence" as before, and since refactor R1 every layout of the Core
        /// layout registry (frames, returns, blocks: conn.bridge.layouts_v1), sections
        /// concatenated. Unknown: NOT_FOUND.
        /// </summary>
        internal static IList<string> Layout (string name)
        {
            if (name == AttitudeTranslation.EvidenceLayoutName)
                return new List<string> (AttitudeTranslation.EvidenceFields);
            var layout = LayoutRegistry.Find (name);
            if (layout != null)
                return layout.FieldNames ();
            throw new ArgumentException (BridgeErrors.Tag (BridgeErrors.NotFound, "bloc inconnu : " +
                name + " (connus : " + string.Join (", ", LayoutRegistry.Names ().ToArray ()) + ")"),
                nameof (name));
        }

        // ---------------------------------------------------------------- the tick

        /// <summary>Capability bits of the options of this entry (0 without options).</summary>
        static long OptionCaps (Entry e)
        {
            long caps = 0;
            if ((e.Options & AttitudeWrite.OptionFamilyBlocking) != 0)
                caps |= CapFamilyBlocking;
            if ((e.Options & AttitudeWrite.OptionServeNotConverged) != 0)
                caps |= CapServeNotConverged;
            if ((e.Options & AttitudeWrite.OptionEvidenceAging) != 0)
                caps |= CapEvidenceAging;
            return caps;
        }

        /// <summary>
        /// the surfaces leave B. AddSurface still runs, on scratch columns, for motif 14
        /// (now informational); the axes every surface answers, and those of the surfaces that
        /// cannot be neutralized, are kept for this step.
        /// </summary>
        static long ClassifySurfaces (Entry e)
        {
            long motif = 0;
            int answering = 0, unneutralizable = 0;
            for (int k = 0; k < 9; k++) {
                surfaceScratchPos [k] = 0.0;
                surfaceScratchNeg [k] = 0.0;
            }
            for (int i = 0; i < e.Surfaces.Count; i++) {
                var surface = e.Surfaces [i];
                if (surface == null)
                    continue;
                if (AttitudeGeometry.AddSurface (surface, surfaceScratchPos, surfaceScratchNeg))
                    motif |= MotifSurfaceApprox;
                int axes = AttitudeGeometry.SurfaceAnsweredAxes (surface);
                // Lot 3, 3.1: a surface ENGAGED by SurfaceModelV1 (armed) is a modeled actuator: it
                // neither asks for the neutralization nor gates the roll (false when the model is off).
                if (axes == 0 || SurfaceEngaged (e, i))
                    continue;
                answering |= axes;
                if (!CanNeutralize (surface))
                    unneutralizable |= axes;
            }
            e.SurfaceAxes = answering;
            e.SurfaceAxesUnneutralizable = unneutralizable;
            return motif;
        }

        /// <summary>
        /// deflectionDirection = 0 on every surface that answers a ctrlState axis and can
        /// be neutralized, for this physics step. False (everything already put back) when the
        /// field cannot be written: the caller then writes no ctrlState this step.
        /// </summary>
        static bool HoldSurfacesNeutral (Entry e)
        {
            var field = DirectionField ();
            if (field == null)
                return false;
            try {
                for (int i = 0; i < e.Surfaces.Count; i++) {
                    var surface = e.Surfaces [i];
                    if (surface == null || AttitudeGeometry.SurfaceAnsweredAxes (surface) == 0 ||
                            !CanNeutralize (surface) || SurfaceEngaged (e, i))
                        continue;
                    object original = field.GetValue (surface);
                    if (original is float && (float)original == 0f)
                        continue;           // already neutral: nothing to hold or restore
                    field.SetValue (surface, boxedZero);
                    e.Neutralized.Add (surface);
                    e.NeutralizedOriginal.Add (original);
                }
                return true;
            } catch (Exception exc) {
                if (++e.FbwExceptions <= 5)
                    BridgeLog.Error ("AttitudeV1 neutralisation des surfaces: " + exc.Message);
                RestoreSurfaces (e);
                return false;
            }
        }

        /// <summary>
        /// Puts back every deflectionDirection this entry holds. Never throws; a destroyed
        /// surface is skipped (its module is gone with it).
        /// </summary>
        static void RestoreSurfaces (Entry e)
        {
            if (e.Neutralized.Count == 0)
                return;
            var field = DirectionField ();
            for (int i = 0; i < e.Neutralized.Count; i++) {
                var surface = e.Neutralized [i];
                if (surface == null || field == null)
                    continue;
                try {
                    field.SetValue (surface, e.NeutralizedOriginal [i]);
                } catch (Exception exc) {
                    BridgeLog.Error ("AttitudeV1 restauration d'une surface: " + exc.Message);
                }
            }
            e.Neutralized.Clear ();
            e.NeutralizedOriginal.Clear ();
        }

        static void RestoreAllSurfaces ()
        {
            foreach (var pair in entries)
                RestoreSurfaces (pair.Value);
        }

        /// <summary>
        /// BetterLateThanNever: the end of the physics step, after every part's FixedUpdate. Lot 3,
        /// 3.1: SurfaceModelV1's capture first (it must know which surfaces were held this step;
        /// nothing without the model), then the restore.
        /// </summary>
        static void OnBetterLateThanNever ()
        {
            try {
                CaptureAllSurfaces ();
            } catch (Exception exc) {
                BridgeLog.Error ("AttitudeV1 BetterLateThanNever (surface model): " + exc);
            }
            try {
                RestoreAllSurfaces ();
            } catch (Exception exc) {
                BridgeLog.Error ("AttitudeV1 BetterLateThanNever: " + exc);
            }
        }

        static void RegisterLate ()
        {
            if (lateRegistered)
                return;
            TimingManager.FixedUpdateAdd (TimingManager.TimingStage.BetterLateThanNever, lateAction);
            lateRegistered = true;
        }

        static void UnregisterLate ()
        {
            if (!lateRegistered)
                return;
            try {
                TimingManager.FixedUpdateRemove (TimingManager.TimingStage.BetterLateThanNever,
                    lateAction);
            } catch (Exception exc) {
                BridgeLog.Error ("AttitudeV1: FixedUpdateRemove (BetterLateThanNever): " + exc.Message);
            }
            lateRegistered = false;
        }

        /// <summary>A surface whose target deflection deflectionDirection = 0 brings to rest: not deployed.</summary>
        static bool CanNeutralize (ModuleControlSurface surface)
        {
            return DirectionField () != null && !surface.deploy;
        }

        static FieldInfo DirectionField ()
        {
            if (!directionResolved) {
                directionResolved = true;
                try {
                    var f = typeof (ModuleControlSurface).GetField ("deflectionDirection",
                        BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                    directionField = f != null && f.FieldType == typeof (float) ? f : null;
                } catch {
                    directionField = null;
                }
            }
            return directionField;
        }
    }
}
