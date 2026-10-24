using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using UnityEngine;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// AttitudeV1, refactor R1 (plan 2.4): the effector registry of an armed vessel, the part
    /// that touches the game. The rules are pure (EffectorRegistry.cs).
    ///
    /// BORN CLOSED: AttitudeEffectorsV1 (mode 1) arms it for one arming; every arming resets it
    /// to 0, and with 0 the tick takes exactly the paths of the build before (RuleOptions is
    /// then the arming's options, unchanged).
    ///
    /// With the registry the family mechanics of option 1 apply (the surfaces leave B, are held
    /// neutral while the loop writes an axis they answer, model motifs 13/14/20 are
    /// informational), with the registry's decisions: a ctrlState axis is frozen only for what
    /// is unmodeled ON THAT AXIS (an RCS module with fullThrust that answers it, a surface that
    /// answers it and cannot be neutralized; all three when the fly-by-wire order is not
    /// verified), and an RCS module in precision mode without useLever keeps its columns,
    /// scaled by its precisionFactor (exact), instead of freezing the three axes.
    /// </summary>
    internal static partial class AttitudeV1
    {
        // Capability bit of the frame header (offset 23), set only with the registry.
        internal const long CapEffectorRegistry = 1L << 14;
        internal const int EffectorsReplyLength = 4;

        static readonly EffectorTable emptyEffectors = new EffectorTable ();
        static readonly double[] rcsModulePos = new double[9], rcsModuleNeg = new double[9];

        /// <summary>
        /// The options as the pure write rules see them: the registry implies option 1, and so does
        /// SurfaceModelV1 armed (lot 3, 3.1: what is not engaged is neutralized as with option 1).
        /// </summary>
        static int RuleOptions (Entry e)
        {
            return e.EffectorRegistry || e.Sm.Mode == SurfaceModel.ModeArmed
                ? e.Options | AttitudeWrite.OptionFamilyBlocking : e.Options;
        }

        // ---------------------------------------------------------------- RPC bodies

        /// <summary>
        /// Arms (1) or disarms (0) the effector registry of an armed vessel. Returns [accepted (1),
        /// protocol of the table (1), mode now, attitude tick].
        /// </summary>
        internal static IList<double> EffectorsConfig (string token, int mode)
        {
            if (!EffectorRegistry.ValidMode (mode))
                throw new ArgumentOutOfRangeException (nameof (mode), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode, "0 arret, 1 registre des effecteurs"));
            var entry = RequireToken (token);
            bool on = mode == EffectorRegistry.ModeOn;
            if (!on && entry.EffectorRegistry && (entry.Options & AttitudeWrite.OptionFamilyBlocking) == 0)
                RestoreSurfaces (entry);
            entry.EffectorRegistry = on;
            entry.Effectors.Enabled = false;     // valid again at the next tick that fills it
            if (on)
                RegisterLate ();
            return new List<double> (EffectorsReplyLength) {
                1.0, EffectorRegistry.Protocol, on ? 1.0 : 0.0, ownTick
            };
        }

        /// <summary>The registry table of the last step of an armed vessel (same layout when off or absent).</summary>
        internal static IList<double> EffectorsRead (string persistentId)
        {
            uint pid = VesselLookup.ParsePersistentId (persistentId);
            Entry entry;
            if (!entries.TryGetValue (pid, out entry))
                return emptyEffectors.Reply ();
            return entry.Effectors.Reply ();
        }

        // ---------------------------------------------------------------- the tick

        /// <summary>
        /// One RCS module under the registry: its columns into the RCS family, scaled by
        /// EffectorRegistry.RcsScale; its status and the axes it answers recorded. Returns what
        /// AddRcsColumns returns (false: precision mode or fullThrust, for motif 13).
        /// </summary>
        static bool AddRcsRegistry (Entry e, ModuleRCS rcs, Vector3 com, Transform reference,
            bool precision)
        {
            for (int k = 0; k < 9; k++) {
                rcsModulePos [k] = 0.0;
                rcsModuleNeg [k] = 0.0;
            }
            bool modeled = AttitudeGeometry.AddRcsColumns (rcs, com, reference, precision,
                rcsModulePos, rcsModuleNeg);
            bool contributes = false;
            for (int k = 0; k < 9 && !contributes; k++)
                contributes = rcsModulePos [k] != 0.0 || rcsModuleNeg [k] != 0.0;
            if (!contributes)
                return modeled;
            var t = e.Effectors;
            double scale = EffectorRegistry.RcsScale (precision, rcs.useLever, rcs.precisionFactor);
            int status = EffectorRegistry.RcsStatus (precision, rcs.useLever, rcs.precisionFactor,
                rcs.fullThrust);
            int axes = EffectorRegistry.Axes (rcs.enablePitch, rcs.enableRoll, rcs.enableYaw);
            for (int k = 0; k < 9; k++) {
                t.RcsPos [k] += scale * rcsModulePos [k];
                t.RcsNeg [k] += scale * rcsModuleNeg [k];
            }
            if (status == EffectorRegistry.StatusUnmodeled)
                t.RcsUnmodeledAxes |= axes;
            else if (status == EffectorRegistry.StatusApproximate)
                t.RcsApproximateAxes |= axes;
            t.Family (EffectorRegistry.FamilyRcs, status, 1);
            if (scale != 1.0)
                t.FamilyScale [EffectorRegistry.FamilyRcs] = scale;
            return modeled;
        }

        /// <summary>
        /// The table of this step: every allocation column with its family, status and action;
        /// per family the worst status, the computed authority (N.m) and alpha (rad/s2).
        /// Called after the frozen axes are set, before the law. Writes nothing to the game.
        /// </summary>
        static void FillEffectorTable (Entry e, AttitudeInput inp, int n, int nGimbalColumns,
            bool planned)
        {
            var t = e.Effectors;
            t.FrozenAxes = e.FrozenAxes;
            t.EnsureColumns (n);
            int firstMotif = nGimbalColumns + 3;
            for (int j = 0; j < n; j++) {
                double bx = inp.B [j], by = inp.B [n + j], bz = inp.B [2 * n + j];
                double half = 0.5 * (inp.UMax [j] - inp.UMin [j]);
                if (j < nGimbalColumns) {
                    t.Column (j, KindGimbal, EffectorRegistry.FamilyGimbal, j % 2,
                        EffectorRegistry.StatusExact,
                        half > 0.0 ? EffectorRegistry.ActionAllocated : EffectorRegistry.ActionFrozen,
                        bx, by, bz, half);
                } else if (j < firstMotif) {
                    int a = j - nGimbalColumns;
                    int bit = 1 << a;
                    int status = EffectorRegistry.StatusExact;
                    int action = EffectorRegistry.ActionAllocated;
                    if ((e.FrozenAxes & bit) != 0) {
                        status = EffectorRegistry.StatusUnmodeled;
                        action = EffectorRegistry.ActionFrozen;
                    } else if ((t.RcsApproximateAxes & bit) != 0) {
                        status = EffectorRegistry.StatusApproximate;
                        action = EffectorRegistry.ActionBounded;
                    }
                    // The ctrlState column is the RCS and wheels' (the surfaces are out of B):
                    // published under the RCS family when the vessel has RCS, else the wheels'.
                    int family = t.FamilyCount [EffectorRegistry.FamilyRcs] > 0
                        ? EffectorRegistry.FamilyRcs : EffectorRegistry.FamilyWheel;
                    t.Column (j, KindCtrlState, family, a, status, action, bx, by, bz, half);
                } else {
                    t.Column (j, KindDiffMotif, EffectorRegistry.FamilyDiffMotif,
                        j == firstMotif ? 0 : 2, EffectorRegistry.StatusExact,
                        half > 0.0 ? EffectorRegistry.ActionAllocated : EffectorRegistry.ActionFrozen,
                        bx, by, bz, half);
                }
            }
            if (nGimbalColumns > 0)
                t.Family (EffectorRegistry.FamilyGimbal, EffectorRegistry.StatusExact, nGimbalColumns / 2);
            if (planned && n > firstMotif)
                t.Family (EffectorRegistry.FamilyDiffMotif, EffectorRegistry.StatusExact, 1);
            int answering = 0, engaged = 0;
            for (int i = 0; i < e.Surfaces.Count; i++)
                if (e.Surfaces [i] != null && AttitudeGeometry.SurfaceAnsweredAxes (e.Surfaces [i]) != 0) {
                    // Lot 3, 3.1: a surface engaged by SurfaceModelV1 (armed) is exact.
                    if (SurfaceEngaged (e, i))
                        engaged++;
                    else
                        answering++;
                }
            if (answering > 0)
                t.Family (EffectorRegistry.FamilySurface, EffectorRegistry.StatusUnmodeled, answering);
            if (engaged > 0)
                t.Family (EffectorRegistry.FamilySurface, EffectorRegistry.StatusExact, engaged);

            EffectorRegistry.AddAuthority (inp.B, n, inp.UMin, inp.UMax, 0, nGimbalColumns,
                t.Authority [EffectorRegistry.FamilyGimbal]);
            EffectorRegistry.AddCtrlAuthority (t.RcsPos, t.RcsNeg, e.FrozenAxes,
                t.Authority [EffectorRegistry.FamilyRcs]);
            EffectorRegistry.AddCtrlAuthority (t.WheelPos, t.WheelNeg, e.FrozenAxes,
                t.Authority [EffectorRegistry.FamilyWheel]);
            // The surfaces' from GetPotentialTorque (q and Mach dependent): comparison only; with
            // engaged surfaces (lot 3, 3.1), their exact static authority instead.
            if (engaged > 0) {
                for (int a = 0; a < 3; a++)
                    t.Authority [EffectorRegistry.FamilySurface] [a] = e.Sm.Authority [a];
            } else {
                EffectorRegistry.AddCtrlAuthority (surfaceScratchPos, surfaceScratchNeg, 0,
                    t.Authority [EffectorRegistry.FamilySurface]);
            }
            if (planned && n > firstMotif)
                EffectorRegistry.AddAuthority (inp.B, n, inp.UMin, inp.UMax, firstMotif, n,
                    t.Authority [EffectorRegistry.FamilyDiffMotif]);
            for (int f = 0; f < EffectorRegistry.FamilyCount; f++)
                EffectorRegistry.Alpha (t.Authority [f], inp.Inertia, t.AlphaRows [f]);
            t.Enabled = true;
        }
    }
}
