using System;

namespace KRPC.Bridge.Model
{
    // -----------------------------------------------------------------------------------
    // Refactor R1 (plan 2.4, audit C §4.2): the EFFECTOR REGISTRY of AttitudeV1. Born closed:
    // armed per vessel by AttitudeEffectorsV1 (mode 1), reset to 0 at every arming; with 0
    // nothing below runs and every frame, return and write is the one of the build before.
    //
    // PURE, like AttitudeWrite.cs: System only, compiled into build/tests.
    //
    // Every column of the allocation gets a MODEL STATUS, and a family of actuators is
    // sanctioned only on the columns it is about, never the vessel:
    //
    //   exact        the column is the model of what the actuator does (gimbals: validated in
    //                flight, slope 0.996-1.008; RCS and reaction wheels outside the cases below;
    //                the differential-thrust motifs). Allocated.
    //   measured     a column whose scale comes from an in-flight measurement (none yet:
    //                reserved for the surfaces' mono-flight identification, plan 3.1).
    //   approximate  a column known to be an UPPER bound of the real effectiveness on its own
    //                axis (b_model >= b_real: INDI then converges monotonically, slowly rather
    //                than overshooting). Allocated, under the per-axis residual guard. Case: RCS
    //                in precision mode with useLever (each nozzle's thrust divided by its lever
    //                distance when above 1: only reduced).
    //   unmodeled    no model, or not a bound: the column must not drive the actuator. Its
    //                effector is NEUTRALIZED while the loop writes an axis it answers (control
    //                surfaces: deflectionDirection = 0 for the physics step, as option 1), or,
    //                when it cannot be, the ctrlState AXES it answers are frozen (B = 0, bounds
    //                at the served u0, not written) - those axes only. Cases: control surfaces
    //                (GetPotentialTorque is a torque times a lever, component-wise), RCS with
    //                fullThrust (thrust jumps to 1 above fullThrustMin: more than the column),
    //                and the ctrlState WRITE path itself when the fly-by-wire order is not
    //                verified (a later callback may overwrite what we write: all three axes).
    //
    // RCS in precision mode WITHOUT useLever is EXACT, scaled: ModuleRCS.FixedUpdate multiplies
    // every nozzle's thrust fraction by precisionFactor after the same clamp, so the realized
    // column is precisionFactor times the full one (decompiled KSP 1.12.5). Option 1 froze all
    // three ctrlState axes there.
    //
    // AUTHORITY PER FAMILY, computed every step, never a threshold: A_f,a = sum over the
    // family's columns j of |B_a,j| times the half span of j ((u_max - u_min) / 2: 0 for a
    // frozen column), in N.m on body axis a; alpha_f,a = A_f,a / I_a in rad/s2. The surfaces'
    // is published from GetPotentialTorque (q and Mach dependent, status unmodeled), for
    // comparison only.
    //
    // The decisions use the statuses only; the authority is published (AttitudeEffectorsReadV1)
    // so the client and the flight logs see, per step, what each family can do.
    // -----------------------------------------------------------------------------------
    internal static class EffectorRegistry
    {
        internal const int Protocol = 1;
        internal const int Schema = 1;
        internal const int ModeOff = 0;
        internal const int ModeOn = 1;

        // Model statuses (the order is the severity).
        internal const int StatusExact = 0;
        internal const int StatusMeasured = 1;
        internal const int StatusApproximate = 2;
        internal const int StatusUnmodeled = 3;
        internal const int StatusAbsent = -1;    // the family has no actuator this step

        // Families (row order of the table).
        internal const int FamilyGimbal = 0;
        internal const int FamilyRcs = 1;
        internal const int FamilyWheel = 2;
        internal const int FamilySurface = 3;
        internal const int FamilyDiffMotif = 4;
        internal const int FamilyCount = 5;

        // Column actions.
        internal const int ActionAllocated = 0;
        internal const int ActionFrozen = 1;       // B = 0, bounds at u0, not written
        internal const int ActionBounded = 2;      // allocated, approximate (upper bound)

        internal const int HeaderStride = 10;
        internal const int FamilyStride = 12;
        internal const int ColumnStride = 10;

        internal static readonly string[] FamilyNames = { "gimbal", "rcs", "wheel", "surface", "diff_motif" };
        internal static readonly string[] StatusNames = { "exact", "measured", "approximate", "unmodeled" };

        internal static bool ValidMode (int mode)
        {
            return mode == ModeOff || mode == ModeOn;
        }

        /// <summary>
        /// The scale of one RCS module's columns: precisionFactor in precision mode without
        /// useLever when it is a finite fraction in (0, 1] (exact, scaled); 1 otherwise.
        /// </summary>
        internal static double RcsScale (bool precisionMode, bool useLever, double precisionFactor)
        {
            if (!precisionMode || useLever)
                return 1.0;
            if (!(precisionFactor > 0.0) || precisionFactor > 1.0 || double.IsInfinity (precisionFactor))
                return 1.0;
            return precisionFactor;
        }

        /// <summary>The status of one RCS module that contributes to an axis.</summary>
        internal static int RcsStatus (bool precisionMode, bool useLever, double precisionFactor, bool fullThrust)
        {
            if (fullThrust)
                return StatusUnmodeled;
            if (!precisionMode)
                return StatusExact;
            if (useLever)
                return StatusApproximate;
            return RcsScale (true, false, precisionFactor) == precisionFactor ? StatusExact : StatusUnmodeled;
        }

        /// <summary>Worse of two statuses (Absent is the best).</summary>
        internal static int Worse (int a, int b)
        {
            return a > b ? a : b;
        }

        /// <summary>
        /// ctrlState axes frozen this step (bit 0 pitch, 1 roll, 2 yaw): all three when the
        /// fly-by-wire order is not verified (armed); the axes an unmodeled RCS module answers;
        /// the axes a surface that cannot be neutralized answers.
        /// </summary>
        internal static int FrozenAxes (bool writePathUnverified, int unmodeledRcsAxes,
            int unneutralizableSurfaceAxes)
        {
            if (writePathUnverified)
                return 7;
            return (unmodeledRcsAxes | unneutralizableSurfaceAxes) & 7;
        }

        /// <summary>The ctrlState axes of a module's pitch/roll/yaw enable flags (bit 0, 1, 2).</summary>
        internal static int Axes (bool pitch, bool roll, bool yaw)
        {
            return (pitch ? 1 : 0) | (roll ? 2 : 0) | (yaw ? 4 : 0);
        }

        /// <summary>
        /// Authority (N.m) of a set of columns on each body axis: sum over columns j in
        /// [from, to) of |B[a * n + j]| times (uMax_j - uMin_j) / 2. Adds into authority[0..2].
        /// </summary>
        internal static void AddAuthority (double[] b, int n, double[] uMin, double[] uMax, int from,
            int to, double[] authority)
        {
            for (int j = from; j < to; j++) {
                double half = 0.5 * (uMax [j] - uMin [j]);
                if (!(half > 0.0))
                    continue;
                for (int a = 0; a < 3; a++) {
                    double v = Math.Abs (b [a * n + j]);
                    if (!double.IsNaN (v) && !double.IsInfinity (v))
                        authority [a] += v * half;
                }
            }
        }

        /// <summary>
        /// Authority (N.m) of ctrlState sub-columns (pos/neg halves, axis a column k at [3k + a])
        /// on each body axis, for the axes not frozen (half span 1: bounds [-1, 1]).
        /// </summary>
        internal static void AddCtrlAuthority (double[] bPos, double[] bNeg, int frozenAxes,
            double[] authority)
        {
            for (int k = 0; k < 3; k++) {
                if ((frozenAxes & (1 << k)) != 0)
                    continue;
                for (int a = 0; a < 3; a++) {
                    double v = Math.Abs (0.5 * (bPos [3 * k + a] + bNeg [3 * k + a]));
                    if (!double.IsNaN (v) && !double.IsInfinity (v))
                        authority [a] += v;
                }
            }
        }

        /// <summary>alpha = A / I per axis (NaN where I is not positive).</summary>
        internal static void Alpha (double[] authority, double[] inertia, double[] alpha)
        {
            for (int a = 0; a < 3; a++)
                alpha [a] = inertia [a] > 0.0 ? authority [a] / inertia [a] : double.NaN;
        }
    }

    /// <summary>
    /// The registry of one armed vessel for the LAST step, preallocated: written by the tick,
    /// read by AttitudeEffectorsReadV1. Pure (System only).
    /// </summary>
    internal sealed class EffectorTable
    {
        internal bool Enabled;
        internal long Tick = -1;
        internal readonly int[] FamilyStatus = new int[EffectorRegistry.FamilyCount];
        internal readonly int[] FamilyCount = new int[EffectorRegistry.FamilyCount];
        internal readonly double[] FamilyScale = new double[EffectorRegistry.FamilyCount];
        internal readonly double[][] Authority = NewRows ();
        internal readonly double[][] AlphaRows = NewRows ();
        internal readonly int[] FamilyColumns = new int[EffectorRegistry.FamilyCount];
        internal readonly int[] FamilyFrozen = new int[EffectorRegistry.FamilyCount];
        internal int FrozenAxes;
        internal int NeutralizedSurfaces;

        // Per-family ctrlState sub-columns of this step (RCS scaled, wheels, surfaces).
        internal readonly double[] RcsPos = new double[9], RcsNeg = new double[9];
        internal readonly double[] WheelPos = new double[9], WheelNeg = new double[9];
        internal int RcsUnmodeledAxes;
        internal int RcsApproximateAxes;

        // Columns of the allocation: kind, axis, status, action, B (3), half span, family.
        internal int Columns;
        internal double[] ColumnRows = new double[0];

        static double[][] NewRows ()
        {
            var rows = new double[EffectorRegistry.FamilyCount][];
            for (int i = 0; i < rows.Length; i++)
                rows [i] = new double[3];
            return rows;
        }

        /// <summary>Start of a step: everything absent, zero authority.</summary>
        internal void Begin (long tick)
        {
            Tick = tick;
            for (int f = 0; f < EffectorRegistry.FamilyCount; f++) {
                FamilyStatus [f] = EffectorRegistry.StatusAbsent;
                FamilyCount [f] = 0;
                FamilyScale [f] = 1.0;
                FamilyColumns [f] = 0;
                FamilyFrozen [f] = 0;
                for (int a = 0; a < 3; a++) {
                    Authority [f] [a] = 0.0;
                    AlphaRows [f] [a] = double.NaN;
                }
            }
            for (int k = 0; k < 9; k++) {
                RcsPos [k] = 0.0;
                RcsNeg [k] = 0.0;
                WheelPos [k] = 0.0;
                WheelNeg [k] = 0.0;
            }
            RcsUnmodeledAxes = 0;
            RcsApproximateAxes = 0;
            FrozenAxes = 0;
            NeutralizedSurfaces = 0;
            Columns = 0;
        }

        internal void Family (int family, int status, int count)
        {
            FamilyStatus [family] = EffectorRegistry.Worse (FamilyStatus [family], status);
            FamilyCount [family] += count;
        }

        internal void EnsureColumns (int n)
        {
            if (ColumnRows.Length < n * EffectorRegistry.ColumnStride)
                ColumnRows = new double[(n + 8) * EffectorRegistry.ColumnStride];
        }

        internal void Column (int j, int kind, int family, int axis, int status, int action,
            double bx, double by, double bz, double halfSpan)
        {
            int at = j * EffectorRegistry.ColumnStride;
            ColumnRows [at] = j;
            ColumnRows [at + 1] = kind;
            ColumnRows [at + 2] = family;
            ColumnRows [at + 3] = axis;
            ColumnRows [at + 4] = status;
            ColumnRows [at + 5] = action;
            ColumnRows [at + 6] = bx;
            ColumnRows [at + 7] = by;
            ColumnRows [at + 8] = bz;
            ColumnRows [at + 9] = halfSpan;
            if (j + 1 > Columns)
                Columns = j + 1;
            if (family >= 0 && family < EffectorRegistry.FamilyCount) {
                FamilyColumns [family]++;
                if (action == EffectorRegistry.ActionFrozen)
                    FamilyFrozen [family]++;
            }
        }

        /// <summary>
        /// The AttitudeEffectorsReadV1 reply: header [protocol, schema, enabled, attitude tick,
        /// families (5), family stride (12), columns, column stride (10), frozen ctrlState axes,
        /// surfaces neutralized], then one row per family [family, status (-1 absent),
        /// modules, scale, authority x y z (N.m), alpha x y z (rad/s2), columns, frozen
        /// columns], then one row per allocation column [column, kind, family, axis, status,
        /// action, b x y z, half span]. The layout is the same whether the registry is armed or
        /// not (NaN values and no column rows while it is not).
        /// </summary>
        internal System.Collections.Generic.List<double> Reply ()
        {
            int families = EffectorRegistry.FamilyCount;
            int columns = Enabled ? Columns : 0;
            var output = new System.Collections.Generic.List<double> (EffectorRegistry.HeaderStride +
                families * EffectorRegistry.FamilyStride + columns * EffectorRegistry.ColumnStride);
            output.Add (EffectorRegistry.Protocol);
            output.Add (EffectorRegistry.Schema);
            output.Add (Enabled ? 1.0 : 0.0);
            output.Add (Tick);
            output.Add (families);
            output.Add (EffectorRegistry.FamilyStride);
            output.Add (columns);
            output.Add (EffectorRegistry.ColumnStride);
            output.Add (Enabled ? FrozenAxes : double.NaN);
            output.Add (Enabled ? NeutralizedSurfaces : double.NaN);
            for (int f = 0; f < families; f++) {
                output.Add (f);
                output.Add (Enabled ? FamilyStatus [f] : double.NaN);
                output.Add (Enabled ? FamilyCount [f] : double.NaN);
                output.Add (Enabled ? FamilyScale [f] : double.NaN);
                for (int a = 0; a < 3; a++)
                    output.Add (Enabled ? Authority [f] [a] : double.NaN);
                for (int a = 0; a < 3; a++)
                    output.Add (Enabled ? AlphaRows [f] [a] : double.NaN);
                output.Add (Enabled ? FamilyColumns [f] : double.NaN);
                output.Add (Enabled ? FamilyFrozen [f] : double.NaN);
            }
            for (int i = 0; i < columns * EffectorRegistry.ColumnStride; i++)
                output.Add (ColumnRows [i]);
            return output;
        }
    }
}
