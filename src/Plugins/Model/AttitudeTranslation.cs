using System;

namespace KRPC.Bridge.Model
{
    // -----------------------------------------------------------------------------------
    // AttitudeV1 lateral force (chantier 19 suite): differential thrust and translation by
    // a COMMON gimbal deflection, as a pre-allocation layer in front of the unchanged law.
    //
    // PURE, like AttitudeMath.cs and AttitudeWrite.cs: System only, no Unity, no KSP, no kRPC
    // type. Compiled three times (the plugin, build/verify, build/tests). The law itself
    // (AttitudeMath) is NOT touched: this file only prepares extra columns and shifted
    // bounds for its weighted least squares, and turns the result into actuator commands.
    //
    // Frame: vessel body (x right = pitch axis, y nose = roll axis, z bottom = yaw axis).
    //
    // Differential thrust. Two MOTIF columns (pitch, yaw) rather than one per engine:
    //   motif_p,i = (z_i - zbar) / max|z - zbar|,  motif_y,i = (x_i - xbar) / max|x - xbar|
    //   with (xbar, zbar) the slope-weighted mean of the engines' nozzle centroids, so that
    //   sum_i s_i motif_i = 0: the TOTAL thrust is conserved (s_i = dT_i / d(throttle_i)).
    //   Engine i is commanded throttle_i = main + d_p motif_p,i + d_y motif_y,i; the column
    //   of motif k is B_k = sum_i motif_k,i s_i (r_i x dir_i) (N.m per unit amplitude).
    //   A vessel whose centroids sit on the axis (lever < MinLeverArmM on either motif) has
    //   no differential authority: refused.
    // Translation. The lateral acceleration the nose does not give (demand - (T/m) horiz(nose))
    //   is produced by tilting every nozzle by the SAME body rotation phi (about body x and
    //   z), converted into each gimbal's own x/y axes. The torque of the REALIZED common
    //   deflection (modelled by the lease's own Lerp ramp) is cancelled in advance by the
    //   differential motifs (feed-forward); the law's allocation then works on the ATTITUDE
    //   part only, around these offsets (shifted bounds).
    //
    // Computed levers (bit 4, modes 5 and 7; precision of the heavy
    // booster). Born closed: modes 0, 1 and 3 never reach any of it.
    //   * The motif lever criterion is the numerical resolution of the levers
    //     (LeverResolution), not the fixed 0.25 m: the heavy booster's 8 clusters (0.133 m)
    //     are accepted.
    //   * The RCS translation (ctrlState.X / .Y) is a lever with its own force AND torque
    //     columns (RcsTranslationColumns, the ModuleRCS rule).
    //   * The common tilt, the RCS translation and the differential are allocated TOGETHER
    //     (JointTranslation: lateral force wanted, zero torque, bounded least squares with the
    //     law's own active-set solver). On the heavy booster the RCS sits 37 m ABOVE the CoM
    //     and the nozzles 10.8 m below: a force in the same direction from both has opposite
    //     torques, so they add their forces while cancelling their torques.
    //   * The published authority is the largest lateral acceleration WITHOUT torque that
    //     allocation reaches, the smallest over the four body directions.
    //
    // Honest translation (bits 16 and 32, on top of bit 4; modes 21/23, 37/39, 53/55). Born
    // closed: modes 0-7 never reach any of it. Measured on the heavy booster (flights
    // a recorded flight and _172225, tower capture; _142810, pad): the body's lateral
    // force along the announced translation is -0.12 / 0.43 / 0.94 x the announced one, the
    // gimbals hold -0.17 / 0.62 / 1.04 of the common tilt (INDI's attitude part -1.17 x the
    // common on 183740). In the capture the AutoPilot aligns the ROLL (ctrlState.roll +-1, the
    // law writes pitch/yaw only) and ModuleRCS combines (pitch, roll, yaw) into ONE rotation
    // vector: on a ring 37 m above the CoM the slightest pitch/yaw turns the lever and fires the
    // whole ring (1.73-1.76 MN.m realized for 0.36 modelled, x 4.8). The law's RCS becomes a
    // bang-bang, the X/Y translation is clipped on the nozzles the rotation fills, the tilt's
    // torque is left unbalanced and INDI cancels the tilt with the gimbals.
    //   * Bit 16, SHARED RCS: (1) the RCS translation columns count only the nozzles the
    //     translation drives linearly over its span under the served rotation (rotation fraction
    //     rho by ModuleRCS's combined rule, RcsRotationFractions, held RcsHoldSeconds:
    //     rho + FracRcs f <= 1, RcsTranslationColumnsHeadroom), each channel keeping the WEAKER of
    //     its + and - columns; (2) the law's RCS pitch / yaw column is left out while ModuleRCS
    //     answers a small command with more than twice the column (RcsRotationNonlinear); (3)
    //     while the law does not write the roll, the gimbals and motifs carry no roll in its
    //     allocation (it no longer fights, with the gimbals, the AutoPilot's roll).
    //   * Bit 32, MEASURED AUTHORITY: the share of the common deflection the gimbals actually
    //     hold (realized deflection projected on the realized common of the previous written
    //     step, AccumulateShare, a forgetting mean over the steps with a common), mono-vol; the
    //     published authority is the model's times that share in [0, 1] (MeasuredFactor), the
    //     layer itself unchanged (it keeps translating, so the share keeps being measured).
    // -----------------------------------------------------------------------------------

    /// <summary>Tunables of the lateral-force layer. Born with the chantier 19 bench values.</summary>
    internal sealed class TranslationParams
    {
        internal const double DefaultFracGimbal = 0.8;
        internal const double DefaultFracDiff = 0.8;
        internal const double DefaultDiffPriority = 0.5;
        /// <summary>
        /// Computed levers (bit 4): largest |ctrlState.X|, |ctrlState.Y| the translation writes.
        /// ModuleRCS.FixedUpdate gives each nozzle the fraction max(v . rot, 0) + max(v . lin, 0)
        /// bounded at 1: the rotation and the translation SHARE a nozzle (on the heavy booster
        /// the yaw nozzles are the ctrlState.X ones, the pitch nozzles the ctrlState.Y ones). At
        /// 0.5 the translation takes at most half of any nozzle (|v . axis| &lt;= 1), so the
        /// attitude's own RCS command (the law's ctrlState columns, priority 3, used last) stays
        /// linear up to |u| = 0.5 and keeps half the torque in every direction; half the RCS
        /// force is also the half of the translation the heavy booster's balance needs (its
        /// gimbals reach 0.8 of their range cancelling the torque of about half the RCS).
        /// </summary>
        internal const double DefaultFracRcs = 0.5;

        /// <summary>Largest share of each gimbal's range the common deflection may use.</summary>
        internal double FracGimbal = DefaultFracGimbal;
        /// <summary>Largest share of each engine's throttle margin the feed-forward may use.</summary>
        internal double FracDiff = DefaultFracDiff;
        /// <summary>
        /// WLS priority of the two motif columns. The gimbal columns keep 1, ctrlState 3: 0.5
        /// makes the differential twice as favourable as a gimbal (the "1 against 2" of the
        /// bench) without changing any ratio of c03d5de.
        /// </summary>
        internal double DiffPriority = DefaultDiffPriority;
        /// <summary>Computed levers (bit 4) only: largest |ctrlState.X/Y| of the translation.</summary>
        internal double FracRcs = DefaultFracRcs;
        /// <summary>
        /// Bit 16 (RCS headroom) only: how long a nozzle's rotation fraction is held (s of game
        /// time, linear decay to 0). The law's pitch/yaw limit cycle reverses the RCS in 0.1-0.3 s
        /// (183740): the headroom of the next step is the worst of the last half second.
        /// </summary>
        internal double RcsHoldSeconds = 0.5;
        /// <summary>Bit 32 (measured authority) only: forgetting time of the measured share (s of steps with a common).</summary>
        internal double ShareTauSeconds = 1.0;
        /// <summary>
        /// Bit 32 only: the share is evidence once the forgetting mean of sum(common^2) reaches
        /// (ShareMinDegrees)^2 per gimbal column; before, the factor is 1 (the model's authority).
        /// </summary>
        internal double ShareMinDegrees = 0.05;
        /// <summary>
        /// Smallest lever (m) of a motif: below, no differential authority. Modes 1 and 3 only;
        /// with bit 4 the criterion is AttitudeTranslation.LeverResolution.
        /// </summary>
        internal double MinLeverArmM = 0.25;
        /// <summary>Smallest throttle margin (fraction) for the differential to exist this step.</summary>
        internal double MinThrottleMargin = 0.005;
        /// <summary>det of the horizontal projection of the body x/z plane (cos^2 of the tilt): 60 deg.</summary>
        internal double MinProjectionDet = 0.25;
        /// <summary>Wind-down (stale request or mode 0): done below this realized common deflection...</summary>
        internal double WindDownDoneDegrees = 0.01;
        /// <summary>... or after this long (game s), whatever the deflection.</summary>
        internal double WindDownMaxSeconds = 1.0;

        internal void Reset ()
        {
            FracGimbal = DefaultFracGimbal;
            FracDiff = DefaultFracDiff;
            DiffPriority = DefaultDiffPriority;
            FracRcs = DefaultFracRcs;
        }
    }

    internal static class AttitudeTranslation
    {
        /// <summary>Layout version of the RPC return (offset 4).</summary>
        internal const int ReturnVersion = 1;
        /// <summary>Layout version of the RPC return while the stored mode has bit 4 (5, 7).</summary>
        internal const int ReturnVersionComputed = 2;
        /// <summary>Layout version while the stored mode has bit 16 or 32 (layout 2 + 7 values).</summary>
        internal const int ReturnVersionHonest = 3;

        // Mode bits (attitude_translation_v1). Valid modes: 0 stop, 1 differential, 3 both,
        // and with bit 4 ("computed levers", born closed) 5 = 1 | 4 and 7 = 1 | 2 | 4; bits 16
        // (RCS headroom) and 32 (measured authority), born closed, only on top of 5 or 7.
        internal const int BitDiff = 1;
        internal const int BitTranslate = 2;
        internal const int BitComputed = 4;
        internal const int BitRcsHeadroom = 16;
        internal const int BitMeasured = 32;
        internal const int BitsHonest = BitRcsHeadroom | BitMeasured;

        // State codes (RPC return offset 5, translation frame offset 5).
        internal const int StateIdle = 0;          // nothing added: behaviour of c03d5de
        internal const int StateDiff = 1;          // motif columns in the WLS
        internal const int StateTranslate = 2;     // motif columns + common deflection
        internal const int StateWindDown = 3;      // stale or stopped: offsets brought back to 0
        internal const int StateUnavailable = 4;   // requested, but no differential this step

        // Reason bits (RPC return offset 6, translation frame offset 6).
        internal const long ReasonNoRequest = 1L << 0;
        internal const long ReasonNotArmedPitchYaw = 1L << 1;     // shadow, or pitch/yaw not in the arm mask
        internal const long ReasonPitchYawNotPermitted = 1L << 2; // write mask, reference older than 0.3 s, warp, stage
        internal const long ReasonNotWriting = 1L << 3;           // the M4 channel fell back this step
        internal const long ReasonStale = 1L << 4;                // ut past ut_valid_until
        internal const long ReasonStopRequested = 1L << 5;        // mode 0
        internal const long ReasonNoDiffAuthority = 1L << 6;      // motifs without lever (engines on the axis, < 2 engines)
        internal const long ReasonNoThrottleMargin = 1L << 7;     // main throttle at 0 or 1
        internal const long ReasonNoTranslationGeometry = 1L << 8;// no thrusting gimbal, singular tilt matrix, no mass
        internal const long ReasonTiltTooLarge = 1L << 9;         // nose more than 60 deg from the local vertical
        internal const long ReasonLimitedByGimbal = 1L << 10;     // common deflection capped at FracGimbal
        internal const long ReasonLimitedByDiff = 1L << 11;       // capped by what the differential can cancel
        internal const long ReasonFeedForwardClamped = 1L << 12;  // realized common torque above the throttle box
        internal const long ReasonThrottleWriteMissed = 1L << 13; // last throttle decision never reached fly-by-wire
        internal const long ReasonWindDownTimeout = 1L << 14;     // wind-down ended by time, not by deflection
        internal const long ReasonNotActiveVessel = 1L << 15;     // the authority exists on the active vessel only
        // Computed levers (bit 4) only.
        internal const long ReasonNoLateralAuthority = 1L << 16;  // the joint allocation reaches no torque-free lateral force
        internal const long ReasonLimitedByRcs = 1L << 17;        // translation capped at frac_rcs of ctrlState.X/Y
        internal const long ReasonRcsModelApprox = 1L << 18;      // RCS precision mode or fullThrust: the RCS columns are approximate
        // Honest translation (bit 16) only.
        internal const long ReasonRcsShared = 1L << 19;           // the law's RCS pitch/yaw left out (combined rotation nonlinear)
        // (option 8192 of AttitudeOptionsV1) only.
        internal const long ReasonServedByAllocation = 1L << 20;  // the engine-by-engine allocation wrote this step, not the layer

        internal static bool ValidMode (int mode)
        {
            if (mode == 0 || mode == BitDiff || mode == (BitDiff | BitTranslate) ||
                    mode == (BitDiff | BitComputed) || mode == (BitDiff | BitTranslate | BitComputed))
                return true;
            // Bits 16 / 32 (born closed): only on top of the computed levers (5 or 7).
            int honest = mode & BitsHonest;
            int rest = mode & ~BitsHonest;
            return honest != 0 &&
                (rest == (BitDiff | BitComputed) || rest == (BitDiff | BitTranslate | BitComputed));
        }

        /// <summary>Bit 16 or 32 in the mode: return layout 3.</summary>
        internal static bool Honest (int mode)
        {
            return (mode & BitsHonest) != 0;
        }

        /// <summary>Bit 4: the computed levers (lever criterion, RCS lever, joint allocation).</summary>
        internal static bool Computed (int mode)
        {
            return (mode & BitComputed) != 0;
        }

        /// <summary>A request drives the layer: received, not a stop, and ut within its validity.</summary>
        internal static bool RequestLive (bool have, int mode, double ut, double utValidUntil)
        {
            return have && mode != 0 && !double.IsNaN (ut) && ut <= utValidUntil;
        }

        /// <summary>
        /// A live request ASKS A LATERAL FORCE: RequestLive and bit 2 (BitTranslate), as the layer's own Translate. A
        /// request of the differential alone (bit 2 off, the client's mode 53 with an acceleration (0, 0, 0)) asks none:
        /// the engine-by-engine allocation aims at no lateral force then (it read RequestLive alone and aimed at
        /// LateralFromGap(0): cancelling the horizontal thrust of the tilted nose, the phantom demand of a recorded flight
        /// and 142012).
        /// </summary>
        internal static bool RequestTranslates (bool have, int mode, double ut, double utValidUntil)
        {
            return RequestLive (have, mode, ut, utValidUntil) && (mode & BitTranslate) != 0;
        }

        /// <summary>
        /// Wind-down end: the realized common deflection is back under the threshold after at
        /// least one wind-down step (so the throttles were written at the main throttle), or
        /// the wind-down lasted WindDownMaxSeconds.
        /// </summary>
        internal static bool WindDownDone (double maxAbsCommonRealized, double elapsed,
            int windDownSteps, TranslationParams p, out bool timeout)
        {
            timeout = false;
            if (elapsed >= p.WindDownMaxSeconds) {
                timeout = true;
                return true;
            }
            return windDownSteps >= 1 && maxAbsCommonRealized < p.WindDownDoneDegrees;
        }

        // ------------------------------------------------------------ small algebra

        /// <summary>Solves [a b; c d] x = r. False when singular (relative) or not finite.</summary>
        internal static bool Solve2 (double a, double b, double c, double d, double r0, double r1,
            out double x0, out double x1)
        {
            x0 = 0.0;
            x1 = 0.0;
            double det = a * d - b * c;
            double scale = Math.Abs (a * d) + Math.Abs (b * c);
            if (!(Math.Abs (det) > 1e-12 * scale) || !(scale > 0.0) || !Finite (det))
                return false;
            x0 = (d * r0 - b * r1) / det;
            x1 = (a * r1 - c * r0) / det;
            return Finite (x0) && Finite (x1);
        }

        internal static bool Finite (double v)
        {
            return !double.IsNaN (v) && !double.IsInfinity (v);
        }

        // ------------------------------------------------------ differential motifs

        /// <summary>
        /// Pitch and yaw motifs of n engines from their slopes s_i (N per unit throttle, &gt; 0)
        /// and the body x and z of their nozzle centroids relative to the CoM. Each motif is
        /// the lever about the slope-weighted mean, normalized to max |motif| = 1, so that
        /// sum s_i motif_i = 0 (total thrust conserved). False (motifs zero) with fewer than
        /// two engines, a non-positive slope, or a motif whose largest lever is below minLever.
        /// </summary>
        internal static bool Motifs (int n, double[] slope, double[] leverX, double[] leverZ,
            double minLever, double[] motifP, double[] motifY)
        {
            double maxP, maxY;
            return Motifs (n, slope, leverX, leverZ, minLever, motifP, motifY, out maxP, out maxY);
        }

        /// <summary>
        /// Motifs, and the largest lever of each (maxP = max |z - zbar|, maxY = max |x - xbar|,
        /// m; 0 when the engines are refused before the levers are read). The same arithmetic
        /// as the overload above: only the two levers are handed out.
        /// </summary>
        internal static bool Motifs (int n, double[] slope, double[] leverX, double[] leverZ,
            double minLever, double[] motifP, double[] motifY, out double maxP, out double maxY)
        {
            maxP = 0.0;
            maxY = 0.0;
            for (int i = 0; i < n; i++) {
                motifP [i] = 0.0;
                motifY [i] = 0.0;
            }
            if (n < 2)
                return false;
            double sw = 0.0, sx = 0.0, sz = 0.0;
            for (int i = 0; i < n; i++) {
                if (!(slope [i] > 0.0) || !Finite (slope [i]) || !Finite (leverX [i]) ||
                        !Finite (leverZ [i]))
                    return false;
                sw = sw + slope [i];
                sx = sx + slope [i] * leverX [i];
                sz = sz + slope [i] * leverZ [i];
            }
            double xm = sx / sw, zm = sz / sw;
            for (int i = 0; i < n; i++) {
                double p = Math.Abs (leverZ [i] - zm);
                double y = Math.Abs (leverX [i] - xm);
                if (p > maxP)
                    maxP = p;
                if (y > maxY)
                    maxY = y;
            }
            // maxP > 0 and maxY > 0 change nothing with the 0.25 m of modes 1 and 3; with a
            // computed criterion (bit 4) they keep a null lever from dividing by zero.
            if (!(maxP >= minLever) || !(maxY >= minLever) || !(maxP > 0.0) || !(maxY > 0.0))
                return false;
            for (int i = 0; i < n; i++) {
                motifP [i] = (leverZ [i] - zm) / maxP;
                motifY [i] = (leverX [i] - xm) / maxY;
            }
            return true;
        }

        /// <summary>
        /// Columns of the two motifs, N.m per unit amplitude: B_k = sum_i motif_k,i s_i t_i,
        /// t_i = torque per newton of engine i (r x dir, body, 3 values at [3i]).
        /// </summary>
        internal static void MotifColumns (int n, double[] slope, double[] motifP,
            double[] motifY, double[] torquePerNewton, double[] columnP, double[] columnY)
        {
            for (int r = 0; r < 3; r++) {
                columnP [r] = 0.0;
                columnY [r] = 0.0;
            }
            for (int i = 0; i < n; i++)
                for (int r = 0; r < 3; r++) {
                    columnP [r] = columnP [r] + motifP [i] * slope [i] * torquePerNewton [3 * i + r];
                    columnY [r] = columnY [r] + motifY [i] * slope [i] * torquePerNewton [3 * i + r];
                }
        }

        /// <summary>
        /// Realized motif amplitudes from the engines' realized throttle commands tau_i:
        /// slope-weighted least squares of tau_i = c + d_p motif_p,i + d_y motif_y,i.
        /// </summary>
        internal static bool RealizedAmplitudes (int n, double[] slope, double[] motifP,
            double[] motifY, double[] tau, out double dP, out double dY)
        {
            dP = 0.0;
            dY = 0.0;
            double sw = 0.0, st = 0.0;
            for (int i = 0; i < n; i++) {
                sw = sw + slope [i];
                st = st + slope [i] * tau [i];
            }
            if (!(sw > 0.0))
                return false;
            double c = st / sw;
            double g00 = 0.0, g01 = 0.0, g11 = 0.0, r0 = 0.0, r1 = 0.0;
            for (int i = 0; i < n; i++) {
                double dev = tau [i] - c;
                g00 = g00 + slope [i] * motifP [i] * motifP [i];
                g01 = g01 + slope [i] * motifP [i] * motifY [i];
                g11 = g11 + slope [i] * motifY [i] * motifY [i];
                r0 = r0 + slope [i] * motifP [i] * dev;
                r1 = r1 + slope [i] * motifY [i] * dev;
            }
            return Solve2 (g00, g01, g01, g11, r0, r1, out dP, out dY);
        }

        /// <summary>
        /// Throttle margin of one engine around the main throttle: min(main - low, high -
        /// main), never negative. The same for every engine (they share the main throttle).
        /// </summary>
        internal static double ThrottleMargin (double mainThrottle, double low, double high)
        {
            if (double.IsNaN (mainThrottle))
                return 0.0;
            double m = Math.Min (mainThrottle - low, high - mainThrottle);
            return m > 0.0 ? m : 0.0;
        }

        /// <summary>
        /// Largest D such that the box |d_p| &lt;= D, |d_y| &lt;= D keeps every engine within its
        /// margin: min_i margin_i / (|motif_p,i| + |motif_y,i|).
        /// </summary>
        internal static double SymmetricHalfWidth (int n, double[] motifP, double[] motifY,
            double[] margin)
        {
            double d = double.PositiveInfinity;
            for (int i = 0; i < n; i++) {
                double w = Math.Abs (motifP [i]) + Math.Abs (motifY [i]);
                if (w > 0.0) {
                    double v = margin [i] / w;
                    if (v < d)
                        d = v;
                }
            }
            if (double.IsInfinity (d) || !(d > 0.0))
                return 0.0;
            return d;
        }

        /// <summary>
        /// Box of the two motif amplitudes this step: the feed-forward (dffP, dffY) first
        /// (scaled down, returned scale &lt; 1, if it alone exceeds a margin), then the same
        /// attitude share r on both motifs with what is left: half widths |dff_k| + r. Every
        /// point of the box keeps every engine within its margin.
        /// </summary>
        internal static double BoxHalfWidths (int n, double[] motifP, double[] motifY,
            double[] margin, ref double dffP, ref double dffY, out double halfP,
            out double halfY, out double share)
        {
            double scale = 1.0;
            for (int i = 0; i < n; i++) {
                double need = Math.Abs (motifP [i]) * Math.Abs (dffP) + Math.Abs (motifY [i]) * Math.Abs (dffY);
                if (need > margin [i]) {
                    double s = margin [i] > 0.0 ? margin [i] / need : 0.0;
                    if (s < scale)
                        scale = s;
                }
            }
            dffP = dffP * scale;
            dffY = dffY * scale;
            double r = double.PositiveInfinity;
            for (int i = 0; i < n; i++) {
                double w = Math.Abs (motifP [i]) + Math.Abs (motifY [i]);
                if (!(w > 0.0))
                    continue;
                double rest = margin [i] - (Math.Abs (motifP [i]) * Math.Abs (dffP) +
                    Math.Abs (motifY [i]) * Math.Abs (dffY));
                double v = rest / w;
                if (v < r)
                    r = v;
            }
            if (double.IsInfinity (r) || !(r > 0.0))
                r = 0.0;
            share = r;
            halfP = Math.Abs (dffP) + r;
            halfY = Math.Abs (dffY) + r;
            return scale;
        }

        /// <summary>
        /// Torque the differential guarantees in ANY pitch/yaw direction with the symmetric
        /// box of half width h: the inradius of the parallelogram {B_xz d, |d_k| &lt;= h},
        /// h |det| / max(|b_p|, |b_y|), on the x (pitch) and z (yaw) rows. N.m.
        /// </summary>
        internal static double Inradius (double[] columnP, double[] columnY, double halfWidth)
        {
            double px = columnP [0], pz = columnP [2], yx = columnY [0], yz = columnY [2];
            double det = px * yz - pz * yx;
            double np = Math.Sqrt (px * px + pz * pz);
            double ny = Math.Sqrt (yx * yx + yz * yz);
            double max = np > ny ? np : ny;
            if (!(max > 0.0) || !(halfWidth > 0.0))
                return 0.0;
            return halfWidth * Math.Abs (det) / max;
        }

        /// <summary>Motif amplitudes whose torque cancels (mx, mz) on the pitch and yaw rows.</summary>
        internal static bool CancelTorque (double[] columnP, double[] columnY, double mx,
            double mz, out double dP, out double dY)
        {
            return Solve2 (columnP [0], columnY [0], columnP [2], columnY [2], -mx, -mz,
                out dP, out dY);
        }

        /// <summary>
        /// Scale (&lt;= +inf) at which the motif amplitudes (dP, dY) reach fracDiff of every
        /// engine's margin: min_i fracDiff margin_i / (|motif_p,i dP| + |motif_y,i dY|).
        /// </summary>
        internal static double DiffCapacityScale (int n, double[] motifP, double[] motifY,
            double[] margin, double dP, double dY, double fracDiff)
        {
            double s = double.PositiveInfinity;
            for (int i = 0; i < n; i++) {
                double need = Math.Abs (motifP [i] * dP) + Math.Abs (motifY [i] * dY);
                if (need > 0.0) {
                    double v = fracDiff * margin [i] / need;
                    if (v < s)
                        s = v;
                }
            }
            return s > 0.0 ? s : 0.0;
        }

        /// <summary>Per-engine throttle offsets of the motif amplitudes.</summary>
        internal static void ThrottleDeltas (int n, double[] motifP, double[] motifY, double dP,
            double dY, double[] delta)
        {
            for (int i = 0; i < n; i++)
                delta [i] = dP * motifP [i] + dY * motifY [i];
        }

        /// <summary>
        /// Write-time scale kappa in [0, 1] of the offsets so that main + kappa delta_i stays
        /// within [low, high] for every engine, with the main throttle of THIS step (the one
        /// the fly-by-wire chain just served). Zero-sum and direction are kept: only the size
        /// shrinks. main 0 (cut) gives kappa 0: no engine is lit by the differential.
        /// </summary>
        internal static double WriteScale (int n, double[] delta, double mainThrottle,
            double low, double high)
        {
            if (double.IsNaN (mainThrottle))
                return 0.0;
            double k = 1.0;
            for (int i = 0; i < n; i++) {
                double d = delta [i];
                if (d > 0.0) {
                    double room = high - mainThrottle;
                    double v = room > 0.0 ? room / d : 0.0;
                    if (v < k)
                        k = v;
                } else if (d < 0.0) {
                    double room = mainThrottle - low;
                    double v = room > 0.0 ? room / -d : 0.0;
                    if (v < k)
                        k = v;
                }
            }
            return k > 0.0 ? k : 0.0;
        }

        // ------------------------------------------------------------- translation

        /// <summary>
        /// Lateral acceleration the common deflection must add, as body x and z components:
        /// gap = horiz(demand) - thrustAccel horiz(nose), with horiz the projection on the
        /// plane orthogonal to up (body frame, need not be unit); then (ax, az) such that
        /// horiz(ax ex + az ez) = gap exactly (2x2 normal equations). False when the nose is
        /// too far from the vertical (det of the projection &lt; minDet) or up is null.
        /// </summary>
        internal static bool LateralFromGap (double[] demand, double[] up, double thrustAccel,
            double minDet, double[] gap, out double ax, out double az, out double det)
        {
            ax = 0.0;
            az = 0.0;
            det = 0.0;
            gap [0] = 0.0;
            gap [1] = 0.0;
            gap [2] = 0.0;
            double un = Math.Sqrt (up [0] * up [0] + up [1] * up [1] + up [2] * up [2]);
            if (!(un > 0.0) || !Finite (un))
                return false;
            double ux = up [0] / un, uy = up [1] / un, uz = up [2] / un;
            double du = demand [0] * ux + demand [1] * uy + demand [2] * uz;
            // horiz(nose), nose = (0, 1, 0): (0,1,0) - uy u
            for (int r = 0; r < 3; r++) {
                double u = r == 0 ? ux : (r == 1 ? uy : uz);
                double dh = demand [r] - du * u;
                double nh = (r == 1 ? 1.0 : 0.0) - uy * u;
                gap [r] = dh - thrustAccel * nh;
            }
            // Horizontal projections of body x and z.
            double exx = 1.0 - ux * ux, exy = -ux * uy, exz = -ux * uz;
            double ezx = -uz * ux, ezy = -uz * uy, ezz = 1.0 - uz * uz;
            double g00 = exx * exx + exy * exy + exz * exz;
            double g01 = exx * ezx + exy * ezy + exz * ezz;
            double g11 = ezx * ezx + ezy * ezy + ezz * ezz;
            double r0 = exx * gap [0] + exy * gap [1] + exz * gap [2];
            double r1 = ezx * gap [0] + ezy * gap [1] + ezz * gap [2];
            det = g00 * g11 - g01 * g01;
            if (!(det >= minDet))
                return false;
            ax = (g11 * r0 - g01 * r1) / det;
            az = (g00 * r1 - g01 * r0) / det;
            return Finite (ax) && Finite (az);
        }

        /// <summary>
        /// Body tilt phi (deg, about body x and z) giving the lateral force (fx, fz) (N, body x
        /// and z) through K = d(F_x, F_z)/d(phi_x, phi_z), row-major {Kxx, Kxz, Kzx, Kzz}.
        /// </summary>
        internal static bool SolveTilt (double[] k, double fx, double fz, out double phiX,
            out double phiZ)
        {
            return Solve2 (k [0], k [1], k [2], k [3], fx, fz, out phiX, out phiZ);
        }

        /// <summary>
        /// Share of a gimbal's directional range used by the deflection (cx, cy): the larger
        /// of cx / (XP or XN) and cy / (YP or YN), by sign. +inf on a null range with a
        /// non-zero deflection.
        /// </summary>
        internal static double GimbalFraction (double cx, double cy, double xn, double xp,
            double yn, double yp)
        {
            double fx = Fraction (cx, xn, xp);
            double fy = Fraction (cy, yn, yp);
            return fx > fy ? fx : fy;
        }

        static double Fraction (double c, double negative, double positive)
        {
            if (c == 0.0)
                return 0.0;
            double range = c > 0.0 ? Math.Abs (positive) : Math.Abs (negative);
            if (!(range > 0.0))
                return double.PositiveInfinity;
            return Math.Abs (c) / range;
        }

        /// <summary>
        /// Announced lateral authority, m/s^2: (T/m) sin(range) min(fracGimbal, fracDiff
        /// diffAuthority / fullCommonTorque); the gimbal share alone when the full common
        /// torque is null. range in degrees.
        /// </summary>
        internal static double LateralAuthority (double thrustAccel, double rangeDeg,
            double fracGimbal, double fracDiff, double diffAuthority, double fullCommonTorque)
        {
            if (!(thrustAccel > 0.0) || !(rangeDeg > 0.0) || !(diffAuthority > 0.0))
                return 0.0;
            double f = fracGimbal;
            if (fullCommonTorque > 0.0) {
                double g = fracDiff * diffAuthority / fullCommonTorque;
                if (g < f)
                    f = g;
            }
            if (!(f > 0.0))
                return 0.0;
            return thrustAccel * Math.Sin (rangeDeg * Math.PI / 180.0) * f;
        }

        /// <summary>Largest singular value of the 2x2 {a, b, c, d}.</summary>
        internal static double MaxSingular2 (double a, double b, double c, double d)
        {
            double s1 = a * a + b * b + c * c + d * d;
            double det = a * d - b * c;
            double disc = s1 * s1 - 4.0 * det * det;
            if (disc < 0.0)
                disc = 0.0;
            return Math.Sqrt ((s1 + Math.Sqrt (disc)) / 2.0);
        }

        // ------------------------------------------------------ ramp and WLS inputs

        /// <summary>
        /// Per-step factor of the gimbal lease ramp (Actuation.ApplyGimbal, QW5): the
        /// stock Lerp by gimbalResponseSpeed x fixed dt, 1 without response speed.
        /// </summary>
        internal static double RampFactor (bool useResponseSpeed, double responseSpeed, double dt)
        {
            if (!useResponseSpeed)
                return 1.0;
            double step = dt > 0.001 ? dt : 0.001;
            double f = responseSpeed * step;
            if (f > 1.0)
                f = 1.0;
            if (!(f > 0.0))
                f = 0.0;
            return f;
        }

        internal static double Ramp (double previous, double target, double factor)
        {
            return previous + (target - previous) * factor;
        }

        /// <summary>
        /// Gimbal column of the law under a common offset: u0 = realized - realized common of
        /// the previous step (the realized ATTITUDE part: the ramp is linear), bounds shifted
        /// by the commanded common c so that c + u stays within [uMin, uMax].
        /// </summary>
        internal static void ShiftGimbal (double realized, double commonRealizedPrevious,
            double common, double uMin, double uMax, out double u0, out double lo, out double hi)
        {
            u0 = realized - commonRealizedPrevious;
            lo = uMin - common;
            hi = uMax - common;
        }

        /// <summary>
        /// Motif column of the law: u0 = realized amplitude - the feed-forward commanded at
        /// the previous step (the realized attitude part), bounds [-half - dff, half - dff]
        /// with an attitude share, [0, 0] without one (wind-down: the law is told the
        /// attitude part goes to zero and compensates it in the same step).
        /// </summary>
        internal static void DiffBounds (double realized, double dffPrevious, double dff,
            double half, bool attitudeShare, out double u0, out double lo, out double hi)
        {
            u0 = realized - dffPrevious;
            if (attitudeShare) {
                lo = -half - dff;
                hi = half - dff;
            } else {
                lo = 0.0;
                hi = 0.0;
            }
        }

        // ------------------------------------------------- computed levers (bit 4)

        /// <summary>
        /// Relative resolution of a lever. The levers are Unity float3 differences of world
        /// positions (nozzle - CoM), rotated into the body frame in float: each component is
        /// known to about 2^-23 |r| = 1.2e-7 |r|, plus a few roundings. A motif lever below
        /// 1e-5 |r| (80 times that) is rounding, not geometry.
        /// </summary>
        internal const double LeverRelativeResolution = 1e-5;

        /// <summary>
        /// Computed lever criterion (bit 4): LeverRelativeResolution x the largest |r| of the
        /// engines' nozzle centroids from the CoM (x, y, z body; the CoM is the origin of every
        /// lever). The heavy booster: |r| 10.83 m, criterion 1.1e-4 m against its 0.133 m.
        /// 0 when no lever is finite.
        /// </summary>
        internal static double LeverResolution (int n, double[] leverX, double[] leverY,
            double[] leverZ)
        {
            double max = 0.0;
            for (int i = 0; i < n; i++) {
                double r = Math.Sqrt (leverX [i] * leverX [i] + leverY [i] * leverY [i] +
                    leverZ [i] * leverZ [i]);
                if (Finite (r) && r > max)
                    max = r;
            }
            return LeverRelativeResolution * max;
        }

        /// <summary>
        /// RCS translation columns (bit 4): force and torque (N, N.m, body, about the CoM) per
        /// unit of a linear command along the body axis (ax, ay, az) (unit: ctrlState.X is
        /// body x = ReferenceTransform.right, ctrlState.Y body z = ReferenceTransform.forward),
        /// ADDED into forcePos / torquePos (command +1) and forceNeg / torqueNeg (per unit of a
        /// NEGATIVE command: the response to -1, times -1). ModuleRCS (KSP 1.12.5, decompiled):
        /// Update makes inputLin = rotation x (X, Z or throttle, Y); FixedUpdate gives nozzle i
        /// the fraction max(v_i . inputLin, 0) (plus the rotation's, the sum bounded at 1) and
        /// the force -v_i x power_i x fraction at its position. lever: nozzle - CoM, 3 per
        /// nozzle; dir: v (thruster up, or forward with useZaxis), unit; power: N at full
        /// demand (&lt;= 0 or not finite: skipped). Linear in the command below the bound of 1.
        /// </summary>
        internal static void RcsTranslationColumns (int count, double[] lever, double[] dir,
            double[] power, double ax, double ay, double az, double[] forcePos, double[] forceNeg,
            double[] torquePos, double[] torqueNeg)
        {
            for (int i = 0; i < count; i++) {
                double p = power [i];
                if (!(p > 0.0) || !Finite (p))
                    continue;
                double vx = dir [3 * i], vy = dir [3 * i + 1], vz = dir [3 * i + 2];
                double rx = lever [3 * i], ry = lever [3 * i + 1], rz = lever [3 * i + 2];
                double dot = vx * ax + vy * ay + vz * az;
                for (int s = 0; s < 2; s++) {
                    double f = s == 0 ? dot : -dot;
                    if (!(f > 0.0))
                        continue;
                    if (f > 1.0)
                        f = 1.0;
                    double k = -p * f;
                    double fx = k * vx, fy = k * vy, fz = k * vz;
                    double tx = ry * fz - rz * fy, ty = rz * fx - rx * fz, tz = rx * fy - ry * fx;
                    double[] force = s == 0 ? forcePos : forceNeg;
                    double[] torque = s == 0 ? torquePos : torqueNeg;
                    double sign = s == 0 ? 1.0 : -1.0;
                    force [0] += sign * fx;
                    force [1] += sign * fy;
                    force [2] += sign * fz;
                    torque [0] += sign * tx;
                    torque [1] += sign * ty;
                    torque [2] += sign * tz;
                }
            }
        }

        /// <summary>
        /// Share of the commanded common deflection the gimbals realize after this step's ramp:
        /// sum c_j n_j / sum c_j^2 over the gimbal columns (the realized one projected on the
        /// commanded one), bounded to [-1, 1]; 1 when nothing is commanded (no deflection to wait
        /// for). The RCS translation is written times this share so that its torque keeps
        /// balancing the torque of the deflection the gimbals have actually reached -- through a
        /// reversal too: while the gimbals still hold the old tilt the share is negative and the
        /// RCS keeps pushing the old way (bounded at 0 before, the RCS stopped for the half ramp
        /// while the old tilt's whole torque stayed: 520 kN.m on the second ground check).
        /// </summary>
        internal static double RealizedShare (double[] common, double[] realized, int count)
        {
            double cc = 0.0, cn = 0.0;
            for (int j = 0; j < count; j++) {
                cc = cc + common [j] * common [j];
                cn = cn + common [j] * realized [j];
            }
            if (!(cc > 0.0) || !Finite (cc) || !Finite (cn))
                return 1.0;
            double k = cn / cc;
            if (!Finite (k))
                return 0.0;
            return k > 1.0 ? 1.0 : (k < -1.0 ? -1.0 : k);
        }

        /// <summary>
        /// Computed levers: each engine's torque per newton about the CoM along the engines' COMMON
        /// thrust direction (the slope-weighted mean of their nozzle directions `dir`, 3 per engine,
        /// body), lever x common direction, into torquePerNewton (3 per engine). A zero-sum motif
        /// keeps only its geometric lever from any direction shared by every engine; what each
        /// engine's OWN tilt adds (L x its gimbal deflection per newton, 0.41 m per degree on the
        /// heavy stack against its 0.133 m motif lever) is the torque of that gimbal's deflection,
        /// which the law re-decides every step: read in the motif columns, it tied the
        /// differential to the gimbals' attitude part (second ground check, period-2 chatter of the
        /// throttles). False, torquePerNewton untouched, when the directions have no common part.
        /// </summary>
        internal static bool CommonDirectionTorques (int count, double[] slope, double[] leverX,
            double[] leverY, double[] leverZ, double[] dir, double[] torquePerNewton)
        {
            double sx = 0.0, sy = 0.0, sz = 0.0;
            for (int i = 0; i < count; i++) {
                double w = slope [i] > 0.0 && Finite (slope [i]) ? slope [i] : 0.0;
                sx += w * dir [3 * i];
                sy += w * dir [3 * i + 1];
                sz += w * dir [3 * i + 2];
            }
            double n = Math.Sqrt (sx * sx + sy * sy + sz * sz);
            if (!(n > 0.0) || !Finite (n))
                return false;
            double dx = sx / n, dy = sy / n, dz = sz / n;
            for (int i = 0; i < count; i++) {
                double rx = leverX [i], ry = leverY [i], rz = leverZ [i];
                torquePerNewton [3 * i] = ry * dz - rz * dy;
                torquePerNewton [3 * i + 1] = rz * dx - rx * dz;
                torquePerNewton [3 * i + 2] = rx * dy - ry * dx;
            }
            return true;
        }

        /// <summary>
        /// Computed levers: the common deflection the gimbals still hold one physics step after
        /// they were handed back to the stock Lerp (toward ctrlState, pitch/yaw fallback): each
        /// column times (1 - f), f the ramp factor of its module (RampFactor; columns 2g and 2g+1
        /// belong to module g). The Lerp is linear: realized = common + attitude part, the common
        /// goes to 0 and the attitude part to ctrlState at the same rate. f outside [0, 1] or not
        /// finite, or a column without module: 0 (the deflection left at once).
        /// </summary>
        internal static void ReleasedCommonFollows (double[] commonRealized, double[] rampFactor)
        {
            int n = Math.Min (commonRealized.Length, 2 * rampFactor.Length);
            for (int j = 0; j < commonRealized.Length; j++) {
                double f = j < n ? rampFactor [j / 2] : double.NaN;
                commonRealized [j] = Finite (f) && f >= 0.0 && f <= 1.0
                    ? Ramp (commonRealized [j], 0.0, f) : 0.0;
            }
        }

        // ------------------------------------------- honest translation (bits 16, 32)

        /// <summary>
        /// Bit 16: the ROTATION fraction of each nozzle (ModuleRCS.FixedUpdate, KSP 1.12.5,
        /// decompiled): rot = inputRot x unit(ProjectOnPlane(r, inputRot)), fraction
        /// max(v . rot, 0), with inputRot the body command (pitch, roll, yaw) along (x, y, z) (the
        /// module's enable flags already applied by the caller). lever: nozzle - CoM, dir: v, 3 per
        /// nozzle, body. Not bounded at 1 (a combined command can ask more); a lever along the
        /// command (no perpendicular part) gives 0, as Unity's normalized of a null vector.
        /// </summary>
        internal static void RcsRotationFractions (int count, double[] lever, double[] dir, double px,
            double ry, double yz, double[] rho)
        {
            double n2 = px * px + ry * ry + yz * yz;
            for (int i = 0; i < count; i++) {
                rho [i] = 0.0;
                if (!(n2 > 0.0) || !Finite (n2))
                    continue;
                double rx = lever [3 * i], rY = lever [3 * i + 1], rz = lever [3 * i + 2];
                double k = (rx * px + rY * ry + rz * yz) / n2;
                double qx = rx - k * px, qy = rY - k * ry, qz = rz - k * yz;
                double qn = Math.Sqrt (qx * qx + qy * qy + qz * qz);
                if (!(qn > 1e-5) || !Finite (qn))
                    continue;
                qx /= qn;
                qy /= qn;
                qz /= qn;
                // rot = inputRot x q
                double ox = ry * qz - yz * qy, oy = yz * qx - px * qz, oz = px * qy - ry * qx;
                double f = dir [3 * i] * ox + dir [3 * i + 1] * oy + dir [3 * i + 2] * oz;
                rho [i] = f > 0.0 && Finite (f) ? f : 0.0;
            }
        }

        /// <summary>
        /// Bit 16: the torque (body, about the CoM) of RCS modules for a ROTATION command (pitch,
        /// roll, yaw), ModuleRCS's combined rule (RcsRotationFractions, each nozzle at min(1,
        /// fraction)), summed into torque[0..2]. Module m: counts[m] nozzles, lever[m] / dir[m] (3 per
        /// nozzle, body), power[m] (N), enable[3m..3m+2] its enablePitch / enableRoll / enableYaw
        /// (1 or 0: ModuleRCS.Update zeroes the disabled components). `rho` is scratch (as many
        /// values as the largest module).
        /// </summary>
        internal static void RcsRotationTorque (int modules, int[] counts, double[][] lever, double[][] dir,
            double[][] power, double[] enable, double px, double ry, double yz, double[] rho, double[] torque)
        {
            torque [0] = torque [1] = torque [2] = 0.0;
            for (int m = 0; m < modules; m++) {
                int count = counts [m];
                RcsRotationFractions (count, lever [m], dir [m], px * enable [3 * m], ry * enable [3 * m + 1],
                    yz * enable [3 * m + 2], rho);
                for (int i = 0; i < count; i++) {
                    double p = power [m] [i];
                    double f = rho [i] > 1.0 ? 1.0 : rho [i];
                    if (!(p > 0.0) || !Finite (p) || !(f > 0.0))
                        continue;
                    double k = -p * f;
                    double fx = k * dir [m] [3 * i], fy = k * dir [m] [3 * i + 1], fz = k * dir [m] [3 * i + 2];
                    double rx = lever [m] [3 * i], rY = lever [m] [3 * i + 1], rz = lever [m] [3 * i + 2];
                    torque [0] += rY * fz - rz * fy;
                    torque [1] += rz * fx - rx * fz;
                    torque [2] += rx * fy - rY * fx;
                }
            }
        }

        /// <summary>
        /// Bit 16: the roll someone else commands (|ctrlState.roll| held RcsHoldSeconds) above which
        /// the gimbals and motifs carry no roll in the law's allocation (the law does not write the
        /// roll). Below, the law keeps damping the roll with its gimbals, as with bit 4.
        /// </summary>
        internal const double RollActiveThreshold = 0.05;
        /// <summary>Bit 16: the law's RCS pitch/yaw is nonlinear beyond this gain over its column.</summary>
        internal const double RcsNonlinearRatio = 2.0;
        /// <summary>Bit 16: the small command the nonlinearity is probed with.</summary>
        internal const double RcsNonlinearProbe = 0.1;

        /// <summary>
        /// Bit 16: is ModuleRCS's answer to a SMALL pitch (axis 0) or yaw (axis 2) command, at the
        /// rotation the step serves (pitch, roll, yaw), more than RcsNonlinearRatio x what the law's
        /// per-axis column predicts? ModuleRCS combines the three commands into ONE rotation vector
        /// (inputRot) and fires each nozzle along inputRot x unit(ProjectOnPlane(r, inputRot)):
        /// with the roll saturated (the AutoPilot aligning the roll, which the law does not write)
        /// a ring of RCS 37 m above the CoM sees its lever turn with the slightest pitch/yaw, and
        /// answers it with its WHOLE torque. Measured on the heavy booster (183740, 172225, roll
        /// served &gt;= 0.9, pitch/yaw 0.05-0.3): 1.73-1.76 MN.m realized for 0.36 MN.m modelled
        /// (x 4.8); roll ~0 (142810): 0.215 for 0.207. Both signs of the probe, from the served
        /// point; the column is the per-axis rule (inputRot = the unit axis) of the same nozzles.
        /// False when the column is null (no RCS on that axis).
        /// </summary>
        internal static bool RcsRotationNonlinear (int modules, int[] counts, double[][] lever, double[][] dir,
            double[][] power, double[] enable, double px, double ry, double yz, int axis, double[] rho,
            double[] scratch)
        {
            if (axis != 0 && axis != 2)
                return false;
            RcsRotationTorque (modules, counts, lever, dir, power, enable, axis == 0 ? 1.0 : 0.0, 0.0,
                axis == 2 ? 1.0 : 0.0, rho, scratch);
            double column = Math.Abs (scratch [axis]);
            if (!(column > 0.0) || !Finite (column))
                return false;
            RcsRotationTorque (modules, counts, lever, dir, power, enable, px, ry, yz, rho, scratch);
            double t0 = scratch [axis];
            for (int s = 0; s < 2; s++) {
                double d = s == 0 ? RcsNonlinearProbe : -RcsNonlinearProbe;
                double p1 = axis == 0 ? Clamp1 (px + d) : px, y1 = axis == 2 ? Clamp1 (yz + d) : yz;
                double step = axis == 0 ? p1 - px : y1 - yz;
                if (step == 0.0)
                    continue;
                RcsRotationTorque (modules, counts, lever, dir, power, enable, p1, ry, y1, rho, scratch);
                double gain = Math.Abs (scratch [axis] - t0) / Math.Abs (step);
                if (!Finite (gain) || gain > RcsNonlinearRatio * column)
                    return true;
            }
            return false;
        }

        static double Clamp1 (double v)
        {
            return v > 1.0 ? 1.0 : (v < -1.0 ? -1.0 : v);
        }

        /// <summary>
        /// Bit 16: held rotation fractions, hold_i = max(now_i, hold_i - drop) (drop = dt over the
        /// hold time); a non-finite hold restarts from now.
        /// </summary>
        internal static void HoldFractions (int count, double[] now, double[] hold, double drop)
        {
            double d = drop > 0.0 && Finite (drop) ? drop : 0.0;
            for (int i = 0; i < count; i++) {
                double h = hold [i] - d;
                if (!Finite (h))
                    h = 0.0;
                hold [i] = now [i] > h ? now [i] : h;
            }
        }

        /// <summary>
        /// Bit 16: RcsTranslationColumns counting only the nozzles the translation can drive
        /// LINEARLY over its whole span: ModuleRCS bounds a nozzle at rho_i + f |command| &lt;= 1
        /// (f = |v . axis| &lt;= 1), so a nozzle with rho_i + fracRcs f &lt;= 1 answers f |command|
        /// exactly up to |command| = fracRcs, and any other is left out of the column (0). With
        /// the held rho (&gt;= the rotation of the step) the column is exact for the nozzles it
        /// counts and the ones it leaves out can only add (the law then takes their torque).
        /// Identical to RcsTranslationColumns when every rho_i + fracRcs f &lt;= 1 (no rotation:
        /// rho 0, f &lt;= 1, fracRcs &lt;= 1). rho null or fracRcs not in (0, 1]: no nozzle left out.
        /// (Measured first with the chord min(f, (1 - rho) / fracRcs): exact at rho 0 and 1 only,
        /// under a pitch/yaw rotation its under-estimate between left 8 x the force's torque
        /// unbalanced at small commands, build/tests TestHeavyHeadroomBalance.)
        /// </summary>
        internal static void RcsTranslationColumnsHeadroom (int count, double[] lever, double[] dir,
            double[] power, double[] rho, double fracRcs, double ax, double ay, double az,
            double[] forcePos, double[] forceNeg, double[] torquePos, double[] torqueNeg)
        {
            bool scale = rho != null && fracRcs > 0.0 && fracRcs <= 1.0;
            for (int i = 0; i < count; i++) {
                double p = power [i];
                if (!(p > 0.0) || !Finite (p))
                    continue;
                double vx = dir [3 * i], vy = dir [3 * i + 1], vz = dir [3 * i + 2];
                double rx = lever [3 * i], ry = lever [3 * i + 1], rz = lever [3 * i + 2];
                double dot = vx * ax + vy * ay + vz * az;
                double r = scale ? rho [i] : 0.0;
                for (int s = 0; s < 2; s++) {
                    double f = s == 0 ? dot : -dot;
                    if (!(f > 0.0))
                        continue;
                    if (f > 1.0)
                        f = 1.0;
                    // Left out: the rotation leaves it less than the whole span (or rho not finite).
                    if (scale && !(r + fracRcs * f <= 1.0))
                        continue;
                    double k = -p * f;
                    double fx = k * vx, fy = k * vy, fz = k * vz;
                    double tx = ry * fz - rz * fy, ty = rz * fx - rx * fz, tz = rx * fy - ry * fx;
                    double[] force = s == 0 ? forcePos : forceNeg;
                    double[] torque = s == 0 ? torquePos : torqueNeg;
                    double sign = s == 0 ? 1.0 : -1.0;
                    force [0] += sign * fx;
                    force [1] += sign * fy;
                    force [2] += sign * fz;
                    torque [0] += sign * tx;
                    torque [1] += sign * ty;
                    torque [2] += sign * tz;
                }
            }
        }

        /// <summary>
        /// Bit 16: each ctrlState channel (X at [0..2], Y at [3..5] of the 6-arrays) keeps the
        /// WEAKER of its + and - columns (smaller force along its own axis: x for X, z for Y) as
        /// both, so the joint allocation's mean column is the one that fires whatever the sign
        /// (with an asymmetric headroom, the mean over-counts the weak sign by up to half). Returns
        /// the share kept (weaker / stronger along the axis, 1 when equal or both zero) per channel.
        /// </summary>
        internal static void WeakerSign (double[] fPos, double[] fNeg, double[] tPos, double[] tNeg,
            out double shareX, out double shareY)
        {
            shareX = shareY = 1.0;
            for (int a = 0; a < 2; a++) {
                int along = a == 0 ? 0 : 5;
                double p = Math.Abs (fPos [along]), n = Math.Abs (fNeg [along]);
                double share = Math.Max (p, n) > 0.0 ? Math.Min (p, n) / Math.Max (p, n) : 1.0;
                if (a == 0)
                    shareX = share;
                else
                    shareY = share;
                if (p == n)
                    continue;
                double[] fw = p < n ? fPos : fNeg, tw = p < n ? tPos : tNeg;
                double[] fs = p < n ? fNeg : fPos, ts = p < n ? tNeg : tPos;
                for (int k = 3 * a; k < 3 * a + 3; k++) {
                    fs [k] = fw [k];
                    ts [k] = tw [k];
                }
            }
        }

        /// <summary>
        /// Bit 32: one step of the measured share of the common deflection. realized: the gimbal
        /// columns' realized deflection (the law's u0 before the shift), commonPrevious: the
        /// realized common of the previous WRITTEN step (CommonRealized), count columns.
        /// Forgetting means num += alpha (sum realized c - num), den += alpha (sum c^2 - den); the
        /// share is num / den. Steps without a common (sum c^2 = 0) change nothing: the evidence
        /// stays what the translation last showed. False (nothing changed) when not finite.
        /// </summary>
        internal static bool AccumulateShare (double[] realized, double[] commonPrevious, int count,
            double alpha, ref double num, ref double den)
        {
            double sn = 0.0, sd = 0.0;
            for (int j = 0; j < count; j++) {
                sn = sn + realized [j] * commonPrevious [j];
                sd = sd + commonPrevious [j] * commonPrevious [j];
            }
            if (!(sd > 0.0) || !Finite (sd) || !Finite (sn) || !(alpha > 0.0) || !Finite (alpha))
                return false;
            double a = alpha > 1.0 ? 1.0 : alpha;
            num = num + a * (sn - num);
            den = den + a * (sd - den);
            return true;
        }

        /// <summary>
        /// Bit 32: the factor of the published authority, num / den bounded to [0, 1] once den
        /// reaches denMin (evidence), 1 before (the model's authority), 1 when not finite.
        /// </summary>
        internal static double MeasuredFactor (double num, double den, double denMin)
        {
            if (!(den >= denMin) || !(den > 0.0) || !Finite (num) || !Finite (den))
                return 1.0;
            double k = num / den;
            if (!Finite (k))
                return 1.0;
            return k > 1.0 ? 1.0 : (k < 0.0 ? 0.0 : k);
        }

        // -----------------------------------------------------------------------------------
        // evidence aging (AttitudeWrite.OptionEvidenceAging, born closed). Without it the
        // forgetting means above only move on a step that carries a sample: a share measured at 0
        // stays 0 while the client, reading the published authority 0, stops asking for the
        // translation that alone could measure it again (an absorbing state, 3-5 s in flight).
        // With it, absence of evidence is not evidence of absence:
        //   * the means forget in GAME time whatever happens: a step without a sample (no common,
        //     no commit, the layer idle in between) is a zero contribution, e^(-elapsed/tau) on
        //     num and den (the share num/den is unchanged, its evidence den decays);
        //   * a step WITH a sample after a regular step (elapsed = dt) is exactly AccumulateShare;
        //   * the published factor relaxes toward the model with the age A of the last sample:
        //     f = 1 - (1 - f_measured) e^(-A/tau). The weight a sample of age A keeps in the
        //     forgetting mean is e^(-A/tau); the missing weight goes to the prior, the model.
        //     A = 0 (a sample this step): f = f_measured, the measured value as before.
        // Hence: with the model above 0, the published authority is 0 only on a step whose fresh
        // sample says 0; one step without a sample later it is >= (1 - e^(-dt/tau)) x the model,
        // half the model after tau ln 2 (0.69 s at tau = 1 s) and 95 % after 3 tau (3.0 s).
        // -----------------------------------------------------------------------------------

        /// <summary>
        /// num and den forget elapsed seconds of game time without a sample (zero
        /// contribution): both times e^(-elapsed/tau). Nothing when elapsed or tau is not a
        /// positive finite number.
        /// </summary>
        internal static void AgeShare (ref double num, ref double den, double elapsed, double tau)
        {
            if (!(elapsed > 0.0) || !Finite (elapsed) || !(tau > 0.0) || !Finite (tau))
                return;
            double k = Math.Exp (-elapsed / tau);
            num = num * k;
            den = den * k;
        }

        /// <summary>
        /// AccumulateShare with the time since the last update. A step that carries a
        /// sample first forgets the time beyond its own step (elapsed - dt, when that exceeds
        /// half a step: two UTs one step apart differ from dt by a rounding, and a regular step
        /// is then exactly AccumulateShare); a step without one (sum c^2 = 0, not finite)
        /// forgets the whole elapsed time. Returns true when a sample was added.
        /// </summary>
        internal static bool AccumulateShareAged (double[] realized, double[] commonPrevious,
            int count, double alpha, double elapsed, double dt, double tau, ref double num,
            ref double den)
        {
            double sn = 0.0, sd = 0.0;
            for (int j = 0; j < count; j++) {
                sn = sn + realized [j] * commonPrevious [j];
                sd = sd + commonPrevious [j] * commonPrevious [j];
            }
            bool sample = sd > 0.0 && Finite (sd) && Finite (sn) && alpha > 0.0 && Finite (alpha);
            if (!sample) {
                AgeShare (ref num, ref den, elapsed, tau);
                return false;
            }
            double beyond = elapsed - dt;
            if (beyond > 0.5 * dt)
                AgeShare (ref num, ref den, beyond, tau);
            double a = alpha > 1.0 ? 1.0 : alpha;
            num = num + a * (sn - num);
            den = den + a * (sd - den);
            return true;
        }

        /// <summary>
        /// the published factor, 1 - (1 - measured) e^(-age/tau), measured the factor of
        /// MeasuredFactor (in [0, 1]); age the game seconds since the last sample (+inf or NaN:
        /// never, the model's 1). In [0, 1].
        /// </summary>
        internal static double AgedFactor (double measured, double age, double tau)
        {
            if (!Finite (measured))
                return 1.0;
            double m = measured > 1.0 ? 1.0 : (measured < 0.0 ? 0.0 : measured);
            if (double.IsNaN (age) || double.IsPositiveInfinity (age))
                return 1.0;
            if (!(age > 0.0) || !(tau > 0.0) || !Finite (tau))
                return m;
            double f = 1.0 - (1.0 - m) * Math.Exp (-age / tau);
            return f > 1.0 ? 1.0 : (f < 0.0 ? 0.0 : f);
        }

        // the named evidence block appended to the AttitudeTranslationV1 return when the
        // arming has AttitudeWrite.OptionEvidenceAging: [protocol, schema, n, n values], at a
        // fixed offset (ReturnLengthHonest) whatever the mode, the shorter layouts padded with
        // NaN. Fields are only ever appended (schema + 1); a client maps them by name
        // (AttitudeLayoutV1("translation_evidence")), never by a copied index.
        internal const int ReturnLengthHonest = 32;
        internal const int EvidenceBlockProtocol = 1;
        internal const int EvidenceBlockSchema = 1;
        internal const string EvidenceLayoutName = "translation_evidence";
        internal static readonly string[] EvidenceFields = {
            "model_authority_ms2",      // the model's lateral authority this step (NaN: not computed)
            "measured_share",           // num / den once evidence (NaN: none)
            "evidence_deg2",            // den, aged to this step (deg^2)
            "evidence_min_deg2",        // the evidence threshold (deg^2)
            "evidence_age_s",           // game s since the last sample (NaN: never)
            "factor_unaged",            // MeasuredFactor, what the factor was before an earlier change
            "factor",                   // AgedFactor, applied to the model's authority
            "servable_authority_ms2",   // = offset 1: model x factor while the layer may act, else 0
            "aging_tau_s",              // tau of the forgetting and of the aging (game s)
            "attitude_tick",            // the step these values belong to
            "ut"                        // its UT
        };

        // -----------------------------------------------------------------------------------
        // the FREE authority (AttitudeWrite.OptionFreeAuthority, born closed; see AttitudeWrite.cs).
        // The joint allocation is solved along the gap twice, with the model's bounds and with what the
        // attitude leaves (FreeGimbalRanges, FreeDiffBounds); the free authority is the model's authority
        // times FreeRatio of the two. Its block is appended to the attitude_translation_v1 return after the
        // evidence block (the evidence block itself then present, NaN without option 2): [protocol, schema,
        // n, n values], mapped by name (AttitudeLayoutV1("translation_free")).
        // -----------------------------------------------------------------------------------
        internal const int FreeBlockProtocol = 1;
        internal const int FreeBlockSchema = 1;
        internal const string FreeLayoutName = "translation_free";
        // Source of the ratio (field "source").
        internal const int FreeNotComputed = 0;   // no planned step with the computed levers
        internal const int FreeAlongGap = 1;      // solved along this step's gap
        internal const int FreeNoDirection = 2;   // no gap this step: the ratio is 1
        internal const int FreeSolveFailed = 3;   // a solve did not converge or the model reaches nothing: 1
        internal static readonly string[] FreeFields = {
            "free_authority_ms2",       // model x ratio while the layer may act, else 0 (NaN: not computed)
            "ratio",                    // along_free / along_model in [0, 1] (NaN: not computed)
            "model_authority_ms2",      // the model's authority this step (offset 25 of layout 3)
            "along_model_ms2",          // the model's reach along the gap
            "along_free_ms2",           // the same with the attitude's realized parts taken out
            "direction_x",              // the gap's unit direction, body x (NaN: none)
            "direction_z",              // body z
            "source",                   // FreeNotComputed .. FreeSolveFailed
            "gimbal_attitude_max",      // largest |attitude part| / its directional range over the gimbal columns
            "diff_attitude_max",        // largest |attitude part of a motif| / the symmetric half width
            "attitude_tick",            // the step these values belong to
            "ut"                        // its UT
        };

        /// <summary>
        /// the directional ranges of one gimbal column for the common, written at ranges[offset]
        /// (negative side) and ranges[offset + 1] (positive side), read by the gimbal-fraction scale at
        /// fracGimbal: the smaller of the range and the room the realized attitude part leaves in the box
        /// divided by fracGimbal (so that the common stays within both fracGimbal of the range and that room),
        /// never below 0. The range itself, to the bit, while the attitude leaves the room. uMin &lt;= 0 &lt;=
        /// uMax.
        /// </summary>
        internal static void FreeGimbalRanges (double attitude, double uMin, double uMax, double fracGimbal,
            double[] ranges, int offset)
        {
            double negative = -uMin, positive = uMax;
            double roomNegative = attitude - uMin, roomPositive = uMax - attitude;
            if (!(roomNegative > 0.0))
                roomNegative = 0.0;
            if (!(roomPositive > 0.0))
                roomPositive = 0.0;
            if (fracGimbal > 0.0) {
                roomNegative = roomNegative / fracGimbal;
                roomPositive = roomPositive / fracGimbal;
            }
            ranges [offset] = roomNegative < negative ? roomNegative : negative;
            ranges [offset + 1] = roomPositive < positive ? roomPositive : positive;
        }

        /// <summary>
        /// the normalized bounds (units of span = the model's |d| bound) of one motif amplitude of the
        /// translation, so that its realized attitude part plus it stays within the symmetric half width;
        /// within [-1, 1], 0 always inside (no room is 0, never a push). span &lt;= 0: [-1, 1].
        /// </summary>
        internal static void FreeDiffBounds (double attitude, double halfWidth, double span, out double lower,
            out double upper)
        {
            lower = -1.0;
            upper = 1.0;
            if (!(span > 0.0) || !Finite (span) || !Finite (attitude) || !Finite (halfWidth))
                return;
            double lo = (-halfWidth - attitude) / span, hi = (halfWidth - attitude) / span;
            lower = lo > 0.0 ? 0.0 : (lo < -1.0 ? -1.0 : lo);
            upper = hi < 0.0 ? 0.0 : (hi > 1.0 ? 1.0 : hi);
        }

        /// <summary>
        /// the ratio of the free authority, alongFree / alongModel bounded to [0, 1]; 1 when the model
        /// reaches nothing along the gap or a value is not finite (FreeSolveFailed: the model's authority).
        /// </summary>
        internal static double FreeRatio (double alongModel, double alongFree, out int source)
        {
            source = FreeAlongGap;
            if (!(alongModel > 0.0) || !Finite (alongModel) || !Finite (alongFree)) {
                source = FreeSolveFailed;
                return 1.0;
            }
            double r = alongFree / alongModel;
            return r > 1.0 ? 1.0 : (r < 0.0 ? 0.0 : r);
        }

        /// <summary>
        /// one step of the free authority. model: the joint allocation of this step, prepared (its
        /// outputs are not touched); free: a scratch instance. (ux, uz) the unit gap, body; attitude: the
        /// realized attitude part of every gimbal column (2 per gimbal, x then y), uMin / uMax their boxes;
        /// attitudeDiffP / Y and halfWidth the motifs'. Returns the ratio, with the reach along the gap with
        /// both bounds and the source. Pure: System only.
        /// </summary>
        internal static double FreeAuthorityRatio (JointTranslation model, JointTranslation free, double ux,
            double uz, double[] attitude, double[] uMin, double[] uMax, double attitudeDiffP,
            double attitudeDiffY, double halfWidth, out double alongModel, out double alongFree, out int source)
        {
            alongModel = double.NaN;
            alongFree = double.NaN;
            source = FreeSolveFailed;
            if (model == null || free == null || !model.Prepared || !Finite (ux) || !Finite (uz))
                return 1.0;
            free.CopyInputs (model);
            if (!free.Prepare ())
                return 1.0;
            alongModel = free.AuthorityAlong (ux, uz);
            double frac = free.FracGimbal;
            for (int g = 0; g < free.Gimbals; g++) {
                if (!free.GimbalUsable [g])
                    continue;
                FreeGimbalRanges (attitude [2 * g], uMin [2 * g], uMax [2 * g], frac, free.GimbalRanges, 4 * g);
                FreeGimbalRanges (attitude [2 * g + 1], uMin [2 * g + 1], uMax [2 * g + 1], frac,
                    free.GimbalRanges, 4 * g + 2);
            }
            FreeDiffBounds (attitudeDiffP, halfWidth, free.DiffMax, out free.Lower [4], out free.Upper [4]);
            FreeDiffBounds (attitudeDiffY, halfWidth, free.DiffMax, out free.Lower [5], out free.Upper [5]);
            alongFree = free.AuthorityAlong (ux, uz);
            return FreeRatio (alongModel, alongFree, out source);
        }
    }

    /// <summary>
    /// Computed levers (bit 4): the joint allocation of the lateral force. Six variables, all
    /// in one bounded least squares solved by the law's own active-set solver
    /// (AttitudeMath.SolveBoxLeastSquares, unchanged, work space preallocated here):
    ///   phi_x, phi_z  body tilt of every nozzle (deg): force K phi, torque Kt phi;
    ///   r_x, r_y      ctrlState.X, ctrlState.Y: force Rf r, torque Rt r (RcsTranslationColumns);
    ///   d_p, d_y      differential motif amplitudes: torque Bd d, no force.
    /// Rows: Fx, Fz (target m (ax, az)), tau_x, tau_y, tau_z (target 0), then one
    /// regularization row per variable. Units: forces / m (m/s^2); torques / (m L), L the
    /// gimbals' own lever (sigma(Kt) / sigma(K)), so a torque row reads as the force it would
    /// take at that lever, times TorqueWeight for pitch and yaw (RollWeight = 0 for roll, which
    /// no lever of the translation can cancel: computed, not allocated). Variables normalized
    /// by their bound (|y| &lt;= 1) for the conditioning. PURE: System only.
    /// </summary>
    internal sealed class JointTranslation
    {
        internal const int Variables = 6;
        internal const int ForceRows = 2;
        internal const int TorqueRows = 3;
        internal const int Rows = ForceRows + TorqueRows + Variables;

        /// <summary>
        /// Lexicographic weight of the torque rows, at the gimbals' lever: the torque comes
        /// first. In a weighted least squares a conflict is shared in the ratio of the squared
        /// weights, so the torque the translation may leave to buy force is 1e-6 of the force
        /// deficit times L. 1e3 and not more: with SqrtGamma the normal matrix stays within a
        /// condition number of (1e3 / 1e-3)^2 = 1e12, 4000 times under 1 / eps.
        /// </summary>
        internal const double TorqueWeight = 1e3;
        /// <summary>
        /// Weight of the ROLL torque row (tau_y): 0 -- the row is kept (its residual is
        /// computed, Torque[1]) but allocates nothing. None of the translation's levers can
        /// cancel roll (the motifs make none, the RCS translation of a symmetric ring none), so a
        /// roll row could only be met by bending the tilt or trading RCS against tilt along the
        /// null space. Its coefficients are the roll of a common tilt of clusters whose thrust
        /// centroid sits about 1 cm off the axis, a few kN.m per degree that every differential
        /// throttle change moves. Measured on the heavy booster model (TranslationTests
        /// .HeavyHover): lexicographic, the tilt hopped from step to step on a pure z demand and
        /// delivered 0.13 of 0.2 m/s^2; at the force rows' weight (1) it still outweighed the
        /// regularization 3 to 1 along the null space and made a period-2 cycle (throttles 0.027
        /// / 0.090 at 0.08, instant engines). The law's roll channel (INDI) takes that roll, as it
        /// does in modes 1 and 3.
        /// </summary>
        internal const double RollWeight = 0.0;
        /// <summary>
        /// Regularization, per unit of each variable's own effect at full span (force / m and
        /// torque / (m L), as the law normalizes by the own effect): it only picks one solution
        /// among the exact ones (six variables, four weighed rows: a two-dimensional null space
        /// at least) and keeps every free subset of full rank. 1e-3 of the force rows: the force target is met to
        /// about 1e-6 when it is reachable (the law's 1e-2 trades attitude for effort, the
        /// translation must not).
        /// </summary>
        internal const double SqrtGamma = 1e-3;
        /// <summary>Active-set iterations: each adds or frees one of 6 bounds; the law's 20 x 3.</summary>
        internal const int MaxIterations = 64;
        /// <summary>A column whose own effect is below this share of the largest is absent (bounds 0).</summary>
        internal const double AbsentColumn = 1e-12;
        /// <summary>
        /// Authority: the target along each direction is this many times the upper bound of
        /// the lateral force (sum |force column| x span), so the closest reachable point is
        /// the far face of the reachable set (the torque the target could buy stays under
        /// 1e-5 of that bound, by the torque weight).
        /// </summary>
        internal const double AuthorityTargetFactor = 10.0;
        /// <summary>
        /// Authority numerically null below this share of the force upper bound: the size of
        /// the solver's own relative imprecision on the force (regularization, weights).
        /// </summary>
        internal const double NullAuthority = 1e-6;
        /// <summary>Force target met (no "limited by" reason) within this share of it.</summary>
        internal const double TargetTolerance = 1e-3;

        // ------------------------------------------------------------------ inputs
        internal double Mass;
        /// <summary>Force per degree of body tilt: {Fx/phi_x, Fx/phi_z, Fz/phi_x, Fz/phi_z}, N.</summary>
        internal readonly double[] K = new double[4];
        /// <summary>Torque per degree of tilt, row r (x, y, z): [2r] phi_x, [2r + 1] phi_z, N.m.</summary>
        internal readonly double[] Kt = new double[6];
        /// <summary>RCS force per unit command: {Fx/X, Fx/Y, Fz/X, Fz/Y}, N.</summary>
        internal readonly double[] Rf = new double[4];
        /// <summary>RCS torque per unit command, row r: [2r] X, [2r + 1] Y, N.m.</summary>
        internal readonly double[] Rt = new double[6];
        /// <summary>Motif torque per unit amplitude, row r: [2r] d_p, [2r + 1] d_y, N.m.</summary>
        internal readonly double[] Bd = new double[6];
        /// <summary>Bounds: |phi| (deg), |r|, |d|.</summary>
        internal double PhiMax, RcsMax, DiffMax;
        /// <summary>
        /// Regularization factor of the two motif variables (tilt and RCS: 1). The plugin sets
        /// 1 / DiffPriority (2 by default): the law favours the differential for the ATTITUDE
        /// (priority 0.5 against the gimbals' 1), so the translation spares it in the inverse
        /// ratio and balances its torque with the RCS first -- the RCS answers at once, the
        /// differential at the engines' throttle response, and every amplitude it takes is
        /// removed from the law's attitude share of the motifs. Measured on the heavy booster
        /// at 0.2 m/s^2: at 1 the differential took 98 % of its bound and the RCS 0.25 of its
        /// 0.5. Only the choice among exact solutions moves: at saturation (the authority) the
        /// bounds decide.
        /// </summary>
        internal double DiffWeight = 1.0;
        /// <summary>The worst gimbal's directional fraction may not exceed this (the whole solution is scaled).</summary>
        internal double FracGimbal = 1.0;
        /// <summary>
        /// Bounds of each variable in units of its span (lower, upper): -1 and 1, the span both ways, for every
        /// caller but an earlier change's free authority, which narrows the motifs' to what the attitude leaves
        /// (AttitudeTranslation.FreeDiffBounds). CopyInputs and a new instance start at -1 / 1.
        /// </summary>
        internal readonly double[] Lower = { -1.0, -1.0, -1.0, -1.0, -1.0, -1.0 };
        internal readonly double[] Upper = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        internal int Gimbals;
        /// <summary>4 per gimbal: axisX.x, axisX.z, axisY.x, axisY.z (body components of its rotation axes).</summary>
        internal double[] GimbalAxes = new double[0];
        /// <summary>4 per gimbal: its directional ranges xn, xp, yn, yp (deg, &gt;= 0).</summary>
        internal double[] GimbalRanges = new double[0];
        internal bool[] GimbalUsable = new bool[0];

        // ---------------------------------------------------------------- prepared
        /// <summary>Lever (m) turning a torque into its force equivalent.</summary>
        internal double Lever;
        /// <summary>Upper bound of the lateral force, N: sum over the present columns of |force| x span.</summary>
        internal double ForceBound;
        internal bool Prepared;
        readonly double[] col = new double[(ForceRows + TorqueRows) * Variables];
        readonly double[] span = new double[Variables];
        readonly double[] effect = new double[Variables];
        readonly bool[] present = new bool[Variables];
        readonly AttitudeWork work = new AttitudeWork ();

        // ----------------------------------------------------------------- outputs
        /// <summary>phi_x, phi_z (deg), r_x, r_y, d_p, d_y, after the gimbal-fraction scale.</summary>
        internal readonly double[] X = new double[Variables];
        internal readonly double[] Force = new double[2];
        internal readonly double[] Torque = new double[3];
        internal double Scale = 1.0;
        internal bool AtPhiBound, AtRcsBound, AtDiffBound, Converged;
        internal int Iterations;
        /// <summary>Authority per direction (+x, -x, +z, -z), m/s^2, of the last Authority call.</summary>
        internal readonly double[] Directions = new double[4];

        internal void Resize (int gimbals)
        {
            if (GimbalUsable.Length != gimbals) {
                GimbalAxes = new double[4 * gimbals];
                GimbalRanges = new double[4 * gimbals];
                GimbalUsable = new bool[gimbals];
            }
            Gimbals = gimbals;
        }

        /// <summary>Clears every input column and bound (the caller then adds what it has).</summary>
        internal void Clear ()
        {
            Mass = 0.0;
            for (int k = 0; k < 4; k++) {
                K [k] = 0.0;
                Rf [k] = 0.0;
            }
            for (int k = 0; k < 6; k++) {
                Kt [k] = 0.0;
                Rt [k] = 0.0;
                Bd [k] = 0.0;
            }
            PhiMax = RcsMax = DiffMax = 0.0;
            DiffWeight = 1.0;
            for (int g = 0; g < GimbalUsable.Length; g++)
                GimbalUsable [g] = false;
            Prepared = false;
        }

        static double Norm2 (double a, double b)
        {
            return Math.Sqrt (a * a + b * b);
        }

        /// <summary>
        /// Columns, lever, spans, own effects and the force bound. False (nothing usable) when
        /// the mass is not positive, an input is not finite, or every column is absent.
        /// </summary>
        internal bool Prepare ()
        {
            Prepared = false;
            ForceBound = 0.0;
            if (!(Mass > 0.0) || !AttitudeTranslation.Finite (Mass))
                return false;
            for (int k = 0; k < 4; k++)
                if (!AttitudeTranslation.Finite (K [k]) || !AttitudeTranslation.Finite (Rf [k]))
                    return false;
            for (int k = 0; k < 6; k++)
                if (!AttitudeTranslation.Finite (Kt [k]) || !AttitudeTranslation.Finite (Rt [k]) ||
                        !AttitudeTranslation.Finite (Bd [k]))
                    return false;
            // Rows 0-1 force, 2-4 torque; column j at [row * Variables + j].
            for (int r = 0; r < ForceRows + TorqueRows; r++)
                for (int j = 0; j < Variables; j++)
                    col [r * Variables + j] = 0.0;
            for (int c = 0; c < 2; c++) {
                col [0 * Variables + c] = K [c];
                col [1 * Variables + c] = K [2 + c];
                col [0 * Variables + 2 + c] = Rf [c];
                col [1 * Variables + 2 + c] = Rf [2 + c];
                for (int r = 0; r < TorqueRows; r++) {
                    col [(2 + r) * Variables + c] = Kt [2 * r + c];
                    col [(2 + r) * Variables + 2 + c] = Rt [2 * r + c];
                    col [(2 + r) * Variables + 4 + c] = Bd [2 * r + c];
                }
            }
            // The lever of the torque rows: the gimbals' own (torque of a tilt over its force);
            // without a gimbal, the RCS's; without either, 1 m (nothing to weigh).
            double sk = AttitudeTranslation.MaxSingular2 (K [0], K [1], K [2], K [3]);
            double skt = AttitudeTranslation.MaxSingular2 (Kt [0], Kt [1], Kt [4], Kt [5]);
            double sr = AttitudeTranslation.MaxSingular2 (Rf [0], Rf [1], Rf [2], Rf [3]);
            double srt = AttitudeTranslation.MaxSingular2 (Rt [0], Rt [1], Rt [4], Rt [5]);
            if (sk > 0.0 && skt > 0.0)
                Lever = skt / sk;
            else if (sr > 0.0 && srt > 0.0)
                Lever = srt / sr;
            else
                Lever = 1.0;
            if (!(Lever > 0.0) || !AttitudeTranslation.Finite (Lever))
                Lever = 1.0;
            span [0] = span [1] = PhiMax > 0.0 && AttitudeTranslation.Finite (PhiMax) ? PhiMax : 0.0;
            span [2] = span [3] = RcsMax > 0.0 && AttitudeTranslation.Finite (RcsMax) ? RcsMax : 0.0;
            span [4] = span [5] = DiffMax > 0.0 && AttitudeTranslation.Finite (DiffMax) ? DiffMax : 0.0;
            double maxEffect = 0.0;
            for (int j = 0; j < Variables; j++) {
                double f2 = 0.0, t2 = 0.0;
                for (int r = 0; r < ForceRows; r++)
                    f2 = f2 + col [r * Variables + j] * col [r * Variables + j];
                for (int r = ForceRows; r < ForceRows + TorqueRows; r++) {
                    // Pitch and yaw at 1, roll at RollWeight (as the rows are weighed).
                    double w = r == ForceRows + 1 ? RollWeight : 1.0;
                    t2 = t2 + w * w * col [r * Variables + j] * col [r * Variables + j];
                }
                // Own effect at full span, acceleration-equivalent (torque at the lever L).
                effect [j] = Math.Sqrt (f2 / (Mass * Mass) + t2 / (Mass * Mass * Lever * Lever)) * span [j];
                if (effect [j] > maxEffect)
                    maxEffect = effect [j];
            }
            if (!(maxEffect > 0.0) || !AttitudeTranslation.Finite (maxEffect))
                return false;
            for (int j = 0; j < Variables; j++) {
                present [j] = effect [j] > AbsentColumn * maxEffect;
                if (present [j])
                    ForceBound = ForceBound + Norm2 (col [0 * Variables + j], col [1 * Variables + j]) * span [j];
            }
            Prepared = true;
            return true;
        }

        /// <summary>
        /// Solves for the lateral force (fx, fz) (N, body) with zero torque. Then the whole
        /// solution is scaled so that the worst usable gimbal stays within FracGimbal of its
        /// directional range (the tilt box is on body axes, a gimbal's box on its own). False
        /// when not prepared or the target is not finite; Converged false when the solver ran
        /// out of iterations or a Cholesky pivot failed (X is then its last feasible iterate).
        /// </summary>
        internal bool Solve (double fx, double fz)
        {
            for (int j = 0; j < Variables; j++)
                X [j] = 0.0;
            Force [0] = Force [1] = 0.0;
            Torque [0] = Torque [1] = Torque [2] = 0.0;
            Scale = 1.0;
            AtPhiBound = AtRcsBound = AtDiffBound = false;
            Converged = false;
            Iterations = 0;
            if (!Prepared || !AttitudeTranslation.Finite (fx) || !AttitudeTranslation.Finite (fz))
                return false;
            work.Resize (Rows, Variables);
            double[] A = work.A, b = work.Bv;
            for (int i = 0; i < Rows * Variables; i++)
                A [i] = 0.0;
            double inv = 1.0 / Mass;
            double invT = TorqueWeight / (Mass * Lever);
            double invRoll = RollWeight / (Mass * Lever);
            for (int j = 0; j < Variables; j++) {
                double s = present [j] ? span [j] : 0.0;
                for (int r = 0; r < ForceRows; r++)
                    A [r * Variables + j] = col [r * Variables + j] * s * inv;
                // Torque rows x (pitch) and z (yaw) lexicographic, y (roll) at RollWeight (0).
                for (int r = ForceRows; r < ForceRows + TorqueRows; r++)
                    A [r * Variables + j] = col [r * Variables + j] * s *
                        (r == ForceRows + 1 ? invRoll : invT);
                // Absent: fixed at 0 by its bounds, never in the free set; any positive weight.
                double wr = j >= 4 && DiffWeight > 0.0 && AttitudeTranslation.Finite (DiffWeight) ? DiffWeight : 1.0;
                A [(ForceRows + TorqueRows + j) * Variables + j] = present [j] ? SqrtGamma * wr * effect [j] : 1.0;
                work.Lb [j] = present [j] ? Lower [j] : 0.0;
                work.Ub [j] = present [j] ? Upper [j] : 0.0;
                work.W [j] = AttitudeMath.SetFree;
            }
            b [0] = fx * inv;
            b [1] = fz * inv;
            for (int i = ForceRows; i < Rows; i++)
                b [i] = 0.0;
            bool converged;
            Iterations = AttitudeMath.SolveBoxLeastSquares (work, Rows, Variables, MaxIterations,
                out converged);
            Converged = converged;
            const double atBound = 1.0 - 1e-9;
            for (int j = 0; j < Variables; j++) {
                double y = work.X [j];
                X [j] = present [j] ? y * span [j] : 0.0;
                bool bound = present [j] && Math.Abs (y) >= atBound;
                if (bound && j < 2)
                    AtPhiBound = true;
                else if (bound && j < 4)
                    AtRcsBound = true;
                else if (bound)
                    AtDiffBound = true;
            }
            // The worst gimbal fraction of the tilt, as ComputeCommon's bound 1.
            double worst = 0.0;
            for (int g = 0; g < Gimbals && g < GimbalUsable.Length; g++) {
                if (!GimbalUsable [g])
                    continue;
                double cx = X [0] * GimbalAxes [4 * g] + X [1] * GimbalAxes [4 * g + 1];
                double cy = X [0] * GimbalAxes [4 * g + 2] + X [1] * GimbalAxes [4 * g + 3];
                double f = AttitudeTranslation.GimbalFraction (cx, cy, GimbalRanges [4 * g],
                    GimbalRanges [4 * g + 1], GimbalRanges [4 * g + 2], GimbalRanges [4 * g + 3]);
                if (f > worst)
                    worst = f;
            }
            if (worst > FracGimbal) {
                Scale = FracGimbal > 0.0 && !double.IsInfinity (worst) ? FracGimbal / worst : 0.0;
                for (int j = 0; j < Variables; j++)
                    X [j] = X [j] * Scale;
            }
            for (int j = 0; j < Variables; j++) {
                Force [0] = Force [0] + col [0 * Variables + j] * X [j];
                Force [1] = Force [1] + col [1 * Variables + j] * X [j];
                for (int r = 0; r < TorqueRows; r++)
                    Torque [r] = Torque [r] + col [(ForceRows + r) * Variables + j] * X [j];
            }
            return true;
        }

        /// <summary>
        /// The largest torque-free lateral acceleration (m/s^2) the allocation reaches, the
        /// smallest over the body directions +x, -x, +z, -z (Directions): each asks
        /// AuthorityTargetFactor x ForceBound along the direction and keeps the component of
        /// the force reached (after the gimbal scale) along it. 0 when not prepared, when a
        /// solve fails or does not converge (its last iterate is within the bounds but may carry
        /// torque: not an honest authority), or when it is numerically null (NullAuthority x
        /// ForceBound / m).
        /// </summary>
        internal double Authority ()
        {
            for (int q = 0; q < 4; q++)
                Directions [q] = 0.0;
            if (!Prepared || !(ForceBound > 0.0))
                return 0.0;
            double target = AuthorityTargetFactor * ForceBound;
            double best = double.PositiveInfinity;
            for (int q = 0; q < 4; q++) {
                double ux = q == 0 ? 1.0 : (q == 1 ? -1.0 : 0.0);
                double uz = q == 2 ? 1.0 : (q == 3 ? -1.0 : 0.0);
                if (!Solve (ux * target, uz * target) || !Converged)
                    return 0.0;
                double a = (Force [0] * ux + Force [1] * uz) / Mass;
                Directions [q] = a;
                if (a < best)
                    best = a;
            }
            if (!(best > NullAuthority * ForceBound / Mass) || double.IsInfinity (best))
                return 0.0;
            return best;
        }

        /// <summary>
        /// the reach along the unit body direction (ux, uz), m/s^2, solved as one direction of
        /// Authority: the component along it of the force reached for a target AuthorityTargetFactor x
        /// ForceBound along it, after the gimbal scale. 0 when numerically null (NullAuthority) or not
        /// prepared; NaN when the solve fails or does not converge.
        /// </summary>
        internal double AuthorityAlong (double ux, double uz)
        {
            if (!Prepared || !(ForceBound > 0.0))
                return 0.0;
            double target = AuthorityTargetFactor * ForceBound;
            if (!Solve (ux * target, uz * target) || !Converged)
                return double.NaN;
            double a = (Force [0] * ux + Force [1] * uz) / Mass;
            if (!(a > NullAuthority * ForceBound / Mass) || double.IsInfinity (a))
                return 0.0;
            return a;
        }

        /// <summary>
        /// every input of src (columns, bounds, gimbal geometry), the variables' bounds back to -1 / 1;
        /// not prepared until Prepare.
        /// </summary>
        internal void CopyInputs (JointTranslation src)
        {
            Mass = src.Mass;
            for (int k = 0; k < 4; k++) {
                K [k] = src.K [k];
                Rf [k] = src.Rf [k];
            }
            for (int k = 0; k < 6; k++) {
                Kt [k] = src.Kt [k];
                Rt [k] = src.Rt [k];
                Bd [k] = src.Bd [k];
            }
            PhiMax = src.PhiMax;
            RcsMax = src.RcsMax;
            DiffMax = src.DiffMax;
            DiffWeight = src.DiffWeight;
            FracGimbal = src.FracGimbal;
            Resize (src.Gimbals);
            for (int g = 0; g < src.Gimbals; g++) {
                GimbalUsable [g] = src.GimbalUsable [g];
                for (int q = 0; q < 4; q++) {
                    GimbalAxes [4 * g + q] = src.GimbalAxes [4 * g + q];
                    GimbalRanges [4 * g + q] = src.GimbalRanges [4 * g + q];
                }
            }
            for (int j = 0; j < Variables; j++) {
                Lower [j] = -1.0;
                Upper [j] = 1.0;
            }
            Prepared = false;
        }
    }
}
