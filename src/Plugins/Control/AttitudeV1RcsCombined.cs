using System;
using KRPC.Bridge.Core;
using KRPC.Bridge.Model;
using UnityEngine;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// AttitudeV1, (option 16384 of AttitudeOptionsV1, born closed): the RCS of the ctrlState channel as ModuleRCS
    /// fires it -- one rotation vector per module, the dead zone per component and per nozzle, each nozzle bounded at
    /// 1. The maths and the measurement behind it are in RcsCombined.cs (Model); this file touches the game.
    ///
    /// At every step of an arming with the option:
    ///   * RcsCombinedPrepare: the nozzles of every RCS module of the cache (AttitudeGeometry.RcsRotationNozzles, the
    ///     conditions of AddRcsColumns: enabled, RCS group on, not for show, not shielded), body frame about the CoM,
    ///     their EPSILON, fullThrust and fullThrustMin, then each module's per-axis columns (RcsCombined.Prepare), and
    ///     the rest of the ctrlState columns (wheels, surfaces: the law's columns less the RCS's). Not modelled (nothing
    ///     below changes this step) in precision mode, which the combined rule does not carry;
    ///   * the in-flight torque of (option 32) by the combined rule (RcsCombinedInFlight, from
    ///     AttitudeV1RcsInFlight.cs);
    ///   * RcsCombinedDecide, at the roll's write: the ctrlState command the RCS answers -- the law's, or its roll held
    ///     (capability bit 26), or its pitch/yaw held (bit 27) when the step writes them too; a held part written just
    ///     under the RCS dead zone (RcsCombined.HeldValue).
    /// Capability bit 25: the nozzles were modelled this step.
    /// </summary>
    internal static partial class AttitudeV1
    {
        // Capability bits of the frame header (offset 23), option 16384 only.
        internal const long CapRcsCombined = 1L << 25;
        internal const long CapRcsRollHeld = 1L << 26;
        internal const long CapRcsPitchYawHeld = 1L << 27;

        internal sealed class RcsCombinedState
        {
            internal readonly RcsNozzleSet Set = new RcsNozzleSet ();
            internal readonly RcsCombinedWork Work = new RcsCombinedWork ();
            // Per module of Entry.Rcs (same index): its module in Set this step, -1 when it adds nothing.
            internal int[] SetIndex = new int[0];
            // The nozzles of this step are in Set (prepared), every module modelled.
            internal bool Modelled;
            // The rest of the ctrlState columns (the law's less the RCS's: wheels, surfaces), per unit, body; column a
            // (0 pitch, 1 roll, 2 yaw) at [3a .. 3a + 2], mean of its halves.
            internal readonly double[] Rest = new double[9];
            // This step's decision (RcsCombined.KeepCommand, RollHeld, PitchYawHeld), the attitude tick it was made at,
            // and the counts since the arming.
            internal int Decision;
            internal long DecisionTick = -1;
            // The steps in a row the roll / the pitch-yaw was left out, up to the last decision.
            internal int RollHeldRun, PitchYawHeldRun;
            internal long RollHeldSteps, PitchYawHeldSteps;

            internal void Reset ()
            {
                Modelled = false;
                Decision = RcsCombined.KeepCommand;
                DecisionTick = -1;
                RollHeldRun = 0;
                PitchYawHeldRun = 0;
                RollHeldSteps = 0;
                PitchYawHeldSteps = 0;
                Set.Clear ();
            }
        }

        // Scratch (Unity main thread only).
        static double[] rcLever = new double[0], rcDir = new double[0], rcPower = new double[0];
        static readonly double[] rcWeight = new double[3];

        static bool RcsCombinedOn (Entry e)
        {
            return (e.Options & AttitudeWrite.OptionRcsCombined) != 0;
        }

        /// <summary>
        /// this step's nozzles of every RCS module of the cache into e.Rc.Set, their columns, and the rest of the
        /// ctrlState columns (from e.BPos / e.BNeg, already summed). Born closed: the caller runs it only with option
        /// 16384. False (not modelled) in precision mode or on an exception. Never throws.
        /// </summary>
        static bool RcsCombinedPrepare (Entry e, Vector3 com, Transform reference, bool precision)
        {
            var c = e.Rc;
            c.Modelled = false;
            c.Set.Clear ();
            int count = e.Rcs.Count;
            if (c.SetIndex.Length != count)
                c.SetIndex = new int[count];
            for (int i = 0; i < count; i++)
                c.SetIndex [i] = -1;
            if (precision)
                return false;
            try {
                for (int i = 0; i < count; i++) {
                    var rcs = e.Rcs [i];
                    if (rcs == null)
                        continue;
                    int nozzles;
                    bool ep, er, ey;
                    if (!AttitudeGeometry.RcsRotationNozzles (rcs, com, reference, ref rcLever, ref rcDir, ref rcPower,
                            out nozzles, out ep, out er, out ey))
                        continue;
                    int first = c.Set.Add (nozzles, ep, er, ey, rcs.EPSILON, rcs.fullThrust, rcs.fullThrustMin);
                    Array.Copy (rcLever, 0, c.Set.Lever, 3 * first, 3 * nozzles);
                    Array.Copy (rcDir, 0, c.Set.Dir, 3 * first, 3 * nozzles);
                    Array.Copy (rcPower, 0, c.Set.Power, first, nozzles);
                    c.SetIndex [i] = c.Set.Modules - 1;
                }
                RcsCombined.Prepare (c.Set, c.Work.Scratch);
            } catch (Exception exc) {
                c.Set.Clear ();
                for (int i = 0; i < count; i++)
                    c.SetIndex [i] = -1;
                if (++e.FbwExceptions <= 5)
                    BridgeLog.Error ("AttitudeV1 RCS combine: " + exc.Message);
                return false;
            }
            var s = c.Set;
            for (int k = 0; k < 9; k++) {
                double rcs = 0.0;
                for (int m = 0; m < s.Modules; m++)
                    rcs += 0.5 * (s.ColPos [9 * m + k] + s.ColNeg [9 * m + k]);
                c.Rest [k] = 0.5 * (e.BPos [k] + e.BNeg [k]) - rcs;
            }
            c.Modelled = true;
            return true;
        }

        /// <summary>
        /// at the roll's write: the decision of RcsCombined.Decide on the command the ctrlState would carry (the
        /// law's pitch/yaw if it writes them this step, else the served ones; its roll), from the served ctrlState
        /// (e.Served, the law's u0), with the law's row weights
        /// (Wv_a / I_a) and the steps in a row each part was left out (when the last decision was at the previous attitude
        /// tick; else none).
        /// pitchYawOwn: the loop writes pitch/yaw this step (the roll alone is then a candidate). Sets e.Rc.Decision and
        /// returns it.
        /// </summary>
        static int RcsCombinedDecide (Entry e, AttitudeInput inp, double pitch, double roll, double yaw, bool pitchYawOwn)
        {
            var c = e.Rc;
            if (c.DecisionTick != ownTick - 1) {
                c.RollHeldRun = 0;
                c.PitchYawHeldRun = 0;
            }
            c.Decision = RcsCombined.KeepCommand;
            c.DecisionTick = ownTick;
            if (!c.Modelled || double.IsNaN (roll)) {
                c.RollHeldRun = 0;
                c.PitchYawHeldRun = 0;
                return c.Decision;
            }
            rcWeight [0] = e.Gains.WvPitchYaw / inp.Inertia [0];
            rcWeight [1] = e.Gains.WvRoll / inp.Inertia [1];
            rcWeight [2] = e.Gains.WvPitchYaw / inp.Inertia [2];
            c.Decision = RcsCombined.Decide (c.Set, e.Served, pitch, roll, yaw, c.Rest, rcWeight, pitchYawOwn, c.RollHeldRun,
                c.PitchYawHeldRun, c.Work);
            c.RollHeldRun = c.Decision == RcsCombined.RollHeld ? c.RollHeldRun + 1 : 0;
            c.PitchYawHeldRun = c.Decision == RcsCombined.PitchYawHeld ? c.PitchYawHeldRun + 1 : 0;
            if (c.Decision == RcsCombined.RollHeld)
                c.RollHeldSteps++;
            else if (c.Decision == RcsCombined.PitchYawHeld)
                c.PitchYawHeldSteps++;
            return c.Decision;
        }

        /// <summary>
        /// in the in-flight torque of module i's (Entry.Rcs index) combined torque at the command its next
        /// Update makes of the served ctrlState minus that at the command it fired, ADDED into torque. False (nothing
        /// added) when the module is not in this step's set or a value is not finite.
        /// </summary>
        static bool RcsCombinedInFlight (Entry e, int i, double[] served, double[] fired, double[] torque)
        {
            var c = e.Rc;
            if (!c.Modelled || i >= c.SetIndex.Length || c.SetIndex [i] < 0)
                return false;
            int m = c.SetIndex [i];
            var w = c.Work;
            for (int a = 0; a < 3; a++)
                if (double.IsNaN (fired [a]) || double.IsInfinity (fired [a]))
                    return false;
            RcsCombined.ModuleCommand (c.Set, m, served [0], served [1], served [2], w.Command);
            w.Combined [0] = w.Combined [1] = w.Combined [2] = 0.0;
            RcsCombined.ModuleTorque (c.Set, m, w.Command [0], w.Command [1], w.Command [2], true, w.Combined);
            w.Fired [0] = w.Fired [1] = w.Fired [2] = 0.0;
            RcsCombined.ModuleTorque (c.Set, m, fired [0], fired [1], fired [2], true, w.Fired);
            double dx = w.Combined [0] - w.Fired [0], dy = w.Combined [1] - w.Fired [1], dz = w.Combined [2] - w.Fired [2];
            if (double.IsNaN (dx + dy + dz) || double.IsInfinity (dx + dy + dz))
                return false;
            torque [0] += dx;
            torque [1] += dy;
            torque [2] += dz;
            return true;
        }
    }
}
