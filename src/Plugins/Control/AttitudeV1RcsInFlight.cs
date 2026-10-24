using System;
using System.Reflection;
using KRPC.Bridge.Core;
using KRPC.Bridge.Model;
using UnityEngine;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// AttitudeV1, (option 32 of AttitudeOptionsV1, born closed): the RCS torque in flight,
    /// taken off the allocation target. The maths and the measurement behind it are in
    /// RcsInFlight.cs (Model); this file touches the game.
    ///
    /// Per RCS module of the vessel cache, at every Earlyish:
    ///   * its own columns (AttitudeGeometry.AddRcsColumns, the ones the WLS sums into the ctrlState
    ///     columns), and whether its model is exact (no precision mode, no fullThrust);
    ///   * what its next Update will make of the served ctrlState (enable flags, its EPSILON);
    ///   * the body command it fired at the PREVIOUS physics step -- its private inputRot read at the
    ///     previous Earlyish, which is what that step's FixedUpdate used (no render frame runs between
    ///     an Earlyish and the FixedUpdates of its own physics step) -- so the in-flight torque is right
    ///     whatever the frame rate (one step late at one physics step per frame, more when several
    ///     share a frame);
    ///   * this step's inputRot, kept for the next step.
    /// The sum over the modules is the in-flight torque. Nothing (and capability bit 16 not set) at the
    /// first step of an arming, after a topology change or a tick gap, or when inputRot cannot be read
    /// (a KSP whose ModuleRCS lacks the field): the law then runs exactly as without the option.
    /// </summary>
    internal static partial class AttitudeV1
    {
        // Capability bit of the frame header (offset 23): the in-flight RCS torque was taken off the
        // allocation target this step (option 32 only).
        internal const long CapRcsInFlight = 1L << 16;

        // ModuleRCS.inputRot (private Vector3, world frame), resolved once.
        static bool inputRotResolved;
        static FieldInfo inputRotField;

        // Scratch (Unity main thread only).
        static readonly double[] inFlightPos = new double[9], inFlightNeg = new double[9];
        static readonly double[] inFlightNext = new double[3], inFlightRealized = new double[3];
        static readonly double[] inFlightScratch = new double[3];

        internal sealed class RcsInFlightState
        {
            // Per module of Entry.Rcs (same index): the body command read from its inputRot at the
            // Earlyish of ReadTick, 3 per module.
            internal double[] Fired = new double[0];
            internal long ReadTick = -1;
            internal ulong TopologyHash;
            // This step: the torque in flight (body, N.m) and whether it was computed.
            internal readonly double[] Torque = new double[3];
            internal bool Valid;
            internal int Modules;

            internal void Reset ()
            {
                ReadTick = -1;
                Valid = false;
                Modules = 0;
                Torque [0] = 0.0;
                Torque [1] = 0.0;
                Torque [2] = 0.0;
            }
        }

        static FieldInfo InputRotField ()
        {
            if (!inputRotResolved) {
                inputRotResolved = true;
                try {
                    var f = typeof (ModuleRCS).GetField ("inputRot",
                        BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                    inputRotField = f != null && f.FieldType == typeof (Vector3) ? f : null;
                } catch {
                    inputRotField = null;
                }
            }
            return inputRotField;
        }

        /// <summary>
        /// this step's RCS torque in flight (e.RcsFlight.Torque, Valid), from the served ctrlState
        /// (pitch, roll, yaw), and this step's inputRot of every module kept for the next; at Earlyish only. Born
        /// closed: the caller runs it only with option 32. Never throws.
        /// </summary>
        static void RcsInFlightStep (Entry e, Vector3 com, Transform reference, bool precision,
            double[] served, ulong topologyHash, int stage)
        {
            var f = e.RcsFlight;
            f.Valid = false;
            f.Modules = 0;
            f.Torque [0] = 0.0;
            f.Torque [1] = 0.0;
            f.Torque [2] = 0.0;
            var field = InputRotField ();
            int count = e.Rcs.Count;
            // Only at Earlyish: the Pump's fallback runs after the parts' FixedUpdates of its step, where
            // inputRot is no longer "what this step will fire" (and the loop never writes there).
            if (field == null || count == 0 || stage != StageEarlyish) {
                f.ReadTick = -1;
                return;
            }
            if (f.Fired.Length != 3 * count) {
                f.Fired = new double[3 * count];
                f.ReadTick = -1;
            }
            // The previous read belongs to this layout and to the physics step just before this one.
            bool previous = f.ReadTick >= 0 && f.ReadTick == ownTick - 1 && f.TopologyHash == topologyHash;
            bool ok = true;
            try {
                for (int i = 0; i < count; i++) {
                    var rcs = e.Rcs [i];
                    double fx = double.NaN, fy = double.NaN, fz = double.NaN;
                    if (rcs != null) {
                        var world = (Vector3)field.GetValue (rcs);
                        Vector3 body = reference.InverseTransformDirection (world);
                        fx = body.x;
                        fy = body.y;
                        fz = body.z;
                    }
                    if (previous && rcs != null && RcsCombinedOn (e) && e.Rc.Modelled) {
                        // (born closed: option 16384): the module's torque by ModuleRCS's combined rule, at the
                        // command its next Update makes of the served ctrlState minus that at the command it fired. A
                        // module out of this step's set (disabled, for show, shielded) adds nothing, as before.
                        inFlightRealized [0] = f.Fired [3 * i];
                        inFlightRealized [1] = f.Fired [3 * i + 1];
                        inFlightRealized [2] = f.Fired [3 * i + 2];
                        if (RcsCombinedInFlight (e, i, served, inFlightRealized, f.Torque))
                            f.Modules++;
                    } else if (previous && rcs != null) {
                        for (int k = 0; k < 9; k++) {
                            inFlightPos [k] = 0.0;
                            inFlightNeg [k] = 0.0;
                        }
                        // The module's own columns; an approximate model (precision, fullThrust) adds nothing.
                        if (AttitudeGeometry.AddRcsColumns (rcs, com, reference, precision, inFlightPos, inFlightNeg)) {
                            RcsInFlight.UpdateCommand (served [0], served [1], served [2], rcs.enablePitch,
                                rcs.enableRoll, rcs.enableYaw, rcs.EPSILON, inFlightNext);
                            inFlightRealized [0] = f.Fired [3 * i];
                            inFlightRealized [1] = f.Fired [3 * i + 1];
                            inFlightRealized [2] = f.Fired [3 * i + 2];
                            if (RcsInFlight.AddInFlight (inFlightPos, inFlightNeg, inFlightNext, inFlightRealized,
                                    inFlightScratch, f.Torque))
                                f.Modules++;
                        }
                    }
                    f.Fired [3 * i] = fx;
                    f.Fired [3 * i + 1] = fy;
                    f.Fired [3 * i + 2] = fz;
                }
            } catch (Exception exc) {
                ok = false;
                if (++e.FbwExceptions <= 5)
                    BridgeLog.Error ("AttitudeV1 couple RCS en vol: " + exc.Message);
            }
            if (!ok) {
                f.ReadTick = -1;
                f.Torque [0] = 0.0;
                f.Torque [1] = 0.0;
                f.Torque [2] = 0.0;
                f.Modules = 0;
                return;
            }
            f.ReadTick = ownTick;
            f.TopologyHash = topologyHash;
            f.Valid = previous && !double.IsNaN (f.Torque [0] + f.Torque [1] + f.Torque [2]) &&
                !double.IsInfinity (f.Torque [0] + f.Torque [1] + f.Torque [2]);
            if (!f.Valid) {
                f.Torque [0] = 0.0;
                f.Torque [1] = 0.0;
                f.Torque [2] = 0.0;
            }
        }

        /// <summary>
        /// the in-flight torque into the law's free torque, after the ctrlState and surface columns
        /// (the surfaces' own M_free, lot 3, is kept and added to). True when the step used it (capability
        /// bit 16).
        /// </summary>
        static bool RcsInFlightApply (Entry e, AttitudeInput inp)
        {
            var f = e.RcsFlight;
            if (!f.Valid)
                return false;
            if (!inp.UseTorqueFree) {
                inp.TorqueFree [0] = 0.0;
                inp.TorqueFree [1] = 0.0;
                inp.TorqueFree [2] = 0.0;
            }
            inp.TorqueFree [0] += f.Torque [0];
            inp.TorqueFree [1] += f.Torque [1];
            inp.TorqueFree [2] += f.Torque [2];
            inp.UseTorqueFree = true;
            return true;
        }
    }
}
