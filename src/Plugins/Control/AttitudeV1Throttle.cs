using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using KRPC.Bridge.Model;
using UnityEngine;
using SCReferenceFrame = KRPC.SpaceCenter.Services.ReferenceFrame;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// AttitudeV1, (option 512 of AttitudeOptionsV1, born closed): the main throttle decided at the
    /// physics step. The decision is MainThrottle.cs (Model, pure); this file touches the game.
    ///
    /// attitude_throttle_v1 stores the client's order (a new sequence; a renewal of the same kind keeps its
    /// runtime: the episode of thrust, its guard, a fired trigger; another kind or STOP starts afresh). At every
    /// Earlyish, after the law, the step's inputs are read (ThrottleStep): the main-throttle engines' realized and
    /// full thrust (finalThrust per unit of currentThrottle while thrusting, MaxThrustOutputAtm at the part's
    /// pressure otherwise, limiter excluded, times thrustPercentage: ModuleEngines.UpdateThrottle asks
    /// mainThrottle x thrustPercentage / 100, decompiled); for a CROSSING order, s = a . V + c (a from the order's
    /// frame to the world, V the vessel's srf_velocity or obt_velocity); for an IGNITE order, the altitude above
    /// h_ref and the vertical speed; for the CALAGE, the law's pointing error, pseudo-command and inertia, the
    /// external torque I w_dot_f - M_f of its own filters (the measure an earlier change checked against kRPC's aerodynamic
    /// torque, slope 0.99-1.005), the RCS + wheels authority of the order (its ctrlState columns' when the order
    /// has none), the gimbals' authority at full main throttle (each gimbal module's columns at the full thrust of
    /// its part's engines, the law's alpha rule over the span; a module mapped per pivot through a scratch
    /// mapping, its free columns), the nose against the boostback direction and the burn axis of the order, the
    /// local vertical (Vessel.upAxis), the mass, and an earlier change's slope from the order. The decision is written at THIS
    /// step's fly-by-wire (ctrlState.mainThrottle, our link last in the chain), before the lateral force's
    /// differential throttles (which take the main throttle of the step). Capability bit 20 on the steps that
    /// decided a value.
    ///
    /// The loop writes only armed (mode 1) with its fly-by-wire link on this vessel, at Earlyish (not in the
    /// pump's fallback), out of warp; else the order is Blocked and the client's throttle flies. At the expiry of
    /// the arming's lease while the loop wrote the throttle at the last step, a lone fly-by-wire link stays on the
    /// vessel and writes 0 until the client's throttle changes (kRPC keeps a disconnected client's throttle,
    /// an earlier change), a new arming of the vessel or the scene's end.
    /// </summary>
    internal static partial class AttitudeV1
    {
        // Capability bit of the frame header (offset 23), option 512 only.
        internal const long CapMainThrottle = 1L << 20;
        internal const int ThrottleReplyLength = 39;

        internal sealed class ThrottleState
        {
            internal readonly ThrottleOrder Order = new ThrottleOrder ();
            internal readonly ThrottleStepInput In = new ThrottleStepInput ();
            internal readonly CalageOutput Out = new CalageOutput ();
            internal readonly GimbalPivotCommand Scratch = new GimbalPivotCommand ();
            internal readonly double[] Parameters = new double[MainThrottle.MaxParameters];
            internal SCReferenceFrame Frame;
            // The decision of this step (Earlyish), consumed by the fly-by-wire callback.
            internal bool Write;
            internal double WriteValue = double.NaN;
            internal long WriteTick = -1;
            // Published.
            internal int State, Motif;
            internal int DecideKind = -1, DecideSequence;
            internal double Decided = double.NaN;
            internal long DecideTick = -1;
            internal double DecideUt = double.NaN;
            internal double ClientSeen = double.NaN, Written = double.NaN;
            internal long FbwTick = -1;
            internal long Writes, Held, Missed;
            internal double Gx = double.NaN, Gz = double.NaN, Mx = double.NaN, Mz = double.NaN;
            internal double FractionLost = double.NaN, DotTarget = double.NaN;

            internal void Reset ()
            {
                Order.Clear ();
                Out.Clear ();
                Frame = null;
                Write = false;
                WriteValue = double.NaN;
                WriteTick = -1;
                State = MainThrottle.StateIdle;
                Motif = MainThrottle.MotifNone;
                DecideKind = -1;
                DecideSequence = 0;
                Decided = double.NaN;
                DecideTick = -1;
                DecideUt = double.NaN;
                ClientSeen = double.NaN;
                Written = double.NaN;
                FbwTick = -1;
                Writes = Held = Missed = 0;
                Gx = Gz = Mx = Mz = double.NaN;
                FractionLost = DotTarget = double.NaN;
                In.ThrustNowN = In.FullThrustN = In.GuardFullThrustN = double.NaN;
            }
        }

        // ---------------------------------------------------------------- RPC body

        /// <summary>
        /// Stores an order of the main throttle (MainThrottle kinds, parameters and validity; see the RPC). Refused
        /// without option 512. Returns the attitude_throttle_v1 reply: accepted 1, or 0 for a sequence not newer (a
        /// read: the order is unchanged).
        /// </summary>
        internal static IList<double> Throttle (string token, int sequence, int kind, SCReferenceFrame frame,
            IList<double> parameters, double utStamp, double utValidUntil)
        {
            var entry = RequireToken (token);
            if ((entry.Options & AttitudeWrite.OptionMainThrottle) == 0)
                throw new InvalidOperationException (BridgeErrors.Tag (BridgeErrors.UnsupportedMode,
                    "manette principale au pas physique : option 512 d'attitude_options_v1 requise (D252 K5)"));
            var th = entry.Th;
            int count = parameters == null ? 0 : parameters.Count;
            if (kind < 0 || kind >= MainThrottle.KindCount || count > MainThrottle.MaxParameters)
                throw new ArgumentOutOfRangeException (nameof (kind), BridgeErrors.Tag (BridgeErrors.UnsupportedMode,
                    "0 arret, 1 rampe (u0, pente), 2 impulsion (u, pas), 3 coupure quand s = 0, 4 allumage quand " +
                    "h <= h_ign(v), 5 calage du flip ; chaque ordre finit par seuil et pas de chaine de la garde (D198)"));
            for (int k = 0; k < count; k++)
                th.Parameters [k] = parameters [k];
            if (!MainThrottle.ValidParameters (kind, th.Parameters, count))
                throw new ArgumentOutOfRangeException (nameof (parameters), BridgeErrors.Tag (BridgeErrors.UnsupportedMode,
                    "parametres de l'ordre " + kind + " : " + MainThrottle.ParameterCounts [kind] +
                    " valeurs attendues, finies"));
            if (!Finite (utStamp) || !Finite (utValidUntil) || utValidUntil < utStamp)
                throw new ArgumentOutOfRangeException ("ut", "validite non finie ou anterieure");
            if ((kind == MainThrottle.KindCrossing || kind == MainThrottle.KindCalage) && frame == null)
                throw new ArgumentNullException (nameof (frame));
            var o = th.Order;
            if (sequence <= o.Sequence)
                return ThrottleReply (entry, 0.0);
            bool renewal = o.Kind == kind && kind != MainThrottle.KindStop;
            o.Kind = kind;
            o.Sequence = sequence;
            o.Count = count;
            for (int k = 0; k < count; k++)
                o.P [k] = th.Parameters [k];
            o.UtStamp = utStamp;
            o.UtValidUntil = utValidUntil;
            if (!renewal)
                o.ResetRuntime ();
            th.Frame = frame;
            return ThrottleReply (entry, 1.0);
        }

        static IList<double> ThrottleReply (Entry e, double accepted)
        {
            var th = e.Th;
            var o = th.Order;
            var c = th.Out;
            bool calage = o.Kind == MainThrottle.KindCalage;
            return new List<double> (ThrottleReplyLength) {
                accepted, MainThrottle.Protocol, MainThrottle.Schema, o.Kind, o.Sequence, th.DecideKind,
                th.DecideSequence, th.State, th.Motif,
                th.Decided, th.Written, th.ClientSeen, th.DecideTick, th.DecideUt, th.FbwTick,
                o.GuardTripped ? 1.0 : 0.0, o.EpisodeOn ? 1.0 : 0.0, o.EpisodeOn ? o.EpisodeMin : double.NaN,
                o.EpisodeSeen ? 1.0 : 0.0, o.Fired ? 1.0 : 0.0, th.Writes, th.Held,
                c.Unrealized, c.Required, c.Cap, th.FractionLost, c.MPar, c.RcsPar, th.Gx, th.Gz, th.Mx, th.Mz,
                th.In.FullThrustN, th.In.ThrustNowN, th.In.Calage.MassKg, th.DotTarget,
                calage && o.Count > 9 ? o.P [9] : double.NaN, th.Missed, ownTick
            };
        }

        // ---------------------------------------------------------------- the step

        /// <summary>
        /// Option 512: this step's decision (Earlyish, after the law), handed to the fly-by-wire callback. Returns
        /// CapMainThrottle when a value is to be written. Nothing without the option.
        /// </summary>
        static long ThrottleStep (Entry e, Vessel vessel, Transform reference, Vector3 com, double ut, double dt,
            int stage, bool warp, int nGimbalColumns)
        {
            var th = e.Th;
            th.Write = false;
            var o = th.Order;
            if (o.Kind < 0)
                return 0L;
            var inp = e.Input;
            var law = e.Output;
            var i = th.In;
            i.Ut = ut;
            i.Dt = dt;
            i.CanWrite = e.Mode == ModeArmed && ReferenceEquals (e.HookedVessel, vessel) &&
                stage == StageEarlyish && !warp;
            ThrottleEngines (e, i);
            i.CrossingValue = double.NaN;
            i.Height = double.NaN;
            i.VerticalSpeed = double.NaN;
            var cal = i.Calage;
            cal.LawComputed = false;
            cal.MassKg = vessel.totalMass * 1000.0;
            if (o.Kind == MainThrottle.KindCrossing && th.Frame != null) {
                try {
                    Vector3d a = th.Frame.DirectionToWorldSpace (new Vector3d (o.P [2], o.P [3], o.P [4]));
                    Vector3d v = o.P [6] == MainThrottle.VelocityOrbital ? vessel.obt_velocity : vessel.srf_velocity;
                    i.CrossingValue = a.x * v.x + a.y * v.y + a.z * v.z + o.P [5];
                } catch {
                    i.CrossingValue = double.NaN;
                }
            } else if (o.Kind == MainThrottle.KindIgnite) {
                i.Height = vessel.altitude - o.P [2];
                i.VerticalSpeed = vessel.verticalSpeed;
            } else if (o.Kind == MainThrottle.KindCalage) {
                ThrottleCalageInputs (e, vessel, reference, com, inp, law, nGimbalColumns);
            }
            // (option 2048): the ring's throttle of this step, taken by a live ramp order (NaN: none).
            i.RingThrottle = RingThrottle (e);
            int state, motif;
            double u = MainThrottle.Decide (o, i, e.Gains, th.Out, out state, out motif);
            th.State = state;
            th.Motif = motif;
            th.DecideKind = o.Kind;
            th.DecideSequence = o.Sequence;
            th.Decided = u;
            th.DecideTick = ownTick;
            th.DecideUt = ut;
            if (state == MainThrottle.StateHeld)
                th.Held++;
            if (double.IsNaN (u))
                return 0L;
            th.Write = true;
            th.WriteValue = u;
            th.WriteTick = ownTick;
            return CapMainThrottle;
        }

        /// <summary>
        /// The main-throttle engines (neither an independent throttle nor a locked one): their realized thrust and
        /// their thrust at full main throttle (N), and the slowest spool among those with a response time.
        /// </summary>
        static void ThrottleEngines (Entry e, ThrottleStepInput i)
        {
            double now = 0.0, full = 0.0, guard = 0.0, accel = double.PositiveInfinity;
            bool finite = false;
            for (int k = 0; k < e.Engines.Count; k++) {
                var engine = e.Engines [k];
                if (engine == null || engine.independentThrottle || engine.throttleLocked)
                    continue;
                now += engine.finalThrust;
                guard += FullThrustKn (engine, true);
                double kn = FullThrustKn (engine);
                if (!(kn > 0.0))
                    continue;
                full += kn;
                if (engine.useEngineResponseTime && !engine.throttleUseAlternate) {
                    finite = true;
                    if (engine.engineAccelerationSpeed < accel)
                        accel = engine.engineAccelerationSpeed;
                }
            }
            i.ThrustNowN = now * 1000.0;
            i.FullThrustN = full * 1000.0;
            i.GuardFullThrustN = guard * 1000.0;
            i.FiniteResponse = finite;
            i.AccelerationSpeed = finite ? accel : double.NaN;
            i.Calage.ThrustN = i.FullThrustN;
        }

        /// <summary>
        /// An engine's thrust at full main throttle (kN): finalThrust per unit of currentThrottle while it thrusts,
        /// else MaxThrustOutputAtm at its part's pressure without the limiter (maxThrust if that fails), times
        /// thrustPercentage / 100; 0 for an engine not ignited, not operational, flamed out, or not on the main
        /// throttle.
        /// </summary>
        static double FullThrustKn (ModuleEngines engine, bool guard = false)
        {
            // an earlier change's guard (guard = true): every ignited engine counts, a flamed-out (no propellant) or inoperative one
            // included -- the thrust that does not come; the calage's authority counts the engines that can thrust.
            if (engine == null || engine.independentThrottle || engine.throttleLocked || !engine.EngineIgnited ||
                    (!guard && (!engine.isOperational || engine.getFlameoutState)))
                return 0.0;
            double k;
            if (engine.currentThrottle > GimbalSpool.ThrottleReadable && engine.finalThrust > 0f) {
                k = engine.finalThrust / engine.currentThrottle;
            } else {
                try {
                    var part = engine.part;
                    k = part == null ? engine.maxThrust : engine.MaxThrustOutputAtm (
                        true, false, (float)part.staticPressureAtm, 310.0, part.atmDensity);
                } catch {
                    k = engine.maxThrust;
                }
            }
            double limiter = Math.Min (100.0, Math.Max (0.0, (double)engine.thrustPercentage)) * 0.01;
            double kn = k * limiter;
            return kn > 0.0 && Finite (kn) ? kn : 0.0;
        }

        /// <summary>The calage's inputs of this step (see the class header).</summary>
        static void ThrottleCalageInputs (Entry e, Vessel vessel, Transform reference, Vector3 com,
            AttitudeInput inp, AttitudeOutput law, int nGimbalColumns)
        {
            var th = e.Th;
            var o = th.Order;
            var c = th.In.Calage;
            int n = inp.N;
            c.LawComputed = law.Computed;
            c.Ex = inp.ReferenceValid ? inp.Error [0] : double.NaN;
            c.Ez = inp.ReferenceValid ? inp.Error [2] : double.NaN;
            c.NuX = law.Computed ? law.Nu [0] : double.NaN;
            c.NuZ = law.Computed ? law.Nu [2] : double.NaN;
            c.Ix = inp.Inertia [0];
            c.Iz = inp.Inertia [2];
            c.Mx = law.Filtered ? inp.Inertia [0] * law.OmegaDotF [0] - law.TorqueF [0] : double.NaN;
            c.Mz = law.Filtered ? inp.Inertia [2] * law.OmegaDotF [2] - law.TorqueF [2] : double.NaN;
            th.Mx = c.Mx;
            th.Mz = c.Mz;
            // RCS + wheels: the order's, else the ctrlState columns the law has this step.
            double rx = o.P [7], rz = o.P [8];
            if (!(rx > 0.0) || !Finite (rx) || !(rz > 0.0) || !Finite (rz)) {
                rx = 0.0;
                rz = 0.0;
                for (int a = 0; a < 3; a++) {
                    int j = nGimbalColumns + a;
                    if (j >= n)
                        break;
                    rx += Math.Abs (inp.B [0 * n + j]);
                    rz += Math.Abs (inp.B [2 * n + j]);
                }
            }
            c.Rx = rx;
            c.Rz = rz;
            ThrottleGimbalAuthority (e, com, reference, inp, out c.Gx, out c.Gz);
            th.Gx = c.Gx;
            th.Gz = c.Gz;
            c.Slope = o.P [6];
            c.DotTarget = double.NaN;
            c.DotAxis = double.NaN;
            c.FractionLost = double.NaN;
            try {
                Vector3 b = Body (reference, th.Frame.DirectionToWorldSpace (new Vector3d (o.P [0], o.P [1], o.P [2])));
                Vector3 h = Body (reference, th.Frame.DirectionToWorldSpace (new Vector3d (o.P [3], o.P [4], o.P [5])));
                Vector3d upWorld = vessel.upAxis;
                Vector3 up = reference.InverseTransformDirection (
                    new Vector3 ((float)upWorld.x, (float)upWorld.y, (float)upWorld.z));
                double bn = Math.Sqrt ((double)b.x * b.x + (double)b.y * b.y + (double)b.z * b.z);
                double hn = Math.Sqrt ((double)h.x * h.x + (double)h.y * h.y + (double)h.z * h.z);
                c.DotTarget = bn > 1e-12 ? b.y / bn : double.NaN;
                c.DotAxis = hn > 1e-12 ? h.y / hn : double.NaN;
                c.FractionLost = MainThrottle.FractionLost (0.0, 1.0, 0.0, up.x, up.y, up.z, b.x, b.y, b.z);
            } catch {
                // A frame that no longer converts: no hemisphere, no cost (the decision pushes nothing).
            }
            th.DotTarget = c.DotTarget;
            th.FractionLost = c.FractionLost;
        }

        static Vector3 Body (Transform reference, Vector3d world)
        {
            return reference.InverseTransformDirection (new Vector3 ((float)world.x, (float)world.y, (float)world.z));
        }

        /// <summary>
        /// The gimbals' authority at full main throttle on body x and z (N.m): every gimbal module's columns at the
        /// full thrust of its part's engines (FullThrustKn; an engine off the main throttle keeps its finalThrust),
        /// at the deflection of this step, times the law's span, summed in absolute value (the law's alpha rule); a
        /// module mapped per pivot through a scratch mapping (its free columns), the step's own mapping untouched.
        /// </summary>
        static void ThrottleGimbalAuthority (Entry e, Vector3 com, Transform reference, AttitudeInput inp,
            out double gx, out double gz)
        {
            gx = 0.0;
            gz = 0.0;
            for (int g = 0; g < e.Gimbals.Count; g++) {
                var gr = e.Gimbals [g];
                var gimbal = gr.Gimbal;
                var engines = gr.PartEngines;
                if (gimbal == null || engines == null)
                    continue;
                int count = engines.Count;
                if (gr.FullThrust == null || gr.FullThrust.Length != count)
                    gr.FullThrust = new double[count];
                for (int k = 0; k < count; k++) {
                    var engine = engines [k];
                    if (engine == null)
                        gr.FullThrust [k] = 0.0;
                    else if (engine.independentThrottle || engine.throttleLocked)
                        gr.FullThrust [k] = engine.finalThrust;
                    else
                        gr.FullThrust [k] = FullThrustKn (engine);
                }
                int jx = 2 * g, jy = 2 * g + 1;
                float dx = (float)inp.U0 [jx], dy = (float)inp.U0 [jy];
                Vector3 cx, cy;
                if (gr.PivotsMapped && AttitudeGeometry.RefreshPivots (gimbal, e.Th.Scratch)) {
                    Vector3 lx, ly;
                    AttitudeGeometry.GimbalColumnsPivots (gimbal, engines, com, reference, e.Th.Scratch, dx, dy,
                        gr.FullThrust, out lx, out ly, out cx, out cy);
                } else {
                    AttitudeGeometry.GimbalColumns (gimbal, engines, com, reference, dx, dy, gr.FullThrust,
                        out cx, out cy);
                }
                double sx = Math.Max (Math.Abs (inp.UMax [jx]), Math.Abs (inp.UMin [jx]));
                double sy = Math.Max (Math.Abs (inp.UMax [jy]), Math.Abs (inp.UMin [jy]));
                gx += Math.Abs ((double)cx.x) * sx + Math.Abs ((double)cy.x) * sy;
                gz += Math.Abs ((double)cx.z) * sx + Math.Abs ((double)cy.z) * sy;
            }
        }

        // ---------------------------------------------------------------- the fly-by-wire link

        /// <summary>
        /// Option 512, in our fly-by-wire link (after kRPC's control inputs and AutoPilot): what the client's
        /// throttle was this step, and the decision of this step's Earlyish written over it, once.
        /// </summary>
        static void WriteMainThrottle (Entry e, FlightCtrlState state)
        {
            var th = e.Th;
            if (th.Order.Kind < 0 && !th.Write)
                return;
            th.ClientSeen = state.mainThrottle;
            th.Written = double.NaN;
            if (th.Write && th.WriteTick == ownTick && e.Mode == ModeArmed && !e.Dropped &&
                    !double.IsNaN (th.WriteValue)) {
                float value = (float)th.WriteValue;
                state.mainThrottle = value;
                th.Written = value;
                th.FbwTick = ownTick;
                th.Writes++;
            }
            th.Write = false;
        }

        // ---------------------------------------------------------------- the throttle given back at expiry

        sealed class ThrottleReturn
        {
            internal Vessel Vessel;
            internal uint PersistentId;
            internal FlightInputCallback Callback;
            internal bool HaveClient;
            internal float Client;
            internal long Steps;
        }

        static readonly List<ThrottleReturn> throttleReturns = new List<ThrottleReturn> ();

        /// <summary>The loop wrote the main throttle at the last fly-by-wire (the arming's tick has moved on since).</summary>
        static bool ThrottleLive (Entry e)
        {
            var th = e.Th;
            return (e.Options & AttitudeWrite.OptionMainThrottle) != 0 && th.Order.Kind > MainThrottle.KindStop &&
                th.FbwTick >= 0 && ownTick - th.FbwTick <= 1;
        }

        /// <summary>
        /// At the lease's expiry of an arming whose loop wrote the main throttle at the last step: a lone link at the
        /// end of the vessel's fly-by-wire chain writes 0 until the throttle kRPC serves (the client's) changes, a
        /// new arming of the vessel or the scene's end. Never throws.
        /// </summary>
        static void InstallThrottleReturn (Entry e, Vessel vessel)
        {
            if (vessel == null)
                return;
            try {
                RemoveThrottleReturn (e.PersistentId);
                var r = new ThrottleReturn { Vessel = vessel, PersistentId = e.PersistentId };
                r.Callback = new FlightInputCallback (s => OnThrottleReturn (r, s));
                vessel.OnFlyByWire = (FlightInputCallback)Delegate.Combine (vessel.OnFlyByWire, r.Callback);
                throttleReturns.Add (r);
                BridgeLog.Info ("AttitudeV1: bail expire, manette principale rendue a zero (D91, D252 K5), vaisseau " +
                    e.PersistentId);
            } catch (Exception exc) {
                BridgeLog.Error ("AttitudeV1: rendu de la manette a l'expiration: " + exc.Message);
            }
        }

        static void OnThrottleReturn (ThrottleReturn r, FlightCtrlState state)
        {
            try {
                if (state == null)
                    return;
                if (!r.HaveClient) {
                    r.Client = state.mainThrottle;
                    r.HaveClient = true;
                } else if (state.mainThrottle != r.Client) {
                    // The client wrote another throttle: it lives, the throttle is its own again.
                    RemoveThrottleReturn (r.PersistentId);
                    return;
                }
                state.mainThrottle = 0f;
                r.Steps++;
            } catch {
                // Nothing may escape the fly-by-wire chain.
            }
        }

        static void RemoveThrottleReturn (uint persistentId)
        {
            for (int k = throttleReturns.Count - 1; k >= 0; k--) {
                var r = throttleReturns [k];
                if (r.PersistentId != persistentId)
                    continue;
                try {
                    if (r.Vessel != null)
                        r.Vessel.OnFlyByWire = (FlightInputCallback)Delegate.Remove (r.Vessel.OnFlyByWire, r.Callback);
                } catch (Exception exc) {
                    BridgeLog.Error ("AttitudeV1: retrait du rendu de la manette: " + exc.Message);
                }
                throttleReturns.RemoveAt (k);
            }
        }

        static void RemoveAllThrottleReturns ()
        {
            for (int k = throttleReturns.Count - 1; k >= 0; k--)
                RemoveThrottleReturn (throttleReturns [k].PersistentId);
        }
    }
}
