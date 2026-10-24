using System;

namespace KRPC.Bridge.Model
{
    // -----------------------------------------------------------------------------------
    // K5 (option 512 of AttitudeOptionsV1, born closed): the MAIN THROTTLE decided at the
    // physics step. Pure maths (System only): compiled as is by build/tests (MainThrottleTests.cs),
    // mirrored operation by operation by the autopilot (the client) and checked
    // against golden vectors (build/tests/golden/attitude_v1_golden_throttle.json).
    //
    // WHY. Every thrust of a flight was decided and written by the Python client: 0.26-0.70 s from
    // the order to the thrust, a cadence guard of 0.4-1.3 s at the descent ignition,
    // and the flip's calage throttle computed on a state 0.2-0.7 s old while
    // the aerodynamic torque swings the nose out of the burn plane at q ~10 kPa: the cross velocity of
    // the flip reached 14-40 m/s on 11 of 42 replayed separations of the super hope.
    //
    // WHAT. The client arms an ORDER (AttitudeThrottleV1, one per tick or once), valid until its game
    // time; at every Earlyish the loop decides the main throttle of the step and its fly-by-wire link,
    // last in the chain after the kRPC AutoPilot's and kRPC's own control inputs, writes it into
    // ctrlState.mainThrottle (ModuleEngines.UpdateThrottle reads it: requestedThrottle = mainThrottle x
    // thrustPercentage / 100, decompiled). The order kinds:
    //   0 STOP        nothing written from this step on (the client's throttle again);
    //   1 SCHEDULE    u(t) = clamp01(u0 + slope (t - t0)), t0 the order's utStamp (a dated ramp);
    //   2 PULSE       u for N physics steps, then the client's throttle;
    //   3 CROSSING    u_before while s(t) = a . V + c > 0, then u_after from the first step s(t) <= 0, latched
    //                 ("cut when s(t) = 0": the client signs s positive before the event; one that has already
    //                 happened when the loop first looks fires at once); V the vessel's surface or orbital velocity;
    //   4 IGNITE      u_before until h <= h_ign(v) = c0 + c1 v + c2 v^2, then u_after, latched ("ignite
    //                 when h <= h_ign(v)"); h the altitude above h_ref, v the vertical speed;
    //   5 CALAGE      the flip's calage throttle (an earlier change's branch `calage`, an earlier change's branch `saturation` on the
    //                 law's REAL pseudo-command, an earlier change's authority guard and cost cap on the loop's own nose,
    // an earlier change's hemisphere rule); see Calage.
    // Every order carries an earlier change's thrust guard (threshold, chain steps): an episode of thrust starts at the
    // first step written above 0; past the chain delay a realized thrust below threshold x the episode's
    // smallest throttle x the full thrust trips the guard, latched for the order (ThrustGuard).
    //
    // SAFETY (the first time the bridge decides a thrust):
    //   * born closed: nothing without option 512, and the order's own validity (game time) on top of the
    //     arming's real-time lease;
    //   * the client's throttle is the fallback, always: the loop writes ctrlState, which kRPC's control
    //     inputs (the client's last throttle) rewrite at every step before our link -- whenever the loop
    //     does not write (order ended, guard, a blocking condition), the client's throttle flies at once;
    //   * no jump on a missing decision (an earlier change's motif): a CALAGE step without the law's decision (tick gap,
    //     filter restart) holds the last written value for at most the chain delay, then writes 0;
    //   * the cost cap of an earlier change, computed at the step from the loop's own nose and mass;
    //   * at the expiry of the arming's lease (a frozen or dead client), the loop zeroes the throttle at the
    //     fly-by-wire until the client writes another value (kRPC keeps a disconnected client's throttle,
    //     an earlier change): AttitudeV1Throttle.cs.
    // -----------------------------------------------------------------------------------

    /// <summary>The flip's calage of one step: inputs (body frame x pitch, z yaw).</summary>
    internal sealed class CalageInput
    {
        internal double Ex, Ez;            // pointing error, rotation vector (rad)
        internal double Mx, Mz;            // external torque estimate I w_dot_f - M_f (N.m)
        internal double Rx, Rz;            // RCS + wheels authority (N.m)
        internal double Gx, Gz;            // gimbals' authority at full main throttle (N.m)
        internal bool LawComputed;         // NuX, NuZ are this step's pseudo-command
        internal double NuX, NuZ;          // the law's pseudo-command (rad/s^2)
        internal double Ix, Iz;            // inertia (kg.m^2)
        internal double GuardRatio;        // AttitudeGains.GuardSaturationRatio
        internal double DotTarget;         // nose . unit(boostback direction)
        internal double DotAxis;           // nose . unit(horizontal burn axis)
        internal double FractionLost;      // MainThrottle.FractionLost (NaN unreadable)
        internal double Slope;             // an earlier change's ballistic slope of the boostback (m/s^2, from the client)
        internal double MassKg;
        internal double ThrustN;           // full main-throttle thrust (N)
    }

    /// <summary>The flip's calage of one step: the decision and what it was made of.</summary>
    internal sealed class CalageOutput
    {
        internal double Throttle;          // 0..1; NaN: no decision (motif SansMesure, SansCardan)
        internal int Motif;
        internal double Unrealized;        // real saturation under RCS + wheels (NaN without the law)
        internal bool Saturated;
        internal double MPar, MPerp, RcsPar;
        internal double Required;          // an earlier change's required throttle (NaN unreadable)
        internal double Cap;               // an earlier change's cost cap (NaN unreadable, +inf nothing lost)

        internal void Clear ()
        {
            Throttle = double.NaN;
            Motif = MainThrottle.MotifNone;
            Unrealized = double.NaN;
            Saturated = false;
            MPar = MPerp = RcsPar = double.NaN;
            Required = Cap = double.NaN;
        }
    }

    /// <summary>One order and what it remembers from one step to the next (per armed vessel).</summary>
    internal sealed class ThrottleOrder
    {
        internal int Kind = -1;            // -1: no order since the arming
        internal int Sequence;
        internal double UtStamp, UtValidUntil;
        internal readonly double[] P = new double[MainThrottle.MaxParameters];
        internal int Count;
        // Runtime, kept across renewals of the same kind.
        internal int Steps;                // steps decided (pulse)
        internal double Last = double.NaN; // last value decided
        internal int HeldSteps;
        internal bool Fired;               // crossing / ignite fired (latched)
        internal bool GuardTripped;        // guard (latched for the order)
        internal bool EpisodeOn;
        internal double EpisodeUt, EpisodeMin;
        internal bool EpisodeSeen;

        internal void ResetRuntime ()
        {
            Steps = 0;
            Last = double.NaN;
            HeldSteps = 0;
            Fired = false;
            GuardTripped = false;
            EpisodeOn = false;
            EpisodeUt = double.NaN;
            EpisodeMin = double.NaN;
            EpisodeSeen = false;
        }

        internal void Clear ()
        {
            Kind = -1;
            Sequence = 0;
            Count = 0;
            UtStamp = UtValidUntil = double.NaN;
            ResetRuntime ();
        }
    }

    /// <summary>The measured inputs of one step's decision (filled by the Control side).</summary>
    internal sealed class ThrottleStepInput
    {
        internal double Ut;
        internal double Dt;
        internal bool CanWrite;            // armed, hooked, Earlyish, no warp, loaded
        internal double CrossingValue;     // s(t) of a crossing order (NaN unreadable)
        internal double Height;            // altitude - h_ref of an ignite order (NaN unreadable)
        internal double VerticalSpeed;     // m/s, positive up
        internal double ThrustNowN;        // realized thrust of the main-throttle engines (N)
        internal double FullThrustN;       // their thrust at full main throttle (N)
        // an earlier change's guard: the thrust at full main throttle of every IGNITED main-throttle engine, a flamed-out or
        // inoperative one included (what the client's guard reads, kRPC's max_thrust over the active engines: an engine
        // starved of propellant is the thrust that does not come). NaN: FullThrustN.
        internal double GuardFullThrustN = double.NaN;
        internal bool FiniteResponse;      // one of them spools (useEngineResponseTime)
        internal double AccelerationSpeed; // the slowest such spool (1/s)
        internal readonly CalageInput Calage = new CalageInput ();
        // (option 2048): the reference ring's throttle at this step (NaN: none); a live SCHEDULE order takes
        // it instead of u0 + slope (t - t0), motif 18.
        internal double RingThrottle = double.NaN;
    }

    internal static class MainThrottle
    {
        internal const int Protocol = 1;
        internal const int Schema = 1;
        internal const int MaxParameters = 12;

        internal const int KindStop = 0;
        internal const int KindSchedule = 1;
        internal const int KindPulse = 2;
        internal const int KindCrossing = 3;
        internal const int KindIgnite = 4;
        internal const int KindCalage = 5;
        internal const int KindCount = 6;

        /// <summary>Parameters of each kind; the last two of every non-stop kind are an earlier change's guard.</summary>
        internal static readonly int[] ParameterCounts = { 0, 4, 4, 9, 8, 12 };

        // Crossing velocity kinds.
        internal const int VelocitySurface = 0;
        internal const int VelocityOrbital = 1;

        // States of a step (published).
        internal const int StateIdle = 0;      // no order (or option off)
        internal const int StateWriting = 1;   // written at this step's fly-by-wire
        internal const int StateHeld = 2;      // the last value held (no decision this step)
        internal const int StateGuard = 3;     // guard tripped: the client's throttle
        internal const int StateEnded = 4;     // validity passed, pulse done, stop: the client's throttle
        internal const int StateBlocked = 5;   // a valid order the loop may not write this step

        // Motifs (calage: an earlier change's names; triggers and schedule).
        internal const int MotifNone = 0;
        internal const int MotifNul = 1;
        internal const int MotifAide = 2;
        internal const int MotifSousRcs = 3;
        internal const int MotifCalage = 4;
        internal const int MotifSaturation = 5;
        internal const int MotifSansCardan = 6;
        internal const int MotifHemisphere = 7;
        internal const int MotifHorsAutorite = 8;
        internal const int MotifCout = 9;
        internal const int MotifSansCout = 10;
        internal const int MotifSansMesure = 11;
        internal const int MotifSansLoi = 12;
        internal const int MotifGardePoussee = 13;
        internal const int MotifSchedule = 14;
        internal const int MotifPulse = 15;
        internal const int MotifAvant = 16;
        internal const int MotifDeclenche = 17;
        internal const int MotifRing = 18;

        // an earlier change's guard.
        internal const int GuardWait = 0;
        internal const int GuardLit = 1;
        internal const int GuardAbsent = 2;

        internal static bool Finite (double v)
        {
            return !double.IsNaN (v) && !double.IsInfinity (v);
        }

        internal static double Clamp01 (double u)
        {
            if (double.IsNaN (u))
                return double.NaN;
            return u > 1.0 ? 1.0 : (u < 0.0 ? 0.0 : u);
        }

        /// <summary>True when the parameters fit the kind (count, and the values the kind needs finite).</summary>
        internal static bool ValidParameters (int kind, double[] p, int count)
        {
            if (kind < 0 || kind >= KindCount || count != ParameterCounts [kind] || (count > 0 && p == null))
                return false;
            switch (kind) {
            case KindSchedule:
                return Finite (p [0]) && Finite (p [1]);
            case KindPulse:
                return Finite (p [0]) && Finite (p [1]) && p [1] >= 1.0;
            case KindCrossing:
                return Finite (p [0]) && Finite (p [1]) && Finite (p [2]) && Finite (p [3]) && Finite (p [4]) &&
                    Finite (p [5]) && (p [6] == VelocitySurface || p [6] == VelocityOrbital);
            case KindIgnite:
                return Finite (p [0]) && Finite (p [1]) && Finite (p [2]) && Finite (p [3]) && Finite (p [4]) &&
                    Finite (p [5]);
            case KindCalage:
                // Directions finite (degenerate ones fail the hemisphere rule); the slope and the RCS authority may
                // be NaN (no cost cap: no push; the loop's own ctrlState columns); the verdict -1, 0 or 1.
                return Finite (p [0]) && Finite (p [1]) && Finite (p [2]) && Finite (p [3]) && Finite (p [4]) &&
                    Finite (p [5]) && (p [9] == -1.0 || p [9] == 0.0 || p [9] == 1.0);
            }
            return true;
        }

        /// <summary>an earlier change's guard parameters of an order: threshold (fraction), chain steps; NaN when it has none.</summary>
        internal static double GuardThreshold (ThrottleOrder o)
        {
            return o.Kind > KindStop && o.Count >= 2 ? o.P [o.Count - 2] : double.NaN;
        }

        internal static double GuardChainSteps (ThrottleOrder o)
        {
            return o.Kind > KindStop && o.Count >= 2 ? o.P [o.Count - 1] : double.NaN;
        }

        /// <summary>The dated ramp: clamp01(u0 + slope (t - t0)); NaN on a non-finite input.</summary>
        internal static double Schedule (double u0, double slope, double t0, double t)
        {
            double u = u0 + slope * (t - t0);
            if (!Finite (u))
                return double.NaN;
            return Clamp01 (u);
        }

        /// <summary>h - h_ign(v), h_ign(v) = c0 + c1 v + c2 v^2: the ignition fires at or below 0.</summary>
        internal static double IgniteMargin (double h, double v, double c0, double c1, double c2)
        {
            return h - (c0 + c1 * v + c2 * v * v);
        }

        /// <summary>
        /// (the client): the throttle whose gimbals cover the whole aerodynamic torque of
        /// the pitch/yaw plane, max(|Mx| / Gx, |Mz| / Gz), unbounded; NaN when an axis of the gimbals has none.
        /// </summary>
        internal static double RequiredThrottle (double mx, double mz, double gx, double gz)
        {
            if (!(gx > 0.0) || !(gz > 0.0))
                return double.NaN;
            double a = Math.Abs (mx) / gx, b = Math.Abs (mz) / gz;
            return a > b ? a : b;
        }

        /// <summary>
        /// (the client): the share of the calage thrust's acceleration the boostback
        /// does not take back, (1 - max(0, n.b)) + max(0, up.(n - (n.b) b)), every vector made unit; NaN when one is
        /// degenerate or not finite.
        /// </summary>
        internal static double FractionLost (double nx, double ny, double nz, double ux, double uy, double uz,
            double bx, double by, double bz)
        {
            double nn = Math.Sqrt (nx * nx + ny * ny + nz * nz);
            double un = Math.Sqrt (ux * ux + uy * uy + uz * uz);
            double bn = Math.Sqrt (bx * bx + by * by + bz * bz);
            if (!(nn > 1e-12) || !(un > 1e-12) || !(bn > 1e-12) || !Finite (nn) || !Finite (un) || !Finite (bn))
                return double.NaN;
            nx /= nn;
            ny /= nn;
            nz /= nn;
            ux /= un;
            uy /= un;
            uz /= un;
            bx /= bn;
            by /= bn;
            bz /= bn;
            double nb = nx * bx + ny * by + nz * bz;
            double hx = nx - nb * bx, hy = ny - nb * by, hz = nz - nb * bz;
            double up = hx * ux + hy * uy + hz * uz;
            return (1.0 - (nb > 0.0 ? nb : 0.0)) + (up > 0.0 ? up : 0.0);
        }

        /// <summary>
        /// (the client): the largest calage throttle whose cost does not exceed its gain,
        /// slope m / (T w); +infinity when nothing is lost (w &lt;= 0); NaN when an input is unreadable or the mass or
        /// the thrust is not positive.
        /// </summary>
        internal static double CostCap (double slope, double massKg, double thrustN, double fractionLost)
        {
            if (!(massKg > 0.0) || !(thrustN > 0.0) || double.IsNaN (slope) || double.IsNaN (fractionLost) ||
                    slope < 0.0 || double.IsInfinity (massKg) || double.IsInfinity (thrustN))
                return double.NaN;
            if (!(fractionLost > 0.0))
                return double.PositiveInfinity;
            return slope * massKg / (thrustN * fractionLost);
        }

        /// <summary>
        /// The flip's calage of one step: the client with par_axe and cout, its order of
        /// tests kept, on the loop's own quantities. Saturation is the law's REAL pseudo-command: on each axis the
        /// torque it asks of the actuators without the engines, I nu - M, against the RCS + wheels authority R,
        /// (|I nu - M| - R) / R, the largest of the two axes beyond the guard ratio of the law (an earlier change's branch on the
        /// prediction of an earlier change, here what the law computed this step). Motifs: SansMesure (RCS authority or
        /// torque unreadable: no decision), Hemisphere (the nose not on the boostback's side or its horizontal part
        /// against the burn axis: 0), then an earlier change's _calage_detail (Nul, Aide, SousRcs: 0; Calage, Saturation: the
        /// smallest throttle whose gimbals cover the aerodynamic torque, clamped to 1; SansCardan: no decision), then
        /// an earlier change's guard of a push (HorsAutorite: the required throttle above 1; Cout: above the cost cap; SansCout:
        /// either unreadable; all 0).
        /// </summary>
        internal static void Calage (CalageInput i, CalageOutput o)
        {
            o.Clear ();
            if (!(i.Rx > 0.0) || !(i.Rz > 0.0) || !Finite (i.Rx) || !Finite (i.Rz) || !Finite (i.Mx) ||
                    !Finite (i.Mz) || !Finite (i.Ex) || !Finite (i.Ez)) {
                o.Motif = MotifSansMesure;
                return;
            }
            if (!(i.DotTarget > 0.0) || !(i.DotAxis > 0.0)) {
                o.Throttle = 0.0;
                o.Motif = MotifHemisphere;
                return;
            }
            bool sature = false;
            if (i.LawComputed && Finite (i.NuX) && Finite (i.NuZ) && Finite (i.Ix) && Finite (i.Iz)) {
                double dx = i.Ix * i.NuX - i.Mx;
                double dz = i.Iz * i.NuZ - i.Mz;
                double irx = (Math.Abs (dx) - i.Rx) / i.Rx;
                double irz = (Math.Abs (dz) - i.Rz) / i.Rz;
                double irr = irx > irz ? irx : irz;
                o.Unrealized = irr;
                sature = irr > i.GuardRatio;
                o.Saturated = sature;
            }
            // the client
            double ne = Math.Sqrt (i.Ex * i.Ex + i.Ez * i.Ez);
            if (!(ne > 1e-9)) {
                o.Throttle = 0.0;
                o.Motif = MotifNul;
                return;
            }
            double ehx = i.Ex / ne, ehz = i.Ez / ne;
            double epx = -ehz, epz = ehx;
            double mPar = i.Mx * ehx + i.Mz * ehz;
            double mPerp = i.Mx * epx + i.Mz * epz;
            double ax = Math.Abs (ehx) > 1e-12 ? i.Rx / Math.Abs (ehx) : double.PositiveInfinity;
            double az = Math.Abs (ehz) > 1e-12 ? i.Rz / Math.Abs (ehz) : double.PositiveInfinity;
            double rcsPar = ax < az ? ax : az;
            o.MPar = mPar;
            o.MPerp = mPerp;
            o.RcsPar = rcsPar;
            int motif = MotifCalage;
            if (!(-mPar > rcsPar)) {
                if (!sature) {
                    o.Throttle = 0.0;
                    o.Motif = !(mPar < 0.0) ? MotifAide : MotifSousRcs;
                    return;
                }
                motif = MotifSaturation;
            }
            if (!(i.Gx > 0.0) || !(i.Gz > 0.0)) {
                o.Motif = MotifSansCardan;
                return;
            }
            double tx = -mPar * ehx - mPerp * epx;
            double tz = -mPar * ehz - mPerp * epz;
            double qx = Math.Abs (tx) / i.Gx, qz = Math.Abs (tz) / i.Gz;
            double q = qx > qz ? qx : qz;
            double u = q < 1.0 ? q : 1.0;
            u = u > 0.0 ? u : 0.0;
            o.Motif = motif;
            o.Throttle = u;
            if (!(u > 0.0))
                return;
            // the client
            double req = RequiredThrottle (i.Mx, i.Mz, i.Gx, i.Gz);
            o.Required = req;
            if (double.IsNaN (req)) {
                o.Throttle = 0.0;
                o.Motif = MotifSansCout;
                return;
            }
            if (req > 1.0) {
                o.Throttle = 0.0;
                o.Motif = MotifHorsAutorite;
                return;
            }
            double cap = CostCap (i.Slope, i.MassKg, i.ThrustN, i.FractionLost);
            o.Cap = cap;
            if (double.IsNaN (cap)) {
                o.Throttle = 0.0;
                o.Motif = MotifSansCout;
                return;
            }
            if (req > cap) {
                o.Throttle = 0.0;
                o.Motif = MotifCout;
            }
        }

        /// <summary>
        /// an earlier change's guard (the client) with the order's threshold and chain: Wait while nothing is
        /// demanded or the delay -- (chain + 1) steps, plus -ln(1 - threshold) / accelerationSpeed for a spooling
        /// engine -- has not passed since the write; Lit once the realized thrust reaches threshold x demanded;
        /// Absent past the delay below it. delay: NaN when there is none.
        /// </summary>
        internal static int ThrustGuard (double ut, double utWritten, double thrustN, double demandedN, double stepS,
            double chainSteps, double threshold, bool finiteResponse, double accelerationSpeed, out double delay)
        {
            delay = double.NaN;
            if (double.IsNaN (utWritten) || !(demandedN > 0.0) || !(stepS > 0.0) || !(chainSteps >= 0.0) ||
                    !(threshold > 0.0) || !(threshold < 1.0))
                return GuardWait;
            double d = (chainSteps + 1.0) * stepS;
            if (finiteResponse) {
                if (!(accelerationSpeed > 0.0))
                    return GuardWait;
                d += -Math.Log (1.0 - threshold) / accelerationSpeed;
            }
            delay = d;
            if (thrustN >= threshold * demandedN)
                return GuardLit;
            if (double.IsNaN (ut) || ut - utWritten <= d + 1e-9)
                return GuardWait;
            return GuardAbsent;
        }

        /// <summary>
        /// One step of an order: the value to write at this step's fly-by-wire (NaN: nothing, the client's throttle)
        /// and the step's state and motif (into the order). The guard of the episode runs first, on the thrust the
        /// previous writes made; a valid order the loop may not write is Blocked (its runtime frozen); a CALAGE step
        /// without a decision holds the last value for at most chain + 1 steps, then 0.
        /// </summary>
        internal static double Decide (ThrottleOrder o, ThrottleStepInput i, AttitudeGains g, CalageOutput c,
            out int state, out int motif)
        {
            c.Clear ();
            state = StateIdle;
            motif = MotifNone;
            if (o.Kind < 0)
                return double.NaN;
            if (o.Kind == KindStop || !(i.Ut <= o.UtValidUntil)) {
                state = StateEnded;
                return Release (o);
            }
            if (!i.CanWrite) {
                state = StateBlocked;
                return Release (o);
            }
            double threshold = GuardThreshold (o), chain = GuardChainSteps (o);
            if (o.EpisodeOn && !o.GuardTripped) {
                double delay;
                double guardFull = Finite (i.GuardFullThrustN) ? i.GuardFullThrustN : i.FullThrustN;
                int gs = ThrustGuard (i.Ut, o.EpisodeUt, i.ThrustNowN, o.EpisodeMin * guardFull, i.Dt, chain,
                    threshold, i.FiniteResponse, i.AccelerationSpeed, out delay);
                if (gs == GuardLit)
                    o.EpisodeSeen = true;
                else if (gs == GuardAbsent)
                    o.GuardTripped = true;
            }
            if (o.GuardTripped) {
                state = StateGuard;
                motif = MotifGardePoussee;
                return Release (o);
            }
            double u;
            state = StateWriting;
            switch (o.Kind) {
            case KindSchedule:
                if (Finite (i.RingThrottle)) {
                    // the ring's throttle of the step, under the order's validity and guard.
                    u = Clamp01 (i.RingThrottle);
                    motif = MotifRing;
                } else {
                    u = Schedule (o.P [0], o.P [1], o.UtStamp, i.Ut);
                    motif = MotifSchedule;
                }
                break;
            case KindPulse:
                if (o.Steps >= (int)Math.Floor (o.P [1])) {
                    state = StateEnded;
                    return Release (o);
                }
                o.Steps++;
                u = Clamp01 (o.P [0]);
                motif = MotifPulse;
                break;
            case KindCrossing:
                if (!o.Fired && Finite (i.CrossingValue) && !(i.CrossingValue > 0.0))
                    o.Fired = true;
                u = Clamp01 (o.Fired ? o.P [1] : o.P [0]);
                motif = o.Fired ? MotifDeclenche : MotifAvant;
                break;
            case KindIgnite:
                if (!o.Fired && Finite (i.Height) && Finite (i.VerticalSpeed) &&
                        !(IgniteMargin (i.Height, i.VerticalSpeed, o.P [3], o.P [4], o.P [5]) > 0.0))
                    o.Fired = true;
                u = Clamp01 (o.Fired ? o.P [1] : o.P [0]);
                motif = o.Fired ? MotifDeclenche : MotifAvant;
                break;
            case KindCalage:
                if (!i.Calage.LawComputed) {
                    double hold = Finite (chain) && chain >= 0.0 ? chain + 1.0 : 0.0;
                    if (Finite (o.Last) && o.HeldSteps < hold) {
                        o.HeldSteps++;
                        state = StateHeld;
                        motif = MotifSansLoi;
                        u = o.Last;
                    } else {
                        motif = MotifSansLoi;
                        u = 0.0;
                    }
                    break;
                }
                o.HeldSteps = 0;
                i.Calage.GuardRatio = g.GuardSaturationRatio;
                Calage (i.Calage, c);
                motif = c.Motif;
                u = double.IsNaN (c.Throttle) ? 0.0 : c.Throttle;
                break;
            default:
                state = StateEnded;
                return Release (o);
            }
            if (double.IsNaN (u)) {
                state = StateEnded;
                return Release (o);
            }
            if (u > 0.0) {
                if (!o.EpisodeOn) {
                    o.EpisodeOn = true;
                    o.EpisodeUt = i.Ut;
                    o.EpisodeMin = u;
                    o.EpisodeSeen = false;
                } else if (u < o.EpisodeMin) {
                    o.EpisodeMin = u;
                }
            } else {
                o.EpisodeOn = false;
            }
            o.Last = u;
            return u;
        }

        /// <summary>
        /// A step the loop does not write: the client's throttle flies, so the episode of thrust ends and no value
        /// is left to hold. Returns NaN.
        /// </summary>
        static double Release (ThrottleOrder o)
        {
            o.EpisodeOn = false;
            o.Last = double.NaN;
            o.HeldSteps = 0;
            return double.NaN;
        }
    }
}
