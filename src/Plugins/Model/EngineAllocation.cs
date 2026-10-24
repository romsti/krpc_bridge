using System;

namespace KRPC.Bridge.Model
{
    // -----------------------------------------------------------------------------------
    // step 1: the ENGINE-BY-ENGINE allocation of AttitudeV1 (option 4096 of AttitudeOptionsV1,
    // born closed), IN SHADOW: it computes and publishes what it would command, never writes. PURE:
    // System only, compiled by the plugin, build/verify and build/tests (EngineAllocationTests.cs).
    //
    // WHY. The law's allocation (AttitudeMath) solves torques only; the lateral force is a layer in
    // front of it (AttitudeTranslation: a common tilt, two differential motifs, the RCS translation),
    // pre-allocated, its torque cancelled by a feed-forward, the law then allocating the attitude part
    // around shifted bounds. The two fight: when the gimbals serve the attitude the share of the
    // common they hold collapses (24-33 % of the heavy booster's last six seconds), and an earlier change
    // works around it. Here every actuator is one column of ONE problem with every row of the wrench.
    //
    // COLUMNS (one variable each): the throttle command of every engine the differential may drive
    // (the motif engines' rule), the two axes of every usable gimbal module, the ctrlState pitch, roll
    // and yaw (RCS + wheels, the law's columns) and the RCS translation ctrlState.X and .Y.
    // ROWS (body frame, x right = pitch axis, y nose = roll axis, z bottom = yaw axis): the force
    // Fx, Fy (axial), Fz and the torque Mx (pitch), My (roll), Mz (yaw).
    //
    // THE VARIABLE is the REALIZED change of each actuator at a common horizon H (physics steps), not
    // the command. KSP moves every actuator by a Lerp toward its command (ModuleGimbal.FixedUpdate and
    // the bridge's lease ramp: factor f = gimbalResponseSpeed dt; ModuleEngines.UpdateThrottle:
    // engineAccelerationSpeed dt up, engineDecelerationSpeed dt down, none without the response time
    // or with the alternate law). Held for H steps a command c moves the realized u0 by
    //     x = phi (c - u0),   phi = 1 - (1 - f)^H,
    // phi of the side the command lies on. So the box of x is exact, [g(lo), g(hi)] with g(c) = phi(c)
    // (c - u0) (monotone), every row is LINEAR in x, and the wrench rows hold on the REALIZED state at
    // the horizon: a common tilt and the differential that cancels its torque are balanced where they
    // ARE, not where they were commanded (the feed-forward on the realized common of the translation
    // layer, built in). The command follows exactly: c = u0 + x / phi(sign x). This is the "gimbal
    // speed" constraint: an actuator realizes at most phi of its way to a bound by the horizon.
    // H is the largest time constant -1 / ln(1 - f) of the actuators present (GimbalSpool.HorizonSteps),
    // so the slowest realizes 1 - 1/e of its command and the others nearly all of theirs; the RCS and
    // the wheels are taken at phi = 1 (the RCS fires at the next step).
    //
    // PRIORITIES, strict (lexicographic), as the maintainer set them: (1) the axial force D asks (every engine
    // at the main throttle D serves, realized at the horizon), (2) pitch and yaw, (3) the lateral force
    // of the translation request (only while one is live), (4) roll. A CASCADE: stage k solves its rows
    // with every earlier level held at what that level achieved as a NEAR-EQUALITY (q = JointQ below: a residual of an
    // earlier level costs 1e9 times one of this level's, in shares of their authorities: never traded in practice), the
    // later levels out. Each stage is a box-constrained regularized least squares:
    //     min sum_{r in level k} (A_r y - b_r)^2 + sum_j R_j^2 (y_j - p_j)^2
    //     s.t. A_r y = v_r (r in levels < k),  lo <= y <= hi,
    // solved by a primal active set whose free subproblem goes through the 6 x 6 matrix
    //     S = diag(q) + A_F D_F^-2 A_F^T  (q_r = 1 weighted, JointQ held; Woodbury: y_F = p_F + D^-2 A_F^T z,
    //     S z = b - A_C y_C - A_F p_F),
    // so a step costs O(6^2 n), not O(n^3): the heavy booster's dozens of columns at 50 Hz.
    // Normalization: y_j = x_j / s_j (s_j the half width of its box); each row divided by its own
    // authority at full span (sum_j |A_rj| (hi - lo) / 2), so pitch and yaw (and Fx and Fz) weigh in
    // shares of what each can reach. Regularization (only picks among exact solutions, as in
    // JointTranslation): R_j = SqrtGamma x priority_j x the column's own normalized effect, priorities
    // the law's (gimbals 1, ctrlState 3) and the translation's differential priority for the
    // throttles; preferred point: the throttles at D's main throttle, the gimbals at neutral, the
    // ctrlState at 0.
    //
    // AUTHORITY, for the solver (published in function of the thrust): the largest lateral
    // acceleration reachable along a body direction (+x, -x, +z, -z) with D's axial force and the
    // pitch/yaw torque held (stages 1 and 2 as equalities), the roll free, at steady state (phi = 1):
    // "free" from the realized state with the law's torque target (what the attitude leaves, an earlier change's
    // notion), "model" from the trim (throttles at a main throttle level, gimbals at neutral, no torque
    // wanted) at the current main throttle and at levels k / LevelGrid of it, the gimbal columns scaled
    // by the thrust of that level.
    // -----------------------------------------------------------------------------------

    internal sealed class EngineAllocation
    {
        internal const int Rows = 6;
        internal const int RowFx = 0, RowFy = 1, RowFz = 2, RowMx = 3, RowMy = 4, RowMz = 5;

        // Levels of the cascade, solved in this order.
        internal const int LevelAxial = 0;
        internal const int LevelPitchYaw = 1;
        internal const int LevelLateral = 2;
        internal const int LevelRoll = 3;
        internal const int Levels = 4;
        internal static readonly int[] RowLevel = {
            LevelLateral, LevelAxial, LevelLateral, LevelPitchYaw, LevelRoll, LevelPitchYaw
        };

        // Kinds of variable.
        internal const int KindThrottle = 1;
        internal const int KindGimbal = 2;
        internal const int KindRotation = 3;   // ctrlState pitch / roll / yaw (RCS + wheels)
        internal const int KindLinear = 4;     // ctrlState X / Y (RCS translation)

        /// <summary>Regularization, the joint translation's (the target met to ~1e-6 when reachable).</summary>
        internal const double SqrtGamma = JointTranslation.SqrtGamma;
        /// <summary>A row (or column) whose authority is below this share of the largest is absent.</summary>
        internal const double AbsentShare = JointTranslation.AbsentColumn;
        /// <summary>
        /// The near-equality: the joint solve's rows and the earlier levels of a cascade stage. Not 0 (an exact equality):
        /// S = q + A D^-2 A^T then stays positive whatever the free set. With exact equalities, a level saturated by its
        /// bounds left a row without a free column: the jittered 6 x 6 gave multipliers of rounding and the active set
        /// cycled (the heavy booster under a torque 40 times its authority: 52 of 1000 steps out of iterations).
        /// </summary>
        internal const double JointQ = 1e-9;
        /// <summary>The joint solve is accepted when every row is met to this share of its authority (JointTranslation's exactness).</summary>
        internal const double ExactShare = JointTranslation.NullAuthority;
        /// <summary>A level is "limited" when its residual exceeds this share of its row's authority.</summary>
        internal const double TargetTolerance = JointTranslation.TargetTolerance;
        /// <summary>Authority: the lateral target along a direction is this many times the row authority.</summary>
        internal const double AuthorityTargetFactor = JointTranslation.AuthorityTargetFactor;

        // Reason bits.
        internal const long ReasonNoLawTarget = 1L << 0;      // the law did not compute: no torque target, no allocation
        internal const long ReasonNoActuator = 1L << 1;       // no variable with an effect
        internal const long ReasonTiltTooLarge = 1L << 2;     // a live lateral demand, nose too far from the vertical
        internal const long ReasonNotConverged = 1L << 3;     // a stage ran out of iterations or its 6 x 6 failed
        internal const long ReasonRcsModelApprox = 1L << 4;   // RCS precision mode or fullThrust: its columns approximate
        internal const long ReasonLimitedAxial = 1L << 5;
        internal const long ReasonLimitedPitchYaw = 1L << 6;
        internal const long ReasonLimitedLateral = 1L << 7;
        internal const long ReasonLimitedRoll = 1L << 8;
        internal const long ReasonRcsDeadZone = 1L << 9;      // an RCS channel's command inside ModuleRCS's dead zone
        internal const long ReasonJitter = 1L << 10;          // an equality made the 6 x 6 singular: jitter added
        internal const long ReasonNoLateralDemand = 1L << 11; // no live translation request: the lateral rows out
        internal const long ReasonDeadZoneSnapped = 1L << 12; // step 2: RCS commands in the dead zone snapped and held

        // States.
        internal const int StateNotComputed = 0;
        internal const int StateSolved = 1;
        internal const int StateNoLaw = 2;
        internal const int StateNoActuator = 3;

        // ------------------------------------------------------------------ inputs
        internal int Count;
        /// <summary>Column of variable j at [6 j + r]: force (N) / torque (N.m) per unit of REALIZED change.</summary>
        internal double[] Col = new double[0];
        internal double[] U0 = new double[0];
        internal double[] Lo = new double[0];
        internal double[] Hi = new double[0];
        internal double[] Pref = new double[0];
        internal double[] Priority = new double[0];
        internal double[] PhiUp = new double[0];
        internal double[] PhiDown = new double[0];
        internal int[] Kind = new int[0];
        /// <summary>
        /// Step 2 (option 8192): a variable ModuleRCS zeroes inside its dead zone (an RCS-only ctrlState channel), for
        /// SolveDeadZone. False everywhere unless the caller sets it.
        /// </summary>
        internal bool[] DeadZoneEligible = new bool[0];
        /// <summary>The wanted REALIZED increment of each row at the horizon (N, N.m).</summary>
        internal readonly double[] Target = new double[Rows];
        /// <summary>A row in the problem (the lateral ones only with a live demand).</summary>
        internal readonly bool[] RowOn = new bool[Rows];

        // ----------------------------------------------------------------- outputs
        internal double[] X = new double[0];
        internal double[] Command = new double[0];
        internal readonly double[] Achieved = new double[Rows];
        internal readonly double[] RowAuthority = new double[Rows];
        internal int Iterations, Stages;
        internal bool Converged;
        /// <summary>The joint solve met every level at once (no cascade stage ran).</summary>
        internal bool JointTaken;
        internal long Reasons;
        /// <summary>SolveDeadZone: the commands snapped out of the RCS dead zone and held.</summary>
        internal int Snapped;

        // ------------------------------------------------------------------ work
        double[] a = new double[0];       // normalized columns [6 j + r]
        double[] span = new double[0];    // s_j
        double[] ylo = new double[0], yhi = new double[0], p = new double[0], r2 = new double[0];
        double[] y = new double[0];
        int[] w = new int[0], warm = new int[0], warmJoint = new int[0];
        double[] snapX = new double[0], snapU0 = new double[0], snapLo = new double[0], snapHi = new double[0];
        bool[] snapped = new bool[0];
        bool haveWarm, haveWarmJoint;
        readonly double[] b = new double[Rows], q = new double[Rows];
        readonly bool[] rowIn = new bool[Rows], present = new bool[Rows];
        readonly double[] scale = new double[Rows];
        readonly CascadeWork cw = new CascadeWork ();

        internal void Resize (int n)
        {
            if (Col.Length < 6 * n) {
                Col = new double[6 * n];
                a = new double[6 * n];
            }
            if (U0.Length < n) {
                U0 = new double[n];
                Lo = new double[n];
                Hi = new double[n];
                Pref = new double[n];
                Priority = new double[n];
                PhiUp = new double[n];
                PhiDown = new double[n];
                Kind = new int[n];
                DeadZoneEligible = new bool[n];
                snapX = new double[n];
                snapU0 = new double[n];
                snapLo = new double[n];
                snapHi = new double[n];
                snapped = new bool[n];
                X = new double[n];
                Command = new double[n];
                span = new double[n];
                ylo = new double[n];
                yhi = new double[n];
                p = new double[n];
                r2 = new double[n];
                y = new double[n];
                w = new int[n];
                warm = new int[n];
                warmJoint = new int[n];
                haveWarm = haveWarmJoint = false;
            }
            if (n != Count)
                haveWarm = haveWarmJoint = false;
            Count = n;
        }

        /// <summary>Forgets the warm start (a new arming, a topology change).</summary>
        internal void ForgetWarm ()
        {
            haveWarm = haveWarmJoint = false;
        }

        /// <summary>g(c) = phi(side) (c - u0): the realized change at the horizon of a command c held.</summary>
        internal static double Realized (double c, double u0, double phiUp, double phiDown)
        {
            double d = c - u0;
            return d >= 0.0 ? phiUp * d : phiDown * d;
        }

        /// <summary>The command c whose held realization at the horizon is x: u0 + x / phi(side); u0 when phi is 0.</summary>
        internal static double CommandOf (double x, double u0, double phiUp, double phiDown)
        {
            double phi = x >= 0.0 ? phiUp : phiDown;
            if (!(phi > 0.0))
                return u0;
            return u0 + x / phi;
        }

        /// <summary>phi = 1 - (1 - f)^H of a Lerp factor f per step at a horizon of H steps (1 for f >= 1, 0 for f &lt;= 0).</summary>
        internal static double Phi (double f, double horizon)
        {
            if (!(f > 0.0) || double.IsNaN (f))
                return 0.0;
            if (f >= 1.0 || double.IsPositiveInfinity (horizon))
                return 1.0;
            if (!(horizon > 0.0))
                return 0.0;
            return 1.0 - Math.Pow (1.0 - f, horizon);
        }

        /// <summary>
        /// The horizon: the largest time constant (GimbalSpool.HorizonSteps) of the given Lerp factors that
        /// move (f &gt; 0); at least one step.
        /// </summary>
        internal static double Horizon (double[] factors, int count)
        {
            double h = 1.0;
            for (int i = 0; i < count; i++) {
                double f = factors [i];
                if (!(f > 0.0) || double.IsNaN (f))
                    continue;
                double t = GimbalSpool.HorizonSteps (f);
                if (t > h && !double.IsInfinity (t))
                    h = t;
            }
            return h;
        }

        /// <summary>
        /// Solves the cascade on the inputs. Outputs X (realized change at the horizon), Command, Achieved
        /// (sum of the columns times X), RowAuthority, Iterations, Stages, Converged and the reason bits of
        /// the solve (limited levels, not converged, jitter, no actuator). levelsUsed: the levels solved (the
        /// authority problems stop at the lateral one). Returns false when no variable has an effect.
        /// </summary>
        internal bool Solve (bool warmStart, int lastLevel = LevelRoll)
        {
            Iterations = 0;
            Stages = 0;
            Converged = true;
            JointTaken = false;
            Reasons = 0;
            int n = Count;
            for (int r = 0; r < Rows; r++) {
                Achieved [r] = 0.0;
                RowAuthority [r] = 0.0;
            }
            for (int j = 0; j < n; j++) {
                X [j] = 0.0;
                Command [j] = U0 [j];
            }
            // Row authority at full span (phi 1): sum |A_rj| (hi - lo) / 2.
            double maxAuthority = 0.0;
            for (int j = 0; j < n; j++) {
                double h = 0.5 * (Hi [j] - Lo [j]);
                if (!(h > 0.0) || !Finite (h))
                    continue;
                for (int r = 0; r < Rows; r++) {
                    double c = Col [6 * j + r];
                    if (Finite (c))
                        RowAuthority [r] += Math.Abs (c) * h;
                }
            }
            for (int r = 0; r < Rows; r++)
                if (RowAuthority [r] > maxAuthority)
                    maxAuthority = RowAuthority [r];
            bool any = false;
            for (int r = 0; r < Rows; r++) {
                // Forces and torques are not comparable: a row is absent against its own kind only.
                double peer = 0.0;
                for (int k = 0; k < Rows; k++)
                    if ((k < 3) == (r < 3) && RowAuthority [k] > peer)
                        peer = RowAuthority [k];
                present [r] = RowAuthority [r] > 0.0 && RowAuthority [r] > AbsentShare * peer &&
                    Finite (RowAuthority [r]);
                scale [r] = present [r] ? 1.0 / RowAuthority [r] : 0.0;
            }
            // Variables: box in x, normalized.
            double maxEffect = 0.0;
            for (int j = 0; j < n; j++) {
                double u0 = U0 [j];
                double xl = Realized (Lo [j], u0, PhiUp [j], PhiDown [j]);
                double xh = Realized (Hi [j], u0, PhiUp [j], PhiDown [j]);
                double s = 0.5 * (xh - xl);
                bool ok = s > 0.0 && Finite (s) && Finite (u0);
                double e2 = 0.0;
                for (int r = 0; r < Rows; r++) {
                    double v = ok && present [r] && Finite (Col [6 * j + r]) ? Col [6 * j + r] * s * scale [r] : 0.0;
                    a [6 * j + r] = v;
                    e2 += v * v;
                }
                double e = Math.Sqrt (e2);
                span [j] = ok ? s : 0.0;
                r2 [j] = e;   // own effect, kept until the regularization below
                if (e > maxEffect)
                    maxEffect = e;
                if (ok) {
                    ylo [j] = xl / s;
                    yhi [j] = xh / s;
                } else {
                    ylo [j] = yhi [j] = 0.0;
                }
            }
            for (int j = 0; j < n; j++) {
                double e = r2 [j];
                bool live = span [j] > 0.0 && e > AbsentShare * maxEffect && maxEffect > 0.0;
                if (!live) {
                    // Absent: fixed where it is (0 if the box holds it, else its nearest bound).
                    for (int r = 0; r < Rows; r++)
                        a [6 * j + r] = 0.0;
                    double z = 0.0;
                    if (z < ylo [j])
                        z = ylo [j];
                    if (z > yhi [j])
                        z = yhi [j];
                    ylo [j] = yhi [j] = z;
                    r2 [j] = 1.0;
                    p [j] = z;
                    continue;
                }
                any = true;
                double prio = Priority [j] > 0.0 && Finite (Priority [j]) ? Priority [j] : 1.0;
                double rr = SqrtGamma * prio * e;
                r2 [j] = rr * rr;
                double pref = Realized (Pref [j], U0 [j], PhiUp [j], PhiDown [j]) / span [j];
                if (!Finite (pref))
                    pref = 0.0;
                p [j] = pref < ylo [j] ? ylo [j] : (pref > yhi [j] ? yhi [j] : pref);
            }
            if (!any) {
                Reasons |= ReasonNoActuator;
                return false;
            }
            // Normalized targets.
            double[] tb = cw.Target;
            for (int r = 0; r < Rows; r++)
                tb [r] = present [r] && Finite (Target [r]) ? Target [r] * scale [r] : 0.0;
            // The joint solve first (warm start only): every level of the problem nearly an equality at once. When
            // it meets every target to ExactShare, it IS the cascade's answer (every level met: no level traded):
            // one stage instead of four, the usual case. Otherwise the cascade, from the stage-1 warm set.
            if (warmStart && Joint (n, lastLevel, tb))
                return Finish (n, lastLevel);
            Start (warmStart && haveWarm ? warm : null);
            // The cascade.
            for (int level = 0; level <= lastLevel && level < Levels; level++) {
                bool stage = false;
                for (int r = 0; r < Rows; r++)
                    if (RowLevel [r] == level && RowOn [r] && present [r])
                        stage = true;
                if (!stage)
                    continue;
                for (int r = 0; r < Rows; r++) {
                    int lv = RowLevel [r];
                    rowIn [r] = RowOn [r] && present [r] && lv <= level;
                    if (!rowIn [r]) {
                        q [r] = 1.0;
                        b [r] = 0.0;
                    } else if (lv < level) {
                        // An earlier level: held at what it achieved.
                        double v = 0.0;
                        for (int j = 0; j < n; j++)
                            v += a [6 * j + r] * y [j];
                        q [r] = JointQ;
                        b [r] = v;
                    } else {
                        q [r] = 1.0;
                        b [r] = tb [r];
                    }
                }
                bool converged;
                bool jitter;
                Iterations += cw.Solve (n, a, b, q, rowIn, r2, p, ylo, yhi, y, w, MaxIterations (n), out converged,
                    out jitter);
                Stages++;
                if (!converged)
                    Converged = false;
                if (jitter)
                    Reasons |= ReasonJitter;
                if (level == 0 && warmStart) {
                    for (int j = 0; j < n; j++)
                        warm [j] = w [j];
                    haveWarm = true;
                }
            }
            if (warmStart) {
                for (int j = 0; j < n; j++)
                    warmJoint [j] = w [j];
                haveWarmJoint = true;
            }
            return Finish (n, lastLevel);
        }

        /// <summary>The start: a working set at its bounds (null: none), the rest at the preferred point.</summary>
        void Start (int[] set)
        {
            for (int j = 0; j < Count; j++) {
                int s0 = set != null ? set [j] : CascadeWork.Free;
                if (yhi [j] - ylo [j] <= CascadeWork.BoundTolerance) {
                    w [j] = CascadeWork.Fixed;
                    y [j] = ylo [j];
                } else if (s0 == CascadeWork.Lower) {
                    w [j] = s0;
                    y [j] = ylo [j];
                } else if (s0 == CascadeWork.Upper) {
                    w [j] = s0;
                    y [j] = yhi [j];
                } else {
                    w [j] = CascadeWork.Free;
                    y [j] = p [j];
                }
            }
        }

        /// <summary>
        /// The joint solve: every row of the problem weighted with q = JointQ (nearly an equality), from the warm set of
        /// the last joint or final stage. True (accepted) when it converged and met every row to ExactShare of its
        /// authority (normalized: ExactShare itself).
        /// </summary>
        bool Joint (int n, int lastLevel, double[] tb)
        {
            Start (haveWarmJoint ? warmJoint : null);
            bool anyRow = false;
            for (int r = 0; r < Rows; r++) {
                rowIn [r] = RowOn [r] && present [r] && RowLevel [r] <= lastLevel;
                q [r] = JointQ;
                b [r] = rowIn [r] ? tb [r] : 0.0;
                anyRow |= rowIn [r];
            }
            if (!anyRow)
                return false;
            bool converged, jitter;
            Iterations += cw.Solve (n, a, b, q, rowIn, r2, p, ylo, yhi, y, w, MaxIterations (n), out converged, out jitter);
            if (!converged)
                return false;
            for (int r = 0; r < Rows; r++) {
                if (!rowIn [r])
                    continue;
                double v = 0.0;
                for (int j = 0; j < n; j++)
                    v += a [6 * j + r] * y [j];
                if (!(Math.Abs (v - b [r]) <= ExactShare))
                    return false;
            }
            for (int j = 0; j < n; j++)
                warmJoint [j] = w [j];
            haveWarmJoint = true;
            JointTaken = true;
            return true;
        }

        /// <summary>The commands, the achieved wrench and the limited levels of the solution y.</summary>
        bool Finish (int n, int lastLevel)
        {
            if (!Converged)
                Reasons |= ReasonNotConverged;
            for (int j = 0; j < n; j++) {
                double x = span [j] > 0.0 ? y [j] * span [j] : 0.0;
                X [j] = x;
                double c = CommandOf (x, U0 [j], PhiUp [j], PhiDown [j]);
                if (c < Lo [j])
                    c = Lo [j];
                if (c > Hi [j])
                    c = Hi [j];
                Command [j] = c;
                for (int r = 0; r < Rows; r++) {
                    double col = Col [6 * j + r];
                    if (Finite (col))
                        Achieved [r] += col * x;
                }
            }
            for (int r = 0; r < Rows; r++) {
                if (!RowOn [r] || !present [r] || RowLevel [r] > lastLevel)
                    continue;
                if (Math.Abs (Target [r] - Achieved [r]) > TargetTolerance * RowAuthority [r]) {
                    int lv = RowLevel [r];
                    Reasons |= lv == LevelAxial ? ReasonLimitedAxial : lv == LevelPitchYaw ? ReasonLimitedPitchYaw
                        : lv == LevelLateral ? ReasonLimitedLateral : ReasonLimitedRoll;
                }
            }
            return true;
        }

        /// <summary>Active-set iterations of one stage: each adds or frees one bound; three passes over the variables.</summary>
        internal static int MaxIterations (int n)
        {
            return 3 * n + 6;
        }

        /// <summary>
        /// The residual of row r as a share of its authority (NaN when the row is not in the problem or absent).
        /// </summary>
        internal double ResidualShare (int r)
        {
            if (!RowOn [r] || !(RowAuthority [r] > 0.0))
                return double.NaN;
            return (Target [r] - Achieved [r]) / RowAuthority [r];
        }

        /// <summary>
        /// The authority along the unit body direction (ux, uz): the largest lateral force increment (N) along it
        /// with the axial force and pitch/yaw torque targets held (stages 1-2 as equalities), roll free. The inputs
        /// must be set with phi = 1 (steady state); Target[Fy], Target[Mx], Target[Mz] the increments to hold.
        /// The lateral rows are switched on with a target AuthorityTargetFactor x their authority along the
        /// direction; the roll off. Returns NaN when a stage did not converge (not an honest authority), 0 when
        /// nothing moves. Leaves Target and RowOn as it found them except the lateral targets.
        /// </summary>
        internal double AuthorityAlong (double ux, double uz)
        {
            bool rollOn = RowOn [RowMy];
            RowOn [RowFx] = true;
            RowOn [RowFz] = true;
            RowOn [RowMy] = false;
            // A first pass sizes the rows (Solve computes RowAuthority before any stage).
            double bound = 0.0;
            for (int j = 0; j < Count; j++) {
                double h = 0.5 * (Hi [j] - Lo [j]);
                if (!(h > 0.0) || !Finite (h))
                    continue;
                double fx = Col [6 * j + RowFx], fz = Col [6 * j + RowFz];
                if (Finite (fx) && Finite (fz))
                    bound += Math.Sqrt (fx * fx + fz * fz) * h;
            }
            Target [RowFx] = AuthorityTargetFactor * bound * ux;
            Target [RowFz] = AuthorityTargetFactor * bound * uz;
            bool ok = Solve (false, LevelLateral);
            RowOn [RowMy] = rollOn;
            if (!ok)
                return 0.0;
            if (!Converged)
                return double.NaN;
            double along = Achieved [RowFx] * ux + Achieved [RowFz] * uz;
            if (!(along > JointTranslation.NullAuthority * bound) || !Finite (along))
                return 0.0;
            return along;
        }

        // ------------------------------------------------------------ step 2: the RCS dead zone

        /// <summary>
        /// What ModuleRCS.Update keeps of a ctrlState component (decompiled, assembly inspection): 0 when its square is below
        /// EPSILON^2, the component itself above (the response is linear there). epsilon NaN or not positive: c.
        /// </summary>
        internal static double DeadZoned (double c, double epsilon)
        {
            if (!(epsilon > 0.0))
                return c;
            return c * c < epsilon * epsilon ? 0.0 : c;
        }

        /// <summary>
        /// A command inside the dead zone snapped to what the RCS realizes nearest: 0 below epsilon / 2, else
        /// +-epsilon. Outside the zone: c.
        /// </summary>
        internal static double SnapDeadZone (double c, double epsilon)
        {
            if (Math.Abs (c) >= epsilon)
                return c;
            if (Math.Abs (c) < 0.5 * epsilon)
                return 0.0;
            return c > 0.0 ? epsilon : -epsilon;
        }

        /// <summary>
        /// Step 2 (option 8192): Solve, then the RCS dead zone in the command. Every eligible variable whose
        /// command lies strictly inside the zone (0 &lt; |c| &lt; epsilon) is snapped (SnapDeadZone) and HELD there: its
        /// realization taken off the targets, its box closed on the snapped command, and the allocation solved again,
        /// so the other actuators take the rest. At most one pass per variable (each pass holds one more or stops).
        /// The outputs then carry the held variables at their snapped command and realization (X, Command,
        /// Achieved), the targets as given, Snapped the count and ReasonDeadZoneSnapped; a pass that did not converge
        /// leaves Converged false. Returns Solve's answer.
        /// </summary>
        internal bool SolveDeadZone (double epsilon, bool warmStart)
        {
            Snapped = 0;
            bool ok = Solve (warmStart);
            if (!ok || !(epsilon > 0.0) || !Finite (epsilon))
                return ok;
            int n = Count;
            for (int j = 0; j < n; j++)
                snapped [j] = false;
            bool notConverged = false;
            for (int pass = 0; pass < n; pass++) {
                bool any = false;
                for (int j = 0; j < n; j++) {
                    if (!DeadZoneEligible [j] || snapped [j])
                        continue;
                    double c = Command [j];
                    if (c == 0.0 || Math.Abs (c) >= epsilon)
                        continue;
                    double cs = SnapDeadZone (c, epsilon);
                    double xs = Realized (cs, U0 [j], PhiUp [j], PhiDown [j]);
                    snapped [j] = true;
                    snapX [j] = xs;
                    snapU0 [j] = U0 [j];
                    snapLo [j] = Lo [j];
                    snapHi [j] = Hi [j];
                    for (int r = 0; r < Rows; r++) {
                        double col = Col [6 * j + r];
                        if (Finite (col))
                            Target [r] -= col * xs;
                    }
                    U0 [j] = cs;
                    Lo [j] = cs;
                    Hi [j] = cs;
                    Snapped++;
                    any = true;
                }
                if (!any)
                    break;
                Solve (warmStart);
                if (!Converged)
                    notConverged = true;
            }
            if (Snapped == 0)
                return true;
            for (int j = 0; j < n; j++) {
                if (!snapped [j])
                    continue;
                double xs = snapX [j];
                X [j] = xs;
                Command [j] = U0 [j];
                U0 [j] = snapU0 [j];
                Lo [j] = snapLo [j];
                Hi [j] = snapHi [j];
                for (int r = 0; r < Rows; r++) {
                    double col = Col [6 * j + r];
                    if (!Finite (col))
                        continue;
                    Achieved [r] += col * xs;
                    Target [r] += col * xs;
                }
            }
            Reasons |= ReasonDeadZoneSnapped;
            if (notConverged) {
                Converged = false;
                Reasons |= ReasonNotConverged;
            }
            return true;
        }

        internal static bool Finite (double v)
        {
            return !double.IsNaN (v) && !double.IsInfinity (v);
        }
    }

    /// <summary>
    /// (option 8192 of AttitudeOptionsV1, born closed): when the engine-by-engine allocation WRITES in place
    /// of the law's allocation and the lateral-force layer, and when it hands them back. PURE (tests, mirrored in
    /// Python: the client of the autopilot).
    ///
    /// The allocation writes on a step where the pitch/yaw channel writes (the law would) and its own solve of THIS
    /// step is sound: solved, converged, neither the axial force nor pitch/yaw limited (a saturation: the law's own
    /// allocation, which the client has flown, takes it), every gimbal the law drives in it, no lease refused at its
    /// last write, no exception. Any of these failing hands the step back to the law and the layer (Fallback*),
    /// WITHOUT a jump: both writers start from the REALIZED state (the law's u0 is the realized deflection, the layer
    /// counts no common of its own after an allocation step, the throttles and the RCS answer the next command), and
    /// the gimbals and throttles move by their own Lerp. After a fallback the allocation takes the hand back only after
    /// ResumeSteps sound solves in a row: the horizon of the actuators, the time the fallback writer's commands need to
    /// be realized (no chatter between two writers). A writing allocation goes on as long as its solves are sound.
    /// </summary>
    internal static class AllocationWriteRule
    {
        internal const int StateOff = 0;        // option 8192 off, or the pitch/yaw channel does not write this step
        internal const int StateWriting = 1;
        internal const int StateFallback = 2;

        internal const long FallbackNotSolved = 1L << 0;
        internal const long FallbackNotConverged = 1L << 1;
        internal const long FallbackSaturated = 1L << 2;      // axial or pitch/yaw limited
        internal const long FallbackStale = 1L << 3;          // no solve of this physics step
        internal const long FallbackLease = 1L << 4;          // a gimbal or throttle refused at the last write
        internal const long FallbackGimbalMissing = 1L << 5;  // a gimbal the law drives is not in the allocation
        internal const long FallbackException = 1L << 6;
        internal const long FallbackResuming = 1L << 7;       // sound again, ResumeSteps not yet reached

        /// <summary>The reasons this step's solve may not write (0: it may).</summary>
        internal static long SolveFallbacks (int state, bool converged, long reasons, bool fresh, bool leaseLost,
            bool gimbalMissing, bool exception)
        {
            long m = 0L;
            if (state != EngineAllocation.StateSolved)
                m |= FallbackNotSolved;
            if (!converged || (reasons & EngineAllocation.ReasonNotConverged) != 0)
                m |= FallbackNotConverged;
            if ((reasons & (EngineAllocation.ReasonLimitedAxial | EngineAllocation.ReasonLimitedPitchYaw)) != 0)
                m |= FallbackSaturated;
            if (!fresh)
                m |= FallbackStale;
            if (leaseLost)
                m |= FallbackLease;
            if (gimbalMissing)
                m |= FallbackGimbalMissing;
            if (exception)
                m |= FallbackException;
            return m;
        }

        /// <summary>The sound solves in a row before the hand is taken back: the horizon in steps, at least one.</summary>
        internal static int ResumeSteps (double horizonSteps)
        {
            double h = horizonSteps > 1.0 && !double.IsInfinity (horizonSteps) ? horizonSteps : 1.0;
            return (int)Math.Ceiling (h - 1e-9);
        }

        /// <summary>
        /// The decision: write or not, the streak of sound solves updated, the reasons published (FallbackResuming
        /// while sound but waiting). wasWriting: the allocation wrote at the previous step (it goes on).
        /// </summary>
        internal static bool Decide (long fallbacks, ref int streak, int resumeSteps, bool wasWriting, out long published)
        {
            if (fallbacks != 0L) {
                streak = 0;
                published = fallbacks;
                return false;
            }
            streak++;
            if (wasWriting || streak >= resumeSteps) {
                published = 0L;
                return true;
            }
            published = FallbackResuming;
            return false;
        }
    }

    /// <summary>
    /// one stage of the cascade, a primal active set on a box with at most six rows, each weighted
    /// (q = 1) or an equality (q = 0), solved through the 6 x 6 Woodbury system (see EngineAllocation).
    /// Preallocated; PURE.
    /// </summary>
    internal sealed class CascadeWork
    {
        internal const int Free = 0;
        internal const int Lower = -1;
        internal const int Upper = 1;
        internal const int Fixed = 2;
        internal const double BoundTolerance = AttitudeMath.BoundTolerance;
        internal const double KktTolerance = AttitudeMath.KktTolerance;
        const int R = EngineAllocation.Rows;

        internal readonly double[] Target = new double[R];
        readonly double[] s = new double[R * R], s2 = new double[R * R], l = new double[R * R];
        readonly double[] rhs = new double[R], z = new double[R], diag0 = new double[R];
        double[] ystar = new double[0], ac = new double[0];

        /// <summary>
        /// One stage. a: normalized columns [6 j + r]; b: row targets; q: 1 weighted, 0 equality; rowIn: the rows
        /// in the stage (the others contribute nothing); r2: regularization weights (squared); p: preferred
        /// point; lo/hi: box; y: feasible start (in), solution (out); w: working set (in/out). Returns the
        /// iterations. converged false when the iterations ran out or the 6 x 6 failed even with jitter.
        /// S and the right-hand side are built once and then moved by rank-one terms as a variable reaches a
        /// bound (rebuilt when an equality row would lose its last free column, and when a bound is released).
        /// </summary>
        internal int Solve (int n, double[] a, double[] b, double[] q, bool[] rowIn, double[] r2, double[] p,
            double[] lo, double[] hi, double[] y, int[] w, int maxIterations, out bool converged, out bool jitter)
        {
            converged = false;
            jitter = false;
            if (ystar.Length < n) {
                ystar = new double[n];
                ac = new double[6 * n];
            }
            // The stage's columns: the rows out of it are zero (their S diagonal 1, their z 0).
            for (int j = 0; j < n; j++)
                for (int r = 0; r < R; r++)
                    ac [6 * j + r] = rowIn [r] ? a [6 * j + r] : 0.0;
            Build (n, b, q, rowIn, r2, p, y, w);
            int iteration = 0;
            while (iteration < maxIterations) {
                iteration++;
                if (!Cholesky6 (q, rowIn, ref jitter))
                    return iteration;
                // Free optimum, then the longest feasible step toward it.
                double step = 1.0;
                int block = -1, side = Free;
                for (int j = 0; j < n; j++) {
                    if (w [j] != Free)
                        continue;
                    int o = 6 * j;
                    double g = ac [o] * z [0] + ac [o + 1] * z [1] + ac [o + 2] * z [2] + ac [o + 3] * z [3] +
                        ac [o + 4] * z [4] + ac [o + 5] * z [5];
                    double v = p [j] + g / r2 [j];
                    ystar [j] = v;
                    double d = v - y [j];
                    if (d > 0.0 && v > hi [j] + BoundTolerance) {
                        double t = (hi [j] - y [j]) / d;
                        if (t < step) {
                            step = t;
                            block = j;
                            side = Upper;
                        }
                    } else if (d < 0.0 && v < lo [j] - BoundTolerance) {
                        double t = (lo [j] - y [j]) / d;
                        if (t < step) {
                            step = t;
                            block = j;
                            side = Lower;
                        }
                    }
                }
                if (step < 0.0)
                    step = 0.0;
                for (int j = 0; j < n; j++)
                    if (w [j] == Free)
                        y [j] = y [j] + step * (ystar [j] - y [j]);
                if (block >= 0) {
                    y [block] = side == Upper ? hi [block] : lo [block];
                    w [block] = side;
                    if (!Leave (block, q, rowIn, r2, p, y))
                        Build (n, b, q, rowIn, r2, p, y, w);
                    continue;
                }
                for (int j = 0; j < n; j++) {
                    if (w [j] != Free)
                        continue;
                    if (y [j] > hi [j])
                        y [j] = hi [j];
                    if (y [j] < lo [j])
                        y [j] = lo [j];
                }
                // Multipliers of the bounds: g_j = r2_j (y_j - p_j) - a_j . z (the gradient with the rows' z).
                int worst = -1;
                double worstValue = -KktTolerance;
                for (int j = 0; j < n; j++) {
                    if (w [j] != Lower && w [j] != Upper)
                        continue;
                    int o = 6 * j;
                    double g = r2 [j] * (y [j] - p [j]) - (ac [o] * z [0] + ac [o + 1] * z [1] + ac [o + 2] * z [2] +
                        ac [o + 3] * z [3] + ac [o + 4] * z [4] + ac [o + 5] * z [5]);
                    double lambda = w [j] == Lower ? g : -g;
                    if (lambda < worstValue) {
                        worstValue = lambda;
                        worst = j;
                    }
                }
                if (worst < 0) {
                    converged = true;
                    return iteration;
                }
                w [worst] = Free;
                Build (n, b, q, rowIn, r2, p, y, w);
            }
            return iteration;
        }

        /// <summary>S = diag(q) + sum_free a_j a_j^T / r2_j, rhs = b - sum_fixed a_j y_j - sum_free a_j p_j, from scratch.</summary>
        void Build (int n, double[] b, double[] q, bool[] rowIn, double[] r2, double[] p, double[] y, int[] w)
        {
            for (int k = 0; k < R * R; k++)
                s [k] = 0.0;
            for (int r = 0; r < R; r++) {
                s [r * R + r] = rowIn [r] ? q [r] : 1.0;
                rhs [r] = rowIn [r] ? b [r] : 0.0;
            }
            for (int j = 0; j < n; j++) {
                int o = 6 * j;
                if (w [j] == Free) {
                    double inv = 1.0 / r2 [j], pj = p [j];
                    for (int r = 0; r < R; r++) {
                        double ar = ac [o + r];
                        if (ar == 0.0)
                            continue;
                        rhs [r] -= ar * pj;
                        double ari = ar * inv;
                        for (int c = 0; c <= r; c++)
                            s [r * R + c] += ari * ac [o + c];
                    }
                } else {
                    double yj = y [j];
                    for (int r = 0; r < R; r++)
                        rhs [r] -= ac [o + r] * yj;
                }
            }
            for (int r = 0; r < R; r++)
                diag0 [r] = s [r * R + r];
        }

        /// <summary>
        /// Variable j leaves the free set, now at its bound y_j: S -= a_j a_j^T / r2_j, rhs += a_j (p_j - y_j). False
        /// (the caller rebuilds) when an equality row's diagonal falls to rounding: its last free column left.
        /// </summary>
        bool Leave (int j, double[] q, bool[] rowIn, double[] r2, double[] p, double[] y)
        {
            int o = 6 * j;
            double inv = 1.0 / r2 [j];
            for (int r = 0; r < R; r++) {
                double ar = ac [o + r];
                if (ar == 0.0)
                    continue;
                rhs [r] += ar * (p [j] - y [j]);
                double ari = ar * inv;
                for (int c = 0; c <= r; c++)
                    s [r * R + c] -= ari * ac [o + c];
            }
            for (int r = 0; r < R; r++)
                if (rowIn [r] && q [r] == 0.0 && !(s [r * R + r] > 1e-9 * diag0 [r]))
                    return false;
            return true;
        }

        /// <summary>
        /// Solves S z = rhs (6 x 6, lower triangle of S filled), on a copy when it must be jittered: a non-positive
        /// pivot (an equality row without rank on the free set) retries once with 1e-12 x the largest diagonal
        /// added to the equality rows. False when that fails too.
        /// </summary>
        bool Cholesky6 (double[] q, bool[] rowIn, ref bool jitter)
        {
            if (Factor (s, l))
                return Back (l, rhs, z);
            double max = 0.0;
            for (int k = 0; k < R; k++)
                if (s [k * R + k] > max)
                    max = s [k * R + k];
            double eps = 1e-12 * (max > 0.0 ? max : 1.0);
            for (int k = 0; k < R * R; k++)
                s2 [k] = s [k];
            for (int k = 0; k < R; k++)
                if (rowIn [k] && q [k] == 0.0)
                    s2 [k * R + k] += eps;
            jitter = true;
            return Factor (s2, l) && Back (l, rhs, z);
        }

        static bool Factor (double[] sm, double[] lm)
        {
            for (int i = 0; i < R; i++) {
                for (int j = 0; j <= i; j++) {
                    double v = sm [i * R + j];
                    for (int k = 0; k < j; k++)
                        v -= lm [i * R + k] * lm [j * R + k];
                    if (i == j) {
                        if (!(v > 0.0) || double.IsInfinity (v))
                            return false;
                        lm [i * R + i] = Math.Sqrt (v);
                    } else {
                        lm [i * R + j] = v / lm [j * R + j];
                    }
                }
            }
            return true;
        }

        static bool Back (double[] lm, double[] r, double[] x)
        {
            for (int i = 0; i < R; i++) {
                double v = r [i];
                for (int k = 0; k < i; k++)
                    v -= lm [i * R + k] * x [k];
                x [i] = v / lm [i * R + i];
            }
            for (int i = R - 1; i >= 0; i--) {
                double v = x [i];
                for (int k = i + 1; k < R; k++)
                    v -= lm [k * R + i] * x [k];
                x [i] = v / lm [i * R + i];
            }
            for (int i = 0; i < R; i++)
                if (double.IsNaN (x [i]) || double.IsInfinity (x [i]))
                    return false;
            return true;
        }
    }
}
