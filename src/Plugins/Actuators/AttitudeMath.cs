using System;

namespace KRPC.Bridge.Actuators
{
    // -----------------------------------------------------------------------------------
    // AttitudeV1 control law, PURE maths. No Unity, no KSP, no kRPC type in this file:
    // it is compiled three times - the real plugin (net472), the stub type-check
    // (build/verify) and the standalone test harness (build/tests), which also writes the
    // golden vectors replayed by the Python mirror (ksp autopilot:
    // guidance/attitude_c_loi.py, tests/test_attitude_c_miroir.py).
    //
    // Keep the arithmetic ORDER of every expression unchanged when editing: the Python
    // mirror repeats it operation by operation, and the golden test compares the two.
    //
    // Frame: vessel body (x right = pitch axis, y nose = roll axis, z bottom = yaw axis),
    // the frame of Vessel.angularVelocity, Vessel.MOI and every torque below.
    //
    // Law (plan GNC v3, R2 section 1.4), per body axis:
    //   H(z)  same 2nd-order low-pass on the measured omega_dot AND on the realized torque
    //   alpha = sum_j |B_aj| * span_j / I_a                       (authority, every tick)
    //   w_r   = sat_wmax( sign(e) * min(K_lin |e|, sqrt(2 eta alpha |e|)) ) + w_ff - w_pch
    //   nu    = w_dot_ff + K_w (w_r - w),  |nu| <= alpha
    //   dM_d  = I (nu - w_dot_f)            M_d = M_f + dM_d      (torque-form INDI)
    //   du    = argmin |W_v (B du - (M_d - M))|^2 + gamma |W_u (u0 + du - u_pref)|^2
    //           lb <= du <= ub, active set, warm start
    //           W_v = w_axis / I (errors in rad/s^2); W_u,j = priority_j * e_j / span_j,
    //           e_j = |W_v B_j| span_j the acceleration of actuator j at full span: the
    //           regularization costs gamma * (own effect)^2, whatever the vessel size
    //   nu_h  = I^-1 (M_d - M - B du)       unrealizable part; w_pch integrates it (PCH)
    //
    // The allocation target is M_d - M (M = realized torque, unfiltered), which is the
    // torque-form INDI command M_f + I (nu - w_dot_f) expressed as an increment from the
    // realized actuator state u0: the filter lag of M_f is compensated, as in synchronized
    // INDI (Smeur 2016).
    // -----------------------------------------------------------------------------------

    /// <summary>Gains and weights of the AttitudeV1 law. Born with the plan R2 values.</summary>
    internal sealed class AttitudeGains
    {
        internal double FilterOmegaC = 18.0;      // rad/s, 2nd-order low-pass of H(z)
        internal double FilterZeta = 0.7;
        internal double KOmegaPitchYaw = 3.0;     // 1/s  ~ 1/(3 tau_gimbal)
        internal double KOmegaRoll = 2.0;         // 1/s  roll: rate damping only
        internal double KLin = 0.75;              // 1/s  ~ K_omega / 4
        internal double Eta = 0.6;                // fraction of authority for braking
        internal double TauPch = 1.0;             // s    leak of the hedge integral
        internal double Gamma = 1e-4;             // regularization, relative to own effect^2
        internal double EffectFloor = 1e-9;       // rad/s^2: keeps a dead actuator regularized
        internal double WvPitchYaw = 1.0;         // torque-error weights (per unit accel)
        internal double WvRoll = 0.3;
        internal double AlphaFloor = 1e-6;        // rad/s^2
        internal double OmegaMaxDefault = 1.0;    // rad/s, kRPC AutoPilot default
        internal double TauResidual = 0.2;        // s, residual guard filter
        internal int MaxIterations = 20;
    }

    /// <summary>Direct-form-I biquad, three independent axes, same coefficients.</summary>
    internal sealed class Biquad3
    {
        internal double B0, B1, B2, A1, A2;
        internal double Dt = -1.0, OmegaC = -1.0, Zeta = -1.0;
        internal bool Primed;
        internal readonly double[] X1 = new double[3];
        internal readonly double[] X2 = new double[3];
        internal readonly double[] Y1 = new double[3];
        internal readonly double[] Y2 = new double[3];

        /// <summary>Tustin with prewarping at omegaC. Recomputed only when a parameter moves.</summary>
        internal void Configure (double dt, double omegaC, double zeta)
        {
            if (dt == Dt && omegaC == OmegaC && zeta == Zeta)
                return;
            Dt = dt;
            OmegaC = omegaC;
            Zeta = zeta;
            double k = omegaC / Math.Tan (omegaC * dt / 2.0);
            double k2 = k * k;
            double w2 = omegaC * omegaC;
            double a0 = k2 + 2.0 * zeta * omegaC * k + w2;
            B0 = w2 / a0;
            B1 = 2.0 * w2 / a0;
            B2 = w2 / a0;
            A1 = (2.0 * w2 - 2.0 * k2) / a0;
            A2 = (k2 - 2.0 * zeta * omegaC * k + w2) / a0;
            Primed = false;
        }

        internal void Reset ()
        {
            Primed = false;
        }

        /// <summary>Filters x (3 values) into y. The first sample primes the steady state.</summary>
        internal void Step (double[] x, double[] y)
        {
            if (!Primed) {
                for (int a = 0; a < 3; a++) {
                    X1 [a] = x [a];
                    X2 [a] = x [a];
                    Y1 [a] = x [a];
                    Y2 [a] = x [a];
                }
                Primed = true;
            }
            for (int a = 0; a < 3; a++) {
                double v = B0 * x [a] + B1 * X1 [a] + B2 * X2 [a] - A1 * Y1 [a] - A2 * Y2 [a];
                X2 [a] = X1 [a];
                X1 [a] = x [a];
                Y2 [a] = Y1 [a];
                Y1 [a] = v;
                y [a] = v;
            }
        }
    }

    /// <summary>Everything the law remembers from one tick to the next, per vessel.</summary>
    internal sealed class AttitudeLawState
    {
        internal readonly Biquad3 FilterOmegaDot = new Biquad3 ();
        internal readonly Biquad3 FilterTorque = new Biquad3 ();
        internal readonly double[] OmegaPch = new double[3];
        internal bool HavePrevious;
        internal readonly double[] PrevOmegaDotF = new double[3];
        internal readonly double[] PrevTorqueF = new double[3];
        internal bool HaveResidualF;
        internal readonly double[] ResidualF = new double[3];
        internal int[] ActiveSet = new int[0];
        internal double ResidualHighSeconds;
        internal double SaturationHighSeconds;

        internal void Reset ()
        {
            FilterOmegaDot.Reset ();
            FilterTorque.Reset ();
            for (int a = 0; a < 3; a++) {
                OmegaPch [a] = 0.0;
                PrevOmegaDotF [a] = 0.0;
                PrevTorqueF [a] = 0.0;
                ResidualF [a] = 0.0;
            }
            HavePrevious = false;
            HaveResidualF = false;
            ActiveSet = new int[0];
            ResidualHighSeconds = 0.0;
            SaturationHighSeconds = 0.0;
        }
    }

    /// <summary>One tick of input. Arrays are reused; N is the actuator count.</summary>
    internal sealed class AttitudeInput
    {
        internal double Dt;
        internal readonly double[] Omega = new double[3];
        internal readonly double[] OmegaDotMeasured = new double[3];
        internal bool OmegaDotValid;              // false: tick gap, filters restart
        internal readonly double[] Torque = new double[3];    // realized, N.m
        internal readonly double[] Inertia = new double[3];   // kg.m^2, diagonal
        internal bool ReferenceValid;
        internal readonly double[] Error = new double[3];     // pointing error rotation vector, rad
        internal readonly double[] OmegaFf = new double[3];
        internal readonly double[] OmegaDotFf = new double[3];
        internal double OmegaMax;                 // <= 0: gains default
        internal int N;
        internal double[] B = new double[0];      // column j of axis a at [a * N + j]
        internal double[] U0 = new double[0];
        internal double[] UMin = new double[0];
        internal double[] UMax = new double[0];
        internal double[] UPref = new double[0];
        internal double[] Priority = new double[0];   // >= 1: higher = used last

        internal void Resize (int n)
        {
            if (N == n && B.Length == 3 * n)
                return;
            N = n;
            B = new double[3 * n];
            U0 = new double[n];
            UMin = new double[n];
            UMax = new double[n];
            UPref = new double[n];
            Priority = new double[n];
        }
    }

    /// <summary>One tick of output. Arrays are reused.</summary>
    internal sealed class AttitudeOutput
    {
        internal bool Computed;                   // reference, law and allocation ran
        internal bool Filtered;                   // H(z), residual and authority ran (no reference needed)
        internal readonly double[] OmegaDotF = new double[3];
        internal readonly double[] TorqueF = new double[3];
        internal readonly double[] Alpha = new double[3];
        internal readonly double[] OmegaSqrt = new double[3];
        internal readonly double[] OmegaRef = new double[3];
        internal readonly double[] OmegaPch = new double[3];
        internal readonly double[] Nu = new double[3];
        internal readonly double[] DeltaTorqueDesired = new double[3];   // I (nu - w_dot_f)
        internal readonly double[] TorqueDesired = new double[3];        // M_f + dM_d
        internal readonly double[] TorqueAllocTarget = new double[3];    // M_d - M
        internal readonly double[] TorqueAllocated = new double[3];      // B du
        internal readonly double[] NuHedge = new double[3];
        internal readonly double[] Residual = new double[3];
        internal readonly double[] ResidualF = new double[3];
        internal double[] DeltaU = new double[0];
        internal double[] U = new double[0];
        internal double[] RegularizationWeight = new double[0];
        internal int[] ActiveSet = new int[0];
        internal int Iterations;
        internal bool Converged;
        internal int ActiveCount;
        internal double SaturationFraction;
        internal bool ResidualGuard;              // |r_f| > 0.5 alpha for more than 1 s
        internal bool SaturationGuard;            // > 50 % saturated for more than 1 s
        internal bool OmegaGuard;                 // |w| > 1.5 w_max

        internal void Resize (int n)
        {
            if (DeltaU.Length == n)
                return;
            DeltaU = new double[n];
            U = new double[n];
            RegularizationWeight = new double[n];
            ActiveSet = new int[n];
        }
    }

    /// <summary>Preallocated work space of the allocation (no per-tick garbage in Unity).</summary>
    internal sealed class AttitudeWork
    {
        internal int M, N;
        internal double[] A = new double[0];      // m x n, row-major
        internal double[] Bv = new double[0];     // m
        internal double[] Lb = new double[0];
        internal double[] Ub = new double[0];
        internal double[] X = new double[0];
        internal double[] P = new double[0];
        internal double[] R = new double[0];      // m
        internal double[] G = new double[0];      // n
        internal double[] H = new double[0];      // n x n
        internal double[] L = new double[0];      // n x n
        internal double[] Y = new double[0];      // n
        internal int[] Free = new int[0];
        internal int[] W = new int[0];

        internal void Resize (int m, int n)
        {
            if (M == m && N == n)
                return;
            M = m;
            N = n;
            A = new double[m * n];
            Bv = new double[m];
            Lb = new double[n];
            Ub = new double[n];
            X = new double[n];
            P = new double[n];
            R = new double[m];
            G = new double[n];
            H = new double[n * n];
            L = new double[n * n];
            Y = new double[n];
            Free = new int[n];
            W = new int[n];
        }
    }

    internal static class AttitudeMath
    {
        // Working-set codes of the bounded least squares.
        internal const int SetFree = 0;
        internal const int SetLower = -1;
        internal const int SetUpper = 1;
        internal const int SetFixed = 2;           // lb == ub: never released
        internal const double BoundTolerance = 1e-12;
        internal const double KktTolerance = 1e-12;

        /// <summary>
        /// Pointing error as a body rotation vector: angle * unit(nose x d), nose = +y.
        /// The roll component is zero by construction. d need not be normalized.
        /// </summary>
        internal static bool PointingError (double dx, double dy, double dz, double[] e)
        {
            double n = Math.Sqrt (dx * dx + dy * dy + dz * dz);
            e [0] = 0.0;
            e [1] = 0.0;
            e [2] = 0.0;
            if (!(n > 1e-12) || double.IsInfinity (n))
                return false;
            double ux = dx / n;
            double uy = dy / n;
            double uz = dz / n;
            // nose (0,1,0) x d = (uz, 0, -ux)
            double s = Math.Sqrt (uz * uz + ux * ux);
            double angle = Math.Atan2 (s, uy);
            if (s > 1e-12) {
                e [0] = angle * (uz / s);
                e [2] = angle * (-ux / s);
            } else if (uy < 0.0) {
                // Antipodal: any axis orthogonal to the nose. Pitch, by convention.
                e [0] = Math.PI;
            }
            return true;
        }

        /// <summary>sign(e) min(kLin |e|, sqrt(2 eta alpha |e|)), saturated at omegaMax.</summary>
        internal static double SqrtReference (double e, double alpha, double eta, double kLin,
            double omegaMax)
        {
            double mag = Math.Abs (e);
            double lin = kLin * mag;
            double sq = Math.Sqrt (2.0 * eta * alpha * mag);
            double w = lin < sq ? lin : sq;
            if (e < 0.0)
                w = -w;
            if (w > omegaMax)
                w = omegaMax;
            if (w < -omegaMax)
                w = -omegaMax;
            return w;
        }

        /// <summary>
        /// One tick of the law. Never throws on finite input; non-finite input leaves
        /// output.Computed false and resets the state.
        /// </summary>
        internal static void Step (AttitudeGains g, AttitudeInput inp, AttitudeLawState st,
            AttitudeWork work, AttitudeOutput o)
        {
            int n = inp.N;
            o.Resize (n);
            o.Computed = false;
            o.Filtered = false;
            double dt = inp.Dt;
            if (!(dt > 0.0) || !Finite3 (inp.Omega) || !Finite3 (inp.Torque) ||
                    !Finite3 (inp.Inertia) || !(inp.Inertia [0] > 0.0) ||
                    !(inp.Inertia [1] > 0.0) || !(inp.Inertia [2] > 0.0) ||
                    !FiniteAll (inp.B, 3 * n) || !FiniteAll (inp.U0, n) ||
                    !FiniteAll (inp.UMin, n) || !FiniteAll (inp.UMax, n) ||
                    !FiniteAll (inp.UPref, n) || !FiniteAll (inp.Priority, n)) {
                st.Reset ();
                return;
            }

            // ------------------------------------------------------------- H(z)
            st.FilterOmegaDot.Configure (dt, g.FilterOmegaC, g.FilterZeta);
            st.FilterTorque.Configure (dt, g.FilterOmegaC, g.FilterZeta);
            if (!inp.OmegaDotValid || !Finite3 (inp.OmegaDotMeasured)) {
                // Tick gap: both filters restart together, never one without the other.
                st.FilterOmegaDot.Reset ();
                st.FilterTorque.Reset ();
                st.HavePrevious = false;
                st.HaveResidualF = false;
                return;
            }
            st.FilterOmegaDot.Step (inp.OmegaDotMeasured, o.OmegaDotF);
            st.FilterTorque.Step (inp.Torque, o.TorqueF);

            // ------------------------------------------------------ INDI residual
            // r = delta(w_dot_f) - I^-1 delta(M_f): what the torque increment did not explain.
            double beta = dt / (g.TauResidual + dt);
            for (int a = 0; a < 3; a++) {
                if (st.HavePrevious)
                    o.Residual [a] = (o.OmegaDotF [a] - st.PrevOmegaDotF [a])
                        - (o.TorqueF [a] - st.PrevTorqueF [a]) / inp.Inertia [a];
                else
                    o.Residual [a] = 0.0;
            }
            if (st.HavePrevious) {
                if (!st.HaveResidualF) {
                    for (int a = 0; a < 3; a++)
                        st.ResidualF [a] = o.Residual [a];
                    st.HaveResidualF = true;
                } else {
                    for (int a = 0; a < 3; a++)
                        st.ResidualF [a] = st.ResidualF [a] + beta * (o.Residual [a] - st.ResidualF [a]);
                }
            }
            for (int a = 0; a < 3; a++) {
                o.ResidualF [a] = st.ResidualF [a];
                st.PrevOmegaDotF [a] = o.OmegaDotF [a];
                st.PrevTorqueF [a] = o.TorqueF [a];
            }
            st.HavePrevious = true;

            // --------------------------------------------------------- authority
            for (int a = 0; a < 3; a++) {
                double sum = 0.0;
                for (int j = 0; j < n; j++) {
                    double span = Math.Abs (inp.UMax [j]) > Math.Abs (inp.UMin [j])
                        ? Math.Abs (inp.UMax [j]) : Math.Abs (inp.UMin [j]);
                    sum = sum + Math.Abs (inp.B [a * n + j]) * span;
                }
                double alpha = sum / inp.Inertia [a];
                o.Alpha [a] = alpha > g.AlphaFloor ? alpha : g.AlphaFloor;
            }

            o.Filtered = true;

            // Guards that do not need a reference.
            double omegaMax = inp.OmegaMax > 0.0 ? inp.OmegaMax : g.OmegaMaxDefault;
            double wNorm = Math.Sqrt (inp.Omega [0] * inp.Omega [0] + inp.Omega [1] * inp.Omega [1]
                + inp.Omega [2] * inp.Omega [2]);
            o.OmegaGuard = wNorm > 1.5 * omegaMax;
            bool residualHigh = false;
            for (int a = 0; a < 3; a++)
                if (Math.Abs (o.ResidualF [a]) > 0.5 * o.Alpha [a])
                    residualHigh = true;
            st.ResidualHighSeconds = residualHigh ? st.ResidualHighSeconds + dt : 0.0;
            o.ResidualGuard = st.ResidualHighSeconds > 1.0;

            if (!inp.ReferenceValid || !Finite3 (inp.Error) || !Finite3 (inp.OmegaFf) ||
                    !Finite3 (inp.OmegaDotFf)) {
                // No reference: filters stay warm, the hedge and the allocation do not run.
                for (int a = 0; a < 3; a++)
                    st.OmegaPch [a] = 0.0;
                st.SaturationHighSeconds = 0.0;
                o.SaturationGuard = false;
                return;
            }

            // ------------------------------------------ reference and pseudo-control
            for (int a = 0; a < 3; a++) {
                double ws = a == 1 ? 0.0 : SqrtReference (inp.Error [a], o.Alpha [a], g.Eta,
                    g.KLin, omegaMax);
                o.OmegaSqrt [a] = ws;
                o.OmegaRef [a] = ws + inp.OmegaFf [a] - st.OmegaPch [a];
                double k = a == 1 ? g.KOmegaRoll : g.KOmegaPitchYaw;
                double nu = inp.OmegaDotFf [a] + k * (o.OmegaRef [a] - inp.Omega [a]);
                if (nu > o.Alpha [a])
                    nu = o.Alpha [a];
                if (nu < -o.Alpha [a])
                    nu = -o.Alpha [a];
                o.Nu [a] = nu;
                o.DeltaTorqueDesired [a] = inp.Inertia [a] * (nu - o.OmegaDotF [a]);
                o.TorqueDesired [a] = o.TorqueF [a] + o.DeltaTorqueDesired [a];
                o.TorqueAllocTarget [a] = o.TorqueDesired [a] - inp.Torque [a];
            }

            // ------------------------------------------------------ allocation (WLS)
            int m = 3 + n;
            work.Resize (m, n);
            double sqrtGamma = Math.Sqrt (g.Gamma);
            for (int i = 0; i < m * n; i++)
                work.A [i] = 0.0;
            for (int a = 0; a < 3; a++) {
                double wv = (a == 1 ? g.WvRoll : g.WvPitchYaw) / inp.Inertia [a];
                for (int j = 0; j < n; j++)
                    work.A [a * n + j] = wv * inp.B [a * n + j];
                work.Bv [a] = wv * o.TorqueAllocTarget [a];
            }
            for (int j = 0; j < n; j++) {
                // Own effect at full span, in the weighted acceleration units of the rows.
                double eff2 = 0.0;
                for (int a = 0; a < 3; a++)
                    eff2 = eff2 + work.A [a * n + j] * work.A [a * n + j];
                double span = Math.Abs (inp.UMax [j]) > Math.Abs (inp.UMin [j])
                    ? Math.Abs (inp.UMax [j]) : Math.Abs (inp.UMin [j]);
                double wu;
                if (span > 1e-9) {
                    double eff = Math.Sqrt (eff2) * span;
                    if (eff < g.EffectFloor)
                        eff = g.EffectFloor;
                    wu = sqrtGamma * inp.Priority [j] * eff / span;
                } else {
                    wu = sqrtGamma;     // zero span: the box fixes it anyway
                }
                o.RegularizationWeight [j] = wu;
                work.A [(3 + j) * n + j] = wu;
                work.Bv [3 + j] = wu * (inp.UPref [j] - inp.U0 [j]);
                work.Lb [j] = inp.UMin [j] - inp.U0 [j];
                work.Ub [j] = inp.UMax [j] - inp.U0 [j];
                if (!(work.Lb [j] <= work.Ub [j])) {
                    // Inverted or non-finite box: this actuator is not moved.
                    work.Lb [j] = 0.0;
                    work.Ub [j] = 0.0;
                }
                int prev = st.ActiveSet.Length == n ? st.ActiveSet [j] : SetFree;
                work.W [j] = prev == SetFixed ? SetFree : prev;
            }
            bool converged;
            int iterations = SolveBoxLeastSquares (work, m, n, g.MaxIterations, out converged);
            if (st.ActiveSet.Length != n)
                st.ActiveSet = new int[n];
            int active = 0;
            for (int j = 0; j < n; j++) {
                o.DeltaU [j] = work.X [j];
                o.U [j] = inp.U0 [j] + work.X [j];
                o.ActiveSet [j] = work.W [j];
                st.ActiveSet [j] = work.W [j];
                if (work.W [j] == SetLower || work.W [j] == SetUpper)
                    active++;
            }
            o.Iterations = iterations;
            o.Converged = converged;
            o.ActiveCount = active;
            o.SaturationFraction = n > 0 ? (double)active / n : 0.0;
            st.SaturationHighSeconds = o.SaturationFraction > 0.5
                ? st.SaturationHighSeconds + dt : 0.0;
            o.SaturationGuard = st.SaturationHighSeconds > 1.0;

            // ------------------------------------------------------- hedge (PCH)
            double leak = dt / g.TauPch;
            for (int a = 0; a < 3; a++) {
                double achieved = 0.0;
                for (int j = 0; j < n; j++)
                    achieved = achieved + inp.B [a * n + j] * o.DeltaU [j];
                o.TorqueAllocated [a] = achieved;
                o.NuHedge [a] = (o.TorqueAllocTarget [a] - achieved) / inp.Inertia [a];
                double pch = st.OmegaPch [a] + dt * o.NuHedge [a] - leak * st.OmegaPch [a];
                if (pch > omegaMax)
                    pch = omegaMax;
                if (pch < -omegaMax)
                    pch = -omegaMax;
                st.OmegaPch [a] = pch;
                o.OmegaPch [a] = pch;
            }
            o.Computed = true;
        }

        /// <summary>
        /// Box-constrained linear least squares, primal active set:
        ///   min |A x - b|^2  s.t.  lb &lt;= x &lt;= ub   (A = work.A, m x n, b = work.Bv).
        /// work.W holds the warm-start working set (0 free, -1 at lb, +1 at ub) and is left
        /// holding the final one (2 = fixed, lb == ub). work.X receives x. A must have full
        /// column rank on every free subset (the regularization rows guarantee it). Returns
        /// the iteration count; converged is false when MaxIterations ran out or a
        /// Cholesky pivot failed (x is then the last feasible iterate).
        /// </summary>
        internal static int SolveBoxLeastSquares (AttitudeWork work, int m, int n, int maxIterations,
            out bool converged)
        {
            double[] A = work.A, b = work.Bv, lb = work.Lb, ub = work.Ub, x = work.X;
            double[] p = work.P, r = work.R, grad = work.G, H = work.H, L = work.L, y = work.Y;
            int[] w = work.W, free = work.Free;
            converged = false;

            // Feasible start: the warm working set at its bounds, the rest at clamp(0).
            for (int j = 0; j < n; j++) {
                if (ub [j] - lb [j] <= BoundTolerance) {
                    w [j] = SetFixed;
                    x [j] = lb [j];
                } else if (w [j] == SetLower) {
                    x [j] = lb [j];
                } else if (w [j] == SetUpper) {
                    x [j] = ub [j];
                } else {
                    w [j] = SetFree;
                    double v = 0.0;
                    if (v < lb [j])
                        v = lb [j];
                    if (v > ub [j])
                        v = ub [j];
                    x [j] = v;
                }
            }

            int iteration = 0;
            while (iteration < maxIterations) {
                iteration++;

                // Residual r = b - A x.
                for (int i = 0; i < m; i++) {
                    double s = b [i];
                    for (int j = 0; j < n; j++)
                        s = s - A [i * n + j] * x [j];
                    r [i] = s;
                }
                int nf = 0;
                for (int j = 0; j < n; j++)
                    if (w [j] == SetFree)
                        free [nf++] = j;

                // Step p on the free variables: min |A_F p - r|^2 (normal equations).
                for (int j = 0; j < n; j++)
                    p [j] = 0.0;
                if (nf > 0) {
                    for (int fi = 0; fi < nf; fi++) {
                        int ci = free [fi];
                        double gs = 0.0;
                        for (int i = 0; i < m; i++)
                            gs = gs + A [i * n + ci] * r [i];
                        y [fi] = gs;
                        for (int fk = 0; fk <= fi; fk++) {
                            int ck = free [fk];
                            double hs = 0.0;
                            for (int i = 0; i < m; i++)
                                hs = hs + A [i * n + ci] * A [i * n + ck];
                            H [fi * n + fk] = hs;
                        }
                    }
                    if (!CholeskySolve (H, L, y, nf, n))
                        return iteration;       // not converged, x feasible
                    for (int fi = 0; fi < nf; fi++)
                        p [free [fi]] = y [fi];
                }

                // Longest feasible fraction of the step.
                double step = 1.0;
                int block = -1;
                int blockSide = SetFree;
                for (int fi = 0; fi < nf; fi++) {
                    int j = free [fi];
                    if (p [j] > 0.0 && x [j] + p [j] > ub [j] + BoundTolerance) {
                        double t = (ub [j] - x [j]) / p [j];
                        if (t < step) {
                            step = t;
                            block = j;
                            blockSide = SetUpper;
                        }
                    } else if (p [j] < 0.0 && x [j] + p [j] < lb [j] - BoundTolerance) {
                        double t = (lb [j] - x [j]) / p [j];
                        if (t < step) {
                            step = t;
                            block = j;
                            blockSide = SetLower;
                        }
                    }
                }
                for (int fi = 0; fi < nf; fi++) {
                    int j = free [fi];
                    x [j] = x [j] + step * p [j];
                }
                if (block >= 0) {
                    x [block] = blockSide == SetUpper ? ub [block] : lb [block];
                    w [block] = blockSide;
                    continue;
                }
                // Clean the full step against round-off, then test the multipliers.
                for (int fi = 0; fi < nf; fi++) {
                    int j = free [fi];
                    if (x [j] > ub [j])
                        x [j] = ub [j];
                    if (x [j] < lb [j])
                        x [j] = lb [j];
                }
                // grad = A^T (A x - b)
                for (int i = 0; i < m; i++) {
                    double s = -b [i];
                    for (int j = 0; j < n; j++)
                        s = s + A [i * n + j] * x [j];
                    r [i] = s;
                }
                int worst = -1;
                double worstValue = -KktTolerance;
                for (int j = 0; j < n; j++) {
                    if (w [j] != SetLower && w [j] != SetUpper)
                        continue;
                    double gs = 0.0;
                    for (int i = 0; i < m; i++)
                        gs = gs + A [i * n + j] * r [i];
                    grad [j] = gs;
                    double lambda = w [j] == SetLower ? gs : -gs;
                    if (lambda < worstValue) {
                        worstValue = lambda;
                        worst = j;
                    }
                }
                if (worst < 0) {
                    converged = true;
                    return iteration;
                }
                w [worst] = SetFree;
            }
            return iteration;
        }

        /// <summary>
        /// Solves H y = g in place (g given in y), H symmetric positive definite of size nf,
        /// lower triangle stored with row stride ld. Returns false on a non-positive pivot.
        /// </summary>
        internal static bool CholeskySolve (double[] H, double[] L, double[] y, int nf, int ld)
        {
            for (int j = 0; j < nf; j++) {
                double d = H [j * ld + j];
                for (int k = 0; k < j; k++)
                    d = d - L [j * ld + k] * L [j * ld + k];
                if (!(d > 0.0))
                    return false;
                double ljj = Math.Sqrt (d);
                L [j * ld + j] = ljj;
                for (int i = j + 1; i < nf; i++) {
                    double s = H [i * ld + j];
                    for (int k = 0; k < j; k++)
                        s = s - L [i * ld + k] * L [j * ld + k];
                    L [i * ld + j] = s / ljj;
                }
            }
            // Forward: L z = g.
            for (int i = 0; i < nf; i++) {
                double s = y [i];
                for (int k = 0; k < i; k++)
                    s = s - L [i * ld + k] * y [k];
                y [i] = s / L [i * ld + i];
            }
            // Backward: L^T y = z.
            for (int i = nf - 1; i >= 0; i--) {
                double s = y [i];
                for (int k = i + 1; k < nf; k++)
                    s = s - L [k * ld + i] * y [k];
                y [i] = s / L [i * ld + i];
            }
            return true;
        }

        internal static bool FiniteAll (double[] v, int count)
        {
            if (v == null || v.Length < count)
                return false;
            for (int i = 0; i < count; i++)
                if (double.IsNaN (v [i]) || double.IsInfinity (v [i]))
                    return false;
            return true;
        }

        internal static bool Finite3 (double[] v)
        {
            return v != null && v.Length >= 3 &&
                !double.IsNaN (v [0]) && !double.IsInfinity (v [0]) &&
                !double.IsNaN (v [1]) && !double.IsInfinity (v [1]) &&
                !double.IsNaN (v [2]) && !double.IsInfinity (v [2]);
        }
    }
}
