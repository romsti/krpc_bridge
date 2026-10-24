using System;

namespace KRPC.Bridge.Model
{
    // -----------------------------------------------------------------------------------
    // (born closed, SurfaceModel.ModeFlagKappa): the PHYSICAL validation of SurfaceModelV1,
    // PURE (System only, compiled into build/tests like SurfaceModel.cs).
    //
    // Why. The identity and the scale guard compare the replica with the module IN THE SAME UNIT:
    // they could not see (the fins' torques given to the law in kN.m as N.m: k = 1.000 and an
    // identity of 2.6e-7 in a recorded flight despite the factor 1000). Only the
    // vessel's own measured angular acceleration can: kappa_a, per body axis, is the slope of
    // what the measured acceleration attributes to the fins' deflections over the torque the LAW
    // was given for them -- 1 when the law's unit, levers and frames are right, 1000 for an earlier change.
    //
    // The regression, per body axis a, per counted physics step, aligned with the INDI (the same
    // raw omega_dot, the same H(z) as AttitudeMath, the same realized torques):
    //
    //     y   = I_a (omega_dot_a(t) - omega_dot_a(t-1)) + (g_a(t) - g_a(t-1))
    //           - (M_nonfin,a(t) - M_nonfin,a(t-1)),    g = omega_m x (I omega_m)
    //     x_d = law scale x sum over the modeled surfaces of [T_i(t) - T_i(deflection(t-1), flow(t))]
    //     x_n = 1000 x [(T_aero(t) - T_aero(t-1)) - sum of the same deflection increments]
    //     H(y) = kappa_a H(x_d) + c_a H(x_n) + e
    //
    // M_nonfin is the law's realized torque of the engines, RCS and wheels (N.m), T_i a modeled
    // surface's realized torque (its module's liftForce at CoL, dragForce at CoP, kN.m), T_aero
    // the whole vessel's aerodynamic torque as the game applied it (every part's cube drag, body
    // lift, lifting-surface forces and rigidbody angular drag, kRPC's Flight.AerodynamicTorque,
    // kN.m), all at the capture of the step, body frame, in double. Increments remove every slow
    // bias; H is the law's own low-pass (zero state, restarted with the estimator's input: the
    // filtered sides stay consistent at every restart).
    //
    // The regressors are those the flights require (a recorded flight, shadow, 50 Hz
    // frames, surface_model_v1 captures and the kRPC aero torque of dynamics_v3):
    //   * the fins' torque alone, in LEVELS, gives a slope of 2.5 (x) and 3.5 (z) -- the 2.3-3.4 of
    //     the diagnostic: the hull's aerodynamic torque is correlated 0.80-0.87 with the fins'
    //     (both follow q and the angle of attack, and the AutoPilot's deflections follow them);
    //     with the hull's torque as a second regressor the slope is 0.89 (x) and 0.975 (z) and the
    //     hull's own 1.01-1.02;
    //   * in filtered increments the fins' total torque alone still gives 1.9-2.0 (2.2-4.1 while
    //     the RCS maneuvers in the coast): the confounder is the same, the hull's torque
    //     increment that follows the rotation; with it, 1.025 +- 0.036 (x), 0.973 +- 0.025 (y),
    //     1.005 +- 0.007 (z);
    //   * the fins' FLOW-driven increment (angle of attack, q) is the hull's twin -- with the fins
    //     folded (range 1e-4 deg) it is all there is, and it gave estimates of 1e4 that a
    //     variance estimator alone does not reject. Hence x_d, the DEFLECTION-driven increment
    //     (the replica at the previous deflection and this step's flow, exactly what the law's
    //     columns predict), as kappa's regressor, and the rest of the aerodynamic increment, the
    //     fins' flow part included, as one nuisance regressor with a free coefficient (published:
    //     the hull readout's own scale, ~1);
    //   * the engines' realized torque is 10-100 times the fins' and its errors follow the same
    //     commands (its free slope 1.06-1.13 in that flight): a step counts only once no engine
    //     torque has been in the filter's memory for BatchSteps steps (the boost-back and the
    //     landing burn are left out; the coast, where the fins are proven before they matter,
    //     is not). The RCS and the wheels stay at their realized torque (free slopes 0.99-1.09).
    //
    // Uncertainty and decision, computed (no tuned constant):
    //   * the steps are grouped in batches of BatchSteps = ceil(ln(1 / IdentityTolerance) /
    //     (zeta omega_c dt)) counted steps -- the time the law's H(z) takes to forget an input to
    //     IdentityTolerance (28 steps, 0.56 s, with the plan R2 gains): the filtered residuals are
    //     correlated within a batch, hardly across;
    //   * kappa's standard error is the delete-one-batch JACKKNIFE: SE^2 = (G - 1) / G sum_b
    //     (kappa_(-b) - kappa)^2, kappa_(-b) the fit without batch b, exact from the batch sums.
    //     It is honest with the flight's heterogeneity: a batch that alone carries the estimate
    //     (a fin deployment, the first seconds of high q) makes its own deletion move kappa;
    //   * tolerance = t x SE, t the Student quantile at 1 - IdentityTolerance / 2 with G - 1
    //     degrees of freedom (a decision is wrong once in a thousand, the replica's own tolerance);
    //   * a step counts on an axis only while the modeled surfaces could act on it: their static
    //     authority there (sum |column x gain|, physical N.m) at least IdentityTolerance of the
    //     loop's whole authority on the axis (alpha x I). Folded grid fins (KRE sets their range and
    //     lift coefficient to 1e-4) are 1e-8 of the RCS: their tiny, flow-driven increments follow
    //     the hull's and gave, in the shadow flight's replay, slopes of -5e5 +- 4e4 that a
    //     variance estimator alone would have called "out of tolerance";
    //   * an axis is PROVEN while [kappa - t SE, kappa + t SE] lies inside the proof band
    //     [ProofMin, ProofMax] = [0.707, 1.414]: the scale guard's band [0.5, 2] (the error factor
    //     INDI tolerates) narrowed by a gain margin of sqrt(2),'s limit cycle was born at
    //     kappa ~2.09; it is OUT OF TOLERANCE, for the arming, once that interval lies
    //     entirely outside the band;
    //     both need 3 batches of the axis. A strict |kappa - 1| <= t SE ("consistent with 1") is
    //     published too, as a diagnostic only: with 10^4 steps it would reject the flight's own
    //     1-3 % physics residue (kappa_x 0.985 +- 0.005 from the shadow flight's published
    //     residuals) and never engage anything. The effective number of batches (Kish, on the
    //     deflection regressor's batch weights) is published; an earlier change judges out of tolerance with
    //     the jackknife's own degrees of freedom (below), which fall with it.
    //
    // Shadow first: armed, every surface not engaged is held neutral, and a surface at rest
    // carries no deflection increment -- the evidence comes from the shadow (and from the moves of
    // the engaged surfaces); the mode may go from shadow to armed without losing it.
    //
    // (a recorded flight, shadow, mode 13): body x was declared out of tolerance
    // during the FLIP (17 km, 4 batches, kappa -0.69 [-1.84, 0.46]) and stayed so for the arming,
    // while the whole flight gave 0.955 +- 0.037. Two causes, both fixed here:
    //   * the PHYSICS: the left-hand side lacked the gyroscopic torque. The vessel's measured
    //     acceleration obeys Euler's equation, I omega_dot = M - omega x (I omega): over the flip
    //     (yaw rate up to 1.18 rad/s, roll rate +-0.05 rad/s) the pitch term (I_z - I_y) w_y w_z is
    //     138 kN.m rms; regressing the measured I omega_dot on the realized RCS + wheel torque and
    //     the aero readout leaves 131 kN.m rms on x without it, 3.6 kN.m with it (slope -0.998).
    //     Its increments follow the AutoPilot's pitch answers to that very disturbance, which the
    //     fins' deflections also follow: the slope took them for the fins' (closed-loop bias).
    //     The left-hand side is now I d(omega_dot) + d(omega_m x I omega_m) - d(M), omega_m the
    //     step's mid rate (the one the measured omega_dot spans; the flight's best fit). Replayed
    //     offline on the flight's 50 Hz trace, kappa_x is 1.000 +- 0.001 and proven in the flip;
    //   * the STATISTICS of the irreversible decision: at the trip, the deflection regressor's
    //     batch weights gave 1.3 effective batches (Kish) -- the deployment slew of the grid fins
    //     carried the estimate -- yet the interval used Student with batches - 1 = 3 degrees of
    //     freedom. With unequal weights the delete-one-batch jackknife's variance has fewer
    //     degrees of freedom than that: its Satterthwaite (Bell-McCaffrey) count, exact for the
    //     jackknife of one (partialled) regressor with independent batches, is
    //       nu = (sum p/(1-p))^2 / (sum p^2 (1-2p)/(1-p)^4 + (sum p^2/(1-p)^2)^2),
    //     p the batches' shares of the partialled deflection regressor -- batches - 1 for equal
    //     shares, down to 1 as one batch carries everything (JackknifeDof). With the flight's weights
    //     the t(batches - 1) rule is wrong 2-6 times more often than 1e-3 (Monte Carlo); t(nu) is
    //     not. Out of tolerance is STICKY for the arming (a real defect seen once must not hide
    //     behind later evidence): it is now judged on kappa +- t(nu) SE, never on one episode. A
    //     proof is re-judged at every batch and only engages while it holds, inside a band that
    //     already keeps a sqrt(2) gain margin: it keeps t(batches - 1) (with nu the super
    //     hope's roll, 2.7 effective batches over a whole flight, would never be proven).
    // Since out_low <= kappa_low and out_high >= kappa_high (nu <= batches - 1), an axis is never
    // proven and out at once. Published (schema 2): the out test's dof, quantile and interval, and
    // the rms of the filtered gyroscopic increment; band_min/band_max are the proof band (
    // schema 1 published the scale band [0.5, 2]).
    //
    // (born closed, SurfaceModel.ModeFlagKappaLeak, 16 ORed with 8): the LEAK GATE of the
    // counted steps. The measured side subtracts the engines', RCS' and wheels' torque at its realized
    // value and multiplies by the inertia: a proportional error of either (eps, delta) is collinear with
    // whatever moves together with the surfaces' deflections, and reads as a bias, not as noise --
    //     kappa_hat = kappa (1 + delta) + (delta + eps) beta,   beta = sum d m / sum d^2,
    // d the filtered deflection regressor, m the filtered increment of the non-surface realized torque,
    // over the counted steps (an earlier change's formula; there beta was the column ratio r of a roll command). A
    // command excites the surfaces AND the RCS and wheels; where the surfaces are weak (the flip, the
    // low q of the coast) |beta| is large: (x/z of the coast alone out of tolerance with 1 % of
    // inertia error, in the emulator) and (the roll proven at the flip on steps the surfaces barely
    // carry). With the flag, a step counts on axis a only while |m_a| <= LeakMax |d_a|, LeakMax =
    // 1 / GainMargin^2 = 1/2 (the surfaces carry at least two thirds of the excited torque increment,
    // an earlier change's threshold): then |sum d m| <= sum |d| |m| <= LeakMax sum d^2, |beta| <= 1/2 by construction,
    // and the leak (|delta| + |eps|) |beta| stays within the inertia's own error. Computed on the
    // REALIZED increments (the RCS dead zone, the wheels' lag, every module's real share), not on the
    // columns: the super hope's RCS + wheel roll column is 122 kN.m per unit while a command inside the
    // RCS dead zone (|u| < 0.05) moves the wheels' 30 kN.m only. Replayed on ten flights of the super
    // hope (07/10-09/10, the emulator), the errors on four of them (19h14, 09h12, 12h25, 11h08):
    // RCS +10 % moves kappa_x by 0.009-0.066 without the gate, by 0.001-0.006 with it; wheels +10 %
    // moves kappa_y by 0.006-0.036 without it, 0.002-0.008 with it;
    // x and z are still proven in the flip on all ten flights, the roll on eight; every mutant (x1.5,
    // x0.6, x2.5, x0.4) stays out of tolerance or unproven. The inertia's own error is
    // multiplicative and stays (kappa (1 + delta)): it is not a leak (kappa_y stays 0.92-0.98). d is
    // compared in PHYSICAL N.m (the caller's physicalPerLaw: x1000 without the unit flag 4): the gate
    // must not depend on the very unit kappa judges, or an earlier change's unit would refuse nearly every step and
    // delay its own detection. Published per axis (schema 3): leak_gate, leak_refused (the steps the
    // gate refused), leak_beta (the measured beta of the counted steps, physical, gate or not).
    //
    // (born closed, SurfaceModel.ModeFlagKappaInertia, 32 ORed with 8): the PHYSICAL inertia of the
    // measured side. kappa_y of the super hope stayed 0.967-0.981 with the leak gate (flights 09/10 19h48,
    // 23h04, 10/10 09h29, 12h11) while x and z read 0.998-1.001, and the nuisance slope of y (the game's
    // own aerodynamic torque, not the replica) read the same 0.968-0.979: a factor common to every roll
    // regressor, the left-hand side's. The inertia is Vessel.MOI (VesselPrecalculate.CalculatePhysicsStats,
    // KSP 1.12.5, decompiled): the rotated rigidbody tensors plus m (|r|^2 E - r r^T) with r = rb.position -
    // CoM, the part's ORIGIN, while the physics puts the mass at rb.worldCenterOfMass = origin + R CoMOffset
    // (FlightIntegrator.UpdateMassStats sets rb.centerOfMass = part.CoMOffset at every step; the CoM itself
    // is summed from worldCenterOfMass). The super hope's four KRE titanium grid fins (0.7 t, CoMOffset
    // 0.6 m along the radius: r 1.81 -> 2.41 m) and four structural wings (0.1 t, CoMOffset (-0.7, -0.6,
    // 0): r 1.90 -> 2.60 m) put 8 349 kg.m^2 of roll inertia outside Vessel.MOI (6 234 on pitch and yaw,
    // 0.07 % of theirs): I_y is served 2.6 % low at the flip (316 400), 3.3 % in the coast (249 000).
    // Identified on the same four flights, one flight at a time, from the realized torques alone (I omega_dot
    // + gyro on the wheels', RCS', engines' and the aerodynamic readout's torques, flip and boostback):
    // I_true / I_served 1.022-1.027 on roll, 0.998-1.009 on pitch and yaw; the wheels alone (exact:
    // AddTorque of -inputVector) 1.029-1.033 at the flip (three flights; the fourth, barely excited, 1.00 +-
    // 0.01), the engines alone 1.025-1.027 in the boostback.
    // With the flag, the measured side multiplies by Vessel.MOI + the missing terms, computed at the tick
    // from the same parts (AddInertiaOffset: m [(|c|^2 - c_a^2) - (|o|^2 - o_a^2)], c and o the mass centre
    // and the origin about the CoM, reference frame, rb.mass as the stock sum). The offset is published per
    // axis whatever the flag (schema 4: inertia_physical, inertia_offset_kg_m2); the law's own inertia is
    // untouched.
    //
    // Mono-flight: reset at every arming and whenever the model is switched on or its unit flag
    // changes (INTER-FLIGHT LEARNING FORBIDDEN). A GUARD, never an adaptation: nothing is scaled by
    // kappa. No allocation per step: every array is built with the estimator.
    // -----------------------------------------------------------------------------------

    /// <summary>constants, the batch length and the Student quantile (pure maths).</summary>
    internal static class SurfaceKappa
    {
        internal const int Protocol = 1;
        /// <summary>
        /// 4: per axis, appended: inertia_physical, inertia_offset_kg_m2. 3: per
        /// axis, appended: leak_gate, leak_refused, leak_beta. 2:
        /// band_min/band_max are the proof band; per axis, appended: out_dof, out_t_quantile, out_low,
        /// out_high, rms_gyro_nm. Schema 1 (6015f9c): 22 per axis, the scale band.
        /// </summary>
        internal const int Schema = 4;
        internal const int HeaderStride = 26;
        internal const int AxisStride = 32;
        internal const int ClockStride = 1;
        internal const int AxisCount = 3;

        /// <summary>kappa's regressor and the nuisance (the rest of the aerodynamic increment).</summary>
        internal const int Regressors = 2;
        /// <summary>Batches kept per axis; when full, adjacent pairs are merged (batch length doubled).</summary>
        internal const int BatchCapacity = 512;
        /// <summary>Per batch: sum dd, dn, nn, dy, ny (d the deflection regressor, n the nuisance).</summary>
        internal const int BatchSums = 5;

        internal const int StateNotProven = 0;
        internal const int StateProven = 1;
        internal const int StateOut = 2;

        /// <summary>
        /// (07/10): the proof band. The scale guard's band [ScaleMin, ScaleMax] = [0.5, 2] is the error
        /// factor INDI tolerates, and the limit cycle of an earlier change was born at kappa ~2.09, right at its edge. Kappa
        /// is judged (proven, or out of tolerance) against that band narrowed by a gain margin of sqrt(2) on
        /// each side: [ScaleMin sqrt 2, ScaleMax / sqrt 2] = [0.707, 1.414]. The kappas measured in flight are
        /// 0.97-1.03, so the margin costs nothing and refuses a replica wrong by a factor 1.5-2.
        /// </summary>
        internal static readonly double GainMargin = Math.Sqrt (2.0);
        internal static readonly double ProofMin = SurfaceModel.ScaleMin * GainMargin;
        internal static readonly double ProofMax = SurfaceModel.ScaleMax / GainMargin;

        /// <summary>
        /// the largest share of the non-surface realized torque increment in the excited one,
        /// relative to the surfaces' (|m| / |d|), for a counted step under the leak gate: 1 / GainMargin^2 =
        /// 1/2, the surfaces carrying at least two thirds -- an earlier change's LeakMax (SurfaceKappaRoll.LeakMax).
        /// </summary>
        internal static readonly double LeakMax = 1.0 / (GainMargin * GainMargin);

        /// <summary>
        /// a step's increments pass the leak gate on an axis: |m| &lt;= LeakMax |d| (d the
        /// filtered deflection regressor, m the filtered non-surface realized increment). False for
        /// non-finite inputs.
        /// </summary>
        internal static bool LeakPasses (double d, double m)
        {
            double ad = Math.Abs (d), am = Math.Abs (m);
            if (double.IsNaN (ad) || double.IsNaN (am) || double.IsInfinity (ad) || double.IsInfinity (am))
                return false;
            return am <= LeakMax * ad;
        }

        /// <summary>
        /// adds to `offset` (body axes, the unit of `mass` times m^2) what one rigidbody's mass puts on
        /// the diagonal of the inertia about the CoM at its mass centre `centre` and not at its origin
        /// `origin` (both about the CoM, body frame, m): mass [(|c|^2 - c_a^2) - (|o|^2 - o_a^2)], the
        /// parallel-axis term the physics has and Vessel.MOI's sum (taken at rb.position) has not. The
        /// rotated tensors of the two sums are the same and cancel. False (nothing added) for non-finite
        /// inputs or a negative mass.
        /// </summary>
        internal static bool AddInertiaOffset (double mass, double[] centre, double[] origin, double[] offset)
        {
            if (!(mass >= 0.0) || double.IsInfinity (mass))
                return false;
            double c2 = 0.0, o2 = 0.0;
            for (int a = 0; a < 3; a++) {
                c2 += centre [a] * centre [a];
                o2 += origin [a] * origin [a];
            }
            if (double.IsNaN (c2) || double.IsInfinity (c2) || double.IsNaN (o2) || double.IsInfinity (o2))
                return false;
            for (int a = 0; a < 3; a++)
                offset [a] += mass * ((c2 - centre [a] * centre [a]) - (o2 - origin [a] * origin [a]));
            return true;
        }

        /// <summary>The probability that a decision is wrong: the replica's own tolerance (1e-3).</summary>
        internal static double FalseAlarm {
            get { return SurfaceModel.IdentityTolerance; }
        }

        /// <summary>
        /// Counted steps per batch: the steps the law's H(z) (Tustin, omega_c, zeta) needs for the
        /// envelope of its impulse response, exp(-zeta omega_c t), to fall below FalseAlarm. 28 with
        /// the plan R2 gains (18 rad/s, 0.7) at 0.02 s. At least 1; 0 for unusable inputs.
        /// </summary>
        internal static int BatchSteps (double dt, double omegaC, double zeta)
        {
            double rate = zeta * omegaC * dt;
            if (!(rate > 0.0) || double.IsInfinity (rate))
                return 0;
            double steps = Math.Ceiling (Math.Log (1.0 / FalseAlarm) / rate);
            if (!(steps >= 1.0))
                return 1;
            return steps > 1e6 ? 1000000 : (int)steps;
        }

        /// <summary>
        /// the degrees of freedom of the delete-one-batch jackknife variance of the slope (its
        /// Satterthwaite / Bell-McCaffrey count, independent batches): with p_b each batch's share of
        /// the regressor's weight, nu = (sum p/(1-p))^2 / (sum p^2 (1-2p)/(1-p)^4 + (sum p^2/(1-p)^2)^2).
        /// count - 1 for equal shares, down to 1 as one batch carries nearly everything (the deletion
        /// of that batch is then the whole variance: one chi-square), never above count - 1; 0 when a
        /// single batch carries it all or nothing is usable. The weight of batch b
        /// is that of the deflection regressor partialled out of the nuisance (Frisch-Waugh, `beta` the
        /// whole fit's dn / nn: dd - 2 beta dn + beta^2 nn), or dd alone when `partial` is false (one
        /// regressor). Reads the batch sums in place: no allocation.
        /// </summary>
        internal static double JackknifeDof (double[] batches, int count, double beta, bool partial)
        {
            double total = 0.0;
            for (int b = 0; b < count; b++)
                total += BatchWeight (batches, b, beta, partial);
            if (!(total > 0.0) || double.IsInfinity (total))
                return 0.0;
            double s1 = 0.0, s2 = 0.0, s3 = 0.0;
            for (int b = 0; b < count; b++) {
                double p = BatchWeight (batches, b, beta, partial) / total;
                double q = 1.0 - p;
                if (!(q > 0.0))
                    return 0.0;
                double r = p / q;
                s1 += r;
                s2 += r * r * (1.0 - 2.0 * p) / (q * q);
                s3 += r * r;
            }
            double den = s2 + s3 * s3;
            double nu = den > 0.0 ? s1 * s1 / den : 0.0;
            return double.IsNaN (nu) || double.IsInfinity (nu) ? 0.0 : Math.Min (nu, count - 1.0);
        }

        static double BatchWeight (double[] batches, int b, double beta, bool partial)
        {
            int at = b * BatchSums;
            double w = partial ? batches [at] - 2.0 * beta * batches [at + 1] + beta * beta * batches [at + 2] : batches [at];
            return w > 0.0 ? w : 0.0;
        }

        // ------------------------------------------------------------- Student's t

        /// <summary>
        /// The p-quantile of Student's t with nu (real, > 0) degrees of freedom, p in (0.5, 1):
        /// bisection on the distribution function (regularized incomplete beta). NaN outside.
        /// </summary>
        internal static double StudentQuantile (double p, double nu)
        {
            if (!(p > 0.5 && p < 1.0) || !(nu > 0.0) || double.IsInfinity (nu))
                return double.NaN;
            double lo = 0.0, hi = 1.0;
            for (int i = 0; i < 2000 && StudentCdf (hi, nu) < p; i++)
                hi *= 2.0;
            for (int i = 0; i < 200; i++) {
                double mid = 0.5 * (lo + hi);
                if (StudentCdf (mid, nu) < p)
                    lo = mid;
                else
                    hi = mid;
                if (hi - lo <= 1e-12 * hi)
                    break;
            }
            return 0.5 * (lo + hi);
        }

        /// <summary>P(T &lt;= t), T Student with nu degrees of freedom.</summary>
        internal static double StudentCdf (double t, double nu)
        {
            if (double.IsNaN (t) || !(nu > 0.0))
                return double.NaN;
            double x = nu / (nu + t * t);
            double tail = 0.5 * RegularizedBeta (0.5 * nu, 0.5, x);
            return t >= 0.0 ? 1.0 - tail : tail;
        }

        /// <summary>I_x(a, b), Numerical Recipes' continued fraction (modified Lentz).</summary>
        internal static double RegularizedBeta (double a, double b, double x)
        {
            if (!(x > 0.0))
                return 0.0;
            if (!(x < 1.0))
                return 1.0;
            double front = Math.Exp (LogGamma (a + b) - LogGamma (a) - LogGamma (b) +
                a * Math.Log (x) + b * Math.Log (1.0 - x));
            if (x < (a + 1.0) / (a + b + 2.0))
                return front * BetaFraction (a, b, x) / a;
            return 1.0 - front * BetaFraction (b, a, 1.0 - x) / b;
        }

        static double BetaFraction (double a, double b, double x)
        {
            const double tiny = 1e-300, eps = 1e-15;
            double qab = a + b, qap = a + 1.0, qam = a - 1.0;
            double c = 1.0, d = 1.0 - qab * x / qap;
            if (Math.Abs (d) < tiny)
                d = tiny;
            d = 1.0 / d;
            double h = d;
            for (int m = 1; m <= 1000; m++) {
                int m2 = 2 * m;
                double aa = m * (b - m) * x / ((qam + m2) * (a + m2));
                d = 1.0 + aa * d;
                if (Math.Abs (d) < tiny)
                    d = tiny;
                c = 1.0 + aa / c;
                if (Math.Abs (c) < tiny)
                    c = tiny;
                d = 1.0 / d;
                h *= d * c;
                aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2));
                d = 1.0 + aa * d;
                if (Math.Abs (d) < tiny)
                    d = tiny;
                c = 1.0 + aa / c;
                if (Math.Abs (c) < tiny)
                    c = tiny;
                d = 1.0 / d;
                double delta = d * c;
                h *= delta;
                if (Math.Abs (delta - 1.0) < eps)
                    break;
            }
            return h;
        }

        static readonly double[] lanczos = {
            0.99999999999980993, 676.5203681218851, -1259.1392167224028, 771.32342877765313,
            -176.61502916214059, 12.507343278686905, -0.13857109526572012, 9.9843695780195716e-6,
            1.5056327351493116e-7
        };

        /// <summary>ln Gamma(x), x &gt; 0 (Lanczos, g = 7, nine coefficients).</summary>
        internal static double LogGamma (double x)
        {
            if (x < 0.5)
                return Math.Log (Math.PI / Math.Abs (Math.Sin (Math.PI * x))) - LogGamma (1.0 - x);
            x -= 1.0;
            double s = lanczos [0];
            for (int i = 1; i < 9; i++)
                s += lanczos [i] / (x + i);
            double t = x + 7.5;
            return 0.5 * Math.Log (2.0 * Math.PI) + (x + 0.5) * Math.Log (t) - t + Math.Log (s);
        }

        // ------------------------------------------------------------- the 2 x 2 fit

        /// <summary>
        /// The least-squares slopes of y on (d, n) from the sums dd, dn, nn, dy, ny (in `s`, at
        /// `at`). With a zero nuisance (vacuum) or collinear regressors, y on d alone (slope n 0,
        /// regressors 1). False when the deflection regressor carries nothing.
        /// </summary>
        internal static bool Solve (double[] s, int at, out double kappa, out double nuisance, out int regressors)
        {
            double dd = s [at], dn = s [at + 1], nn = s [at + 2], dy = s [at + 3], ny = s [at + 4];
            kappa = double.NaN;
            nuisance = 0.0;
            regressors = 0;
            if (!(dd > 0.0) || double.IsInfinity (dd))
                return false;
            double det = dd * nn - dn * dn;
            if (nn > 0.0 && det > 1e-12 * dd * nn) {
                kappa = (nn * dy - dn * ny) / det;
                nuisance = (dd * ny - dn * dy) / det;
                regressors = 2;
            } else {
                kappa = dy / dd;
                regressors = 1;
            }
            return !double.IsNaN (kappa) && !double.IsInfinity (kappa);
        }
    }

    /// <summary>one body axis of the estimator -- its batches and its verdict.</summary>
    internal sealed class SurfaceKappaAxis
    {
        internal readonly double[] Batches = new double[SurfaceKappa.BatchCapacity * SurfaceKappa.BatchSums];
        internal int Count;                                       // closed batches
        internal readonly double[] Current = new double[SurfaceKappa.BatchSums];
        internal readonly double[] Total = new double[SurfaceKappa.BatchSums];   // closed batches only
        internal double SumYy, SumYyCurrent;
        internal double SumGg, SumGgCurrent;                     // the filtered gyroscopic increment
        internal long Steps, StepsCurrent;
        readonly double[] scratch = new double[SurfaceKappa.BatchSums];

        // Verdict (after each closed batch).
        internal double Kappa = double.NaN, Se = double.NaN, T = double.NaN;
        internal double Nuisance = double.NaN, NuisanceSe = double.NaN, EffectiveBatches = double.NaN;
        internal double RmsY = double.NaN, RmsFins = double.NaN, RmsNuisance = double.NaN, RmsResidual = double.NaN;
        // the out-of-tolerance test, on the jackknife's own degrees of freedom.
        internal double OutDof = double.NaN, OutT = double.NaN, OutLow = double.NaN, OutHigh = double.NaN;
        internal double RmsGyro = double.NaN;
        internal int RegressorsUsed;
        internal bool Proven, Tripped, Consistent;
        // the leak of the counted steps -- sum d m and sum d^2 (d the filtered deflection
        // regressor, m the filtered non-surface realized increment), and the steps the gate refused.
        internal double SumDm, SumDd;
        internal long LeakRefused;

        /// <summary>beta = sum d m / sum d^2 over the counted steps (NaN before any).</summary>
        internal double LeakBeta {
            get { return SumDd > 0.0 ? SumDm / SumDd : double.NaN; }
        }

        internal void Reset ()
        {
            Count = 0;
            for (int k = 0; k < SurfaceKappa.BatchSums; k++) {
                Current [k] = 0.0;
                Total [k] = 0.0;
            }
            SumYy = SumYyCurrent = 0.0;
            SumGg = SumGgCurrent = 0.0;
            SumDm = SumDd = 0.0;
            LeakRefused = 0;
            Steps = StepsCurrent = 0;
            Kappa = Se = T = Nuisance = NuisanceSe = EffectiveBatches = double.NaN;
            RmsY = RmsFins = RmsNuisance = RmsResidual = double.NaN;
            OutDof = OutT = OutLow = OutHigh = RmsGyro = double.NaN;
            RegressorsUsed = 0;
            Proven = Tripped = Consistent = false;
        }

        internal int State {
            get {
                return Tripped ? SurfaceKappa.StateOut : Proven ? SurfaceKappa.StateProven : SurfaceKappa.StateNotProven;
            }
        }

        /// <summary>
        /// One counted step: filtered deflection regressor d, nuisance n, left-hand side y, and the
        /// filtered gyroscopic increment g that y includes (published only).
        /// </summary>
        internal void Add (double d, double n, double y, double g)
        {
            Current [0] += d * d;
            Current [1] += d * n;
            Current [2] += n * n;
            Current [3] += d * y;
            Current [4] += n * y;
            SumYyCurrent += y * y;
            SumGgCurrent += g * g;
            StepsCurrent++;
        }

        /// <summary>the leak sums of a counted step (published only, nothing decided on them).</summary>
        internal void AddLeak (double d, double m)
        {
            SumDm += d * m;
            SumDd += d * d;
        }

        /// <summary>
        /// Closes the current batch (nothing is stored when the axis counted no step in it); true
        /// when the store was full and every pair was merged.
        /// </summary>
        internal bool Close ()
        {
            if (StepsCurrent == 0)
                return false;
            int at = Count * SurfaceKappa.BatchSums;
            for (int k = 0; k < SurfaceKappa.BatchSums; k++) {
                Batches [at + k] = Current [k];
                Total [k] += Current [k];
                Current [k] = 0.0;
            }
            SumYy += SumYyCurrent;
            SumYyCurrent = 0.0;
            SumGg += SumGgCurrent;
            SumGgCurrent = 0.0;
            Steps += StepsCurrent;
            StepsCurrent = 0;
            Count++;
            if (Count < SurfaceKappa.BatchCapacity)
                return false;
            // Full: adjacent batches merged (still valid clusters, twice as long).
            int half = Count / 2;
            for (int b = 0; b < half; b++)
                for (int k = 0; k < SurfaceKappa.BatchSums; k++)
                    Batches [b * SurfaceKappa.BatchSums + k] = Batches [2 * b * SurfaceKappa.BatchSums + k] +
                        Batches [(2 * b + 1) * SurfaceKappa.BatchSums + k];
            Count = half;
            return true;
        }

        /// <summary>The fit, its jackknife standard error, t (Student, Count - 1) and the verdict.</summary>
        internal void Evaluate ()
        {
            double t = Count >= 2 ? SurfaceKappa.StudentQuantile (1.0 - 0.5 * SurfaceKappa.FalseAlarm, Count - 1.0) :
                double.NaN;
            T = t;
            double kappa, nuisance;
            int regressors;
            if (!SurfaceKappa.Solve (Total, 0, out kappa, out nuisance, out regressors)) {
                Kappa = Se = Nuisance = NuisanceSe = double.NaN;
                OutDof = OutT = OutLow = OutHigh = double.NaN;
                RegressorsUsed = 0;
                Proven = false;
                Consistent = false;
                Rms (double.NaN, 0.0);
                return;
            }
            Kappa = kappa;
            Nuisance = regressors == 2 ? nuisance : double.NaN;
            RegressorsUsed = regressors;
            Rms (kappa, regressors == 2 ? nuisance : 0.0);
            // Delete-one-batch jackknife, exact from the batch sums.
            double vk = 0.0, vn = 0.0, sumW = 0.0, sumW2 = 0.0;
            bool finite = Count >= 2;
            for (int b = 0; b < Count && finite; b++) {
                int at = b * SurfaceKappa.BatchSums;
                for (int k = 0; k < SurfaceKappa.BatchSums; k++)
                    scratch [k] = Total [k] - Batches [at + k];
                if (regressors == 1) {
                    // The same model without the nuisance (it carried nothing in the whole fit).
                    scratch [1] = 0.0;
                    scratch [2] = 0.0;
                    scratch [4] = 0.0;
                }
                double kb, nb;
                int rb;
                if (!SurfaceKappa.Solve (scratch, 0, out kb, out nb, out rb) || rb != regressors) {
                    finite = false;
                    break;
                }
                vk += (kb - kappa) * (kb - kappa);
                vn += (nb - nuisance) * (nb - nuisance);
                double w = Batches [at];
                sumW += w;
                sumW2 += w * w;
            }
            double factor = Count >= 2 ? (Count - 1.0) / Count : double.NaN;
            Se = finite ? Math.Sqrt (factor * vk) : double.PositiveInfinity;
            NuisanceSe = finite && regressors == 2 ? Math.Sqrt (factor * vn) : double.NaN;
            EffectiveBatches = sumW2 > 0.0 ? sumW * sumW / sumW2 : 0.0;
            // the jackknife's own degrees of freedom, from the batches' shares of the
            // deflection regressor partialled out of the nuisance (that of kappa in the fit).
            OutDof = finite ? SurfaceKappa.JackknifeDof (Batches, Count, regressors == 2 ? Total [1] / Total [2] : 0.0,
                regressors == 2) : 0.0;
            OutT = OutDof > 0.0 ? SurfaceKappa.StudentQuantile (1.0 - 0.5 * SurfaceKappa.FalseAlarm, OutDof) : double.NaN;
            double tolerance = t * Se;
            double outTolerance = OutT * Se;
            OutLow = kappa - outTolerance;
            OutHigh = kappa + outTolerance;
            Consistent = !double.IsNaN (tolerance) && Math.Abs (kappa - 1.0) <= tolerance;
            // Out of tolerance (sticky for the arming): the interval of the jackknife's own degrees
            // of freedom entirely outside the proof band -- never on evidence one batch carries.
            bool judgedOut = Count >= 3 && !double.IsNaN (outTolerance) && !double.IsInfinity (outTolerance);
            if (judgedOut && (OutLow > SurfaceKappa.ProofMax || OutHigh < SurfaceKappa.ProofMin))
                Tripped = true;
            bool judged = Count >= 3 && !double.IsNaN (tolerance) && !double.IsInfinity (tolerance);
            if (!judged) {
                Proven = false;
                return;
            }
            double lo = kappa - tolerance, hi = kappa + tolerance;
            Proven = !Tripped && lo >= SurfaceKappa.ProofMin && hi <= SurfaceKappa.ProofMax;
        }

        void Rms (double kappa, double nuisance)
        {
            if (Steps <= 0) {
                RmsY = RmsFins = RmsNuisance = RmsResidual = RmsGyro = double.NaN;
                return;
            }
            double n = Steps;
            RmsY = Math.Sqrt (SumYy / n);
            RmsGyro = Math.Sqrt (SumGg / n);
            RmsFins = Math.Sqrt (Total [0] / n);
            RmsNuisance = Math.Sqrt (Total [2] / n);
            if (double.IsNaN (kappa)) {
                RmsResidual = double.NaN;
                return;
            }
            // sum (y - k d - c n)^2 = yy - 2 k dy - 2 c ny + k^2 dd + 2 k c dn + c^2 nn
            double ssr = SumYy - 2.0 * kappa * Total [3] - 2.0 * nuisance * Total [4] + kappa * kappa * Total [0] +
                2.0 * kappa * nuisance * Total [1] + nuisance * nuisance * Total [2];
            RmsResidual = Math.Sqrt (Math.Max (0.0, ssr) / n);
        }
    }

    /// <summary>
    /// the per-vessel estimator (three body axes), fed once per attitude tick with the INDI's
    /// measurement and the capture of the previous physics step (see the file header).
    /// </summary>
    internal sealed class SurfaceKappaEstimator
    {
        internal readonly SurfaceKappaAxis[] Axes = {
            new SurfaceKappaAxis (), new SurfaceKappaAxis (), new SurfaceKappaAxis ()
        };
        readonly Biquad3 filterY = new Biquad3 (), filterD = new Biquad3 (), filterN = new Biquad3 ();
        readonly Biquad3 filterG = new Biquad3 ();             // the gyroscopic increment (published)
        readonly Biquad3 filterM = new Biquad3 ();             // the non-surface realized increment
        readonly double[] zero = new double[3];
        readonly double[] inY = new double[3], inD = new double[3], inN = new double[3], inG = new double[3];
        readonly double[] outY = new double[3], outD = new double[3], outN = new double[3], outG = new double[3];
        readonly double[] inM = new double[3], outM = new double[3];
        readonly double[] previousOmegaDot = new double[3], previousNonFin = new double[3];
        // the body rate of the last tick and the gyroscopic torque of the last step.
        readonly double[] previousOmega = new double[3], previousGyro = new double[3], gyro = new double[3];
        bool havePrevious, previousEngine, primed, previousGyroValid;
        long previousTick = long.MinValue, previousOmegaTick = long.MinValue;
        int batchFill;

        internal int BaseBatchSteps;      // computed at the first usable step of an arming
        internal int BatchSteps;          // doubled at every merge
        /// <summary>counted steps in the batch being filled (0 right after a batch closed).</summary>
        internal int BatchFill {
            get { return batchFill; }
        }
        internal long EngineQuietSteps;   // counted steps since an engine torque was in the filters
        internal long StepsSeen, StepsCounted, Restarts;
        /// <summary>the axes (bits 0-2) the last step was counted on (after the leak gate; 0 when not counted).</summary>
        internal int LastCountedAxes;
        internal long LastTick = -1;
        internal double LastUt = double.NaN;

        internal void Reset ()
        {
            for (int a = 0; a < 3; a++)
                Axes [a].Reset ();
            havePrevious = previousEngine = primed = previousGyroValid = false;
            previousTick = previousOmegaTick = long.MinValue;
            batchFill = 0;
            BaseBatchSteps = BatchSteps = 0;
            EngineQuietSteps = 0;
            StepsSeen = StepsCounted = Restarts = 0;
            LastCountedAxes = 0;
            LastTick = -1;
            LastUt = double.NaN;
            filterY.Reset ();
            filterD.Reset ();
            filterN.Reset ();
            filterG.Reset ();
            filterM.Reset ();
        }

        /// <summary>Axes (bits 0-2) proven now and not out of tolerance.</summary>
        internal int ProvenAxes ()
        {
            int axes = 0;
            for (int a = 0; a < 3; a++)
                if (Axes [a].Proven && !Axes [a].Tripped)
                    axes |= 1 << a;
            return axes;
        }

        /// <summary>Axes (bits 0-2) out of tolerance for the arming.</summary>
        internal int TrippedAxes ()
        {
            int axes = 0;
            for (int a = 0; a < 3; a++)
                if (Axes [a].Tripped)
                    axes |= 1 << a;
            return axes;
        }

        /// <summary>
        /// the gyroscopic torque omega x (I omega) (N.m, body, diagonal inertia) at the rate `w`.
        /// </summary>
        internal static void Gyroscopic (double[] w, double[] inertia, double[] g)
        {
            double lx = inertia [0] * w [0], ly = inertia [1] * w [1], lz = inertia [2] * w [2];
            g [0] = w [1] * lz - w [2] * ly;
            g [1] = w [2] * lx - w [0] * lz;
            g [2] = w [0] * ly - w [1] * lx;
        }

        /// <summary>
        /// One attitude tick. `measured`: the law's omega_dot is valid (consecutive ticks, no warp, no
        /// topology change); `omega`: the vessel's body rate this tick (rad/s, the one omega_dot
        /// differences); `increment`: the capture of the previous physics step brought both
        /// increments (deflection, law unit; nuisance, N.m), each body frame. nonFin: the law's
        /// engine + RCS + wheel torque (N.m); engine: an engine torque in it; axes (bits 0-2): the
        /// axes the surfaces can act on this step (their authority there, see the file header).
        /// the left-hand side adds the increment of the gyroscopic torque at the step's mid
        /// rate, (omega(t) + omega(t-1)) / 2 -- Euler's equation, I omega_dot = M - omega x I omega;
        /// a step needs the rates of the three ticks t-2, t-1, t. `leakGate` (born false)
        /// counts an axis only while the filtered non-surface increment is at most LeakMax of the
        /// filtered deflection regressor in PHYSICAL N.m (SurfaceKappa.LeakPasses; `physicalPerLaw`
        /// converts the law's unit to N.m: 1 with an earlier change's flag, 1000 without -- the gate must not depend
        /// on the very unit kappa judges, or an earlier change's unit would refuse nearly every step and delay its own
        /// detection); refused steps are counted per axis (LeakRefused); the leak sums of the counted
        /// steps are kept whatever the gate (leak_beta, physical). Returns whether the step was counted
        /// (on the axes the gate let through). No allocation.
        /// </summary>
        internal bool Step (long tick, double ut, bool measured, double dt, double omegaC, double zeta,
            double[] omega, double[] omegaDot, double[] inertia, double[] nonFin, bool engine, bool increment,
            double[] deflection, double[] nuisance, int axes, bool leakGate = false, double physicalPerLaw = 1.0)
        {
            StepsSeen++;
            LastCountedAxes = 0;
            LastTick = tick;
            LastUt = ut;
            bool engineNow = engine || previousEngine;   // a cut-off is an increment too
            previousEngine = engine;
            bool consecutive = havePrevious && tick == previousTick + 1;
            // this step's gyroscopic torque, from this tick's rate and the last tick's.
            bool omegaFinite = Finite (omega);
            bool gyroValid = omegaFinite && Finite (inertia) && previousOmegaTick != long.MinValue &&
                tick == previousOmegaTick + 1;
            if (gyroValid) {
                for (int a = 0; a < 3; a++)
                    inG [a] = 0.5 * (omega [a] + previousOmega [a]);   // the mid rate (inG as scratch)
                Gyroscopic (inG, inertia, gyro);
                gyroValid = Finite (gyro);
            }
            bool usable = measured && consecutive && increment && gyroValid && previousGyroValid &&
                Finite (omegaDot) && Finite (inertia) && Finite (nonFin) && Finite (deflection) && Finite (nuisance) &&
                dt > 0.0;
            if (measured && Finite (omegaDot) && Finite (nonFin)) {
                if (usable) {
                    for (int a = 0; a < 3; a++) {
                        double g = gyro [a] - previousGyro [a];
                        inY [a] = inertia [a] * (omegaDot [a] - previousOmegaDot [a]) + g - (nonFin [a] - previousNonFin [a]);
                        inG [a] = g;
                        inD [a] = deflection [a];
                        inN [a] = nuisance [a];
                        inM [a] = nonFin [a] - previousNonFin [a];   // an earlier change
                    }
                }
                for (int a = 0; a < 3; a++) {
                    previousOmegaDot [a] = omegaDot [a];
                    previousNonFin [a] = nonFin [a];
                }
                havePrevious = true;
                previousTick = tick;
            } else {
                havePrevious = false;
            }
            // The rate and gyroscopic history, whatever the measurement.
            for (int a = 0; a < 3; a++)
                previousGyro [a] = gyroValid ? gyro [a] : 0.0;
            previousGyroValid = gyroValid;
            if (omegaFinite) {
                for (int a = 0; a < 3; a++)
                    previousOmega [a] = omega [a];
                previousOmegaTick = tick;
            } else {
                previousOmegaTick = long.MinValue;
            }
            if (!usable) {
                primed = false;         // the next usable step restarts every filter from zero
                return false;
            }
            if (BaseBatchSteps <= 0) {
                BaseBatchSteps = SurfaceKappa.BatchSteps (dt, omegaC, zeta);
                BatchSteps = BaseBatchSteps;
                if (BaseBatchSteps <= 0)
                    return false;
            }
            if (!primed || filterY.Dt != dt || filterY.OmegaC != omegaC || filterY.Zeta != zeta) {
                // Zero state: the three filtered sides start together, from nothing.
                Restart (filterY, dt, omegaC, zeta);
                Restart (filterD, dt, omegaC, zeta);
                Restart (filterN, dt, omegaC, zeta);
                Restart (filterG, dt, omegaC, zeta);
                Restart (filterM, dt, omegaC, zeta);
                primed = true;
                Restarts++;
                // Nothing older is in the filters: only this step's engine increment can be.
                EngineQuietSteps = engineNow ? 0 : BaseBatchSteps;
            } else if (engineNow) {
                EngineQuietSteps = 0;
            } else {
                EngineQuietSteps++;
            }
            filterY.Step (inY, outY);
            filterD.Step (inD, outD);
            filterN.Step (inN, outN);
            filterG.Step (inG, outG);
            filterM.Step (inM, outM);
            if (EngineQuietSteps < BaseBatchSteps)
                return false;
            for (int a = 0; a < 3; a++) {
                if ((axes & (1 << a)) == 0)
                    continue;
                // (born closed): the surfaces must carry at least 2/3 of the excited increment
                // (both in physical N.m: H is linear, the unit's factor commutes with it).
                double physicalD = outD [a] * physicalPerLaw;
                if (leakGate && !SurfaceKappa.LeakPasses (physicalD, outM [a])) {
                    Axes [a].LeakRefused++;
                    continue;
                }
                Axes [a].Add (outD [a], outN [a], outY [a], outG [a]);
                Axes [a].AddLeak (physicalD, outM [a]);
                LastCountedAxes |= 1 << a;
            }
            StepsCounted++;
            if (++batchFill >= BatchSteps) {
                batchFill = 0;
                bool merged = false;
                for (int a = 0; a < 3; a++) {
                    bool closed = Axes [a].StepsCurrent > 0;
                    merged |= Axes [a].Close ();
                    if (closed)
                        Axes [a].Evaluate ();
                }
                if (merged)
                    BatchSteps *= 2;
            }
            return true;
        }

        void Restart (Biquad3 f, double dt, double omegaC, double zeta)
        {
            f.Configure (dt, omegaC, zeta);
            f.Reset ();
            f.Step (zero, outY);       // primes the steady state at zero (outY is overwritten next)
        }

        static bool Finite (double[] v)
        {
            for (int i = 0; i < v.Length; i++)
                if (double.IsNaN (v [i]) || double.IsInfinity (v [i]))
                    return false;
            return true;
        }
    }
}
