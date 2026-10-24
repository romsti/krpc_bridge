using System;

namespace KRPC.Bridge.Model
{
    /// <summary>
    /// K2 (option 128 of AttitudeOptionsV1, born closed): the thrust a gimbal command will act
    /// with. Pure maths (System only): compiled as is by build/tests (GimbalTrimTests.cs).
    ///
    /// WHY. The gimbal columns are proportional to the engine's finalThrust. At a spool start the
    /// thrust is a few percent of what it will be when the gimbal has moved: the law allocates on
    /// those small columns, asks deflections the coming thrust does not need, and the thrust arrives
    /// with the gimbals at their stops (the heavy booster's descent ignition: 1.0-3.7 MN.m for
    /// 0.4-0.9 s, |omega| times 2 to 4).
    ///
    /// WHAT KSP DOES (Assembly-CSharp 1.12.5, decompiled through assembly inspection):
    ///   * ModuleEngines.UpdateThrottle, every physics step: with useEngineResponseTime,
    ///     currentThrottle = Mathf.Lerp (currentThrottle, requestedThrottle, speed * fixedDeltaTime),
    ///     speed = engineAccelerationSpeed going up, engineDecelerationSpeed going down, set to
    ///     requestedThrottle once within 1e-4; without it, currentThrottle = requestedThrottle. Mathf.Lerp
    ///     clamps its factor to [0, 1]. An engine with throttleUseAlternate follows another rate law
    ///     (protected rates): it is taken as reaching requestedThrottle at once, the upper bound;
    ///   * finalThrust follows currentThrottle (CalculateThrust; the lateral-force layer already reads
    ///     thrust = K currentThrottle, K = finalThrust / currentThrottle);
    ///   * ModuleGimbal.FixedUpdate moves each pivot by a Lerp of factor gimbalResponseSpeed *
    ///     fixedDeltaTime per step toward its target (AttitudeTranslation.RampFactor), the bridge's lease
    ///     the same.
    ///
    /// THE HORIZON. A gimbal command reaches 1 - 1/e of its way after the time constant of that Lerp,
    /// -1 / ln(1 - f) steps (f the per-step factor); an instant gimbal (no response speed, or f = 1)
    /// acts at the next step. The columns are evaluated at the thrust of that horizon: the effectiveness
    /// at the time the command takes effect. Never below the thrust now: in a spool DOWN the realized
    /// columns are kept, so that the true effectiveness over the modelled one stays in (0, 1], where the
    /// incremental law converges without overshoot.
    /// </summary>
    internal static class GimbalSpool
    {
        /// <summary>KSP's snap of currentThrottle onto requestedThrottle (UpdateThrottle).</summary>
        internal const double ThrottleSnap = 1e-4;

        /// <summary>
        /// Below this currentThrottle the thrust per unit of throttle is not read from finalThrust (the
        /// lateral-force layer's own convention, AttitudeV1Translation.CollectMotifEngines).
        /// </summary>
        internal const double ThrottleReadable = 1e-3;

        /// <summary>
        /// The horizon in physics steps of a gimbal whose per-step Lerp factor is rampFactor
        /// (AttitudeTranslation.RampFactor): -1 / ln(1 - f), the time constant; 1 for an instant gimbal
        /// (f >= 1); +infinity for a gimbal that does not move (f &lt;= 0, or not finite).
        /// </summary>
        internal static double HorizonSteps (double rampFactor)
        {
            if (!(rampFactor > 0.0))
                return double.PositiveInfinity;
            if (rampFactor >= 1.0)
                return 1.0;
            return -1.0 / Math.Log (1.0 - rampFactor);
        }

        /// <summary>
        /// The throttle the stock spool reaches after `steps` physics steps (closed form of the Lerp,
        /// continuous in steps): requested + (current - requested) (1 - r)^steps, r = speed * dt clamped
        /// to [0, 1]; requested at once without the response time, with the alternate law, within the
        /// snap, or after an infinite horizon. A non-finite input returns current.
        /// </summary>
        internal static double ExpectedThrottle (double current, double requested, bool responseTime,
            bool alternate, double accelerationSpeed, double decelerationSpeed, double dt, double steps)
        {
            if (double.IsNaN (current) || double.IsInfinity (current) || double.IsNaN (requested) ||
                    double.IsInfinity (requested))
                return current;
            if (!responseTime || alternate || Math.Abs (current - requested) <= ThrottleSnap)
                return requested;
            double speed = current < requested ? accelerationSpeed : decelerationSpeed;
            double r = speed * dt;
            if (!(r > 0.0))
                return current;
            if (r >= 1.0 || double.IsPositiveInfinity (steps))
                return requested;
            if (!(steps > 0.0))
                return current;
            return requested + (current - requested) * Math.Pow (1.0 - r, steps);
        }

        /// <summary>
        /// The thrust (kN) a gimbal command acts with: K x expectedThrottle, K = finalThrust /
        /// currentThrottle while the engine thrusts (currentThrottle above ThrottleReadable and a
        /// positive finalThrust), else idleThrustPerThrottle (its full thrust at this atmosphere,
        /// limiter excluded: requestedThrottle already carries it); never below finalThrust. A
        /// non-finite or non-positive K, or a non-finite expected throttle, returns finalThrust.
        /// </summary>
        internal static double ExpectedThrust (double finalThrust, double currentThrottle,
            double idleThrustPerThrottle, double expectedThrottle)
        {
            double k = currentThrottle > ThrottleReadable && finalThrust > 0.0
                ? finalThrust / currentThrottle : idleThrustPerThrottle;
            if (!(k > 0.0) || double.IsInfinity (k) || double.IsNaN (expectedThrottle) ||
                    double.IsInfinity (expectedThrottle))
                return finalThrust;
            double expected = k * expectedThrottle;
            return expected > finalThrust ? expected : finalThrust;
        }
    }
}
