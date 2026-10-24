using System;
using System.Collections.Generic;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// The pure part of conn.tower.hinge_lock_at_rest_v1: one watcher per robotic hinge, fed one
    /// angle per physics step, that says WHEN to lock (catch tower, 06/10/2026).
    ///
    /// WHY. KSP refuses to lock a hinge that moves (BaseServo.EngageServoLock: nothing when
    /// IsMoving()), and ModuleRoboticServoHinge.IsMoving compares to 0.049 deg/s a MOVING AVERAGE
    /// of the absolute rate over 20 physics steps (BaseServo.CalculateAverageRateOfMovement:
    /// velocityReadings, velocityAverageReadings = 20; decompiled). A hinge that vibrates on the
    /// spot (a recorded flight: +-0.08 to 0.17 deg at the hinge, period 1.15-1.2 s,
    /// undamped) keeps that average near (2/pi) A w ~ 0.3 deg/s even at its turning points: the
    /// lock is refused for ever, the capture refused with it. Its INSTANTANEOUS rate, however,
    /// goes through zero twice a period. This watcher locks on the first physics step whose
    /// instantaneous rate |d angle| / dt is at or below the threshold the client passes.
    ///
    /// No constant of its own: the threshold and the time limit come from the client (who
    /// computes them: KSP's own 0.049 deg/s, the budget left to the movement). The only rules
    /// here are the order of the tests and the angle unwrapping.
    ///
    /// PURE: System only (build/tests/tower). The Unity side (HingeRestLocks) reads the angle,
    /// applies the lock and fills the KSP fields of the status.
    /// </summary>
    public sealed class HingeRestWatch
    {
        /// <summary>Protocol of the hinge_lock_at_rest_v1 status (a field removed or moved raises it).</summary>
        public const int Protocol = 1;

        /// <summary>Schema of the status (a field appended raises it).</summary>
        public const int Schema = 1;

        /// <summary>No watcher was ever armed for this part in this scene.</summary>
        public const int StateNone = 0;
        /// <summary>Armed: watching the hinge, not locked yet.</summary>
        public const int StateArmed = 1;
        /// <summary>Locked by the watcher, at a step whose instantaneous rate was under the threshold.</summary>
        public const int StateLocked = 2;
        /// <summary>The hinge was already locked when armed, or got locked by someone else: nothing done.</summary>
        public const int StateAlreadyLocked = 3;
        /// <summary>The time limit ran out before a step came under the threshold: nothing done.</summary>
        public const int StateExpired = 4;
        /// <summary>Cancelled by the client (hinge_lock_cancel_v1), or replaced by a new arming.</summary>
        public const int StateCancelled = 5;
        /// <summary>The vessel was unloaded, packed or destroyed, or the part or its module is gone.</summary>
        public const int StateLost = 6;
        /// <summary>The lock was applied and the hinge did not say locked, or applying it threw.</summary>
        public const int StateFailed = 7;

        /// <summary>The status fields, in wire order (layout hinge_lock_at_rest_v1, section "status").</summary>
        public static readonly string[] Fields = {
            "protocol", "state", "elapsed_s", "lock_angle_deg", "lock_rate_deg_s", "max_rate_deg_s",
            "timeout_s", "steps", "min_rate_deg_s", "ksp_average_rate_deg_s", "locked", "angle_deg",
            "turning_points"
        };

        /// <summary>Length of the status (Fields.Length).</summary>
        public const int StatusLength = 13;

        /// <summary>What the Unity side does after a step.</summary>
        public enum Decision
        {
            /// <summary>Keep watching.</summary>
            Watch,
            /// <summary>Lock the hinge now (then report with Locked or Failed).</summary>
            Lock,
            /// <summary>The watcher just ended (expired, already locked): nothing to do.</summary>
            Ended
        }

        public readonly double MaxRate;
        public readonly double Timeout;
        public readonly double ArmedAt;

        public int State { get; private set; }
        public double EndedAt { get; private set; }
        public double LockAngle { get; private set; }
        public double LockRate { get; private set; }
        public int Steps { get; private set; }
        public double MinRate { get; private set; }
        public int TurningPoints { get; private set; }
        public double LastAngle { get; private set; }
        public double LastRate { get; private set; }

        double lastDelta;
        double lastStepTime = double.NaN;

        HingeRestWatch (double maxRate, double timeout, double armedAt)
        {
            MaxRate = maxRate;
            Timeout = timeout;
            ArmedAt = armedAt;
            State = StateArmed;
            EndedAt = double.NaN;
            LockAngle = double.NaN;
            LockRate = double.NaN;
            MinRate = double.NaN;
            LastAngle = double.NaN;
            LastRate = double.NaN;
        }

        /// <summary>
        /// Why these arguments are refused, or null when they are valid: a threshold and a time
        /// limit that are finite and strictly positive.
        /// </summary>
        public static string Invalid (double maxRate, double timeout)
        {
            if (double.IsNaN (maxRate) || double.IsInfinity (maxRate) || maxRate <= 0.0)
                return "max_rate_deg_s must be finite and > 0";
            if (double.IsNaN (timeout) || double.IsInfinity (timeout) || timeout <= 0.0)
                return "timeout_s must be finite and > 0";
            return null;
        }

        /// <summary>
        /// A watcher armed at <paramref name="now"/> (game time, s). <paramref name="alreadyLocked"/>:
        /// the hinge is locked right now, so the watcher ends at once (StateAlreadyLocked).
        /// </summary>
        public static HingeRestWatch Arm (double maxRate, double timeout, double now, double angle, bool alreadyLocked)
        {
            var why = Invalid (maxRate, timeout);
            if (why != null)
                throw new ArgumentException (why);
            var w = new HingeRestWatch (maxRate, timeout, now);
            // The angle at arming is for the status only, never a rate sample: the arming RPC may run
            // in the same physics step as the next watched step, and a difference taken within one
            // step is zero (a false rest).
            w.LastAngle = angle;
            if (alreadyLocked)
                w.End (StateAlreadyLocked, now);
            return w;
        }

        /// <summary>The signed shortest difference b - a between two angles in degrees, in (-180, 180].</summary>
        public static double AngleDelta (double a, double b)
        {
            double d = (b - a) % 360.0;
            if (d > 180.0)
                d -= 360.0;
            else if (d <= -180.0)
                d += 360.0;
            return d;
        }

        /// <summary>
        /// One physics step. <paramref name="now"/>: game time (UT, s), for the time limit;
        /// <paramref name="stepTime"/>: the physics clock of this step (Time.fixedTime, s), for the
        /// rate; <paramref name="angle"/>: the hinge's angle now (deg); <paramref name="lockedNow"/>:
        /// whether it is locked now. In this order: a hinge locked by someone else ends the watcher
        /// (AlreadyLocked); then the instantaneous rate |angle - previous| / (stepTime - previous
        /// stepTime), only between two DIFFERENT physics steps (the first step after arming, or a
        /// second call within one step, only records); at or under the threshold: Lock, even on
        /// the last step of the window; then the time limit (elapsed strictly greater): Expired.
        /// </summary>
        public Decision Step (double now, double stepTime, double angle, bool lockedNow)
        {
            if (State != StateArmed)
                return Decision.Ended;
            if (lockedNow) {
                LastAngle = angle;
                End (StateAlreadyLocked, now);
                return Decision.Ended;
            }
            double rate = double.NaN;
            bool sampled = !double.IsNaN (lastStepTime);
            if (sampled && !(stepTime > lastStepTime))
                return Decision.Watch;          // same physics step: nothing new to compare
            Steps++;
            if (sampled && !double.IsNaN (LastAngle) && !double.IsNaN (angle)) {
                double delta = AngleDelta (LastAngle, angle);
                rate = Math.Abs (delta) / (stepTime - lastStepTime);
                if (delta != 0.0) {
                    if (lastDelta != 0.0 && (delta > 0.0) != (lastDelta > 0.0))
                        TurningPoints++;
                    lastDelta = delta;
                }
                if (double.IsNaN (MinRate) || rate < MinRate)
                    MinRate = rate;
            }
            LastAngle = angle;
            LastRate = rate;
            lastStepTime = stepTime;
            if (!double.IsNaN (rate) && rate <= MaxRate) {
                LockAngle = angle;
                LockRate = rate;
                return Decision.Lock;
            }
            if (now - ArmedAt > Timeout) {
                End (StateExpired, now);
                return Decision.Ended;
            }
            return Decision.Watch;
        }

        /// <summary>The Unity side applied the lock: did the hinge say locked afterwards?</summary>
        public void Locked (bool took, double now)
        {
            if (State != StateArmed)
                return;
            End (took ? StateLocked : StateFailed, now);
        }

        /// <summary>Ends an armed watcher with a final state (Cancelled, Lost, Failed...); no-op otherwise.</summary>
        public void End (int state, double now)
        {
            if (State != StateArmed)
                return;
            State = state;
            EndedAt = now;
        }

        /// <summary>Game time since arming, frozen at the end.</summary>
        public double Elapsed (double now)
        {
            return (State == StateArmed ? now : EndedAt) - ArmedAt;
        }

        /// <summary>
        /// The status (layout hinge_lock_at_rest_v1). <paramref name="kspAverageRate"/>,
        /// <paramref name="lockedNow"/> (1, 0 or NaN) and <paramref name="angleNow"/> are read by
        /// the Unity side.
        /// </summary>
        public List<double> Status (double now, double kspAverageRate, double lockedNow, double angleNow)
        {
            var r = new List<double> (StatusLength) {
                Protocol, State, Elapsed (now), LockAngle, LockRate, MaxRate, Timeout, Steps, MinRate,
                kspAverageRate, lockedNow, angleNow, TurningPoints
            };
            return r;
        }

        /// <summary>The status of a part that has no watcher: protocol, StateNone, NaN elsewhere.</summary>
        public static List<double> NoneStatus (double lockedNow, double angleNow)
        {
            var r = new List<double> (StatusLength);
            for (int i = 0; i < StatusLength; i++)
                r.Add (double.NaN);
            r [0] = Protocol;
            r [1] = StateNone;
            r [7] = 0;
            r [10] = lockedNow;
            r [11] = angleNow;
            r [12] = 0;
            return r;
        }
    }
}
