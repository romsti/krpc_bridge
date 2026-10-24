// -----------------------------------------------------------------------------------
// Hinge lock at rest (conn.tower.hinge_lock_at_rest_v1): pure tests. See TowerTests.csproj.
//
// Usage:
//     dotnet run --project build/tests/tower/TowerTests.csproj -c Release
//
// The watcher is fed what KSP would give it: one angle per physics step of 0.02 s, the step's
// physics clock and game time. a recorded flight (heavy booster, south v5 tower):
// at the opening the "+" arm vibrated on the spot, +-0.08 to 0.17 deg at the hinge, period
// 1.15-1.2 s, and KSP refused its lock for 18 s (IsMoving: a moving average of the absolute
// rate over 20 steps, compared to 0.049 deg/s).
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;

namespace KRPC.Bridge.Tower
{
    public static partial class Program
    {
        static int failures;
        static int checks;

        // KSP's own threshold of IsMoving (ModuleRoboticServoHinge.IsMoving, decompiled): what the
        // client passes as max_rate_deg_s.
        const double KspThreshold = 0.049;
        // KSP's physics step and the length of IsMoving's moving average (BaseServo, decompiled).
        const double Dt = 0.02;
        const int KspAverageSteps = 20;

        public static int Main (string[] args)
        {
            TestInvalidArguments ();
            TestAngleDelta ();
            TestFirstStepOnlyRecords ();
            TestSameStepIsNotARest ();
            TestHingeAtRestLocksOnSecondStep ();
            TestSinusoidOf132656AtEveryPhase ();
            TestTraceOf132656 ();
            TestTimeout ();
            TestSlowStepOnTheLastStepLocks ();
            TestLockedBySomeoneElse ();
            TestLockThatDoesNotTake ();
            TestFinalStatesAreFinal ();
            TestTurningPoints ();
            TestStatus ();
            TestLayoutDescriptor ();
            // conn.tower_site (TowerSiteTests.cs): after the descriptor test, which needs an empty registry.
            TowerSiteTests ();
            // conn.tower_site.tower_bubble_v1 (TowerBubbleTests.cs).
            TowerBubbleTests ();
            // conn.tower_site.tower_ground_v1 (TowerGroundTests.cs).
            TowerGroundTests ();
            // The tower package (TowerPackageTests.cs): the craft search and the rows of tower_package_v1.
            TowerPackageTests ();

            Console.WriteLine ();
            Console.WriteLine ("checks: " + checks + ", failures: " + failures);
            return failures == 0 ? 0 : 1;
        }

        // ------------------------------------------------------------------ helpers

        static void Check (bool ok, string what)
        {
            checks++;
            if (ok)
                return;
            failures++;
            Console.WriteLine ("FAIL: " + what);
        }

        static void Equal<T> (T expected, T actual, string what)
        {
            Check (EqualityComparer<T>.Default.Equals (expected, actual),
                   what + " (expected " + expected + ", got " + actual + ")");
        }

        static void Near (double expected, double actual, double tol, string what)
        {
            Check (Math.Abs (expected - actual) <= tol, what + " (expected " + expected + " +- " + tol + ", got " + actual + ")");
        }

        static bool Throws (Action a)
        {
            try {
                a ();
            } catch (ArgumentException) {
                return true;
            }
            return false;
        }

        /// <summary>The outcome of a watcher run on an angle signal.</summary>
        sealed class Run
        {
            internal HingeRestWatch W;
            internal double LockTime = double.NaN;
            internal double KspAverageAtLock = double.NaN;
        }

        /// <summary>
        /// Arms at <paramref name="tArm"/> (s) and steps at every multiple of Dt from there, as KSP's
        /// physics would (game time = physics clock here), until the watcher locks or ends or
        /// <paramref name="tEnd"/>. The lock always takes. Also computes IsMoving's average at the
        /// lock step from the same signal.
        /// </summary>
        static Run Watch (Func<double, double> angle, double tArm, double tEnd, double maxRate, double timeout)
        {
            var r = new Run { W = HingeRestWatch.Arm (maxRate, timeout, tArm, angle (tArm), false) };
            for (long n = (long) Math.Ceiling (tArm / Dt - 1e-9); ; n++) {
                double t = n * Dt;
                if (t > tEnd)
                    break;
                var d = r.W.Step (t, t, angle (t), false);
                if (d == HingeRestWatch.Decision.Lock) {
                    r.W.Locked (true, t);
                    r.LockTime = t;
                    try {
                        r.KspAverageAtLock = KspAverage (angle, t);
                    } catch (ArgumentOutOfRangeException) {
                        // fewer than 21 samples of the signal before the lock (the start of a trace)
                    }
                    break;
                }
                if (d == HingeRestWatch.Decision.Ended)
                    break;
            }
            return r;
        }

        /// <summary>BaseServo.CalculateAverageRateOfMovement: mean of |d angle| / dt over the last 20 steps.</summary>
        static double KspAverage (Func<double, double> angle, double t)
        {
            double sum = 0.0;
            for (int j = 0; j < KspAverageSteps; j++) {
                double t1 = t - j * Dt;
                sum += Math.Abs (angle (t1) - angle (t1 - Dt)) / Dt;
            }
            return sum / KspAverageSteps;
        }

        // ------------------------------------------------------------------ the trace of 132656
        // The "+" hinge of a recorded flight (`trace_bras` of the capture_bras events of
        // the client: UT minus the UT of the opening request, 216296.0; angle as kRPC's current_angle,
        // which is modelInitialAngle + currentTransformAngle, the angle this watcher reads), from the
        // end of the opening (the "-" hinge locked at 4.64 s) to the pilot's echec:temps at 9.5 s.
        static readonly double[,] Trace132656Plus = {
            { 4.54, 87.701 }, { 4.64, 87.754 }, { 4.76, 87.797 }, { 4.88, 87.813 }, { 5.00, 87.807 },
            { 5.12, 87.781 }, { 5.22, 87.749 }, { 5.34, 87.723 }, { 5.46, 87.715 }, { 5.56, 87.741 },
            { 5.72, 87.780 }, { 5.82, 87.819 }, { 5.92, 87.855 }, { 6.04, 87.867 }, { 6.14, 87.853 },
            { 6.26, 87.826 }, { 6.38, 87.773 }, { 6.48, 87.739 }, { 6.60, 87.725 }, { 6.74, 87.734 },
            { 6.86, 87.772 }, { 6.98, 87.821 }, { 7.08, 87.864 }, { 7.20, 87.883 }, { 7.32, 87.876 },
            { 7.42, 87.833 }, { 7.58, 87.782 }, { 7.70, 87.740 }, { 7.80, 87.722 }, { 7.94, 87.736 },
            { 8.04, 87.776 }, { 8.16, 87.831 }, { 8.30, 87.880 }, { 8.44, 87.893 }, { 8.56, 87.867 },
            { 8.70, 87.812 }, { 8.82, 87.751 }, { 8.96, 87.717 }, { 9.06, 87.718 }, { 9.20, 87.761 },
            { 9.36, 87.836 }, { 9.50, 87.894 }
        };

        /// <summary>
        /// The trace between its samples (0.10-0.18 s apart), at the physics step: a cubic Hermite
        /// with Catmull-Rom slopes (smooth: the rate goes through zero at a turning point instead of
        /// jumping, as a physical hinge's does), one-sided at both ends.
        /// </summary>
        static double TraceAngle (double t)
        {
            var p = Trace132656Plus;
            int n = p.GetLength (0);
            if (t < p [0, 0] - 1e-9 || t > p [n - 1, 0] + 1e-9)
                throw new ArgumentOutOfRangeException ("t", t, "outside the trace");
            int i = 0;
            while (i < n - 2 && t > p [i + 1, 0])
                i++;
            double t0 = p [i, 0], t1 = p [i + 1, 0], h = t1 - t0, s = (t - t0) / h;
            double m0 = Slope (i) * h, m1 = Slope (i + 1) * h;
            double s2 = s * s, s3 = s2 * s;
            return (2 * s3 - 3 * s2 + 1) * p [i, 1] + (s3 - 2 * s2 + s) * m0 + (-2 * s3 + 3 * s2) * p [i + 1, 1] + (s3 - s2) * m1;
        }

        static double Slope (int i)
        {
            var p = Trace132656Plus;
            int n = p.GetLength (0);
            if (i == 0)
                return (p [1, 1] - p [0, 1]) / (p [1, 0] - p [0, 0]);
            if (i == n - 1)
                return (p [n - 1, 1] - p [n - 2, 1]) / (p [n - 1, 0] - p [n - 2, 0]);
            return (p [i + 1, 1] - p [i - 1, 1]) / (p [i + 1, 0] - p [i - 1, 0]);
        }

        // ------------------------------------------------------------------ tests

        static void TestInvalidArguments ()
        {
            foreach (var bad in new [] { double.NaN, double.PositiveInfinity, double.NegativeInfinity, 0.0, -0.049 }) {
                Check (HingeRestWatch.Invalid (bad, 1.0) != null, "max rate " + bad + " refused");
                Check (HingeRestWatch.Invalid (0.049, bad) != null, "timeout " + bad + " refused");
                Check (Throws (() => HingeRestWatch.Arm (bad, 1.0, 0.0, 0.0, false)), "Arm throws on max rate " + bad);
                Check (Throws (() => HingeRestWatch.Arm (0.049, bad, 0.0, 0.0, false)), "Arm throws on timeout " + bad);
            }
            Check (HingeRestWatch.Invalid (1e-9, 1e-9) == null, "any finite positive pair is valid (no constant here)");
            Check (HingeRestWatch.Invalid (1e6, 1e6) == null, "no upper bound either");
        }

        static void TestAngleDelta ()
        {
            Near (2.0, HingeRestWatch.AngleDelta (359.0, 1.0), 1e-12, "359 -> 1 is +2");
            Near (-2.0, HingeRestWatch.AngleDelta (1.0, 359.0), 1e-12, "1 -> 359 is -2");
            Near (0.05, HingeRestWatch.AngleDelta (87.80, 87.85), 1e-9, "87.80 -> 87.85");
            Near (180.0, HingeRestWatch.AngleDelta (0.0, 180.0), 1e-12, "half a turn is +180");
            Near (180.0, HingeRestWatch.AngleDelta (180.0, 0.0), 1e-12, "and +180 the other way (in (-180, 180])");
        }

        static void TestFirstStepOnlyRecords ()
        {
            // a hinge that moves fast, armed: the first step has no previous sample, never a rate
            var w = HingeRestWatch.Arm (KspThreshold, 5.0, 100.0, 87.0, false);
            Equal (HingeRestWatch.StateArmed, w.State, "armed");
            Equal (HingeRestWatch.Decision.Watch, w.Step (100.0, 50.0, 87.0, false), "first step only records");
            Equal (1, w.Steps, "one step counted");
            Check (double.IsNaN (w.MinRate) && double.IsNaN (w.LastRate), "no rate yet");
            Equal (HingeRestWatch.Decision.Watch, w.Step (100.02, 50.02, 87.2, false), "10 deg/s: watch");
            Near (10.0, w.LastRate, 1e-6, "rate = |d angle| / d physics clock");
        }

        static void TestSameStepIsNotARest ()
        {
            // The arming RPC can run in the same physics step as the watcher's next FixedUpdate, and two
            // reads within one step give the same angle: never a rate of zero (a false rest).
            var w = HingeRestWatch.Arm (KspThreshold, 5.0, 0.0, 87.5, false);
            Equal (HingeRestWatch.Decision.Watch, w.Step (0.0, 10.0, 87.5, false), "step 1 records");
            Equal (HingeRestWatch.Decision.Watch, w.Step (0.0, 10.0, 87.5, false), "same physics step again: nothing");
            Equal (1, w.Steps, "the repeat is not counted");
            Check (double.IsNaN (w.MinRate), "and gives no rate");
            Equal (HingeRestWatch.Decision.Watch, w.Step (0.02, 10.02, 87.7, false), "next step moving: watch");
            Equal (HingeRestWatch.Decision.Watch, w.Step (0.02, 10.0, 87.7, false), "an older physics clock: nothing");
        }

        static void TestHingeAtRestLocksOnSecondStep ()
        {
            var w = HingeRestWatch.Arm (KspThreshold, 5.0, 0.0, 90.0, false);
            Equal (HingeRestWatch.Decision.Watch, w.Step (0.0, 0.0, 90.0, false), "records");
            Equal (HingeRestWatch.Decision.Lock, w.Step (0.02, 0.02, 90.0, false), "at rest: lock on the second step");
            Near (90.0, w.LockAngle, 1e-12, "lock angle");
            Near (0.0, w.LockRate, 1e-12, "lock rate");
            Equal (HingeRestWatch.StateArmed, w.State, "still armed until the lock is applied");
            w.Locked (true, 0.02);
            Equal (HingeRestWatch.StateLocked, w.State, "locked");
            Near (0.02, w.Elapsed (99.0), 1e-12, "elapsed frozen at the lock");
        }

        static void TestSinusoidOf132656AtEveryPhase ()
        {
            // The vibration of 132656 as the client's simulated hinge has it (+-0.085 deg, 1.2 s), its
            // measured period (1.15 s), and the amplitude after the failure (+-0.17 deg): armed at 60
            // phases of a period, the watcher locks at the FIRST turning point it can see (within a
            // physics step and a half of it), slower than KSP's threshold, where IsMoving's average
            // says 0.29-0.64 deg/s.
            foreach (var vib in new [] { new [] { 0.085, 1.2 }, new [] { 0.085, 1.15 }, new [] { 0.17, 1.15 }, new [] { 0.076, 1.15 } }) {
                double a = vib [0], period = vib [1], w = 2.0 * Math.PI / period;
                Func<double, double> angle = t => 87.80 + a * Math.Sin (w * t);
                string tag = "+-" + a + " deg, " + period + " s";
                int locked = 0;
                double worst = 0.0, slowestAverage = double.PositiveInfinity;
                for (int k = 0; k < 60; k++) {
                    double tArm = 10.0 + k * period / 60.0 + 0.0037;
                    var r = Watch (angle, tArm, tArm + 10.0, KspThreshold, 10.0);
                    if (r.W.State != HingeRestWatch.StateLocked)
                        continue;
                    locked++;
                    // the turning points: w t = pi/2 + m pi. The first step after arming only records: the
                    // first turning point the watcher can see is the first one past its second sample,
                    // at most two physics steps after arming (one closer may be caught, or not).
                    double m = Math.Ceiling ((w * (tArm + 2.0 * Dt) - Math.PI / 2.0) / Math.PI);
                    double next = (Math.PI / 2.0 + m * Math.PI) / w;
                    double nearest = (Math.PI / 2.0 + Math.Round ((w * r.LockTime - Math.PI / 2.0) / Math.PI) * Math.PI) / w;
                    worst = Math.Max (worst, Math.Abs (r.LockTime - nearest));
                    Check (r.LockTime >= tArm, tag + ": never before arming");
                    Check (r.LockTime <= next + 1.5 * Dt, tag + " armed at " + tArm.ToString ("F4") + ": the first turning point ("
                           + next.ToString ("F3") + "), locked at " + r.LockTime.ToString ("F3"));
                    Check (r.W.LockRate <= KspThreshold, tag + ": lock rate under the threshold");
                    Check (Math.Abs (r.W.LockAngle - 87.80) >= a * Math.Cos (w * 2.0 * Dt) - 1e-9, tag + ": locked near an extreme");
                    slowestAverage = Math.Min (slowestAverage, r.KspAverageAtLock);
                }
                Equal (60, locked, tag + ": locked at every phase");
                Check (worst <= 1.5 * Dt, tag + ": within a physics step and a half of the turning point (worst " + worst.ToString ("F4") + " s)");
                Check (slowestAverage > 5.0 * KspThreshold, tag + ": IsMoving's average at the lock is " + slowestAverage.ToString ("F3")
                       + " deg/s: the stock lock is refused there");
            }
        }

        static void TestTraceOf132656 ()
        {
            // Through the whole vibration, IsMoving's average never comes under 0.15 deg/s: the stock lock
            // refused at every step, as in flight.
            double minAverage = double.PositiveInfinity;
            for (double t = 4.54 + (KspAverageSteps + 1) * Dt; t <= 9.5; t += Dt)
                minAverage = Math.Min (minAverage, KspAverage (TraceAngle, t));
            Check (minAverage > 3.0 * KspThreshold, "132656: IsMoving's average never under " + minAverage.ToString ("F3") + " deg/s");

            // Armed when the client recognises the vibration (6.74 s): locked at the next turning
            // point, the top near 7.2 s (87.883 at 7.20, 87.876 at 7.32), 0.5 s later.
            var r = Watch (TraceAngle, 6.74, 9.5, KspThreshold, 2.72);
            Equal (HingeRestWatch.StateLocked, r.W.State, "132656 armed at 6.74 s: locked");
            Near (7.22, r.LockTime, 0.041, "132656: at the top of the swing");
            Near (87.884, r.W.LockAngle, 0.004, "132656: lock angle");
            Check (r.W.LockRate <= KspThreshold, "132656: lock rate " + r.W.LockRate.ToString ("F4") + " deg/s");
            Check (r.KspAverageAtLock > 0.3, "132656: IsMoving's average at that step " + r.KspAverageAtLock.ToString ("F3") + " deg/s");
            Equal (0, r.W.TurningPoints, "132656: locked at the first reversal, before seeing it");

            // Armed anywhere in the vibration: always the first turning point after arming.
            var cases = new [] { new [] { 4.64, 4.90 }, new [] { 5.00, 5.44 }, new [] { 6.00, 6.02 }, new [] { 7.50, 7.82 }, new [] { 8.50, 9.02 } };
            foreach (var c in cases) {
                var rc = Watch (TraceAngle, c [0], 9.5, KspThreshold, 5.0);
                Equal (HingeRestWatch.StateLocked, rc.W.State, "132656 armed at " + c [0] + ": locked");
                Near (c [1], rc.LockTime, 0.041, "132656 armed at " + c [0] + ": next turning point");
                double lo = double.PositiveInfinity, hi = double.NegativeInfinity;
                for (double t = rc.LockTime - 0.1; t <= Math.Min (9.5, rc.LockTime + 0.1); t += 0.005) {
                    lo = Math.Min (lo, TraceAngle (t));
                    hi = Math.Max (hi, TraceAngle (t));
                }
                Check (Math.Min (Math.Abs (rc.W.LockAngle - lo), Math.Abs (rc.W.LockAngle - hi)) <= 0.002,
                       "132656 armed at " + c [0] + ": the lock angle is the local extreme");
            }

            // The stock path on the same trace (EngageServoLock asked at every step): never.
            int stockLocks = 0;
            for (double t = 6.74; t <= 9.5; t += Dt)
                if (!(KspAverage (TraceAngle, t) > KspThreshold))
                    stockLocks++;
            Equal (0, stockLocks, "132656: the stock lock never takes on that trace");
        }

        static void TestTimeout ()
        {
            // 1 deg/s, never slow enough: expired at the first step whose game time since arming is
            // strictly over the limit; the elapsed time is frozen there.
            var w = HingeRestWatch.Arm (KspThreshold, 0.5, 0.0, 80.0, false);
            HingeRestWatch.Decision d = HingeRestWatch.Decision.Watch;
            double t = 0.0;
            for (int n = 0; n <= 100 && d == HingeRestWatch.Decision.Watch; n++) {
                t = n * Dt;
                d = w.Step (t, t, 80.0 + 1.0 * t, false);
            }
            Equal (HingeRestWatch.Decision.Ended, d, "ended");
            Equal (HingeRestWatch.StateExpired, w.State, "expired");
            Near (0.52, t, 1e-9, "at the first step past 0.5 s");
            Near (0.52, w.Elapsed (100.0), 1e-9, "elapsed frozen");
            Near (1.0, w.MinRate, 1e-6, "slowest rate seen");
            Check (double.IsNaN (w.LockAngle) && double.IsNaN (w.LockRate), "no lock angle");
            Equal (27, w.Steps, "steps watched");
            Equal (HingeRestWatch.Decision.Ended, w.Step (1.0, 1.0, 80.0, false), "ended for good");
            w.Locked (true, 1.0);
            Equal (HingeRestWatch.StateExpired, w.State, "a late lock does not change it");
        }

        static void TestSlowStepOnTheLastStepLocks ()
        {
            // The order of the tests: a slow step past the time limit still locks (the rate first).
            var w = HingeRestWatch.Arm (KspThreshold, 0.1, 0.0, 80.0, false);
            for (int n = 0; n <= 5; n++)
                Equal (HingeRestWatch.Decision.Watch, w.Step (n * Dt, n * Dt, 80.0 + n * Dt, false), "moving at 1 deg/s, step " + n);
            Equal (HingeRestWatch.Decision.Lock, w.Step (0.12, 0.12, 80.10, false), "stopped at 0.12 s, past 0.1 s: lock");
        }

        static void TestLockedBySomeoneElse ()
        {
            var w = HingeRestWatch.Arm (KspThreshold, 5.0, 0.0, 85.0, true);
            Equal (HingeRestWatch.StateAlreadyLocked, w.State, "locked when armed: ends at once");
            Equal (HingeRestWatch.Decision.Ended, w.Step (0.02, 0.02, 85.0, false), "nothing more");
            Near (0.0, w.Elapsed (5.0), 1e-12, "elapsed 0");

            w = HingeRestWatch.Arm (KspThreshold, 5.0, 0.0, 85.0, false);
            w.Step (0.0, 0.0, 85.0, false);
            w.Step (0.02, 0.02, 85.1, false);
            Equal (HingeRestWatch.Decision.Ended, w.Step (0.04, 0.04, 85.1, true), "the stock lock took first");
            Equal (HingeRestWatch.StateAlreadyLocked, w.State, "already locked");
            Check (double.IsNaN (w.LockAngle), "not locked by the watcher");
        }

        static void TestLockThatDoesNotTake ()
        {
            var w = HingeRestWatch.Arm (KspThreshold, 5.0, 0.0, 85.0, false);
            w.Step (0.0, 0.0, 85.0, false);
            Equal (HingeRestWatch.Decision.Lock, w.Step (0.02, 0.02, 85.0, false), "lock");
            w.Locked (false, 0.02);
            Equal (HingeRestWatch.StateFailed, w.State, "the hinge did not say locked: failed");
        }

        static void TestFinalStatesAreFinal ()
        {
            var w = HingeRestWatch.Arm (KspThreshold, 5.0, 0.0, 85.0, false);
            w.End (HingeRestWatch.StateCancelled, 1.0);
            Equal (HingeRestWatch.StateCancelled, w.State, "cancelled");
            w.End (HingeRestWatch.StateLost, 2.0);
            Equal (HingeRestWatch.StateCancelled, w.State, "the first end state stays");
            Equal (HingeRestWatch.Decision.Ended, w.Step (3.0, 3.0, 85.0, false), "no step after the end");
            Near (1.0, w.Elapsed (10.0), 1e-12, "elapsed frozen at the cancel");
        }

        static void TestTurningPoints ()
        {
            // a threshold no sampled step reaches: the reversals of direction are counted (diagnostic)
            Func<double, double> angle = t => 87.80 + 0.085 * Math.Sin (2.0 * Math.PI * t / 1.2 + 0.3);
            var r = Watch (angle, 0.0, 3.6 - 1e-9, 1e-9, 100.0);
            Equal (HingeRestWatch.StateArmed, r.W.State, "never slow enough");
            Equal (6, r.W.TurningPoints, "three periods: six turning points");
            Check (r.W.MinRate < 0.05 && r.W.MinRate > 1e-9, "slowest rate seen near a turning point");
        }

        static void TestStatus ()
        {
            Equal (HingeRestWatch.StatusLength, HingeRestWatch.Fields.Length, "Fields and StatusLength agree");
            var none = HingeRestWatch.NoneStatus (1.0, 90.05);
            Equal (HingeRestWatch.StatusLength, none.Count, "none: length");
            Near (HingeRestWatch.Protocol, none [0], 0, "none: protocol");
            Near (HingeRestWatch.StateNone, none [1], 0, "none: state");
            Near (1.0, none [10], 0, "none: locked now");
            Near (90.05, none [11], 0, "none: angle now");
            Check (double.IsNaN (none [2]) && double.IsNaN (none [3]) && double.IsNaN (none [5]), "none: NaN elsewhere");

            var w = HingeRestWatch.Arm (0.03, 2.5, 100.0, 87.0, false);
            w.Step (100.0, 0.0, 87.0, false);
            w.Step (100.02, 0.02, 87.01, false);
            w.Step (100.04, 0.04, 87.0102, false);
            w.Locked (true, 100.04);
            var s = w.Status (200.0, 0.31, 1.0, 87.0101);
            Equal (HingeRestWatch.StatusLength, s.Count, "status length");
            var names = HingeRestWatch.Fields;
            Func<string, double> f = n => s [Array.IndexOf (names, n)];
            Near (1, f ("protocol"), 0, "protocol");
            Near (HingeRestWatch.StateLocked, f ("state"), 0, "state");
            Near (0.04, f ("elapsed_s"), 1e-9, "elapsed frozen at the lock");
            Near (87.0102, f ("lock_angle_deg"), 1e-12, "lock angle");
            Near (0.01, f ("lock_rate_deg_s"), 1e-6, "lock rate");
            Near (0.03, f ("max_rate_deg_s"), 0, "max rate as armed");
            Near (2.5, f ("timeout_s"), 0, "timeout as armed");
            Near (3, f ("steps"), 0, "steps");
            Near (0.01, f ("min_rate_deg_s"), 1e-6, "slowest rate");
            Near (0.31, f ("ksp_average_rate_deg_s"), 0, "KSP average (from the Unity side)");
            Near (1.0, f ("locked"), 0, "locked now (from the Unity side)");
            Near (87.0101, f ("angle_deg"), 0, "angle now (from the Unity side)");
            Near (0, f ("turning_points"), 0, "turning points");
        }

        static void TestLayoutDescriptor ()
        {
            // Without an assembly named KRPC.Bridge.Core (this test has the registry compiled in): nothing,
            // and the reason.
            Check (TowerLayoutDeclaration.FindRegistry () == null, "no Core assembly here");
            Check (TowerLayoutDeclaration.Declare ().StartsWith ("no layout registry", StringComparison.Ordinal), "the reason");
            Check (LayoutRegistry.Find (TowerLayoutDeclaration.HingeLockLayout) == null, "nothing declared");

            // Into the real registry (the source the Core compiles), twice: one layout, read back as
            // conn.bridge.layout_v1 serves it.
            string said = TowerLayoutDeclaration.DeclareInto (typeof (LayoutRegistry));
            TowerLayoutDeclaration.DeclareInto (typeof (LayoutRegistry));
            Check (said.StartsWith ("layout hinge_lock_at_rest_v1 declared", StringComparison.Ordinal), "declared: " + said);
            Equal (1, LayoutRegistry.Names ().FindAll (n => n == TowerLayoutDeclaration.HingeLockLayout).Count, "declared once");
            Equal (0, LayoutRegistry.Problems ().Count, "the descriptor fits StatusLength");
            var lines = LayoutRegistry.Lines (TowerLayoutDeclaration.HingeLockLayout);
            var head = new Dictionary<string, string> ();
            foreach (var kv in lines [0].Split ('\t')) {
                int eq = kv.IndexOf ('=');
                head [kv.Substring (0, eq)] = kv.Substring (eq + 1);
            }
            Equal ("hinge_lock_at_rest_v1", head ["layout"], "layout");
            Equal ("Tower", head ["service"], "service");
            Equal ("1", head ["layout_protocol"], "protocol");
            Equal ("1", head ["layout_schema"], "schema");
            Equal (LayoutRegistry.KindStatus, head ["kind"], "kind");
            Equal ("1", head ["sections"], "one section");
            Equal ("13", head ["fields"], "13 fields");
            Equal ("#section\tstatus\t1\t13\t0\t", lines [1], "the section, read once, stride 13");
            Equal (2 + HingeRestWatch.Fields.Length, lines.Count, "one line per field");
            for (int i = 0; i < HingeRestWatch.Fields.Length; i++) {
                var cells = lines [2 + i].Split ('\t');
                Equal ("status", cells [0], "field " + i + ": section");
                Equal (i.ToString (), cells [1], "field " + i + ": offset");
                Equal (HingeRestWatch.Fields [i], cells [2], "field " + i + ": name");
            }
            var units = new Dictionary<string, string> ();
            for (int i = 0; i < HingeRestWatch.Fields.Length; i++) {
                var cells = lines [2 + i].Split ('\t');
                units [cells [2]] = cells [3];
            }
            Equal ("s", units ["elapsed_s"], "unit of elapsed_s");
            Equal ("deg", units ["lock_angle_deg"], "unit of lock_angle_deg");
            Equal ("deg/s", units ["lock_rate_deg_s"], "unit of lock_rate_deg_s");
            Equal ("deg/s", units ["ksp_average_rate_deg_s"], "unit of ksp_average_rate_deg_s");
            Equal ("", units ["steps"], "steps: a count");
            Equal ("", units ["turning_points"], "turning_points: a count");
        }
    }
}
