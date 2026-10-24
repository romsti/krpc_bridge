// the CLOSED loop of the armed SurfaceModelV1, on one body axis, with the law of the
// plugin (AttitudeMath.Step), the command law and the actuator of the replica (SurfaceModel
// Target, TargetGradient, Step) and a rigid body whose fins push with their REAL torque.
//
// a recorded flight (first armed flight, bridge ad35d85): the replica's forces are
// the module's liftForce/dragForce, in KSP's force unit, the kilonewton (Qlift is in Pa, the force
// goes to the part's rigidbody, whose mass is in tonnes); the law adds them to RCS, wheel and
// engine torques it has converted to N.m. Every fin torque the law saw (M, the sub-columns of B,
// M_free) was 1000 times too small: in the 50 Hz frames the measured angular acceleration is
// RCS + wheel + 870-1030 x the replica's fin torque, on every axis. The INDI increment then
// overshoots by the ratio kappa of the true to the modeled ctrlState column, and diverges when
// kappa passes 2: in that flight the loop's roll (written in the coast once the fins were engaged)
// started its 3 Hz limit cycle as kappa crossed 2 (q ~ 7 kPa) and was saturated at +-1 from
// kappa ~ 12 (q ~ 68 kPa).
//
// The scenarios are built from that flight's own numbers (inertias, the published RCS/wheel
// columns, the fins' columns and gains of surface_model_v1.jsonl, the gimbal columns of the frames).
// LawScale 1 is the build that flew (torques in kN.m given to the law as N.m), the scale
// (SurfaceModel.LawTorqueScale) is the fix. Writes nothing into the golden vectors.

using System;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    internal static class SurfaceLoopTests
    {
        static int checks;
        static int failures;
        static readonly bool trace = Environment.GetEnvironmentVariable ("BRIDGE_D165_TRACE") == "1";

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.WriteLine ("FAIL (surface loop): " + what);
            }
        }

        internal sealed class Scenario
        {
            internal string Name;
            internal int Axis;                  // body axis of the loop: 0 pitch (x), 1 roll (y)
            internal double Inertia;            // kg.m^2
            internal double RcsColumn;          // N.m per unit of the ctrlState axis (law and plant)
            internal double RcsAlpha;           // the RCS torque's first-order lag per step (flight fit)
            internal double WheelColumn;        // reaction wheel, N.m per unit
            internal double WheelAlpha = 0.4;   // ModuleReactionWheel's lerp (exact in the flight's frames)
            internal int Fins;
            internal double FinColumnNative;    // per fin, KSP force unit x m per degree (kN.m/deg)
            internal double FinGain;            // deg of target per unit of the ctrlState axis
            internal double FinSpeed = 40.0;    // actuatorSpeed, deg/s
            internal double LawScale;           // what the law multiplies the replica's torques by
            internal int Gimbals;               // identical gimbal columns on the axis
            internal double GimbalColumnFull;   // N.m per degree, each, at full throttle
            internal double GimbalRange = 10.5; // deg
            internal double ThrottleStart, ThrottleEnd, ThrottleRampS;
            internal bool ScalarRoll;           // coast: roll written by RollCommand, pitch/yaw the AutoPilot's
            internal double OmegaMax = 1.0;
            internal double Omega0, U0;
            internal double Seconds = 10.0;
        }

        internal sealed class Result
        {
            internal double PeakOmega, LatePeakOmega, LatePeakU, SaturatedFraction, LateFinTorqueRms;
            internal double PeakAngleDeg, LateAngleDeg;
            internal double LatePeriodTicks;    // mean period of omega's sign changes, last 2 s (0: none)
            internal bool Finite = true;
        }

        const double Dt = 0.02;

        static SurfaceCommand FinCommand (Scenario s)
        {
            // target = Range * clamp(CPitch pitch - KRoll roll) * Authority / 100 (Direction 1):
            // the gain per unit of the scenario's axis is FinGain below the clamp.
            var c = new SurfaceCommand {
                Range = 35.0, Authority = 100.0, Direction = 1.0, DeployTerm = 0.0, Speed = s.FinSpeed,
                Exponential = false, Flip = false
            };
            if (s.Axis == 1)
                c.KRoll = -s.FinGain / c.Range;
            else
                c.CPitch = s.FinGain / c.Range;
            return c;
        }

        internal static Result Simulate (Scenario s)
        {
            var res = new Result ();
            var g = new AttitudeGains ();
            var st = new AttitudeLawState ();
            var work = new AttitudeWork ();
            var o = new AttitudeOutput ();
            var inp = new AttitudeInput ();
            int ng = s.Gimbals;
            int n = ng + 3;
            int jCtrl = ng + s.Axis;
            int a = s.Axis;
            inp.Resize (n);
            inp.Dt = Dt;
            var cmd = FinCommand (s);
            var grad = new double[3];
            double[] ctrl = new double[3];
            ctrl [a] = s.U0;
            double deflection = SurfaceModel.Target (cmd, ctrl [0], ctrl [1], ctrl [2]);
            double gimbal = 0.0;
            double omega = s.Omega0, omegaPrevious = s.Omega0, angle = 0.0;
            double realizedLaw = double.NaN;
            double rcsTorque = s.RcsColumn * s.U0, wheelTorque = s.WheelColumn * s.U0;
            double instantColumn = s.RcsColumn + s.WheelColumn;
            int steps = (int)Math.Round (s.Seconds / Dt);
            int lateFrom = steps - (int)Math.Round (2.0 / Dt);
            int saturated = 0;
            int crossings = 0;
            double lastSign = 0.0;
            double finSquares = 0.0;
            int finCount = 0;
            for (int k = 0; k < steps; k++) {
                double t = k * Dt;
                double throttle = s.ThrottleRampS > 0.0
                    ? s.ThrottleStart + (s.ThrottleEnd - s.ThrottleStart) * Math.Min (1.0, t / s.ThrottleRampS)
                    : s.ThrottleEnd;
                double gimbalColumn = s.GimbalColumnFull * throttle;
                // ---- Earlyish: the law, from the state the last step left.
                for (int x = 0; x < 3; x++) {
                    inp.Omega [x] = x == a ? omega : 0.0;
                    inp.OmegaDotMeasured [x] = x == a ? (omega - omegaPrevious) / Dt : 0.0;
                    inp.Inertia [x] = x == a ? s.Inertia : s.Inertia;
                    inp.Torque [x] = 0.0;
                    inp.TorqueFree [x] = 0.0;
                    inp.Error [x] = 0.0;
                    inp.OmegaFf [x] = 0.0;
                    inp.OmegaDotFf [x] = 0.0;
                }
                inp.OmegaDotValid = k > 0;
                inp.ReferenceValid = true;
                if (a != 1)
                    inp.Error [a] = -angle;
                inp.OmegaMax = s.OmegaMax;
                double finNative = s.Fins * s.FinColumnNative;
                if (k == 0)
                    realizedLaw = rcsTorque + wheelTorque + ng * gimbalColumn * gimbal + s.LawScale * finNative * deflection;
                inp.Torque [a] = realizedLaw;
                for (int j = 0; j < 3 * n; j++)
                    inp.B [j] = 0.0;
                for (int j = 0; j < ng; j++) {
                    inp.B [a * n + j] = gimbalColumn;
                    inp.U0 [j] = gimbal;
                    inp.UMin [j] = -s.GimbalRange;
                    inp.UMax [j] = s.GimbalRange;
                    inp.UPref [j] = 0.0;
                    inp.Priority [j] = 1.0;
                }
                for (int x = 0; x < 3; x++) {
                    int j = ng + x;
                    inp.U0 [j] = ctrl [x];
                    inp.UPref [j] = 0.0;
                    inp.Priority [j] = 3.0;
                    if (x == a) {
                        inp.UMin [j] = -1.0;
                        inp.UMax [j] = 1.0;
                    } else {
                        // The other ctrlState axes are frozen (an earlier change rule: no authority, bounds at u0).
                        inp.UMin [j] = ctrl [x];
                        inp.UMax [j] = ctrl [x];
                    }
                }
                // The engaged fins' sub-column and their free torque (AttitudeV1Surfaces.SurfacesColumns).
                SurfaceModel.TargetGradient (cmd, ctrl [0], ctrl [1], ctrl [2], grad);
                inp.B [a * n + jCtrl] = instantColumn + s.LawScale * finNative * grad [a];
                double free = SurfaceModel.Step (cmd, deflection, SurfaceModel.Target (cmd, ctrl [0], ctrl [1], ctrl [2]), Dt) -
                    deflection;
                inp.TorqueFree [a] = s.LawScale * finNative * free;
                inp.UseTorqueFree = true;
                AttitudeMath.Step (g, inp, st, work, o);
                // ---- the writes (fly-by-wire for ctrlState; the leased gimbals at Earlyish).
                if (o.Computed) {
                    if (s.ScalarRoll) {
                        double command;
                        if (AttitudeWrite.RollCommand (ctrl [a], o.TorqueAllocTarget [a], inp.B [a * n + jCtrl], out command))
                            ctrl [a] = command;
                    } else {
                        ctrl [a] = Math.Max (-1.0, Math.Min (1.0, ctrl [a] + o.DeltaU [jCtrl]));
                        if (ng > 0)
                            gimbal = o.U [0];
                    }
                }
                if (Math.Abs (ctrl [a]) >= 0.999)
                    saturated++;
                // ---- the parts: the fins move toward the target of this step's ctrlState.
                double target = SurfaceModel.Target (cmd, ctrl [0], ctrl [1], ctrl [2]);
                deflection = SurfaceModel.Step (cmd, deflection, target, Dt);
                // RCS and wheel: first-order lags (the flight's frames), the law reads what they realized.
                rcsTorque = s.RcsAlpha * rcsTorque + (1.0 - s.RcsAlpha) * s.RcsColumn * ctrl [a];
                wheelTorque = s.WheelAlpha * wheelTorque + (1.0 - s.WheelAlpha) * s.WheelColumn * ctrl [a];
                double instant = rcsTorque + wheelTorque;
                double gimbalTorque = ng * gimbalColumn * gimbal;
                // KSP applies the fins' force in kN: 1000 N per unit of the replica.
                double finTrue = 1000.0 * finNative * deflection;
                double torqueTrue = instant + gimbalTorque + finTrue;
                // ---- integration, then the capture the next Earlyish reads (law units).
                omegaPrevious = omega;
                omega += Dt * torqueTrue / s.Inertia;
                angle += Dt * omega;
                realizedLaw = instant + gimbalTorque + s.LawScale * finNative * deflection;
                if (double.IsNaN (omega) || double.IsInfinity (omega)) {
                    res.Finite = false;
                    break;
                }
                double w = Math.Abs (omega);
                res.PeakOmega = Math.Max (res.PeakOmega, w);
                res.PeakAngleDeg = Math.Max (res.PeakAngleDeg, Math.Abs (angle) * 180.0 / Math.PI);
                if (k >= lateFrom) {
                    double sign = Math.Sign (omega);
                    if (sign != 0.0 && lastSign != 0.0 && sign != lastSign && w > 1e-6)
                        crossings++;
                    if (sign != 0.0)
                        lastSign = sign;
                    res.LatePeakOmega = Math.Max (res.LatePeakOmega, w);
                    res.LatePeakU = Math.Max (res.LatePeakU, Math.Abs (ctrl [a]));
                    res.LateAngleDeg = Math.Max (res.LateAngleDeg, Math.Abs (angle) * 180.0 / Math.PI);
                    finSquares += finTrue * finTrue;
                    finCount++;
                }
                if (trace)
                    Console.WriteLine ("{0} k={1} w={2:G4} u={3:G4} defl={4:G4} gim={5:G4} Mtrue={6:G4} Mfin={7:G4} tgt={8:G4} free={9:G4}",
                        s.Name, k, omega, ctrl [a], deflection, gimbal, torqueTrue, finTrue,
                        o.Computed ? o.TorqueAllocTarget [a] : double.NaN, inp.TorqueFree [a]);
            }
            res.SaturatedFraction = (double)saturated / steps;
            res.LatePeriodTicks = crossings > 1 ? 2.0 * (steps - lateFrom) / crossings : 0.0;
            res.LateFinTorqueRms = finCount > 0 ? Math.Sqrt (finSquares / finCount) : 0.0;
            return res;
        }

        // ------------------------------------------------------------- the flight's numbers

        // Roll in the coast (frames of 133848, ut 258560-258600): roll inertia 246 131 kg.m^2, the
        // published RCS + wheel roll column -1.19e5 N.m per unit (wheel -3.0e4, lerp 0.4 per step,
        // exact; RCS a first-order lag of 0.52 per step, the pitch fit of the disarmed flight
        // 134626, steady gain = the published column); each of the 4 fins: roll gain
        // -31.7 deg/unit, roll column +14.0 (replica unit) per degree at q = 98.6 kPa, in
        // proportion to q (9.36 at 67.7 kPa).
        static Scenario RollCoast (double qPa, double lawScale)
        {
            return new Scenario {
                Name = "roll-coast-q" + (qPa / 1000.0).ToString ("0") + "-x" + lawScale.ToString ("0"),
                Axis = 1, Inertia = 246131.0, RcsColumn = -0.89e5, RcsAlpha = 0.52, WheelColumn = -0.30e5, Fins = 4,
                FinColumnNative = 14.0 * qPa / 98600.0, FinGain = -31.7, LawScale = lawScale,
                ScalarRoll = true, Omega0 = 0.01, U0 = 0.0, Seconds = 12.0
            };
        }

        // Pitch at the ignition (frames 258599.8-258601): pitch inertia 9.47e6 kg.m^2, the
        // published RCS + wheel pitch column -5.5e5 N.m per unit (lags as for the roll); 2 fins answer pitch, gain -35 deg/unit,
        // column 121.5 per degree at 98.6 kPa; gimbals: 18 columns, 3.47e5 N.m/deg on pitch in all at
        // throttle 0.30 (9 columns of 1.28e5 each at full throttle here), +-10.5 deg; throttle
        // 0 -> 0.34 in 0.5 s.
        static Scenario PitchIgnition (double qPa, double lawScale, double omega0, double u0)
        {
            return new Scenario {
                Name = "pitch-ignition-q" + (qPa / 1000.0).ToString ("0") + "-x" + lawScale.ToString ("0"),
                Axis = 0, Inertia = 9.47e6, RcsColumn = -5.2e5, RcsAlpha = 0.52, WheelColumn = -0.30e5, Fins = 2,
                FinColumnNative = 121.5 * qPa / 98600.0, FinGain = -35.0, LawScale = lawScale,
                Gimbals = 9, GimbalColumnFull = 1.28e5, ThrottleStart = 0.0, ThrottleEnd = 0.34,
                ThrottleRampS = 0.5, ScalarRoll = false, OmegaMax = 0.14, Omega0 = omega0, U0 = u0,
                Seconds = 8.0
            };
        }

        static void Report (Scenario s, Result r)
        {
            Console.WriteLine ("  {0,-28} peak|w| {1,7:F4}  late peak|w| {2,9:F6}  late|u| {3,5:F3}  saturated {4,5:P0}  late fin rms {5,9:F0} N.m  peak angle {6,5:F1} deg  period {7,4:F1} steps",
                s.Name, r.PeakOmega, r.LatePeakOmega, r.LatePeakU, r.SaturatedFraction, r.LateFinTorqueRms, r.PeakAngleDeg,
                r.LatePeriodTicks);
        }

        internal static int Run (out int checkCount)
        {
            double fix = SurfaceModel.LawTorqueScale (true);
            Check (SurfaceModel.LawTorqueScale (false) == 1.0 && fix == 1000.0,
                "the law's scale of the replica's torques, kN.m -> N.m (1 without the option)");

            // ---- roll in the coast: the flight's limit cycle, and the fix.
            // In the flight: the cycle starts as kappa crosses 2 (q ~ 7 kPa), |omega_y| 0.036 rad/s
            // and |roll| 0.49 at q ~ 66 kPa, saturated from q ~ 85 kPa, period ~16 steps.
            double[] qs = { 3000.0, 7000.0, 20000.0, 40000.0, 68000.0, 98000.0 };
            double previousLate = 0.0;
            foreach (double q in qs) {
                var bug = RollCoast (q, 1.0);
                var rb = Simulate (bug);
                Report (bug, rb);
                var ok = RollCoast (q, fix);
                var rf = Simulate (ok);
                Report (ok, rf);
                // kappa = true / modeled roll column of the ctrlState axis.
                double fin = 4 * 14.0 * q / 98600.0 * 31.7;
                double kappa = (1.19e5 + 1000.0 * fin) / (1.19e5 + fin);
                if (kappa < 1.6)
                    Check (rb.LatePeakOmega < 0.01 * bug.Omega0, "kappa " + kappa.ToString ("F2") +
                        " < 2: the build that flew is stable (" + bug.Name + ")");
                if (kappa > 3.5) {
                    Check (rb.LatePeakOmega > 0.1 * bug.Omega0 && rb.LatePeakU > 0.05 && rb.LatePeriodTicks > 8.0 &&
                        rb.LatePeriodTicks < 24.0, "kappa " + kappa.ToString ("F2") +
                        " > 2: the build that flew reproduces the roll limit cycle (" + bug.Name + ")");
                    Check (rb.LatePeakOmega > previousLate, "the cycle grows with q (" + bug.Name + ")");
                    previousLate = rb.LatePeakOmega;
                }
                Check (rf.Finite && rf.LatePeakOmega < 0.01 * ok.Omega0 && rf.LatePeakU < 0.05,
                    "the roll converges with the fins' torque in N.m (" + ok.Name + ")");
            }
            Check (previousLate > 0.03, "at q = 98 kPa the cycle reaches the flight's amplitude");

            // ---- a large roll disturbance at the highest q: the fins in N.m dominate the ctrlState
            // column (kappa of the build that flew ~ 16) and are rate-limited; the loop must still
            // settle, without a sustained cycle.
            var big = RollCoast (98000.0, fix);
            big.Omega0 = 0.2;
            big.Name = "roll-coast-q98-x1000-w0.2";
            var rbig = Simulate (big);
            Report (big, rbig);
            Check (rbig.Finite && rbig.LatePeakOmega < 1e-3 && rbig.LatePeakU < 0.05,
                "a 0.2 rad/s roll at 98 kPa settles with the fins in N.m");

            // ---- pitch at the ignition: from rest, and from the flight's takeover state (the
            // AutoPilot's -1, 0.28 rad/s inherited from the coast).
            foreach (var start in new[] { new[] { 0.02, 0.0 }, new[] { -0.28, -1.0 } }) {
                var bug = PitchIgnition (98000.0, 1.0, start [0], start [1]);
                var rb = Simulate (bug);
                Report (bug, rb);
                var ok = PitchIgnition (98000.0, fix, start [0], start [1]);
                var rf = Simulate (ok);
                Report (ok, rf);
                Check (rf.Finite && rf.LatePeakOmega < 0.005, "pitch at the ignition settles (" + ok.Name +
                    ", w0 " + start [0] + ")");
            }
            checkCount = checks;
            return failures;
        }
    }
}
