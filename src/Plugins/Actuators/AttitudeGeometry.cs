using System;
using System.Collections.Generic;
using UnityEngine;

namespace KRPC.Bridge.Actuators
{
    /// <summary>
    /// Realized torques and control-effectiveness columns for AttitudeV1, in the vessel
    /// body frame (right, forward, bottom; x pitch, y roll, z yaw), about the current CoM.
    ///
    /// The gimbal columns repeat DynamicsExt.GimbalColumns on purpose instead of calling
    /// it: DynamicsExtV1 is the transport flown first (step M0) and must stay byte-identical
    /// to the deployed build. Factor the two once ExtV1 is re-qualified.
    /// </summary>
    internal static class AttitudeGeometry
    {
        const double DegToRad = Math.PI / 180.0;

        internal static Vector3 Add (Vector3 a, Vector3 b)
        {
            return new Vector3 (a.x + b.x, a.y + b.y, a.z + b.z);
        }

        internal static Vector3 Sub (Vector3 a, Vector3 b)
        {
            return new Vector3 (a.x - b.x, a.y - b.y, a.z - b.z);
        }

        internal static Vector3 Scale (Vector3 a, float s)
        {
            return new Vector3 (a.x * s, a.y * s, a.z * s);
        }

        internal static float Dot (Vector3 a, Vector3 b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        internal static Vector3 Unit (Vector3 a)
        {
            float n = (float)Math.Sqrt ((double)a.x * a.x + (double)a.y * a.y + (double)a.z * a.z);
            return n > 1e-12f ? new Vector3 (a.x / n, a.y / n, a.z / n) : new Vector3 (0f, 0f, 0f);
        }

        static double Multiplier (ModuleEngines engine, int t)
        {
            if (engine.thrustTransformMultipliers != null &&
                    t < engine.thrustTransformMultipliers.Count)
                return engine.thrustTransformMultipliers [t];
            return 1.0;
        }

        static double MultiplierWeightSum (ModuleEngines engine, int count)
        {
            double sum = 0.0;
            for (int t = 0; t < count; t++)
                sum += Math.Abs (Multiplier (engine, t));
            return sum;
        }

        /// <summary>
        /// Realized engine torque, world frame, N.m: sum over nozzles of r x F with
        /// F = -finalThrust * weight * forward (the v3 realized wrench, same split).
        /// </summary>
        internal static Vector3 EngineTorqueWorld (ModuleEngines engine, Vector3 com)
        {
            var torque = new Vector3 (0f, 0f, 0f);
            int count = engine.thrustTransforms == null ? 0 : engine.thrustTransforms.Count;
            double weightSum = MultiplierWeightSum (engine, count);
            if (weightSum <= 1e-12 || !(Math.Abs (engine.finalThrust) > 0f))
                return torque;
            for (int t = 0; t < count; t++) {
                var nozzle = engine.thrustTransforms [t];
                if (nozzle == null)
                    continue;
                float forceN = (float)(engine.finalThrust * 1000.0 * (Multiplier (engine, t) / weightSum));
                Vector3 force = Scale (Unit (nozzle.forward), -forceN);
                torque = Add (torque, DynamicsV3.Cross3 (Sub (nozzle.position, com), force));
            }
            return torque;
        }

        /// <summary>
        /// Columns dM/d(delta_x) and dM/d(delta_y) of one gimbal module, N.m per degree,
        /// body frame, at the realized deflection. Same derivation as DynamicsExtV1: see
        /// the header of DynamicsExt.GimbalColumns and docs/API.md.
        /// </summary>
        internal static void GimbalColumns (ModuleGimbal gimbal, List<ModuleEngines> partEngines,
            Vector3 com, Transform reference, float dx, float dy,
            out Vector3 columnXBody, out Vector3 columnYBody)
        {
            Vector3 columnX = new Vector3 (0f, 0f, 0f);
            Vector3 columnY = new Vector3 (0f, 0f, 0f);
            int count = (gimbal.gimbalTransforms == null || gimbal.initRots == null) ? 0
                : Math.Min (gimbal.gimbalTransforms.Count, gimbal.initRots.Count);
            for (int i = 0; i < count; i++) {
                var pivot = gimbal.gimbalTransforms [i];
                if (pivot == null)
                    continue;
                Quaternion parentRotation = pivot.parent != null
                    ? pivot.parent.rotation : Quaternion.identity;
                Quaternion q0 = parentRotation * gimbal.initRots [i];
                Vector3 xAxisLocal = Scale (Vector3.right, gimbal.xMult);
                Vector3 second = gimbal.flipYZ ? Vector3.forward : Vector3.up;
                Vector3 axisX = Unit (q0 * xAxisLocal);
                Vector3 axisY = Unit (q0 * Quaternion.AngleAxis (dx, xAxisLocal)
                                      * Scale (second, gimbal.yMult));
                for (int e = 0; e < partEngines.Count; e++) {
                    var engine = partEngines [e];
                    int transformCount = engine.thrustTransforms == null
                        ? 0 : engine.thrustTransforms.Count;
                    double weightSum = MultiplierWeightSum (engine, transformCount);
                    if (weightSum <= 1e-12)
                        continue;
                    for (int t = 0; t < transformCount; t++) {
                        var nozzle = engine.thrustTransforms [t];
                        if (nozzle == null || !nozzle.IsChildOf (pivot))
                            continue;
                        float thrustN = (float)(engine.finalThrust * 1000.0 *
                            (Multiplier (engine, t) / weightSum));
                        Vector3 direction = Unit (nozzle.forward);
                        Vector3 force = Scale (direction, -thrustN);
                        Vector3 lever = Sub (nozzle.position, com);
                        Vector3 offset = Sub (nozzle.position, pivot.position);
                        columnX = Add (columnX, TorqueDerivative (axisX, direction, force, lever,
                            offset, thrustN));
                        columnY = Add (columnY, TorqueDerivative (axisY, direction, force, lever,
                            offset, thrustN));
                    }
                }
            }
            columnXBody = reference.InverseTransformDirection (Scale (columnX, (float)DegToRad));
            columnYBody = reference.InverseTransformDirection (Scale (columnY, (float)DegToRad));
        }

        /// <summary>d(r x F)/d(theta) for a rotation of the nozzle about axis, per radian.</summary>
        static Vector3 TorqueDerivative (Vector3 axis, Vector3 direction, Vector3 force,
            Vector3 lever, Vector3 offset, float thrustN)
        {
            Vector3 dForce = Scale (DynamicsV3.Cross3 (axis, direction), -thrustN);
            Vector3 dLever = DynamicsV3.Cross3 (axis, offset);
            return Add (DynamicsV3.Cross3 (dLever, force), DynamicsV3.Cross3 (lever, dForce));
        }

        /// <summary>Realized RCS torque, world frame, N.m (thrustForces of the last FixedUpdate).</summary>
        internal static Vector3 RcsTorqueWorld (ModuleRCS rcs, Vector3 com)
        {
            var torque = new Vector3 (0f, 0f, 0f);
            if (rcs.isJustForShow)
                return torque;
            int count = rcs.thrusterTransforms == null ? 0 : rcs.thrusterTransforms.Count;
            int forces = rcs.thrustForces == null ? 0 : rcs.thrustForces.Length;
            for (int i = 0; i < Math.Min (count, forces); i++) {
                float thrust = rcs.thrustForces [i];
                var thruster = rcs.thrusterTransforms [i];
                if (thruster == null || !(thrust > 0f))
                    continue;
                Vector3 axis = rcs.useZaxis ? thruster.forward : thruster.up;
                Vector3 f = Scale (axis, -1000f * thrust);
                torque = Add (torque, DynamicsV3.Cross3 (Sub (thruster.position, com), f));
            }
            return torque;
        }

        /// <summary>
        /// Maximum force of one RCS nozzle, kN: ModuleRCS.CalculateThrust at full demand,
        /// flowMult * maxFuelFlow * thrustPercentage% * realISP * G * ispMult, full tanks.
        /// </summary>
        internal static double RcsNozzleForceKn (ModuleRCS rcs)
        {
            if (!rcs.requiresFuel)
                return 1.0;
            return rcs.flowMult * rcs.maxFuelFlow * ((double)rcs.thrustPercentage * 0.01)
                * (double)rcs.realISP * rcs.G * rcs.ispMult;
        }

        /// <summary>
        /// Adds the columns of one RCS module to the ctrlState channel: the torque produced
        /// by a unit pitch/roll/yaw command (bPos, axis a at [3a..3a+2], per unit u &gt; 0)
        /// and the torque per unit of a negative command (bNeg, so torque = u * bNeg for
        /// u &lt; 0). Reproduces ModuleRCS.FixedUpdate: inputRot = rotation * command,
        /// rot = inputRot x unit(ProjectOnPlane(r, inputRot)), fraction max(v . rot, 0).
        /// Linear below full thrust for a single-axis command, which is what a column is.
        /// Returns false when a nonlinearity the model ignores is active (precision mode,
        /// fullThrust), for the frame's flags.
        /// </summary>
        internal static bool AddRcsColumns (ModuleRCS rcs, Vector3 com, Transform reference,
            bool precisionMode, double[] bPos, double[] bNeg)
        {
            if (!rcs.moduleIsEnabled || !rcs.rcsEnabled || !rcs.rcs_active || rcs.isJustForShow)
                return true;
            if (rcs.part != null && rcs.part.ShieldedFromAirstream && !rcs.shieldedCanThrust)
                return true;
            int count = rcs.thrusterTransforms == null ? 0 : rcs.thrusterTransforms.Count;
            double power = RcsNozzleForceKn (rcs);
            if (!(power > 0.0) || count == 0)
                return true;
            Vector3[] axes = { reference.right, reference.up, reference.forward };
            bool[] enabled = { rcs.enablePitch, rcs.enableRoll, rcs.enableYaw };
            for (int a = 0; a < 3; a++) {
                if (!enabled [a])
                    continue;
                Vector3 axis = Unit (axes [a]);
                for (int s = 0; s < 2; s++) {
                    float sign = s == 0 ? 1f : -1f;
                    Vector3 inputRot = Scale (axis, sign);
                    var torque = new Vector3 (0f, 0f, 0f);
                    for (int i = 0; i < count; i++) {
                        var thruster = rcs.thrusterTransforms [i];
                        if (thruster == null)
                            continue;
                        Vector3 r = Sub (thruster.position, com);
                        // ProjectOnPlane(r, n) = r - n (r . n) / |n|^2, |n| = 1 here.
                        Vector3 rPerp = Sub (r, Scale (inputRot, Dot (r, inputRot)));
                        Vector3 rot = DynamicsV3.Cross3 (inputRot, Unit (rPerp));
                        Vector3 v = rcs.useZaxis ? thruster.forward : thruster.up;
                        float fraction = Dot (v, rot);
                        if (!(fraction > 0f))
                            continue;
                        if (fraction > 1f)
                            fraction = 1f;
                        Vector3 f = Scale (v, (float)(-1000.0 * power * fraction));
                        torque = Add (torque, DynamicsV3.Cross3 (r, f));
                    }
                    Vector3 body = reference.InverseTransformDirection (torque);
                    double[] target = s == 0 ? bPos : bNeg;
                    // bNeg is torque per unit of u for u < 0: the command -1 gave 'body'.
                    double k = s == 0 ? 1.0 : -1.0;
                    target [3 * a] += k * body.x;
                    target [3 * a + 1] += k * body.y;
                    target [3 * a + 2] += k * body.z;
                }
            }
            return !(precisionMode || rcs.fullThrust);
        }

        /// <summary>
        /// Reaction wheel: realized torque (body, N.m) and its ctrlState columns.
        /// ModuleReactionWheel.ActiveUpdate applies AddTorque(rotation * -inputVector) with
        /// inputVector = (pitch * PitchTorque, roll * RollTorque, yaw * YawTorque) * authority,
        /// kN.m, lerped at torqueResponseSpeed.
        /// </summary>
        internal static bool AddWheel (ModuleReactionWheel wheel, double[] torqueBody,
            double[] bPos, double[] bNeg)
        {
            if (!wheel.moduleIsEnabled || !wheel.operational ||
                    wheel.wheelState != ModuleReactionWheel.WheelState.Active)
                return false;
            torqueBody [0] -= wheel.inputVector.x * 1000.0;
            torqueBody [1] -= wheel.inputVector.y * 1000.0;
            torqueBody [2] -= wheel.inputVector.z * 1000.0;
            if (wheel.actuatorModeCycle != 0)
                return true;        // SAS-only or pilot-only: not driven by the autopilot path
            double authority = Math.Min (100.0, Math.Max (0.0, (double)wheel.authorityLimiter)) * 0.01;
            double[] per = { wheel.PitchTorque, wheel.RollTorque, wheel.YawTorque };
            for (int a = 0; a < 3; a++) {
                double column = -per [a] * authority * 1000.0;
                bPos [3 * a + a] += column;
                bNeg [3 * a + a] += column;
            }
            return true;
        }

        /// <summary>
        /// Control surface: ITorqueProvider.GetPotentialTorque magnitudes, applied with the
        /// sign every stock effector shares (a positive command gives a negative body torque,
        /// as the wheel and RCS formulas above). APPROXIMATE: the stock estimate scales the
        /// lever component-wise. Flagged in every frame that uses it.
        /// </summary>
        internal static bool AddSurface (ModuleControlSurface surface, double[] bPos, double[] bNeg)
        {
            var provider = surface as ITorqueProvider;
            if (provider == null || !surface.moduleIsEnabled)
                return false;
            Vector3 pos, neg;
            provider.GetPotentialTorque (out pos, out neg);
            double[] p = { Math.Abs (pos.x), Math.Abs (pos.y), Math.Abs (pos.z) };
            double[] n = { Math.Abs (neg.x), Math.Abs (neg.y), Math.Abs (neg.z) };
            bool any = false;
            for (int a = 0; a < 3; a++) {
                if (!(p [a] > 0.0) && !(n [a] > 0.0))
                    continue;
                if (double.IsNaN (p [a]) || double.IsInfinity (p [a]) ||
                        double.IsNaN (n [a]) || double.IsInfinity (n [a]))
                    continue;
                bPos [3 * a + a] -= p [a] * 1000.0;
                bNeg [3 * a + a] -= n [a] * 1000.0;
                any = true;
            }
            return any;
        }
    }
}
