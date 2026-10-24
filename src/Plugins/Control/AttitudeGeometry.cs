using System;
using System.Collections.Generic;
using UnityEngine;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// Realized torques and control-effectiveness columns for AttitudeV1, in the vessel
    /// body frame (right, forward, bottom; x pitch, y roll, z yaw), about the current CoM.
    ///
    /// The gimbal columns repeat DynamicsExt.GimbalColumns on purpose instead of calling
    /// it: DynamicsExtV1 is the transport flown first (step M0) and must stay byte-identical
    /// to the deployed build. Factor the two once ExtV1 is re-qualified.
    ///
    /// GimbalColumns (and GimbalAxesAndForce) model ONE (x, y) applied to every pivot in
    /// its own frame -- what the lease does. For a module whose pivots are not parallel (AR-1
    /// Cluster) the *Pivots variants model the per-pivot mapping of GimbalPivots.cs instead,
    /// when the client armed it (AttitudeGimbalPivotsV1); the V1 functions are unchanged.
    /// </summary>
    internal static class AttitudeGeometry
    {
        const double DegToRad = Math.PI / 180.0;

        /// <summary>
        /// a x b in float, component by component (the arithmetic of Unity's Vector3.Cross).
        /// The one definition for Control and Telemetry (refactor R3: it was DynamicsV3.Cross3).
        /// </summary>
        internal static Vector3 Cross3 (Vector3 a, Vector3 b)
        {
            return new Vector3 (
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x);
        }

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
            return EngineTorqueWorld (engine, com, engine.finalThrust);
        }

        /// <summary>
        /// EngineTorqueWorld at the thrust thrustKn (kN) instead of finalThrust, the nozzles
        /// where they are. With thrustKn = finalThrust: the same operations, the same floats.
        /// </summary>
        internal static Vector3 EngineTorqueWorld (ModuleEngines engine, Vector3 com, double thrustKn)
        {
            var torque = new Vector3 (0f, 0f, 0f);
            int count = engine.thrustTransforms == null ? 0 : engine.thrustTransforms.Count;
            double weightSum = MultiplierWeightSum (engine, count);
            if (weightSum <= 1e-12 || !(Math.Abs (thrustKn) > 0.0))
                return torque;
            for (int t = 0; t < count; t++) {
                var nozzle = engine.thrustTransforms [t];
                if (nozzle == null)
                    continue;
                float forceN = (float)(thrustKn * 1000.0 * (Multiplier (engine, t) / weightSum));
                Vector3 force = Scale (Unit (nozzle.forward), -forceN);
                torque = Add (torque, Cross3 (Sub (nozzle.position, com), force));
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
            GimbalColumns (gimbal, partEngines, com, reference, dx, dy, null, out columnXBody, out columnYBody);
        }

        /// <summary>
        /// GimbalColumns with each engine's thrust taken from thrustKn (kN, aligned with
        /// partEngines) instead of finalThrust; null = finalThrust, the same operations and floats.
        /// </summary>
        internal static void GimbalColumns (ModuleGimbal gimbal, List<ModuleEngines> partEngines,
            Vector3 com, Transform reference, float dx, float dy, double[] thrustKn,
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
                    double kn = thrustKn != null && e < thrustKn.Length ? thrustKn [e] : engine.finalThrust;
                    for (int t = 0; t < transformCount; t++) {
                        var nozzle = engine.thrustTransforms [t];
                        if (nozzle == null || !nozzle.IsChildOf (pivot))
                            continue;
                        float thrustN = (float)(kn * 1000.0 *
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

        /// <summary>
        /// Lateral force (chantier 19 suite). Body-frame rotation axes of one gimbal module at
        /// the realized x deflection - the axes GimbalColumns differentiates about, averaged
        /// over its pivots and made unit - and its force columns dF/d(delta_x), dF/d(delta_y)
        /// (N per degree, body), same derivation as the torque columns (d force = axis x
        /// direction x -thrust). False when the module has no usable pivot.
        /// </summary>
        internal static bool GimbalAxesAndForce (ModuleGimbal gimbal, List<ModuleEngines> partEngines,
            Transform reference, float dx, out Vector3 axisXBody, out Vector3 axisYBody,
            out Vector3 forceXBody, out Vector3 forceYBody)
        {
            Vector3 axisXSum = new Vector3 (0f, 0f, 0f), axisYSum = new Vector3 (0f, 0f, 0f);
            Vector3 forceX = new Vector3 (0f, 0f, 0f), forceY = new Vector3 (0f, 0f, 0f);
            int pivots = 0;
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
                axisXSum = Add (axisXSum, axisX);
                axisYSum = Add (axisYSum, axisY);
                pivots++;
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
                        forceX = Add (forceX, Scale (Cross3 (axisX, direction), -thrustN));
                        forceY = Add (forceY, Scale (Cross3 (axisY, direction), -thrustN));
                    }
                }
            }
            axisXBody = reference.InverseTransformDirection (Unit (axisXSum));
            axisYBody = reference.InverseTransformDirection (Unit (axisYSum));
            forceXBody = reference.InverseTransformDirection (Scale (forceX, (float)DegToRad));
            forceYBody = reference.InverseTransformDirection (Scale (forceY, (float)DegToRad));
            return pivots > 0;
        }

        /// <summary>
        /// Lateral force (chantier 19 suite). Nozzle centroid of one engine relative to the
        /// CoM (thrust-transform multipliers as weights) and its torque per newton of thrust,
        /// sum_t w_t (r_t x -forward_t), both in the body frame. False without a nozzle.
        /// </summary>
        internal static bool EngineLever (ModuleEngines engine, Vector3 com, Transform reference,
            out Vector3 centroidBody, out Vector3 torquePerNewtonBody)
        {
            var centroid = new Vector3 (0f, 0f, 0f);
            var torque = new Vector3 (0f, 0f, 0f);
            centroidBody = centroid;
            torquePerNewtonBody = torque;
            int count = engine.thrustTransforms == null ? 0 : engine.thrustTransforms.Count;
            double weightSum = MultiplierWeightSum (engine, count);
            if (weightSum <= 1e-12)
                return false;
            double used = 0.0;
            for (int t = 0; t < count; t++) {
                var nozzle = engine.thrustTransforms [t];
                if (nozzle == null)
                    continue;
                float w = (float)(Math.Abs (Multiplier (engine, t)) / weightSum);
                Vector3 lever = Sub (nozzle.position, com);
                Vector3 direction = Scale (Unit (nozzle.forward), -1f);
                centroid = Add (centroid, Scale (lever, w));
                torque = Add (torque, Scale (Cross3 (lever, direction), w));
                used += w;
            }
            if (!(used > 1e-9))
                return false;
            // Missing nozzles: the centroid of the present ones.
            centroid = Scale (centroid, (float)(1.0 / used));
            torque = Scale (torque, (float)(1.0 / used));
            centroidBody = reference.InverseTransformDirection (centroid);
            torquePerNewtonBody = reference.InverseTransformDirection (torque);
            return true;
        }

        /// <summary>
        /// Computed levers: an engine's thrust direction (body unit vector, the thrust-transform
        /// multipliers as weights), sum_t w_t (-forward_t), at the nozzles' realized orientation.
        /// False without a nozzle or when the directions cancel.
        /// </summary>
        internal static bool EngineDirection (ModuleEngines engine, Transform reference,
            out Vector3 directionBody)
        {
            directionBody = new Vector3 (0f, 0f, 0f);
            int count = engine.thrustTransforms == null ? 0 : engine.thrustTransforms.Count;
            double weightSum = MultiplierWeightSum (engine, count);
            if (weightSum <= 1e-12)
                return false;
            var sum = new Vector3 (0f, 0f, 0f);
            for (int t = 0; t < count; t++) {
                var nozzle = engine.thrustTransforms [t];
                if (nozzle == null)
                    continue;
                float w = (float)(Math.Abs (Multiplier (engine, t)) / weightSum);
                sum = Add (sum, Scale (Unit (nozzle.forward), -w));
            }
            double n = Math.Sqrt ((double)sum.x * sum.x + (double)sum.y * sum.y + (double)sum.z * sum.z);
            if (!(n > 1e-9))
                return false;
            directionBody = reference.InverseTransformDirection (Scale (sum, (float)(1.0 / n)));
            return true;
        }

        // ------------------------------------------------------------------ pivots

        // Scratch of the pivot paths (Unity main thread only, like every caller).
        static readonly double[] pivotAxisX = new double[3], pivotAxisY = new double[3];
        static readonly double[] pivotDir = new double[3], pivotLever = new double[3];
        static readonly double[] pivotOffset = new double[3];
        static readonly double[] pivotCX = new double[3], pivotCY = new double[3];
        static readonly double[] pivotFX = new double[3], pivotFY = new double[3];
        static readonly double[] pivotColX = new double[3], pivotColY = new double[3];
        static readonly double[] pivotForceX = new double[3], pivotForceY = new double[3];
        static readonly double[] pivotFreeX = new double[3], pivotFreeY = new double[3];

        static void Put (double[] target, Vector3 v)
        {
            target [0] = v.x;
            target [1] = v.y;
            target [2] = v.z;
        }

        static void Zero (double[] v)
        {
            v [0] = 0.0;
            v [1] = 0.0;
            v [2] = 0.0;
        }

        static Vector3 ToVector (double[] v)
        {
            return new Vector3 ((float)v [0], (float)v [1], (float)v [2]);
        }

        /// <summary>
        /// fills p with the module's pivots AT REST (world unit axes of each pivot's two
        /// lease rotations: parent rotation x initRots[i] applied to xMult right and to yMult up
        /// -- forward with flipYZ --, as ApplyGimbal and ModuleGimbal.FixedUpdate rotate), its box
        /// (ranges x limiter) and the per-pivot map (GimbalPivots.Build). True when the module is
        /// mapped (two pivots or more, not all parallel to pivot 0): false = the V1 path.
        /// </summary>
        internal static bool RefreshPivots (ModuleGimbal gimbal, GimbalPivotCommand p)
        {
            int count = (gimbal.gimbalTransforms == null || gimbal.initRots == null) ? 0
                : Math.Min (gimbal.gimbalTransforms.Count, gimbal.initRots.Count);
            p.Resize (count);
            if (count < 2)
                return false;
            Vector3 xAxisLocal = Scale (Vector3.right, gimbal.xMult);
            Vector3 second = Scale (gimbal.flipYZ ? Vector3.forward : Vector3.up, gimbal.yMult);
            for (int i = 0; i < count; i++) {
                int a = GimbalPivots.AxesStride * i;
                var pivot = gimbal.gimbalTransforms [i];
                if (pivot == null) {
                    for (int k = 0; k < GimbalPivots.AxesStride; k++)
                        p.Axes [a + k] = 0.0;
                    continue;
                }
                Quaternion parentRotation = pivot.parent != null
                    ? pivot.parent.rotation : Quaternion.identity;
                Quaternion q0 = parentRotation * gimbal.initRots [i];
                Vector3 ax = Unit (q0 * xAxisLocal);
                Vector3 ay = Unit (q0 * second);
                p.Axes [a] = ax.x;
                p.Axes [a + 1] = ax.y;
                p.Axes [a + 2] = ax.z;
                p.Axes [a + 3] = ay.x;
                p.Axes [a + 4] = ay.y;
                p.Axes [a + 5] = ay.z;
            }
            double limiter = Math.Min (100.0, Math.Max (0.0, (double)gimbal.gimbalLimiter)) * 0.01;
            p.XMin = -Math.Abs ((double)gimbal.gimbalRangeXN) * limiter;
            p.XMax = Math.Abs ((double)gimbal.gimbalRangeXP) * limiter;
            p.YMin = -Math.Abs ((double)gimbal.gimbalRangeYN) * limiter;
            p.YMax = Math.Abs ((double)gimbal.gimbalRangeYP) * limiter;
            return GimbalPivots.Build (p.Axes, count, p.Map);
        }

        /// <summary>
        /// the module deflection (degrees, frame of pivot 0) of a mapped module that STOCK
        /// drives: pivot 0's own rotation. With a response speed, stock keeps every pivot's lerped
        /// rotation in oldActuationLocal[i]; without, actuationLocal holds the LAST pivot's
        /// (FixedUpdate loops over the pivots), brought back through its row.
        /// </summary>
        internal static void PivotZeroDeflection (ModuleGimbal gimbal, GimbalPivotCommand p,
            out float dx, out float dy)
        {
            dx = gimbal.actuationLocal.x;
            dy = gimbal.actuationLocal.y;
            if (gimbal.useGimbalResponseSpeed && gimbal.oldActuationLocal != null &&
                    gimbal.oldActuationLocal.Length > 0) {
                dx = gimbal.oldActuationLocal [0].x;
                dy = gimbal.oldActuationLocal [0].y;
                return;
            }
            double x, y;
            int last = p.Count - 1;
            if (last >= 0 && GimbalPivots.ModuleFromPivot (p.Map, last, dx, dy, out x, out y)) {
                dx = (float)x;
                dy = (float)y;
            }
        }

        /// <summary>
        /// the columns of a MAPPED module (N.m per degree of the module command, body): per
        /// pivot, dM/d(x_i) and dM/d(y_i) at its realized rotations (x_i, y_i) -- the V1 formula
        /// (GimbalPivots.AddNozzleDerivative) over the nozzles below that pivot --, then the chain
        /// rule through the pivot's row (GimbalPivots.AddPivotColumns). A component clamped at the
        /// box contributes nothing: the exact local derivative, what INDI needs.
        ///
        /// freeXBody / freeYBody: the same chain rule with EVERY component free (the linear model
        /// of the module, no pivot at its bound), for the law's authority. Measured in the clamps
        /// (ground check an earlier change, 27/09): under the law the WLS drives a mapped module to its box
        /// corner, |w| up to 2.83 deg, beyond the 2 deg where every pivot component is free; 2 or 4
        /// of its 12 components clamp and the local columns fall to sqrt(31)/6 and sqrt(19)/6 of
        /// the free ones (70 890 and 55 500 N.m/deg published for 76 460). The authority sum
        /// |B| x span read on them fell to about 0.66 of the torque the lease realized: the nu
        /// bound, the sqrt reference and the guards would see the authority collapse at
        /// saturation.
        /// </summary>
        internal static void GimbalColumnsPivots (ModuleGimbal gimbal, List<ModuleEngines> partEngines,
            Vector3 com, Transform reference, GimbalPivotCommand p, float dx, float dy,
            out Vector3 columnXBody, out Vector3 columnYBody, out Vector3 freeXBody,
            out Vector3 freeYBody)
        {
            GimbalColumnsPivots (gimbal, partEngines, com, reference, p, dx, dy, null, out columnXBody,
                out columnYBody, out freeXBody, out freeYBody);
        }

        /// <summary>
        /// GimbalColumnsPivots with each engine's thrust from thrustKn (kN, aligned with
        /// partEngines); null = finalThrust, the same operations.
        /// </summary>
        internal static void GimbalColumnsPivots (ModuleGimbal gimbal, List<ModuleEngines> partEngines,
            Vector3 com, Transform reference, GimbalPivotCommand p, float dx, float dy, double[] thrustKn,
            out Vector3 columnXBody, out Vector3 columnYBody, out Vector3 freeXBody,
            out Vector3 freeYBody)
        {
            PivotSums (gimbal, partEngines, com, p, dx, dy, pivotColX, pivotColY, null, null,
                pivotFreeX, pivotFreeY, false, p.TorqueCols, null, thrustKn);
            GimbalPivots.PerDegree (pivotColX);
            GimbalPivots.PerDegree (pivotColY);
            GimbalPivots.PerDegree (pivotFreeX);
            GimbalPivots.PerDegree (pivotFreeY);
            columnXBody = reference.InverseTransformDirection (ToVector (pivotColX));
            columnYBody = reference.InverseTransformDirection (ToVector (pivotColY));
            freeXBody = reference.InverseTransformDirection (ToVector (pivotFreeX));
            freeYBody = reference.InverseTransformDirection (ToVector (pivotFreeY));
            // the per-pivot torque columns too (body, per degree), for the torque the module
            // REALIZES at any command (feed-forward of a common deflection, exact authority).
            PivotColumnsToBody (reference, p.TorqueCols, p.Count);
            p.TorqueColsValid = true;
        }

        /// <summary>
        /// lateral force: GimbalAxesAndForce for a MAPPED module. The module rotates every
        /// pivot by one world rotation, w = x X0 + y Y0: its axes are pivot 0's (Y0 turned by the
        /// x rotation, as V1 does per pivot); its force columns are the chain rule of the pivots'.
        ///
        /// those force columns are the FREE ones (every pivot component free: the module's
        /// linear model from rest), not the local ones at (dx, dy). The translation solves a
        /// TOTAL deflection from rest (tilt matrix, K phi = m a): read on the local columns, a
        /// clamped component counted nothing and the tilt matrix fell -- to 0.67 along a pivot
        /// axis as soon as |w| passes the range, 0.38 at the 0.8 box's diagonal --, so the solved
        /// tilt jumped up by as much, pushing the module further into its clamps. The free
        /// columns are exact inside the disc |w| &lt;= range and within 6 % at the 0.8 box. The
        /// per-pivot force columns go to p.ForceCols (the force the module realizes).
        /// </summary>
        internal static bool GimbalAxesAndForcePivots (ModuleGimbal gimbal,
            List<ModuleEngines> partEngines, Transform reference, GimbalPivotCommand p, float dx,
            float dy, out Vector3 axisXBody, out Vector3 axisYBody, out Vector3 forceXBody,
            out Vector3 forceYBody)
        {
            axisXBody = axisYBody = forceXBody = forceYBody = new Vector3 (0f, 0f, 0f);
            var pivot0 = p.Count > 0 && gimbal.gimbalTransforms != null
                ? gimbal.gimbalTransforms [0] : null;
            if (pivot0 == null)
                return false;
            PivotSums (gimbal, partEngines, new Vector3 (0f, 0f, 0f), p, dx, dy, pivotColX,
                pivotColY, pivotForceX, pivotForceY, null, null, true, null, p.ForceCols, null);
            PivotColumnsToBody (reference, p.ForceCols, p.Count);
            p.ForceColsValid = true;
            Quaternion parentRotation = pivot0.parent != null
                ? pivot0.parent.rotation : Quaternion.identity;
            Quaternion q0 = parentRotation * gimbal.initRots [0];
            Vector3 xAxisLocal = Scale (Vector3.right, gimbal.xMult);
            Vector3 second = gimbal.flipYZ ? Vector3.forward : Vector3.up;
            Vector3 axisX = Unit (q0 * xAxisLocal);
            Vector3 axisY = Unit (q0 * Quaternion.AngleAxis (dx, xAxisLocal)
                                  * Scale (second, gimbal.yMult));
            GimbalPivots.PerDegree (pivotForceX);
            GimbalPivots.PerDegree (pivotForceY);
            axisXBody = reference.InverseTransformDirection (axisX);
            axisYBody = reference.InverseTransformDirection (axisY);
            forceXBody = reference.InverseTransformDirection (ToVector (pivotForceX));
            forceYBody = reference.InverseTransformDirection (ToVector (pivotForceY));
            return true;
        }

        /// <summary>
        /// per-pivot columns (GimbalPivots.ColumnStride per pivot), world per radian, to
        /// body per degree in place -- the pipeline of the module columns (per degree in double,
        /// then the float body conversion).
        /// </summary>
        static void PivotColumnsToBody (Transform reference, double[] cols, int count)
        {
            for (int k = 0; k < 2 * count; k++) {
                int o = 3 * k;
                Vector3 b = reference.InverseTransformDirection (new Vector3 (
                    (float)(cols [o] * DegToRad), (float)(cols [o + 1] * DegToRad),
                    (float)(cols [o + 2] * DegToRad)));
                cols [o] = b.x;
                cols [o + 1] = b.y;
                cols [o + 2] = b.z;
            }
        }

        /// <summary>
        /// Module sums of a mapped module, per radian, world: torque columns (colX, colY) about
        /// com and, when forceX is not null, force columns. Per pivot: its rotations for the
        /// module command (clamped), its axes at the realized x rotation, the derivative over its
        /// nozzles, then the chain rule. When freeX is not null: the torque columns with every
        /// component free (no clamp), in the same pass. ForceFree sums the force columns
        /// with every component free; pivotTorque / pivotForce (not null) receive each pivot's
        /// own columns (GimbalPivots.ColumnStride per pivot, world per radian; zero for a
        /// missing pivot). The module sums are unchanged by these three. ThrustKn not
        /// null = each engine's thrust (kN, aligned with partEngines) instead of finalThrust.
        /// </summary>
        static void PivotSums (ModuleGimbal gimbal, List<ModuleEngines> partEngines, Vector3 com,
            GimbalPivotCommand p, float dx, float dy, double[] colX, double[] colY,
            double[] forceX, double[] forceY, double[] freeX, double[] freeY, bool forceFree,
            double[] pivotTorque, double[] pivotForce, double[] thrustKn)
        {
            Zero (colX);
            Zero (colY);
            if (forceX != null) {
                Zero (forceX);
                Zero (forceY);
            }
            if (freeX != null) {
                Zero (freeX);
                Zero (freeY);
            }
            for (int k = 0; k < GimbalPivots.ColumnStride * p.Count; k++) {
                if (pivotTorque != null)
                    pivotTorque [k] = 0.0;
                if (pivotForce != null)
                    pivotForce [k] = 0.0;
            }
            Vector3 xAxisLocal = Scale (Vector3.right, gimbal.xMult);
            Vector3 second = gimbal.flipYZ ? Vector3.forward : Vector3.up;
            int count = Math.Min (p.Count, gimbal.gimbalTransforms == null
                ? 0 : gimbal.gimbalTransforms.Count);
            for (int i = 0; i < count; i++) {
                var pivot = gimbal.gimbalTransforms [i];
                if (pivot == null)
                    continue;
                double xi, yi;
                bool xFree, yFree;
                GimbalPivots.PivotCommand (p.Map, i, dx, dy, p.XMin, p.XMax, p.YMin, p.YMax,
                    out xi, out yi, out xFree, out yFree);
                Quaternion parentRotation = pivot.parent != null
                    ? pivot.parent.rotation : Quaternion.identity;
                Quaternion q0 = parentRotation * gimbal.initRots [i];
                Put (pivotAxisX, Unit (q0 * xAxisLocal));
                Put (pivotAxisY, Unit (q0 * Quaternion.AngleAxis ((float)xi, xAxisLocal)
                                       * Scale (second, gimbal.yMult)));
                Zero (pivotCX);
                Zero (pivotCY);
                Zero (pivotFX);
                Zero (pivotFY);
                for (int e = 0; e < partEngines.Count; e++) {
                    var engine = partEngines [e];
                    int transformCount = engine.thrustTransforms == null
                        ? 0 : engine.thrustTransforms.Count;
                    double weightSum = MultiplierWeightSum (engine, transformCount);
                    if (weightSum <= 1e-12)
                        continue;
                    double kn = thrustKn != null && e < thrustKn.Length ? thrustKn [e] : engine.finalThrust;
                    for (int t = 0; t < transformCount; t++) {
                        var nozzle = engine.thrustTransforms [t];
                        if (nozzle == null || !nozzle.IsChildOf (pivot))
                            continue;
                        double thrustN = kn * 1000.0 *
                            (Multiplier (engine, t) / weightSum);
                        Put (pivotDir, Unit (nozzle.forward));
                        Put (pivotLever, Sub (nozzle.position, com));
                        Put (pivotOffset, Sub (nozzle.position, pivot.position));
                        GimbalPivots.AddNozzleDerivative (pivotAxisX, pivotDir, pivotLever,
                            pivotOffset, thrustN, pivotCX, 0, forceX != null ? pivotFX : null);
                        GimbalPivots.AddNozzleDerivative (pivotAxisY, pivotDir, pivotLever,
                            pivotOffset, thrustN, pivotCY, 0, forceX != null ? pivotFY : null);
                    }
                }
                GimbalPivots.AddPivotColumns (p.Map, i, xFree, yFree, pivotCX, pivotCY, colX, colY);
                if (freeX != null)
                    GimbalPivots.AddPivotColumns (p.Map, i, true, true, pivotCX, pivotCY, freeX, freeY);
                if (forceX != null)
                    GimbalPivots.AddPivotColumns (p.Map, i, forceFree || xFree, forceFree || yFree,
                        pivotFX, pivotFY, forceX, forceY);
                int c = GimbalPivots.ColumnStride * i;
                for (int r = 0; r < 3; r++) {
                    if (pivotTorque != null) {
                        pivotTorque [c + r] = pivotCX [r];
                        pivotTorque [c + 3 + r] = pivotCY [r];
                    }
                    if (pivotForce != null) {
                        pivotForce [c + r] = pivotFX [r];
                        pivotForce [c + 3 + r] = pivotFY [r];
                    }
                }
            }
        }

        /// <summary>d(r x F)/d(theta) for a rotation of the nozzle about axis, per radian.</summary>
        static Vector3 TorqueDerivative (Vector3 axis, Vector3 direction, Vector3 force,
            Vector3 lever, Vector3 offset, float thrustN)
        {
            Vector3 dForce = Scale (Cross3 (axis, direction), -thrustN);
            Vector3 dLever = Cross3 (axis, offset);
            return Add (Cross3 (dLever, force), Cross3 (lever, dForce));
        }

        /// <summary>Realized RCS torque, world frame, N.m (thrustForces of the last FixedUpdate).</summary>
        internal static Vector3 RcsTorqueWorld (ModuleRCS rcs, Vector3 com)
        {
            var torque = new Vector3 (0f, 0f, 0f);
            if (rcs.isJustForShow)
                return torque;
            var force = new Vector3 (0f, 0f, 0f);
            double thrustSum = 0.0;
            AddRcsWrenchWorld (rcs, com, ref force, ref torque, ref thrustSum);
            return torque;
        }

        /// <summary>
        /// The realized wrench of one RCS module (thrustForces of the last FixedUpdate), world
        /// frame, about com, ADDED nozzle by nozzle in order into force (N) and torque (N.m);
        /// thrustSum (kN) counts every firing nozzle, for show or not. ModuleRCS.FixedUpdate
        /// (KSP 1.12.5, also inherited by ModuleRCSFX) resets thrustForces[] every tick, then
        /// applies AddForceAtPosition(-v * thrustForces[i], thruster position) with v = useZaxis
        /// ? forward : up, in kN - except for a module marked isJustForShow, which fills
        /// thrustForces but applies nothing.
        ///
        /// The one computation of DynamicsExtV1's RCS rows and of AttitudeV1's realized RCS
        /// torque (refactor R3; audit A 3.2 found it written twice, with the same operations).
        /// The sums are written through the references as they grow, so a caller that catches
        /// an exception keeps what was added before it, as the two copies did.
        /// </summary>
        internal static void AddRcsWrenchWorld (ModuleRCS rcs, Vector3 com, ref Vector3 force,
            ref Vector3 torque, ref double thrustSum)
        {
            int count = rcs.thrusterTransforms == null ? 0 : rcs.thrusterTransforms.Count;
            int forces = rcs.thrustForces == null ? 0 : rcs.thrustForces.Length;
            for (int i = 0; i < Math.Min (count, forces); i++) {
                float thrust = rcs.thrustForces [i];
                var thruster = rcs.thrusterTransforms [i];
                if (thruster == null || !(thrust > 0f))
                    continue;
                thrustSum += thrust;
                if (rcs.isJustForShow)
                    continue;
                Vector3 axis = rcs.useZaxis ? thruster.forward : thruster.up;
                Vector3 f = Scale (axis, -1000f * thrust);
                force = Add (force, f);
                torque = Add (torque, Cross3 (Sub (thruster.position, com), f));
            }
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
            // Scratch (Unity main thread only): no arrays allocated per module and per step.
            Vector3[] axes = rcsAxesScratch;
            axes [0] = reference.right;
            axes [1] = reference.up;
            axes [2] = reference.forward;
            bool[] enabled = rcsEnabledScratch;
            enabled [0] = rcs.enablePitch;
            enabled [1] = rcs.enableRoll;
            enabled [2] = rcs.enableYaw;
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
                        Vector3 rot = Cross3 (inputRot, Unit (rPerp));
                        Vector3 v = rcs.useZaxis ? thruster.forward : thruster.up;
                        float fraction = Dot (v, rot);
                        if (!(fraction > 0f))
                            continue;
                        if (fraction > 1f)
                            fraction = 1f;
                        Vector3 f = Scale (v, (float)(-1000.0 * power * fraction));
                        torque = Add (torque, Cross3 (r, f));
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

        static readonly Vector3[] rcsAxesScratch = new Vector3[3];
        static readonly bool[] rcsEnabledScratch = new bool[3];
        static readonly double[] wheelPerScratch = new double[3];
        static readonly double[] surfacePosScratch = new double[3], surfaceNegScratch = new double[3];

        // Scratch of the RCS translation columns (Unity main thread only, like every caller).
        static double[] rcsLever = new double[0], rcsDir = new double[0], rcsPower = new double[0];

        /// <summary>
        /// Computed levers (bit 4): adds the translation columns of one RCS module, body frame,
        /// about com: force (N) and torque (N.m) per unit of ctrlState.X at [0..2] and of
        /// ctrlState.Y at [3..5] of fPos / tPos (command +1) and fNeg / tNeg (per unit of a
        /// negative command). The same module conditions as AddRcsColumns (enabled, RCS group
        /// on, not for show, not shielded) and the axis flags ModuleRCS.Update reads: enableX
        /// for X (body x = ReferenceTransform.right), enableY for Y (body z =
        /// ReferenceTransform.forward). The maths is AttitudeTranslation.RcsTranslationColumns
        /// on the nozzles' levers, directions (thruster up, forward with useZaxis) and full
        /// power (RcsNozzleForceKn). Returns false when a nonlinearity the model ignores is
        /// active (precision mode, fullThrust) on a module that contributes.
        /// </summary>
        internal static bool AddRcsTranslationColumns (ModuleRCS rcs, Vector3 com,
            Transform reference, bool precisionMode, double[] fPos, double[] fNeg, double[] tPos,
            double[] tNeg)
        {
            if (!rcs.moduleIsEnabled || !rcs.rcsEnabled || !rcs.rcs_active || rcs.isJustForShow)
                return true;
            if (rcs.part != null && rcs.part.ShieldedFromAirstream && !rcs.shieldedCanThrust)
                return true;
            if (!rcs.enableX && !rcs.enableY)
                return true;
            int count = rcs.thrusterTransforms == null ? 0 : rcs.thrusterTransforms.Count;
            double power = RcsNozzleForceKn (rcs) * 1000.0;
            if (!(power > 0.0) || count == 0)
                return true;
            if (rcsPower.Length < count) {
                rcsLever = new double[3 * count];
                rcsDir = new double[3 * count];
                rcsPower = new double[count];
            }
            FillRcsNozzles (rcs, com, reference, power, count, rcsLever, rcsDir, rcsPower, false);
            for (int a = 0; a < 2; a++) {
                if (!(a == 0 ? rcs.enableX : rcs.enableY))
                    continue;
                for (int k = 0; k < 3; k++) {
                    columnFPos [k] = 0.0;
                    columnFNeg [k] = 0.0;
                    columnTPos [k] = 0.0;
                    columnTNeg [k] = 0.0;
                }
                AttitudeTranslation.RcsTranslationColumns (count, rcsLever, rcsDir, rcsPower,
                    a == 0 ? 1.0 : 0.0, 0.0, a == 0 ? 0.0 : 1.0, columnFPos, columnFNeg,
                    columnTPos, columnTNeg);
                for (int k = 0; k < 3; k++) {
                    fPos [3 * a + k] += columnFPos [k];
                    fNeg [3 * a + k] += columnFNeg [k];
                    tPos [3 * a + k] += columnTPos [k];
                    tNeg [3 * a + k] += columnTNeg [k];
                }
            }
            return !(precisionMode || rcs.fullThrust);
        }

        /// <summary>
        /// The nozzles of one RCS module as the translation columns read them, body frame about
        /// com: lever (nozzle - com) and direction (thruster up, or forward with useZaxis), 3 per
        /// nozzle, and the same full power for each. A missing thruster gets power 0 and, with
        /// zeroMissing, a zero lever and direction (without it, they keep what the arrays held:
        /// the computed-levers path always did, and its power 0 cancels them). The one loop of
        /// the three RCS translation paths (refactor R3; audit A 3.2).
        /// </summary>
        static void FillRcsNozzles (ModuleRCS rcs, Vector3 com, Transform reference, double power,
            int count, double[] lever, double[] dir, double[] powerOut, bool zeroMissing)
        {
            for (int i = 0; i < count; i++) {
                var thruster = rcs.thrusterTransforms [i];
                if (thruster == null) {
                    powerOut [i] = 0.0;
                    if (zeroMissing) {
                        lever [3 * i] = lever [3 * i + 1] = lever [3 * i + 2] = 0.0;
                        dir [3 * i] = dir [3 * i + 1] = dir [3 * i + 2] = 0.0;
                    }
                    continue;
                }
                Vector3 r = reference.InverseTransformDirection (Sub (thruster.position, com));
                Vector3 v = reference.InverseTransformDirection (rcs.useZaxis ? thruster.forward : thruster.up);
                lever [3 * i] = r.x;
                lever [3 * i + 1] = r.y;
                lever [3 * i + 2] = r.z;
                dir [3 * i] = v.x;
                dir [3 * i + 1] = v.y;
                dir [3 * i + 2] = v.z;
                powerOut [i] = power;
            }
        }

        static readonly double[] columnFPos = new double[3], columnFNeg = new double[3];
        static readonly double[] columnTPos = new double[3], columnTNeg = new double[3];
        static double[] rcsRho = new double[0];

        /// <summary>
        /// Honest translation, bit 16: the nozzles of one RCS module as ModuleRCS fires them for a
        /// ROTATION -- levers (nozzle - com) and directions (thruster up, or forward with useZaxis),
        /// body, 3 per nozzle, full power (N, RcsNozzleForceKn) -- into arrays grown as needed, and
        /// the module's enablePitch / enableRoll / enableYaw. False (nothing to count) under the
        /// conditions of AddRcsColumns (disabled, RCS group off, for show, shielded, no power).
        /// </summary>
        internal static bool RcsRotationNozzles (ModuleRCS rcs, Vector3 com, Transform reference,
            ref double[] lever, ref double[] dir, ref double[] power, out int count, out bool enablePitch,
            out bool enableRoll, out bool enableYaw)
        {
            count = 0;
            enablePitch = enableRoll = enableYaw = false;
            if (!rcs.moduleIsEnabled || !rcs.rcsEnabled || !rcs.rcs_active || rcs.isJustForShow)
                return false;
            if (rcs.part != null && rcs.part.ShieldedFromAirstream && !rcs.shieldedCanThrust)
                return false;
            int n = rcs.thrusterTransforms == null ? 0 : rcs.thrusterTransforms.Count;
            double p = RcsNozzleForceKn (rcs) * 1000.0;
            if (!(p > 0.0) || n == 0)
                return false;
            if (lever == null || lever.Length < 3 * n)
                lever = new double[3 * n];
            if (dir == null || dir.Length < 3 * n)
                dir = new double[3 * n];
            if (power == null || power.Length < n)
                power = new double[n];
            FillRcsNozzles (rcs, com, reference, p, n, lever, dir, power, true);
            count = n;
            enablePitch = rcs.enablePitch;
            enableRoll = rcs.enableRoll;
            enableYaw = rcs.enableYaw;
            return true;
        }

        /// <summary>
        /// Honest translation, bit 16 (born closed): AddRcsTranslationColumns where each nozzle's
        /// translation share is bounded by the headroom the ROTATION leaves it
        /// (AttitudeTranslation.RcsTranslationColumnsHeadroom). served: ctrlState pitch, roll, yaw
        /// as the step found it (the law's pitch/yaw, the AutoPilot's roll), each zeroed when the
        /// module's enablePitch / enableRoll / enableYaw is off, as ModuleRCS.Update reads them;
        /// the rotation fraction of every nozzle (RcsRotationFractions) is held in `hold` (one
        /// value per thruster, reallocated when the count changes) with the linear decay `drop`
        /// per step. Same module conditions, same return as AddRcsTranslationColumns.
        /// </summary>
        internal static bool AddRcsTranslationColumnsHeadroom (ModuleRCS rcs, Vector3 com,
            Transform reference, bool precisionMode, double[] served, ref double[] hold, double drop,
            double fracRcs, double[] fPos, double[] fNeg, double[] tPos, double[] tNeg)
        {
            if (!rcs.moduleIsEnabled || !rcs.rcsEnabled || !rcs.rcs_active || rcs.isJustForShow)
                return true;
            if (rcs.part != null && rcs.part.ShieldedFromAirstream && !rcs.shieldedCanThrust)
                return true;
            if (!rcs.enableX && !rcs.enableY)
                return true;
            int count = rcs.thrusterTransforms == null ? 0 : rcs.thrusterTransforms.Count;
            double power = RcsNozzleForceKn (rcs) * 1000.0;
            if (!(power > 0.0) || count == 0)
                return true;
            if (rcsPower.Length < count) {
                rcsLever = new double[3 * count];
                rcsDir = new double[3 * count];
                rcsPower = new double[count];
            }
            if (rcsRho.Length < count)
                rcsRho = new double[count];
            if (hold == null || hold.Length != count)
                hold = new double[count];
            FillRcsNozzles (rcs, com, reference, power, count, rcsLever, rcsDir, rcsPower, true);
            double p = served != null && served.Length >= 3 && rcs.enablePitch ? served [0] : 0.0;
            double rl = served != null && served.Length >= 3 && rcs.enableRoll ? served [1] : 0.0;
            double yw = served != null && served.Length >= 3 && rcs.enableYaw ? served [2] : 0.0;
            AttitudeTranslation.RcsRotationFractions (count, rcsLever, rcsDir, p, rl, yw, rcsRho);
            AttitudeTranslation.HoldFractions (count, rcsRho, hold, drop);
            for (int a = 0; a < 2; a++) {
                if (!(a == 0 ? rcs.enableX : rcs.enableY))
                    continue;
                for (int k = 0; k < 3; k++) {
                    columnFPos [k] = 0.0;
                    columnFNeg [k] = 0.0;
                    columnTPos [k] = 0.0;
                    columnTNeg [k] = 0.0;
                }
                AttitudeTranslation.RcsTranslationColumnsHeadroom (count, rcsLever, rcsDir, rcsPower,
                    hold, fracRcs, a == 0 ? 1.0 : 0.0, 0.0, a == 0 ? 0.0 : 1.0, columnFPos, columnFNeg,
                    columnTPos, columnTNeg);
                for (int k = 0; k < 3; k++) {
                    fPos [3 * a + k] += columnFPos [k];
                    fNeg [3 * a + k] += columnFNeg [k];
                    tPos [3 * a + k] += columnTPos [k];
                    tNeg [3 * a + k] += columnTNeg [k];
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
            double[] per = wheelPerScratch;
            per [0] = wheel.PitchTorque;
            per [1] = wheel.RollTorque;
            per [2] = wheel.YawTorque;
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
            double[] p = surfacePosScratch;
            p [0] = Math.Abs (pos.x);
            p [1] = Math.Abs (pos.y);
            p [2] = Math.Abs (pos.z);
            double[] n = surfaceNegScratch;
            n [0] = Math.Abs (neg.x);
            n [1] = Math.Abs (neg.y);
            n [2] = Math.Abs (neg.z);
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

        /// <summary>
        /// the ctrlState axes a control surface follows (bit 0 pitch, 1 roll, 2 yaw), as
        /// ModuleControlSurface.FixedCtrlSurfaceUpdate reads them: the module enabled, a range
        /// and an authority limiter that are not 0, and the axis not ignored. Whatever the
        /// dynamic pressure: at q = 0 the surface still moves (GetPotentialTorque, motif 14, is
        /// 0 there, this is not).
        /// </summary>
        internal static int SurfaceAnsweredAxes (ModuleControlSurface surface)
        {
            if (surface == null || !surface.moduleIsEnabled || !(surface.ctrlSurfaceRange != 0f) ||
                    !(surface.authorityLimiter != 0f))
                return 0;
            int axes = 0;
            if (!surface.ignorePitch)
                axes |= 1;
            if (!surface.ignoreRoll)
                axes |= 2;
            if (!surface.ignoreYaw)
                axes |= 4;
            return axes;
        }
    }
}
