using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using KRPC.Bridge.Model;
using UnityEngine;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// conn.control.gimbal_authority_v1, read-only: the columns of every gimbal module of a
    /// loaded vessel at the REST pose of its pivots and at ONE kN of thrust, pivots included, and the
    /// authority per kN they give. The pure maths is GimbalAuthority.cs and GimbalPivots.cs (Model).
    ///
    /// Per gimbal module (AttitudeV1's discovery: a ModuleGimbal, the ModuleEngines of its part):
    ///   * one kN split over the part's engines by their maxThrust (equal shares when none has one), each
    ///     engine's kN over its nozzles by the stock multipliers (AttitudeGeometry's split);
    ///   * every nozzle under pivot i carried to the pivot's rest rotation (parent.rotation x initRots[i]):
    ///     GimbalAuthority.RestNozzle;
    ///   * per pivot, the derivative of r x F about its two rest axes (GimbalPivots.AddNozzleDerivative,
    ///     the formula of AttitudeGeometry: the lever is the nozzle's, its offset from the pivot counts);
    ///   * a module whose pivots are not parallel (GimbalPivots.Build: the AR-1 Cluster) is chained
    ///     through its per-pivot rows, every component free (its free columns: what the law's alpha reads
    ///     under attitude_gimbal_pivots_v1 mode 1), and its EXACT authority is the largest torque it realizes
    ///     over its box (GimbalPivots.ExactAuthority); the per-pivot rows are published. Any other
    ///     module applies one (x, y) to every pivot in the pivot's own frame (AttitudeGeometry.GimbalColumns)
    ///     and its exact authority is the alpha rule, sum |column| x span (a box is realized at its corners).
    /// The vessel's authority per kN of the summed maxThrust of ALL its engines (the normalization of the
    /// an earlier change): sum over modules of the module's authority per kN times its part's maxThrust,
    /// over that sum. Multiplied by the vessel's available thrust (kN), it is the authority at full thrust.
    /// Lengths are about the CURRENT centre of mass, in the vessel body frame (ReferenceTransform).
    /// Nothing is written; the vessel need not be armed nor the active one.
    /// </summary>
    internal static class GimbalAuthorityRead
    {
        internal const int Protocol = 1;
        internal const int Schema = 1;
        internal const int HeaderStride = 21;
        internal const int ModuleStride = 31;
        internal const int PivotStride = 12;
        const double DegToRad = Math.PI / 180.0;

        static readonly double[] qNow = new double[4], qRest = new double[4];
        static readonly double[] pivotPos = new double[3], nozzlePos = new double[3], nozzleDir = new double[3];
        static readonly double[] restPos = new double[3], restDir = new double[3], lever = new double[3];
        static readonly double[] offset = new double[3], axisX = new double[3], axisY = new double[3];
        static readonly double[] cX = new double[3], cY = new double[3];
        static readonly double[] colX = new double[3], colY = new double[3];
        static readonly double[] authority = new double[3], exact = new double[3], work = new double[3];

        /// <summary>The answer (see docs/API.md, "Gimbal authority per kN"). "" = the active vessel.</summary>
        internal static IList<double> Read (string persistentId)
        {
            Vessel v;
            uint pid;
            if (string.IsNullOrEmpty (persistentId)) {
                v = FlightGlobals.ActiveVessel;
                pid = v != null ? v.persistentId : 0u;
                if (v != null && !v.loaded)
                    v = null;
            } else {
                pid = VesselLookup.ParsePersistentId (persistentId);
                v = VesselLookup.FindLoaded (pid);
            }
            var modules = new List<double> ();
            var pivots = new List<double> ();
            int moduleCount = 0, pivotCount = 0;
            double totalMax = 0.0, gimbalMax = 0.0, restOk = 1.0;
            var vesselAuthority = new double[3];
            var vesselExact = new double[3];
            var reference = v != null ? v.ReferenceTransform : null;
            if (v != null && reference != null) {
                Vector3 com = v.CurrentCoM;
                for (int p = 0; p < v.parts.Count; p++) {
                    var part = v.parts [p];
                    if (part == null)
                        continue;
                    var partEngines = new List<ModuleEngines> ();
                    for (int m = 0; m < part.Modules.Count; m++) {
                        var engine = part.Modules [m] as ModuleEngines;
                        if (engine != null) {
                            partEngines.Add (engine);
                            totalMax += engine.maxThrust;
                        }
                    }
                    int ordinal = 0;
                    for (int m = 0; m < part.Modules.Count; m++) {
                        var gimbal = part.Modules [m] as ModuleGimbal;
                        if (gimbal == null)
                            continue;
                        double partMax = 0.0;
                        for (int e = 0; e < partEngines.Count; e++)
                            partMax += partEngines [e].maxThrust;
                        if (!Module (gimbal, part, ordinal, partEngines, partMax, com, reference, moduleCount,
                                modules, pivots, ref pivotCount))
                            restOk = 0.0;
                        int row = moduleCount * ModuleStride;
                        for (int a = 0; a < 3; a++) {
                            vesselAuthority [a] += modules [row + 19 + a] * partMax;
                            vesselExact [a] += modules [row + 22 + a] * partMax;
                        }
                        gimbalMax += partMax;
                        moduleCount++;
                        ordinal++;
                    }
                }
            }
            var o = new List<double> (HeaderStride + modules.Count + pivots.Count);
            o.Add (Protocol);
            o.Add (Schema);
            o.Add (pid);
            o.Add (v != null && reference != null ? 1.0 : 0.0);
            o.Add (Ut ());
            o.Add (moduleCount);
            o.Add (ModuleStride);
            o.Add (pivotCount);
            o.Add (PivotStride);
            o.Add (totalMax);
            o.Add (gimbalMax);
            for (int a = 0; a < 3; a++)
                o.Add (totalMax > 0.0 ? vesselAuthority [a] / totalMax : double.NaN);
            for (int a = 0; a < 3; a++)
                o.Add (totalMax > 0.0 ? vesselExact [a] / totalMax : double.NaN);
            Vector3 comBody = v != null && reference != null
                ? reference.InverseTransformDirection (AttitudeGeometry.Sub (v.CurrentCoM, reference.position))
                : new Vector3 (float.NaN, float.NaN, float.NaN);
            o.Add (comBody.x);
            o.Add (comBody.y);
            o.Add (comBody.z);
            o.Add (restOk);
            o.AddRange (modules);
            o.AddRange (pivots);
            return o;
        }

        /// <summary>
        /// One module's row (ModuleStride values) and its pivot rows. False when a pivot or a nozzle could
        /// not be carried to rest (a missing transform): its columns then miss that nozzle.
        /// </summary>
        static bool Module (ModuleGimbal gimbal, Part part, int ordinal, List<ModuleEngines> partEngines,
            double partMax, Vector3 com, Transform reference, int index, List<double> modules,
            List<double> pivotRows, ref int pivotCount)
        {
            bool ok = true;
            var p = new GimbalPivotCommand ();
            bool mapped = AttitudeGeometry.RefreshPivots (gimbal, p);
            int count = (gimbal.gimbalTransforms == null || gimbal.initRots == null) ? 0
                : Math.Min (gimbal.gimbalTransforms.Count, gimbal.initRots.Count);
            double limiter = Math.Min (100.0, Math.Max (0.0, (double)gimbal.gimbalLimiter)) * 0.01;
            double xMin = -Math.Abs ((double)gimbal.gimbalRangeXN) * limiter;
            double xMax = Math.Abs ((double)gimbal.gimbalRangeXP) * limiter;
            double yMin = -Math.Abs ((double)gimbal.gimbalRangeYN) * limiter;
            double yMax = Math.Abs ((double)gimbal.gimbalRangeYP) * limiter;
            Vector3 xAxisLocal = AttitudeGeometry.Scale (Vector3.right, gimbal.xMult);
            Vector3 second = AttitudeGeometry.Scale (gimbal.flipYZ ? Vector3.forward : Vector3.up, gimbal.yMult);
            for (int r = 0; r < 3; r++) {
                colX [r] = 0.0;
                colY [r] = 0.0;
            }
            var perPivot = new double[GimbalPivots.ColumnStride * Math.Max (count, 1)];
            var pivotCentroid = new double[3];
            int centroidCount = 0;
            for (int i = 0; i < count; i++) {
                var pivot = gimbal.gimbalTransforms [i];
                if (pivot == null)
                    continue;
                Quaternion parentRotation = pivot.parent != null ? pivot.parent.rotation : Quaternion.identity;
                Quaternion q0 = parentRotation * gimbal.initRots [i];
                Put4 (qRest, q0);
                Put4 (qNow, pivot.rotation);
                Put3 (pivotPos, AttitudeGeometry.Sub (pivot.position, com));
                for (int r = 0; r < 3; r++)
                    pivotCentroid [r] += pivotPos [r];
                centroidCount++;
                Put3 (axisX, AttitudeGeometry.Unit (q0 * xAxisLocal));
                Put3 (axisY, AttitudeGeometry.Unit (q0 * second));
                for (int r = 0; r < 3; r++) {
                    cX [r] = 0.0;
                    cY [r] = 0.0;
                }
                for (int e = 0; e < partEngines.Count; e++) {
                    var engine = partEngines [e];
                    int transforms = engine.thrustTransforms == null ? 0 : engine.thrustTransforms.Count;
                    double weightSum = 0.0;
                    for (int t = 0; t < transforms; t++)
                        weightSum += Math.Abs (Multiplier (engine, t));
                    if (weightSum <= 1e-12)
                        continue;
                    double share = partMax > 0.0 ? engine.maxThrust / partMax : 1.0 / partEngines.Count;
                    for (int t = 0; t < transforms; t++) {
                        var nozzle = engine.thrustTransforms [t];
                        if (nozzle == null || !nozzle.IsChildOf (pivot))
                            continue;
                        double thrustN = 1000.0 * share * (Multiplier (engine, t) / weightSum);
                        Put3 (nozzlePos, AttitudeGeometry.Sub (nozzle.position, com));
                        Put3 (nozzleDir, AttitudeGeometry.Unit (nozzle.forward));
                        GimbalAuthority.RestNozzle (pivotPos, qNow, qRest, nozzlePos, nozzleDir, restPos, restDir);
                        if (!Finite3 (restPos) || !Finite3 (restDir)) {
                            ok = false;
                            continue;
                        }
                        for (int r = 0; r < 3; r++) {
                            lever [r] = restPos [r];
                            offset [r] = restPos [r] - pivotPos [r];
                        }
                        GimbalPivots.AddNozzleDerivative (axisX, restDir, lever, offset, thrustN, cX, 0, null);
                        GimbalPivots.AddNozzleDerivative (axisY, restDir, lever, offset, thrustN, cY, 0, null);
                    }
                }
                int c = GimbalPivots.ColumnStride * i;
                for (int r = 0; r < 3; r++) {
                    perPivot [c + r] = cX [r];
                    perPivot [c + 3 + r] = cY [r];
                }
                if (mapped)
                    GimbalPivots.AddPivotColumns (p.Map, i, true, true, cX, cY, colX, colY);
                else
                    for (int r = 0; r < 3; r++) {
                        colX [r] += cX [r];
                        colY [r] += cY [r];
                    }
            }
            // World per radian to body per degree (the pipeline of the attitude columns, in double).
            ToBodyPerDegree (reference, colX);
            ToBodyPerDegree (reference, colY);
            for (int k = 0; k < 2 * count; k++) {
                var v = new double[] { perPivot [3 * k], perPivot [3 * k + 1], perPivot [3 * k + 2] };
                ToBodyPerDegree (reference, v);
                perPivot [3 * k] = v [0];
                perPivot [3 * k + 1] = v [1];
                perPivot [3 * k + 2] = v [2];
            }
            double spanX = GimbalAuthority.Span (xMin, xMax), spanY = GimbalAuthority.Span (yMin, yMax);
            GimbalAuthority.ModuleAuthority (colX, colY, spanX, spanY, authority);
            if (mapped) {
                GimbalPivots.ExactAuthority (p.Map, count, perPivot, xMin, xMax, yMin, yMax, -spanX, spanX,
                    -spanY, spanY, exact, work);
            } else {
                for (int r = 0; r < 3; r++)
                    exact [r] = authority [r];
            }
            bool usable = gimbal.moduleIsEnabled && gimbal.gimbalActive && !gimbal.gimbalLock;
            Vector3 centroidBody = centroidCount > 0
                ? reference.InverseTransformDirection (new Vector3 ((float)(pivotCentroid [0] / centroidCount),
                    (float)(pivotCentroid [1] / centroidCount), (float)(pivotCentroid [2] / centroidCount)))
                : new Vector3 (float.NaN, float.NaN, float.NaN);
            modules.Add (part.flightID);
            modules.Add (part.persistentId);
            modules.Add (ordinal);
            modules.Add (count);
            modules.Add (mapped ? 1.0 : 0.0);
            modules.Add (usable ? 1.0 : 0.0);
            modules.Add (limiter);
            modules.Add (xMin);
            modules.Add (xMax);
            modules.Add (yMin);
            modules.Add (yMax);
            modules.Add (partMax);
            modules.Add (gimbal.useGimbalResponseSpeed ? (double)gimbal.gimbalResponseSpeed : double.NaN);
            for (int r = 0; r < 3; r++)
                modules.Add (colX [r]);
            for (int r = 0; r < 3; r++)
                modules.Add (colY [r]);
            for (int r = 0; r < 3; r++)
                modules.Add (authority [r]);
            for (int r = 0; r < 3; r++)
                modules.Add (exact [r]);
            modules.Add (centroidBody.x);
            modules.Add (centroidBody.y);
            modules.Add (centroidBody.z);
            modules.Add (ok ? 1.0 : 0.0);
            modules.Add (partEngines.Count);
            modules.Add (mapped ? count : 0);
            if (mapped) {
                for (int i = 0; i < count; i++) {
                    pivotRows.Add (index);
                    pivotRows.Add (i);
                    for (int k = 0; k < GimbalPivots.MapStride; k++)
                        pivotRows.Add (p.Map [GimbalPivots.MapStride * i + k]);
                    for (int k = 0; k < GimbalPivots.ColumnStride; k++)
                        pivotRows.Add (perPivot [GimbalPivots.ColumnStride * i + k]);
                    pivotCount++;
                }
            }
            return ok;
        }

        static double Multiplier (ModuleEngines engine, int t)
        {
            if (engine.thrustTransformMultipliers != null && t < engine.thrustTransformMultipliers.Count)
                return engine.thrustTransformMultipliers [t];
            return 1.0;
        }

        static void ToBodyPerDegree (Transform reference, double[] v)
        {
            Vector3 b = reference.InverseTransformDirection (new Vector3 ((float)(v [0] * DegToRad),
                (float)(v [1] * DegToRad), (float)(v [2] * DegToRad)));
            v [0] = b.x;
            v [1] = b.y;
            v [2] = b.z;
        }

        static void Put3 (double[] target, Vector3 v)
        {
            target [0] = v.x;
            target [1] = v.y;
            target [2] = v.z;
        }

        static void Put4 (double[] target, Quaternion q)
        {
            target [0] = q.x;
            target [1] = q.y;
            target [2] = q.z;
            target [3] = q.w;
        }

        static bool Finite3 (double[] v)
        {
            for (int r = 0; r < 3; r++)
                if (double.IsNaN (v [r]) || double.IsInfinity (v [r]))
                    return false;
            return true;
        }

        static double Ut ()
        {
            try {
                return Planetarium.fetch != null ? Planetarium.GetUniversalTime () : double.NaN;
            } catch (Exception) {
                return double.NaN;
            }
        }
    }
}
