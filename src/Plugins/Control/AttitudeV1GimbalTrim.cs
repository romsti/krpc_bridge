using System;
using KRPC.Bridge.Model;
using UnityEngine;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// AttitudeV1, (option 128 of AttitudeOptionsV1, born closed): the gimbals without thrust go
    /// to neutral, and their columns are taken at the thrust that comes. The maths is GimbalSpool.cs and
    /// AttitudeMath's TrimNullColumns (Model); this file touches the game.
    ///
    /// At every Earlyish, per gimbal module of the vessel cache, for each engine of its part (the engines
    /// whose torque its columns differentiate):
    ///   * the gimbal's horizon: the time constant of its stock Lerp (AttitudeTranslation.RampFactor,
    ///     GimbalSpool.HorizonSteps);
    ///   * the throttle the engine's stock spool reaches over that horizon (requestedThrottle,
    ///     currentThrottle, useEngineResponseTime, its two speeds; GimbalSpool.ExpectedThrottle), for an
    ///     engine ignited, operational and not flamed out (any other: its thrust now);
    ///   * the thrust at that throttle: finalThrust per unit of currentThrottle while it thrusts, else
    ///     MaxThrustOutputAtm at the part's pressure, limiter excluded (maxThrust if that call fails);
    ///     never below finalThrust (GimbalSpool.ExpectedThrust).
    /// The module's columns (and a mapped module's free and per-pivot columns) are computed at those
    /// thrusts instead of finalThrust. For the part's first gimbal module, the torque that thrust change
    /// adds with the nozzles where they are, sum of r x F at the expected thrust minus at finalThrust
    /// (AttitudeGeometry.EngineTorqueWorld), is taken off the allocation target (AttitudeInput.TorqueFree):
    /// the law sees now the torque the spool brings, including the misalignment of the thrust line, and
    /// moves the gimbals to the trim of the coming thrust. An engine without a gimbal module keeps its
    /// thrust now (no column, no prediction). Capability bit 18 on a step where one expected thrust
    /// differs from finalThrust; bit 17 on a step where the law regularized a null gimbal column.
    /// Without the option nothing here runs: GimbalColumns reads finalThrust as before.
    /// </summary>
    internal static partial class AttitudeV1
    {
        // Capability bits of the frame header (offset 23), option 128 only.
        internal const long CapGimbalTrimNull = 1L << 17;
        internal const long CapGimbalSpool = 1L << 18;

        /// <summary>
        /// Option 128: each gimbal module's expected engine thrusts (GimbalRef.SpoolThrust, aligned with its
        /// PartEngines) and the spool's free torque (Entry.SpoolFree, body, N.m). Returns CapGimbalSpool
        /// when an expected thrust differs from finalThrust.
        /// </summary>
        static long GimbalTrimPrepare (Entry e, Vector3 com, Transform reference, double dt)
        {
            var freeWorld = new Vector3 (0f, 0f, 0f);
            bool spool = false;
            for (int g = 0; g < e.Gimbals.Count; g++) {
                var gr = e.Gimbals [g];
                var engines = gr.PartEngines;
                int count = engines == null ? 0 : engines.Count;
                if (gr.SpoolThrust == null || gr.SpoolThrust.Length != count)
                    gr.SpoolThrust = new double[count];
                var gimbal = gr.Gimbal;
                double steps = GimbalSpool.HorizonSteps (gimbal == null ? 1.0 : AttitudeTranslation.RampFactor (
                    gimbal.useGimbalResponseSpeed, gimbal.gimbalResponseSpeed, dt));
                for (int i = 0; i < count; i++) {
                    var engine = engines [i];
                    double now = engine == null ? 0.0 : engine.finalThrust;
                    double kn = engine == null ? 0.0 : ExpectedThrust (engine, dt, steps);
                    gr.SpoolThrust [i] = kn;
                    if (kn == now)
                        continue;
                    spool = true;
                    if (gr.Ordinal == 0)
                        freeWorld = AttitudeGeometry.Add (freeWorld, AttitudeGeometry.Sub (
                            AttitudeGeometry.EngineTorqueWorld (engine, com, kn),
                            AttitudeGeometry.EngineTorqueWorld (engine, com)));
                }
            }
            Vector3 freeBody = reference.InverseTransformDirection (freeWorld);
            e.SpoolFree [0] = freeBody.x;
            e.SpoolFree [1] = freeBody.y;
            e.SpoolFree [2] = freeBody.z;
            return spool ? CapGimbalSpool : 0L;
        }

        /// <summary>One engine's thrust (kN) at the horizon of its gimbal (see the class header).</summary>
        static double ExpectedThrust (ModuleEngines engine, double dt, double steps)
        {
            double now = engine.finalThrust;
            if (!engine.EngineIgnited || !engine.isOperational || engine.getFlameoutState)
                return now;
            double throttle = GimbalSpool.ExpectedThrottle (engine.currentThrottle, engine.requestedThrottle,
                engine.useEngineResponseTime, engine.throttleUseAlternate, engine.engineAccelerationSpeed,
                engine.engineDecelerationSpeed, dt, steps);
            double idle = double.NaN;
            if (!(engine.currentThrottle > GimbalSpool.ThrottleReadable && engine.finalThrust > 0f)) {
                try {
                    var part = engine.part;
                    idle = part == null ? engine.maxThrust : engine.MaxThrustOutputAtm (
                        true, false, (float)part.staticPressureAtm, 310.0, part.atmDensity);
                } catch {
                    idle = engine.maxThrust;
                }
            }
            return GimbalSpool.ExpectedThrust (now, engine.currentThrottle, idle, throttle);
        }

        /// <summary>Option 128: the spool's free torque added to the allocation target's (the law subtracts it).</summary>
        static void GimbalTrimApply (Entry e, AttitudeInput inp)
        {
            if (!inp.UseTorqueFree) {
                inp.TorqueFree [0] = 0.0;
                inp.TorqueFree [1] = 0.0;
                inp.TorqueFree [2] = 0.0;
            }
            inp.TorqueFree [0] += e.SpoolFree [0];
            inp.TorqueFree [1] += e.SpoolFree [1];
            inp.TorqueFree [2] += e.SpoolFree [2];
            inp.UseTorqueFree = true;
        }
    }
}
