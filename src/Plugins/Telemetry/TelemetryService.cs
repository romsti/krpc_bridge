using System;
using System.Collections.Generic;
using KRPC.Service;
using KRPC.Service.Attributes;
using UnityEngine;
using KRPC.Bridge.Control;

namespace KRPC.Bridge.Telemetry
{
    /// <summary>
    /// conn.telemetry - what observes: the physics-rate state of the vessels and of their
    /// actuators, read-only (refactor R3: the observing half of the former conn.actuators; the
    /// commanding half is conn.control, in KRPC.Bridge.Control).
    ///
    /// EngineSample returns rows of nine doubles: flight_id, module ordinal, ignited, realized
    /// throttle, realized thrust, maximum thrust, thrust limiter, independent mode, independent
    /// percentage.
    ///
    /// GimbalSample returns rows of eight doubles: flight_id, module ordinal, lock, limiter,
    /// range, then local actuation X/Y/Z in degrees. Outside a lease, that actuation is the
    /// state ModuleGimbal computed. During a lease, it is the setting applied directly to the
    /// nozzle transforms.
    /// </summary>
    [KRPCService (Name = "Telemetry", GameScene = GameScene.Flight)]
    public static partial class TelemetryService
    {
        [KRPCProperty]
        public static bool Available { get { return true; } }

        [KRPCProcedure]
        public static string Ping ()
        {
            return "pong";
        }

        [KRPCProcedure]
        public static IList<double> EngineSample ()
        {
            var output = new List<double> ();
            var vessel = VesselLookup.RequireActive ();
            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                int ordinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var engine = part.Modules [m] as ModuleEngines;
                    if (engine == null)
                        continue;
                    output.Add (part.flightID);
                    output.Add (ordinal++);
                    output.Add (engine.EngineIgnited ? 1.0 : 0.0);
                    output.Add (engine.currentThrottle);
                    output.Add (engine.finalThrust);
                    output.Add (engine.maxThrust);
                    output.Add (engine.thrustPercentage);
                    output.Add (engine.independentThrottle ? 1.0 : 0.0);
                    output.Add (engine.independentThrottlePercentage);
                }
            }
            return output;
        }

        [KRPCProcedure]
        public static IList<double> GimbalSample ()
        {
            var output = new List<double> ();
            var vessel = VesselLookup.RequireActive ();
            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                int ordinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var gimbal = part.Modules [m] as ModuleGimbal;
                    if (gimbal == null)
                        continue;
                    output.Add (part.flightID);
                    output.Add (ordinal++);
                    output.Add (gimbal.gimbalLock ? 1.0 : 0.0);
                    output.Add (gimbal.gimbalLimiter);
                    output.Add (gimbal.gimbalRange);
                    output.Add (gimbal.actuationLocal.x);
                    output.Add (gimbal.actuationLocal.y);
                    output.Add (gimbal.actuationLocal.z);
                }
            }
            return output;
        }

        /// <summary>
        /// Thrust direction of every ModuleEngines in the vessel frame.
        ///
        /// Rows of five doubles: flight_id, engine ordinal, then direction X/Y/Z. Reads the
        /// thrustTransforms of the current ModuleEngines instead of keeping kRPC's Thruster
        /// wrappers, which can go stale after a revert. The axes are right, forward and down,
        /// as kRPC's Vessel.reference_frame.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> ThrustDirectionSample ()
        {
            var output = new List<double> ();
            var vessel = VesselLookup.RequireActive ();
            var reference = vessel.ReferenceTransform;
            if (reference == null)
                throw new InvalidOperationException (KRPC.Bridge.Core.BridgeErrors.Tag (
                    KRPC.Bridge.Core.BridgeErrors.NotLoaded, "repère du vaisseau absent"));
            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                int ordinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var engine = part.Modules [m] as ModuleEngines;
                    if (engine == null)
                        continue;
                    Vector3 world = Vector3.zero;
                    float weight = 0f;
                    if (engine.thrustTransforms == null ||
                            engine.thrustTransforms.Count == 0)
                        throw new InvalidOperationException (
                            "thrustTransforms absents : part " + part.flightID);
                    for (int i = 0; i < engine.thrustTransforms.Count; i++) {
                        var transform = engine.thrustTransforms [i];
                        if (transform == null)
                            throw new InvalidOperationException (
                                "thrustTransform nul : part " + part.flightID);
                        float multiplier = 1f;
                        if (engine.thrustTransformMultipliers != null &&
                                i < engine.thrustTransformMultipliers.Count)
                            multiplier = engine.thrustTransformMultipliers [i];
                        world += multiplier * transform.forward;
                        weight += Math.Abs (multiplier);
                    }
                    if (weight <= 0f || world.sqrMagnitude <= 1e-12f)
                        throw new InvalidOperationException (
                            "direction de poussée nulle : part " + part.flightID);
                    Vector3 local = reference.InverseTransformDirection (
                        world.normalized);
                    output.Add (part.flightID);
                    output.Add (ordinal++);
                    output.Add (local.x);
                    output.Add (local.y);
                    output.Add (local.z);
                }
            }
            return output;
        }

        /// <summary>
        /// Thrust position of every ModuleEngines: lever arm from the centre of mass, in the
        /// vessel frame.
        ///
        /// Rows of five doubles: flight_id, engine ordinal, then position X/Y/Z (m). Like
        /// ThrustDirectionSample, reads the current thrustTransforms DIRECTLY, so it answers
        /// even when kRPC's Thruster wrappers return a null transform (seen on some craft,
        /// engine cold AND hot). Origin = CurrentCoM, axes right / forward / down as kRPC's
        /// Vessel.reference_frame: the position IS then the lever arm r_i the S7 capability
        /// descriptor expects.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> ThrustPositionSample ()
        {
            var output = new List<double> ();
            var vessel = VesselLookup.RequireActive ();
            var reference = vessel.ReferenceTransform;
            if (reference == null)
                throw new InvalidOperationException (KRPC.Bridge.Core.BridgeErrors.Tag (
                    KRPC.Bridge.Core.BridgeErrors.NotLoaded, "repère du vaisseau absent"));
            Vector3 com = vessel.CurrentCoM;
            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                int ordinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var engine = part.Modules [m] as ModuleEngines;
                    if (engine == null)
                        continue;
                    Vector3 positionWorld = Vector3.zero;
                    float weight = 0f;
                    if (engine.thrustTransforms == null ||
                            engine.thrustTransforms.Count == 0)
                        throw new InvalidOperationException (
                            "thrustTransforms absents : part " + part.flightID);
                    for (int i = 0; i < engine.thrustTransforms.Count; i++) {
                        var transform = engine.thrustTransforms [i];
                        if (transform == null)
                            throw new InvalidOperationException (
                                "thrustTransform nul : part " + part.flightID);
                        float multiplier = 1f;
                        if (engine.thrustTransformMultipliers != null &&
                                i < engine.thrustTransformMultipliers.Count)
                            multiplier = engine.thrustTransformMultipliers [i];
                        float absoluteWeight = Math.Abs (multiplier);
                        positionWorld += absoluteWeight * transform.position;
                        weight += absoluteWeight;
                    }
                    if (weight <= 0f)
                        throw new InvalidOperationException (
                            "poids de poussée nul : part " + part.flightID);
                    Vector3 local = reference.InverseTransformDirection (
                        (positionWorld / weight) - com);
                    output.Add (part.flightID);
                    output.Add (ordinal++);
                    output.Add (local.x);
                    output.Add (local.y);
                    output.Add (local.z);
                }
            }
            return output;
        }
    }
}
