using System;
using System.Collections.Generic;
using System.Globalization;
using KRPC.Service;
using KRPC.Service.Attributes;
using UnityEngine;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// conn.control - what flies: per-actuator authority, the atomic protocol v2, and the
    /// AttitudeV1 loop (refactor R3: the commanding half of the former conn.actuators; the
    /// observing half is conn.telemetry, in KRPC.Bridge.Telemetry). No loop and no wait inside
    /// an RPC.
    ///
    /// The stock thrust limiter that kRPC exposes is a ceiling. LeaseIndependentThrottle drives
    /// the other stock channel of ModuleEngines: an absolute per-engine setting that no longer
    /// follows the main throttle. The lease lasts at most one second and restores the previous
    /// values when the client does not renew it.
    ///
    /// LeaseGimbal turns off ModuleGimbal's stock computation for a while and applies an
    /// absolute deflection, bounded by the module's ranges and limiter. The configured response
    /// speed still applies. The lease always restores the state, the actuation and the original
    /// rotations.
    /// </summary>
    [KRPCService (Name = "Control", GameScene = GameScene.Flight)]
    public static partial class ControlService
    {
        [KRPCProperty]
        public static bool Available { get { return true; } }

        [KRPCProcedure]
        public static string Ping ()
        {
            return "pong";
        }

        /// <summary>
        /// Leases the independent throttle of one engine (percentage 0-100) of the active vessel,
        /// for leaseSeconds of real time (0.05-1 s), restored at expiry.
        /// </summary>
        [KRPCProcedure]
        public static bool LeaseIndependentThrottle (
            string flightId, int engineOrdinal, float percentage, float leaseSeconds = 0.25f)
        {
            Actuation.RequireLegacyControlAvailable ();
            var engine = VesselLookup.FindEngine (flightId, engineOrdinal);
            Actuation.Command (engine, percentage, leaseSeconds);
            return true;
        }

        /// <summary>Releases the lease of an engine's independent throttle and restores it.</summary>
        [KRPCProcedure]
        public static bool ReleaseIndependentThrottle (string flightId, int engineOrdinal)
        {
            Actuation.RequireLegacyControlAvailable ();
            return Actuation.ReleaseThrottle (VesselLookup.FindEngine (flightId, engineOrdinal));
        }

        /// <summary>
        /// Leases the absolute deflection of a gimbal, in local X/Y degrees. The setting is
        /// bounded by the directional ranges and the stock limiter; the part's configured
        /// response speed still applies.
        /// </summary>
        [KRPCProcedure]
        public static bool LeaseGimbal (
            string flightId, int gimbalOrdinal, float xDegrees, float yDegrees,
            float leaseSeconds = 0.25f)
        {
            Actuation.RequireLegacyControlAvailable ();
            Actuation.CommandGimbal (
                VesselLookup.FindGimbal (flightId, gimbalOrdinal), xDegrees, yDegrees, leaseSeconds);
            return true;
        }

        /// <summary>Releases a gimbal lease and gives the stock computation back.</summary>
        [KRPCProcedure]
        public static bool ReleaseGimbal (string flightId, int gimbalOrdinal)
        {
            Actuation.RequireLegacyControlAvailable ();
            return Actuation.ReleaseGimbal (VesselLookup.FindGimbal (flightId, gimbalOrdinal));
        }

        /// <summary>Releases every legacy lease; returns how many were restored.</summary>
        [KRPCProcedure]
        public static int ReleaseAll ()
        {
            Actuation.RequireLegacyControlAvailable ();
            return Actuation.RestoreAll ();
        }
    }
}
