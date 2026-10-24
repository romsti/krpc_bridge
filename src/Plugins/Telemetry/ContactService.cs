using System.Collections.Generic;
using KRPC.Service.Attributes;

namespace KRPC.Bridge.Telemetry
{
    public static partial class TelemetryService
    {
        /// <summary>
        /// ContactV1 (refactor lot 3): the contact of a vessel with the ground, by leg and by part,
        /// and the master tick of each first contact, read-only. Layout contact_v1 (protocol 1,
        /// schema 1; conn.bridge.layout_v1("contact_v1")): a header of 45 doubles, then
        /// wheel_count rows of wheel_stride doubles, one per wheel module (ModuleWheelBase: legs
        /// and wheels), in part order. Live fields are read when the call runs; the latched ticks
        /// come from a watcher that runs at every physics step inside the actuator pump, right
        /// after the V3 capture, so a tick here is the master_tick of the V3 frame of the same step.
        /// The active vessel is always watched; another loaded vessel is watched for 3000 master
        /// ticks after each call that names it (the first call starts its watch). A vessel not
        /// loaded answers found 0 and no row; this procedure never throws for that. The vessel
        /// is in contact when a wheel is grounded or a part has Part.GroundContact or
        /// PermanentGroundContact; a collision with another vessel alone is latched apart
        /// (collision_tick). Transitions are also published on the event bus V2 (contact.first,
        /// contact.leg, contact.all_legs, contact.landed, contact.situation_landed,
        /// contact.collision, contact.lost). Forces in N, lengths in m, speeds in m/s, angles in
        /// rad, normals in the vessel body frame.
        /// </summary>
        /// <param name="persistentId">The vessel's persistentId as a decimal string; empty for the active vessel.</param>
        [KRPCProcedure]
        public static IList<double> ContactV1 (string persistentId)
        {
            return ContactWatch.Read (persistentId);
        }

        /// <summary>
        /// read-only: the geometry of a vessel's legs and wheels, one row per wheel module
        /// (ModuleWheelBase) in the order of contact_v1. Layout legs_geometry_v1 (protocol 1, schema 1): a
        /// header of 14 doubles (protocol, schema, persistentId, found, UT, wheel count, wheel stride (42), legs,
        /// legs deployed, wheels grounded, the deepest deployed leg's foot at full extension below the centre
        /// of mass along the nose, the same for the wheel's lowest point now, the mean live contact point below
        /// the centre of mass, height from terrain), then per wheel: part persistentId, part flightID, module
        /// ordinal, wheel type, deployed, deploy position, collider ready, the collider's origin (body, m from
        /// the centre of mass) and down axis, the collider's local centre, radius, suspension distance,
        /// suspension anchor, spring and damper rates, the foot at full extension (radius + suspension distance x
        /// anchor below the origin) and the wheel's lowest point now (KSP's WheelCenter + WheelDown x
        /// WheelRadius), grounded, the live contact point, the contact depth, the suspension module's
        /// suspensionDistance, springRatio, damperRatio, maximumLoad, targetPosition, the damage module's
        /// impactTolerance, stressTolerance, isDamaged, the part's crashTolerance, the wheel module's radius.
        /// Body frame about the current centre of mass, metres; NaN for what the part has not. A vessel not
        /// loaded: found 0, no row. See docs/API.md, "Legs geometry".
        /// </summary>
        /// <param name="persistentId">The vessel's persistentId as a decimal string; empty for the active vessel.</param>
        [KRPCProcedure]
        public static IList<double> LegsGeometryV1 (string persistentId)
        {
            return LegsGeometry.Read (persistentId);
        }
    }
}
