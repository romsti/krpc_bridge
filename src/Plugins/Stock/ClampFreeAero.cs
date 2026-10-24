using System;
using System.Collections.Generic;
using System.Reflection;
using KRPC.SpaceCenter.ExtensionMethods;
using UnityEngine;

using SCBody = KRPC.SpaceCenter.Services.CelestialBody;
using SCFrame = KRPC.SpaceCenter.Services.ReferenceFrame;
using SCVessel = KRPC.SpaceCenter.Services.Vessel;

namespace KRPC.Bridge.Stock
{
    /// <summary>
    /// The aerodynamic force kRPC's Flight.SimulateAerodynamicForceAt computes, with or without
    /// the vessel's launch clamps.
    ///
    /// WHY. Terminal Guidance's ascent tunes its pitch program on the pad, on the drag that
    /// SimulateAerodynamicForceAt gives at nine (altitude, Mach) points. That call sums every part
    /// of the vessel, and on the pad the launch clamps are parts of it; they stay on the ground at
    /// lift-off. On 03/10/2026 the drag measured in flight was 13 to 19 % below the pad
    /// measurement on a 1 709 t lander with four clamps, 29 to 35 % below on a 579 t orbiter with
    /// four clamps: the ascent flew faster and higher than its own simulation.
    ///
    /// HOW. The same loop as kRPC's stock path (StockAerodynamics.SimAeroWrench with
    /// useLegacyUniformFlow, as SimAeroForce calls it): every part not shielded from the
    /// airstream and with a rigid body sees the same airflow, and its lift and drag come from
    /// kRPC's own StockAerodynamics.SimPartAeroForce, reached by reflection because it is
    /// internal. Without clamps, the parts carrying a LaunchClamp module are skipped. With the
    /// clamps the result must equal SimulateAerodynamicForceAt, which lets the caller check this
    /// copy against kRPC before trusting it. FAR replaces the stock model: refused.
    /// </summary>
    public static class ClampFreeAero
    {
        const BindingFlags StaticMembers = BindingFlags.Static | BindingFlags.NonPublic | BindingFlags.Public;
        const BindingFlags InstanceMembers = BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public;

        static MethodInfo atmosphere;
        static MethodInfo partForce;

        static void Load ()
        {
            if (atmosphere != null && partForce != null)
                return;
            var sa = typeof (SCVessel).Assembly.GetType (
                "KRPC.SpaceCenter.ExtensionMethods.StockAerodynamics", true);
            atmosphere = sa.GetMethod ("GetAtmosphericState", StaticMembers);
            partForce = sa.GetMethod ("SimPartAeroForce", StaticMembers);
            if (atmosphere == null || partForce == null)
                throw new InvalidOperationException (
                    "kRPC StockAerodynamics: GetAtmosphericState or SimPartAeroForce not found");
        }

        static T Field<T> (object o, string name)
        {
            var f = o.GetType ().GetField (name, InstanceMembers);
            if (f == null)
                throw new InvalidOperationException ("kRPC StockAerodynamics: no field " + name);
            return (T)f.GetValue (o);
        }

        static bool FarPresent ()
        {
            var t = typeof (SCVessel).Assembly.GetType ("KRPC.SpaceCenter.ExternalAPI.FAR", false);
            var available = t == null ? null : t.GetProperty ("IsAvailable", StaticMembers);
            return available != null && (bool)available.GetValue (null, null);
        }

        static bool IsClamp (Part p)
        {
            return p.FindModuleImplementing<LaunchClamp> () != null;
        }

        /// <summary>
        /// [Fx, Fy, Fz] in newtons in the reference frame, then the count of clamp parts
        /// left out and the count of parts summed. Arguments as SimulateAerodynamicForceAt.
        /// </summary>
        public static IList<double> Force (SCVessel vessel, SCBody body,
                                           Tuple<double, double, double> position,
                                           Tuple<double, double, double> velocity,
                                           Tuple<double, double, double, double> rotation,
                                           SCFrame referenceFrame, bool withoutClamps)
        {
            if (vessel == null)
                throw new ArgumentNullException ("vessel");
            if (body == null)
                throw new ArgumentNullException ("body");
            if (referenceFrame == null)
                throw new ArgumentNullException ("referenceFrame");
            if (FarPresent ())
                throw new InvalidOperationException ("FAR replaces the stock aerodynamics");
            Load ();
            var v = vessel.InternalVessel;
            var celestial = body.InternalBody;
            Vector3d velocityWorld = referenceFrame.VelocityToWorldSpace (position.ToVector (), velocity.ToVector ());
            Vector3d positionWorld = referenceFrame.PositionToWorldSpace (position.ToVector ());
            QuaternionD rotationWorld = referenceFrame.RotationToWorldSpace (rotation.ToQuaternion ());
            QuaternionD current = v.ReferenceTransform.rotation;
            QuaternionD offset = rotationWorld * current.Inverse ();
            var r = new List<double> { 0.0, 0.0, 0.0, 0.0, 0.0 };
            object air = atmosphere.Invoke (null, new object[] {
                positionWorld, celestial, Planetarium.GetUniversalTime () });
            double density = Field<double> (air, "Density");
            if (density <= 0.0)
                return r;
            double speedOfSound = Field<double> (air, "SpeedOfSound");
            Vector3 airflow = offset.Inverse () * (velocityWorld - celestial.getRFrmVel (positionWorld));
            Vector3d force = Vector3d.zero;
            int clamps = 0;
            int summed = 0;
            for (int i = 0; i < v.Parts.Count; i++) {
                Part p = v.Parts [i];
                if (p.ShieldedFromAirstream || p.Rigidbody == null)
                    continue;
                if (withoutClamps && IsClamp (p)) {
                    clamps++;
                    continue;
                }
                object f = partForce.Invoke (null, new object[] { p, airflow, density, speedOfSound });
                force += offset * Field<Vector3d> (f, "Lift") + offset * Field<Vector3d> (f, "Drag");
                summed++;
            }
            Vector3d inFrame = referenceFrame.DirectionFromWorldSpace (force * 1000.0);
            r [0] = inFrame.x;
            r [1] = inFrame.y;
            r [2] = inFrame.z;
            r [3] = clamps;
            r [4] = summed;
            return r;
        }
    }
}
