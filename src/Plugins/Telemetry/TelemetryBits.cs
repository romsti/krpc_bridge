using System;
using KRPC.Bridge.Core;
using KRPC.Bridge.Control;

namespace KRPC.Bridge.Telemetry
{
    /// <summary>
    /// The capability masks the Telemetry layouts publish (DynamicsV3, DynamicsExtV1), declared
    /// once into the Core v2 bit registry FROM THE CONSTANTS THAT DEFINE THEM (each checked to be
    /// exactly one bit, ControlBits.Bit). Names are those of the client decoders
    /// (python/dynamics_v3.py) where they exist.
    ///
    /// Read-only description: declaring changes nothing that flies.
    /// </summary>
    internal static class TelemetryBits
    {
        internal const string Owner = "Telemetry";
        internal const string DynamicsCapabilities = "dynamics_v3.capabilities";
        internal const string ExtCapabilities = "dynamics_ext_v1.capabilities";
        internal const string ContactSources = "contact_v1.sources";

        static bool declared;

        /// <summary>Declares every set once (TelemetryAddon.Awake). Never throws into the game.</summary>
        internal static void Declare ()
        {
            if (declared)
                return;
            declared = true;
            try {
                DeclareDynamics ();
                DeclareExt ();
                DeclareContact ();
            } catch (Exception exc) {
                BridgeLog.Error ("Telemetry", "bit registry: " + exc.Message);
            }
        }

        static void D (string set, long mask, string name, string kind, string note = "")
        {
            BitRegistry.Declare (set, Owner, ControlBits.Bit (mask), name, kind, note);
        }

        static void DeclareDynamics ()
        {
            string k = BitRegistry.KindCapability;
            string set = DynamicsCapabilities;
            D (set, DynamicsV3.CapPosition, "position", k);
            D (set, DynamicsV3.CapOrbitalVelocity, "orbital_velocity", k);
            D (set, DynamicsV3.CapSurfaceVelocity, "surface_velocity", k);
            D (set, DynamicsV3.CapSurfaceAcceleration, "surface_acceleration_derived", k);
            D (set, DynamicsV3.CapOrbitalAcceleration, "orbital_acceleration_derived", k);
            D (set, DynamicsV3.CapRotation, "rotation", k);
            D (set, DynamicsV3.CapAngularVelocity, "angular_velocity", k);
            D (set, DynamicsV3.CapAngularAcceleration, "angular_acceleration_derived", k);
            D (set, DynamicsV3.CapMass, "mass", k);
            D (set, DynamicsV3.CapCoM, "center_of_mass", k);
            D (set, DynamicsV3.CapMoi, "moi", k);
            D (set, DynamicsV3.CapGravity, "gravity", k);
            D (set, DynamicsV3.CapAtmosphere, "atmosphere", k);
            D (set, DynamicsV3.CapAeroForce, "aero_force", k);
            D (set, DynamicsV3.CapAeroTorque, "aero_torque", k);
            D (set, DynamicsV3.CapDragVector, "drag_vector", k);
            D (set, DynamicsV3.CapLiftVector, "lift_vector", k);
            D (set, DynamicsV3.CapBodySurfaceVelocity, "body_surface_velocity", k);
            D (set, DynamicsV3.CapAoA, "aoa", k);
            D (set, DynamicsV3.CapSideslip, "sideslip", k);
            D (set, DynamicsV3.CapGlobalThrottle, "global_throttle", k);
            D (set, DynamicsV3.CapActuators, "actuators", k);
            D (set, DynamicsV3.CapRealizedEngineWrench, "realized_engine_wrench", k);
            D (set, DynamicsV3.CapKrpcLiveAeroForce, "krpc_live_aero_force", k);
            D (set, DynamicsV3.CapKrpcLiveAeroTorque, "krpc_live_aero_torque", k);
            D (set, DynamicsV3.CapKrpcLiveAeroComponents, "krpc_live_aero_components", k);
            D (set, DynamicsV3.CapKrpcAeroAngles, "krpc_aero_angles", k);
            D (set, DynamicsV3.CapKrpcAeroThermo, "krpc_aero_thermo", k);
            D (set, DynamicsV3.CapExternalForceResidual, "external_force_residual", k);
            D (set, DynamicsV3.CapUnexplainedNonAeroForce, "unexplained_non_aero_force", k);
            D (set, DynamicsV3.CapRealizedEngineForceOnVessel, "realized_engine_force_on_vessel", k);
        }

        static void DeclareContact ()
        {
            string k = BitRegistry.KindInformational;
            string set = ContactSources;
            D (set, ContactLatch.SourceWheel, "wheel_grounded", k, "a wheel's WheelState.grounded");
            D (set, ContactLatch.SourcePart, "part_ground_contact", k,
                "a part without a wheel module has GroundContact or PermanentGroundContact");
            D (set, ContactLatch.SourceWheelPart, "wheel_part_ground_contact", k,
                "the GroundContact flag of a part that carries a wheel (KSP's landing detection)");
        }

        static void DeclareExt ()
        {
            string k = BitRegistry.KindCapability;
            string set = ExtCapabilities;
            D (set, DynamicsExt.CapRotationalStateBody, "rotational_state_body", k);
            D (set, DynamicsExt.CapMoiKgM2, "moi_kg_m2", k);
            D (set, DynamicsExt.CapRcsRealized, "rcs_realized", k);
            D (set, DynamicsExt.CapReactionWheelRealized, "reaction_wheel_realized", k);
            D (set, DynamicsExt.CapControlTorque, "control_torque", k);
            D (set, DynamicsExt.CapEngineAvailability, "engine_availability", k);
            D (set, DynamicsExt.CapControlSurfaces, "control_surfaces", k);
            D (set, DynamicsExt.CapBMatrix, "b_matrix", k);
            D (set, DynamicsExt.CapIndiDeltaOmegaDot, "indi_delta_omega_dot", k);
            D (set, DynamicsExt.CapIndiPredictionB, "indi_prediction_b", k);
            D (set, DynamicsExt.CapIndiPredictionM, "indi_prediction_m", k);
            D (set, DynamicsExt.CapIndiPredictionAero, "indi_prediction_aero", k);
            D (set, DynamicsExt.CapIndiResidualB, "indi_residual_b", k);
            D (set, DynamicsExt.CapIndiResidualM, "indi_residual_m", k);
            D (set, DynamicsExt.CapResponseTracking, "response_tracking", k);
        }
    }
}
