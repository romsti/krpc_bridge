using System;
using KRPC.Bridge.Core;
using KRPC.Bridge.Control;

namespace KRPC.Bridge.Telemetry
{
    /// <summary>
    /// The schema descriptors of every Telemetry frame and return (refactor R1, plan 2.5; split
    /// from the Actuators layouts in R3), declared once into the Core layout registry:
    /// conn.bridge.layout_v1(name) serves them. Protocol and schema numbers come from the
    /// constants that version each layout; every fixed section is checked against the constant
    /// that sizes it (LayoutRegistry.Expect). Field names are the ones the client decoders already
    /// use (python/dynamics_v3.py). The history wrapper and the actuator rows shared with
    /// control_snapshot_v2 are Control's (ControlLayouts).
    ///
    /// Read-only description: declaring changes nothing that flies.
    /// </summary>
    internal static class TelemetryLayouts
    {
        internal const string Service = "Telemetry";
        const string Owner = "Telemetry";
        static bool declared;

        static string[] X (string prefix, string suffix = "")
        {
            return LayoutRegistry.Xyz (prefix, suffix);
        }

        static string[] J (params object[] parts)
        {
            return LayoutRegistry.Join (parts);
        }

        /// <summary>Declares every layout once (TelemetryAddon.Awake). Never throws into the game.</summary>
        internal static void Declare ()
        {
            if (declared)
                return;
            declared = true;
            try {
                Dynamics ();
                Ext ();
                Aero ();
                Samples ();
                Contact ();
                LegsGeometryLayout ();
            } catch (Exception exc) {
                BridgeLog.Error ("Telemetry", "layout registry: " + exc.Message);
            }
        }

        static LayoutRegistry.Layout L (string name, int protocol, int schema, string kind, string note = "")
        {
            return LayoutRegistry.Declare (name, Service, Owner, protocol, schema, kind, note);
        }

        // ------------------------------------------------------------------ DynamicsV3

        internal static readonly string[] V3Header = {
            "protocol", "schema", "physics_tick", "ut", "physics_dt", "vessel_persistent_id",
            "topology_generation", "last_accepted_sequence", "last_applied_sequence",
            "last_applied_tick", "last_result", "capability_mask", "state_stride", "engine_count",
            "engine_stride", "gimbal_count", "gimbal_stride", "transform_count", "transform_stride",
            "history_capacity", "history_count_before_capture", "capture_phase"
        };

        internal static readonly string[] V3State = J (
            X ("position_world_"), X ("orbital_velocity_world_"), X ("surface_velocity_world_"),
            X ("surface_accel_derived_"), X ("orbital_accel_derived_"),
            "rotation_world_x", "rotation_world_y", "rotation_world_z", "rotation_world_w",
            X ("angular_velocity_world_"), X ("angular_accel_derived_"), "mass_tonnes",
            X ("com_world_"), X ("moi_native_"), X ("gravity_world_"),
            "atm_density", "static_pressure_kpa", "dynamic_pressure_kpa", "mach",
            "altitude_asl_m", "radar_altitude_m", "latitude_deg", "longitude_deg", "global_throttle",
            X ("aero_force_raw_"), X ("aero_torque_raw_"), X ("drag_vector_raw_"), X ("lift_vector_raw_"),
            "surface_speed_ms", "orbital_speed_ms", "gee_force_raw", "situation", "mission_time_s", "packed",
            X ("surface_velocity_reference_"), "aoa_raw", "sideslip_raw",
            // Schema 2, canonical SI.
            X ("engine_force_body_n_"), X ("engine_force_world_n_"), X ("engine_torque_body_nm_"),
            X ("engine_torque_world_nm_"), X ("aero_force_body_n_"), X ("aero_force_world_n_"),
            X ("aero_torque_body_nm_"), X ("aero_torque_world_nm_"), X ("aero_lift_body_n_"),
            X ("aero_drag_body_n_"), X ("aero_side_force_body_n_"),
            "dynamic_pressure_pa", "static_pressure_pa", "atmosphere_density_kg_m3",
            "speed_of_sound_ms", "true_air_speed_ms", "aoa_deg", "sideslip_deg",
            X ("external_force_residual_world_n_"), X ("external_force_residual_body_n_"),
            X ("unexplained_non_aero_force_world_n_"));

        static void Dynamics ()
        {
            var l = L ("dynamics_v3", DynamicsV3.ProtocolVersion, DynamicsV3.SchemaVersion,
                LayoutRegistry.KindFrame, "dynamics_snapshot_v3, dynamics_frames_v3, tracked_dynamics_*_v3");
            l.Once ("header", V3Header);
            l.Rows ("state", "1", "@state_stride", V3State);
            l.Rows ("engines", "@engine_count", "@engine_stride", ControlLayouts.EngineRow);
            l.Rows ("gimbals", "@gimbal_count", "@gimbal_stride", ControlLayouts.GimbalRow);
            l.Rows ("thrust_transforms", "@transform_count", "@transform_stride", ControlLayouts.TransformRow);
            // Schema 3 (refactor R3): appended at the end of the layout.
            l.Once ("clock", "master_tick");
            LayoutRegistry.Expect (l, "header", DynamicsV3.HeaderStride);
            LayoutRegistry.Expect (l, "state", DynamicsV3.StateStride);
            LayoutRegistry.Expect (l, "engines", DynamicsV3.EngineStride);
            LayoutRegistry.Expect (l, "gimbals", DynamicsV3.GimbalStride);
            LayoutRegistry.Expect (l, "thrust_transforms", DynamicsV3.TransformStride);
            LayoutRegistry.Expect (l, "clock", DynamicsV3.ClockStride);

            var s = L ("dynamics_status_v3", DynamicsV3.ProtocolVersion, DynamicsV3.StatusSchemaVersion,
                LayoutRegistry.KindStatus);
            s.Once ("status", "protocol", "schema", "latest_tick", "history_count", "history_capacity",
                "capability_mask", "latest_applied_sequence", "latest_applied_tick", "actuator_result");

            var t = L ("dynamics_tracked_v3", DynamicsTracked.ProtocolVersion, 1, LayoutRegistry.KindStatus,
                "tracked_vessels_v3");
            t.Once ("header", "protocol", "count", "stride");
            t.Rows ("vessels", "@count", "@stride", "persistent_id", "loaded", "lease_remaining_s",
                "topology_generation", "latest_tick", "history_count");
            LayoutRegistry.Expect (t, "vessels", DynamicsTracked.RegistryStride);

            var r = L ("dynamics_reflection_check_v1", 1, 1, LayoutRegistry.KindReturn);
            r.Once ("check", "protocol", "v3_length", "v3_mismatches", "first_v3_mismatch", "aero_length",
                "aero_mismatches", "first_aero_mismatch", "legacy_capture_us", "cached_capture_us");
        }

        // ------------------------------------------------------------------ DynamicsExtV1

        static void Ext ()
        {
            var l = L ("dynamics_ext_v1", DynamicsExt.ProtocolVersion, DynamicsExt.SchemaVersion,
                LayoutRegistry.KindFrame, "dynamics_ext_snapshot_v1, dynamics_ext_frames_v1, tracked_dynamics_ext_frames_v1");
            l.Once ("header", "protocol", "schema", "physics_tick", "ut", "physics_dt",
                "vessel_persistent_id", "topology_generation", "capability_mask", "state_stride",
                "engine_count", "engine_stride", "surface_count", "surface_stride", "rcs_count",
                "rcs_stride", "b_count", "b_stride", "response_count", "response_stride");
            l.Rows ("state", "1", "@state_stride", J (
                X ("omega_body_rad_s_"), X ("omega_dot_body_rad_s2_"), X ("moi_body_kg_m2_"),
                X ("rcs_force_body_n_"), X ("rcs_force_world_n_"), X ("rcs_torque_body_nm_"),
                X ("reaction_wheel_torque_body_nm_"), X ("control_torque_body_nm_"),
                "engines_available_thrust_kn", "engines_max_thrust_kn",
                "engines_realized_mass_flow_kg_s", "engines_requested_mass_flow_kg_s",
                X ("indi_delta_omega_dot_rad_s2_"), X ("indi_pred_b_rad_s2_"), X ("indi_pred_m_rad_s2_"),
                X ("indi_pred_aero_rad_s2_"), X ("indi_residual_b_rad_s2_"), X ("indi_residual_m_rad_s2_"),
                X ("indi_residual_b_filtered_rad_s2_"),
                "indi_filter_tau_s", "indi_lag_ticks", "indi_valid_streak"));
            l.Rows ("engines", "@engine_count", "@engine_stride", "flight_id", "ordinal",
                "available_thrust_kn", "max_thrust_kn", "thrust_per_throttle_kn",
                "realized_mass_flow_kg_s", "requested_mass_flow_kg_s", "isp_at_pressure_s",
                "static_pressure_atm");
            l.Rows ("surfaces", "@surface_count", "@surface_stride", "flight_id", "kind", "ordinal",
                "deflection_deg", "action_deg", "ctrl_surface_range_deg", "authority_limiter_pct",
                "deploy", "deploy_angle_deg", "current_deploy_angle_deg", "actuator_speed_deg_s",
                "use_exponential_speed", "ignore_mask");
            l.Rows ("rcs", "@rcs_count", "@rcs_stride", J ("flight_id", "ordinal", "module_enabled",
                "rcs_enabled", "rcs_active", "just_for_show", "nozzle_count", "thrust_sum_kn",
                "thruster_power_kn", "realized_isp_s", X ("force_body_n_"), X ("torque_body_nm_")));
            l.Rows ("b", "@b_count", "@b_stride", "flight_id", "ordinal", "kind", "axis", "b_body_x",
                "b_body_y", "b_body_z", "u", "du");
            l.Rows ("responses", "@response_count", "@response_stride", "kind", "flight_id", "ordinal",
                "sequence", "apply_tick", "first_response_tick", "baseline_x", "command_x", "realized_x",
                "baseline_y", "command_y", "realized_y");
            // Schema 2 (refactor R3): appended at the end of the layout.
            l.Once ("clock", "master_tick");
            LayoutRegistry.Expect (l, "header", DynamicsExt.HeaderStride);
            LayoutRegistry.Expect (l, "state", DynamicsExt.StateStride);
            LayoutRegistry.Expect (l, "engines", DynamicsExt.EngineStride);
            LayoutRegistry.Expect (l, "surfaces", DynamicsExt.SurfaceStride);
            LayoutRegistry.Expect (l, "rcs", DynamicsExt.RcsStride);
            LayoutRegistry.Expect (l, "b", DynamicsExt.BStride);
            LayoutRegistry.Expect (l, "responses", DynamicsExt.ResponseStride);
            LayoutRegistry.Expect (l, "clock", DynamicsExt.ClockStride);

            var s = L ("dynamics_ext_status_v1", DynamicsExt.ProtocolVersion, DynamicsExt.StatusSchemaVersion,
                LayoutRegistry.KindStatus);
            s.Once ("status", "protocol", "schema", "armed", "lease_remaining_s", "latest_tick",
                "history_count", "history_capacity", "capability_mask");
        }

        // ------------------------------------------------------------------ aero

        static void Aero ()
        {
            var l = L ("aero_actuator_v1", AeroActuatorV1.ProtocolVersion, AeroActuatorV1.SchemaVersion,
                LayoutRegistry.KindFrame, "module_kind 1 ModuleAeroSurface, 2 ModuleControlSurface, " +
                "3 ModuleAnimateGeneric, 4 ModuleDeployableAero");
            l.Once ("header", "protocol", "schema", "physics_tick", "ut", "row_count", "row_stride");
            l.Rows ("rows", "@row_count", "@row_stride", "part_flight_id", "module_kind", "module_ordinal",
                "deploy_bool", "deploy_fraction", "authority_limiter", "deploy_angle_deg",
                "animation_open_hint", "enabled_hint");
            // Schema 2 (refactor R3): appended at the end of the layout.
            l.Once ("clock", "master_tick", "vessel_persistent_id");
            LayoutRegistry.Expect (l, "header", AeroActuatorV1.HeaderStride);
            LayoutRegistry.Expect (l, "rows", AeroActuatorV1.RowStride);
            LayoutRegistry.Expect (l, "clock", AeroActuatorV1.ClockStride);

            var s = L ("aero_actuator_status_v1", AeroActuatorV1.ProtocolVersion, AeroActuatorV1.StatusSchemaVersion,
                LayoutRegistry.KindStatus);
            s.Once ("status", "protocol", "schema", "latest_tick", "history_count", "history_capacity");

            var w = L ("aero_wrench_batch_v1", AeroWrenchBatch.ProtocolVersion, 1, LayoutRegistry.KindReturn,
                "frame_code: the body frame the force and torque are in (non-rotating or rotating)");
            w.Once ("header", "protocol", "count", "stride", "frame_code");
            w.Rows ("states", "@count", "@stride", "ok", "force_x_n", "force_y_n", "force_z_n",
                "torque_x_nm", "torque_y_nm", "torque_z_nm");
            LayoutRegistry.Expect (w, "states", AeroWrenchBatch.OutputStride);
        }

        // ------------------------------------------------------------------ contact (lot 3)

        internal static readonly string[] ContactHeader = J (
            "protocol", "schema", "persistent_id", "found", "tick", "ut", "watched", "watch_start_tick",
            "last_step_tick", "started_in_contact", "contact_now", "landed", "splashed", "situation",
            "parts_ground_contact", "parts_permanent_contact", "parts_colliding", "wheels_grounded", "legs",
            "legs_deployed", "episodes", "first_episode_tick", "airborne_since_tick", "first_contact_tick",
            "first_contact_ut", "first_contact_source", "first_contact_part_pid", "first_contact_vertical_speed_ms",
            "first_contact_horizontal_speed_ms", "first_contact_tilt_rad", "all_legs_tick", "landed_tick",
            "situation_landed_tick", "lost_tick", "collision_tick", "height_from_terrain_m",
            X ("terrain_normal_body_"), "terrain_tilt_rad", "tilt_rad", "vertical_speed_ms", "horizontal_speed_ms",
            "wheel_count", "wheel_stride");

        internal static readonly string[] ContactWheel = J (
            "part_persistent_id", "part_flight_id", "module_ordinal", "wheel_type", "deploy_position",
            "deployed_position", "deployed", "grounded", "module_grounded", "part_ground_contact", "downforce_n",
            "suspension_force_n", "suspension_compression", "contact_depth_m", "compression_rate_ms",
            "normalized_load", X ("hit_normal_body_"), "hit_layer", "first_grounded_tick", "first_grounded_ut",
            "first_grounded_compression_rate_ms", "grounded_transitions", "last_change_tick");

        static void Contact ()
        {
            var c = L ("contact_v1", ContactWatch.Protocol, ContactWatch.Schema, LayoutRegistry.KindReturn,
                "contact_v1(persistent_id); ticks are master ticks (-1 none), first_contact_source bits in " +
                "bits_v1('contact_v1.sources'), wheel_type 0 FREE 1 MOTORIZED 2 LEG, hit_layer 15 terrain, " +
                "0 a part (barge, tower), -1 none");
            c.Once ("header", ContactHeader);
            c.Rows ("wheels", "@wheel_count", "@wheel_stride", ContactWheel);
            LayoutRegistry.Expect (c, "header", ContactWatch.HeaderStride);
            LayoutRegistry.Expect (c, "wheels", ContactWatch.WheelStride);
        }

        // ------------------------------------------------------------------ legs geometry

        static void LegsGeometryLayout ()
        {
            var g = L ("legs_geometry_v1", LegsGeometry.Protocol, LegsGeometry.Schema, LayoutRegistry.KindReturn,
                "legs_geometry_v1(persistent_id); rows in the order of contact_v1; body frame about the current " +
                "centre of mass, metres; *_below_com_m along -nose (body -y); foot_extended: radius + " +
                "suspension_distance x suspension_anchor below the collider's origin; wheel_lowest_now: KSP's " +
                "WheelCenter + WheelDown x WheelRadius; NaN for what the part has not");
            g.Once ("header", "protocol", "schema", "persistent_id", "found", "ut", "wheel_count", "wheel_stride",
                "legs", "legs_deployed", "wheels_grounded", "foot_extended_below_com_m", "wheel_lowest_below_com_m",
                "hit_below_com_m", "height_from_terrain_m");
            g.Rows ("wheels", "@wheel_count", "@wheel_stride", J ("part_persistent_id", "part_flight_id",
                "module_ordinal", "wheel_type", "deployed", "deploy_position", "collider_ready",
                X ("collider_origin_m_"), X ("down_"), X ("collider_center_local_m_"), "radius_m",
                "suspension_distance_m", "suspension_anchor", "spring_rate", "damper_rate",
                X ("foot_extended_m_"), X ("wheel_lowest_now_m_"), "grounded", X ("hit_point_m_"),
                "contact_depth_m", "suspension_distance_config_m", "spring_ratio", "damper_ratio",
                "maximum_load", "target_position", "impact_tolerance", "stress_tolerance", "damaged",
                "part_crash_tolerance_ms", "module_radius_m"));
            LayoutRegistry.Expect (g, "header", LegsGeometry.HeaderStride);
            LayoutRegistry.Expect (g, "wheels", LegsGeometry.WheelStride);
        }

        // ------------------------------------------------------------------ samples

        static void Samples ()
        {
            var e = L ("engine_sample", 1, 1, LayoutRegistry.KindReturn, "rows until the end of the list");
            e.Rows ("rows", "*", "9", "flight_id", "ordinal", "ignited", "current_throttle", "final_thrust_kn",
                "max_thrust_kn", "thrust_percentage", "independent_throttle", "independent_percentage");
            var g = L ("gimbal_sample", 1, 1, LayoutRegistry.KindReturn, "rows until the end of the list");
            g.Rows ("rows", "*", "8", "flight_id", "ordinal", "locked", "limiter", "range", "actual_x_deg",
                "actual_y_deg", "actual_z_deg");
            var d = L ("thrust_direction_sample", 1, 1, LayoutRegistry.KindReturn, "rows until the end of the list; body frame");
            d.Rows ("rows", "*", "5", "flight_id", "ordinal", "direction_x", "direction_y", "direction_z");
            var p = L ("thrust_position_sample", 1, 1, LayoutRegistry.KindReturn, "rows until the end of the list; body frame");
            p.Rows ("rows", "*", "5", "flight_id", "ordinal", "lever_x_m", "lever_y_m", "lever_z_m");
        }
    }
}
