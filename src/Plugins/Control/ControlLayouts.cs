using System;
using KRPC.Bridge.Core;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// The schema descriptors of every Control frame and return (refactor R1, plan 2.5; split in
    /// R3: Telemetry declares its own in TelemetryLayouts), declared once into the Core layout
    /// registry: conn.bridge.layout_v1(name) and conn.control.attitude_layout_v1(name) serve
    /// them. Protocol and schema numbers come from the constants that version each layout;
    /// every fixed section is checked against the constant that sizes it
    /// (LayoutRegistry.Expect), and the replay harness fails on a mismatch. Field names are the
    /// ones the client decoders already use (the client), in
    /// English where those were French.
    ///
    /// The history wrapper (history_v1) and the actuator rows shared by control_snapshot_v2 and
    /// the DynamicsV3 frame (EngineRow, GimbalRow, TransformRow) are declared here, below
    /// Telemetry, which uses them.
    ///
    /// Read-only description: declaring changes nothing that flies.
    /// </summary>
    internal static class ControlLayouts
    {
        internal const string Service = "Control";
        const string Owner = "Control";
        static bool declared;

        static string[] X (string prefix, string suffix = "")
        {
            return LayoutRegistry.Xyz (prefix, suffix);
        }

        static string[] J (params object[] parts)
        {
            return LayoutRegistry.Join (parts);
        }

        /// <summary>Declares every layout once (Actuation.Awake). Never throws into the game.</summary>
        internal static void Declare ()
        {
            if (declared)
                return;
            declared = true;
            try {
                History ();
                Control ();
                Attitude ();
                Translation ();
                Allocation ();
            } catch (Exception exc) {
                BridgeLog.Error ("Control", "layout registry: " + exc.Message);
            }
        }

        static LayoutRegistry.Layout L (string name, int protocol, int schema, string kind, string note = "")
        {
            return LayoutRegistry.Declare (name, Service, Owner, protocol, schema, kind, note);
        }

        // ------------------------------------------------------------------ the history wrapper

        internal static readonly string[] HistoryHeader = {
            "protocol", "schema", "frame_count", "oldest_tick", "newest_tick", "dropped_before"
        };

        static void History ()
        {
            var l = L ("history_v1", 1, 1, LayoutRegistry.KindHistory,
                "every *_frames_* RPC: protocol and schema are the frame layout's; then frame_count frames, " +
                "each prefixed by its length, oldest first, newer than since_tick, at most 128");
            l.Once ("header", HistoryHeader);
            l.Frames ("frames", "@frame_count", "");
        }

        // ------------------------------------------------------------------ shared actuator rows

        /// <summary>The engine row of the DynamicsV3 frame; control_snapshot_v2 publishes its first 18.</summary>
        internal static readonly string[] EngineRow = {
            "flight_id", "ordinal", "ignited", "requested_throttle", "current_throttle",
            "final_thrust_kn", "max_thrust_kn", "thrust_percentage", "independent_throttle",
            "independent_percentage", "finite_response", "acceleration_speed",
            "deceleration_speed", "requested_mass_flow", "propellant_requirement_met",
            "real_isp_s", "thrust_transform_count", "leased", "flameout", "min_thrust_kn"
        };

        internal static readonly string[] GimbalRow = {
            "flight_id", "ordinal", "locked", "stock_active", "limiter", "range",
            "range_x_negative", "range_x_positive", "range_y_negative", "range_y_positive",
            "finite_response", "response_speed", "actual_x_deg", "actual_y_deg", "actual_z_deg",
            "transform_count", "leased", "target_x_deg", "target_y_deg"
        };

        internal static readonly string[] TransformRow = {
            "flight_id", "engine_ordinal", "transform_index", "multiplier",
            "lever_x", "lever_y", "lever_z", "direction_x", "direction_y", "direction_z"
        };

        // ------------------------------------------------------------------ control v2

        static void Control ()
        {
            var l = L ("control_snapshot_v2", Actuation.ProtocolVersion, 1, LayoutRegistry.KindReturn);
            l.Once ("header", "protocol", "physics_tick", "ut", "physics_dt", "vessel_persistent_id",
                "topology_generation", "last_applied_sequence", "last_result", "engine_count",
                "engine_stride", "gimbal_count", "gimbal_stride", "transform_count", "transform_stride");
            var engines = new string[18];
            Array.Copy (EngineRow, engines, 18);
            l.Rows ("engines", "@engine_count", "@engine_stride", engines);
            l.Rows ("gimbals", "@gimbal_count", "@gimbal_stride", GimbalRow);
            l.Rows ("thrust_transforms", "@transform_count", "@transform_stride", TransformRow);
            // The strides are local constants of Actuation.CaptureSnapshot: 14, 18, 19, 10.
            LayoutRegistry.Expect (l, "header", 14);
            LayoutRegistry.Expect (l, "engines", 18);
            LayoutRegistry.Expect (l, "gimbals", 19);
            LayoutRegistry.Expect (l, "thrust_transforms", 10);

            var s = L ("control_v2", Actuation.ProtocolVersion, Actuation.ControlStatusSchema,
                LayoutRegistry.KindStatus,
                "control_status_v2; result codes 0 none, 1 queued, 2 applied, -1 frame expired, " +
                "-2 authority lost, -3 topology changed, -4 apply failed, -5 superseded");
            s.Once ("status", "protocol", "physics_tick", "topology_generation", "owner_active",
                "owner_ttl_s", "pending_sequence", "pending_tick", "last_accepted_sequence",
                "last_applied_sequence", "result", "last_applied_tick",
                // Schema 2 (refactor R3).
                "vessel_persistent_id");

            var i = L ("inactive_status_v1", Actuation.InactiveStatusVersion, 1, LayoutRegistry.KindStatus,
                "attitude_inactive_status_v1");
            i.Once ("header", "version", "count", "stride");
            i.Rows ("authorities", "@count", "@stride", "persistent_id", "gimbals_leased",
                "throttles_leased", "lease_remaining_s", "now_active", "held");
            LayoutRegistry.Expect (i, "authorities", Actuation.InactiveStatusStride);
        }

        // ------------------------------------------------------------------ AttitudeV1

        internal static readonly string[] AttitudeHeader = {
            "protocol", "schema", "attitude_tick", "ut", "physics_dt", "vessel_persistent_id",
            "topology_generation", "stage", "pump_physics_tick", "fixed_time", "mode",
            "axes_mask", "motif_mask", "consumed_sequence", "consumed_ut_stamp",
            "transport_delay_s", "cost_us", "iterations", "converged", "n_columns",
            "state_stride", "row_count", "row_stride", "capability_mask"
        };

        internal static readonly string[] AttitudeState = J (
            X ("error_rad_"), X ("omega_rad_s_"), X ("omega_dot_measured_rad_s2_"), X ("omega_dot_f_rad_s2_"),
            X ("alpha_rad_s2_"), X ("omega_sqrt_rad_s_"), X ("omega_ref_rad_s_"), X ("omega_pch_rad_s_"),
            X ("nu_rad_s2_"), X ("inertia_kg_m2_"), X ("torque_nm_"), X ("torque_f_nm_"),
            X ("delta_torque_desired_nm_"), X ("torque_desired_nm_"), X ("torque_alloc_target_nm_"),
            X ("torque_allocated_nm_"), X ("nu_hedge_rad_s2_"), X ("residual_rad_s2_"), X ("residual_f_rad_s2_"),
            X ("reference_body_"), "ctrl_served_pitch", "ctrl_served_roll", "ctrl_served_yaw",
            X ("omega_ff_body_rad_s_"), "omega_max_rad_s", "saturation_fraction", "active_count", "guard_mask",
            X ("rcs_torque_nm_"), X ("wheel_torque_nm_"), X ("engine_torque_nm_"),
            // Schema 2 (M3).
            X ("unrealized_"), X ("omega_max_axis_rad_s_"), "axis_guard_mask", "roll_coupling",
            "write_state", "roll_command", "fbw_tick", "fbw_ap_pitch", "fbw_ap_roll", "fbw_ap_yaw",
            "fbw_written_roll", "chain_own_index", "chain_krpc_index", "chain_length",
            "chain_reorders", "writes", "reference_write_mask",
            // Schema 3 (M4).
            "write_state_pitch_yaw", "fbw_written_pitch", "fbw_written_yaw", "gimbals_leased",
            "gimbal_authority", "contact", "reference_age_s", X ("omega_guard_limit_rad_s_"));

        internal static readonly string[] AttitudeRow = {
            "kind", "flight_id", "ordinal", "axis", "u0", "du", "u_min", "u_max", "active_set",
            "b_pos_x", "b_pos_y", "b_pos_z", "b_neg_x", "b_neg_y", "b_neg_z", "priority"
        };

        static void Attitude ()
        {
            var l = L ("attitude_v1", AttitudeV1.ProtocolVersion, AttitudeV1.SchemaVersion,
                LayoutRegistry.KindFrame, "attitude_frames_v1; row kind 1 gimbal, 3 ctrlState, 4 differential motif; " +
                "motif_mask: bits_v1(attitude_v1.motifs); capability_mask: bits_v1(attitude_v1.capabilities)");
            l.Once ("header", AttitudeHeader);
            l.Rows ("state", "1", "@state_stride", AttitudeState);
            l.Rows ("rows", "@row_count", "@row_stride", AttitudeRow);
            // Schema 4 (refactor R3): appended at the end of the layout.
            l.Once ("clock", "master_tick");
            LayoutRegistry.Expect (l, "header", AttitudeV1.HeaderStride);
            LayoutRegistry.Expect (l, "state", AttitudeV1.StateStride);
            LayoutRegistry.Expect (l, "rows", AttitudeV1.RowStride);
            LayoutRegistry.Expect (l, "clock", AttitudeV1.ClockStride);

            var s = L ("attitude_status_v1", AttitudeV1.ProtocolVersion, AttitudeV1.StatusSchemaVersion,
                LayoutRegistry.KindStatus);
            s.Once ("header", "protocol", "schema", "count", "stride", "earlyish_registered",
                "earlyish_calls", "fallback_ticks", "attitude_tick", "ut");
            s.Rows ("vessels", "@count", "@stride", "persistent_id", "mode", "axes_mask",
                "lease_remaining_s", "loaded", "last_motif", "sequence", "consumed_sequence",
                "reference_age_s", "latest_tick", "history_count", "computed_ticks", "exceptions",
                "cost_last_us", "cost_max_us", "accepted_references", "write_mask", "order_verified",
                "writes", "fly_by_wire_calls", "chain_reorders", "fallback_steps", "pitch_yaw_writes",
                "gimbal_authority", "pitch_yaw_fallback_steps");
            LayoutRegistry.Expect (s, "header", AttitudeV1.StatusHeaderStride);
            LayoutRegistry.Expect (s, "vessels", AttitudeV1.StatusRowStride);

            var p = L ("attitude_order_probe_v1", 1, 1, LayoutRegistry.KindReturn,
                "attitude_order_probe_read_v1; stage 1 precalc, 2 earlyish, 3 normal, 4 flight integrator, " +
                "5 late, 6 better late than never, 7 watcher, 8 fly-by-wire");
            p.Once ("header", "version", "armed", "row_count", "row_stride");
            p.Rows ("rows", "@row_count", "@row_stride", "frame", "call_index", "stage", "fixed_time", "ut",
                "pump_tick", "render_frame", "served_pitch", "first_gimbal_x", "first_engine_thrust",
                "omega_x", "first_rcs_thrust_sum", "real_time");
            LayoutRegistry.Expect (p, "rows", AttitudeProbe.RowStride);

            var o = L ("attitude_options_v1", AttitudeWrite.OptionsVersion, 1, LayoutRegistry.KindReturn,
                "options: bits_v1(attitude_v1.options); surface_axes bit 0 pitch, 1 roll, 2 yaw");
            o.Once ("reply", "accepted", "version", "options", "supported_options", "surfaces_answering",
                "surfaces_neutralizable", "surface_axes", "attitude_tick");
            LayoutRegistry.Expect (o, "reply", AttitudeV1.OptionsReplyLength);

            var c = L ("attitude_capabilities_v1", AttitudeV1.CapabilitiesVersion, 1, LayoutRegistry.KindReturn);
            c.Once ("reply", "version", "supported_options", "evidence_block_protocol",
                "evidence_block_schema", "evidence_field_count", "surface_neutralizer");

            var g = L ("attitude_gimbal_pivots_v1", GimbalPivots.Version, 1, LayoutRegistry.KindReturn);
            g.Once ("reply", "accepted", "version", "mode", "gimbal_modules", "multi_pivot_modules",
                "mapped_modules", "most_pivots", "attitude_tick");
            LayoutRegistry.Expect (g, "reply", AttitudeV1.PivotsReplyLength);

            var e = L ("attitude_effectors_v1", EffectorRegistry.Protocol, EffectorRegistry.Schema,
                LayoutRegistry.KindStatus, "attitude_effectors_read_v1; family 0 gimbal, 1 rcs, 2 wheel, 3 surface, " +
                "4 diff_motif; status -1 absent, 0 exact, 1 measured, 2 approximate, 3 unmodeled; " +
                "action 0 allocated, 1 frozen, 2 bounded");
            e.Once ("header", "protocol", "schema", "enabled", "attitude_tick", "family_count",
                "family_stride", "column_count", "column_stride", "frozen_axes", "surfaces_neutralized");
            e.Rows ("families", "@family_count", "@family_stride", J ("family", "status", "modules", "scale",
                X ("authority_", "_nm"), X ("alpha_", "_rad_s2"), "columns", "frozen_columns"));
            e.Rows ("columns", "@column_count", "@column_stride", "column", "kind", "family", "axis",
                "status", "action", "b_x", "b_y", "b_z", "half_span");
            LayoutRegistry.Expect (e, "header", EffectorRegistry.HeaderStride);
            LayoutRegistry.Expect (e, "families", EffectorRegistry.FamilyStride);
            LayoutRegistry.Expect (e, "columns", EffectorRegistry.ColumnStride);

            var ec = L ("attitude_effectors_config_v1", EffectorRegistry.Protocol, 1, LayoutRegistry.KindReturn,
                "attitude_effectors_v1");
            ec.Once ("reply", "accepted", "protocol", "mode", "attitude_tick");
            LayoutRegistry.Expect (ec, "reply", AttitudeV1.EffectorsReplyLength);

            Surfaces ();
            GimbalAuthorityLayout ();
            ThrottleLayout ();
            RingLayout ();
        }

        // ------------------------------------------------------------------ the main throttle

        static void ThrottleLayout ()
        {
            var t = L ("attitude_throttle_v1", MainThrottle.Protocol, MainThrottle.Schema, LayoutRegistry.KindReturn,
                "attitude_throttle_v1 (option 512); kind 0 stop, 1 ramp, 2 pulse, 3 cut when s = 0, 4 ignition when " +
                "h <= h_ign(v), 5 flip calage; state 0 idle, 1 writing, 2 held, 3 guard, 4 ended, 5 blocked; motif 0 " +
                "none, 1 nul, 2 aide, 3 sous_rcs, 4 calage, 5 saturation, 6 sans_cardan, 7 hemisphere, 8 hors_autorite, " +
                "9 cout, 10 sans_cout, 11 sans_mesure, 12 sans_loi, 13 garde_poussee, 14 schedule, 15 pulse, 16 avant, " +
                "17 declenche, 18 anneau (the ramp order takes the reference ring's throttle, D252 K7); kind and sequence: the order now; decide_*, state, motif, decided: the last Earlyish's decision; " +
                "written, client_throttle: the last fly-by-wire (fly_by_wire_tick); a sequence not newer reads without change");
            t.Once ("reply", "accepted", "protocol", "schema", "kind", "sequence", "decide_kind", "decide_sequence",
                "state", "motif", "decided",
                "written", "client_throttle", "decide_tick", "decide_ut", "fly_by_wire_tick", "guard_tripped",
                "episode_on", "episode_min", "episode_seen", "fired", "writes", "held", "unrealized", "required",
                "cost_cap", "fraction_lost", "m_par_nm", "rcs_par_nm", "gimbal_authority_x_nm",
                "gimbal_authority_z_nm", "torque_ext_x_nm", "torque_ext_z_nm", "full_thrust_n", "thrust_now_n",
                "mass_kg", "dot_target", "verdict", "missed", "attitude_tick");
            LayoutRegistry.Expect (t, "reply", AttitudeV1.ThrottleReplyLength);
        }

        // ------------------------------------------------------------------ the reference ring

        static void RingLayout ()
        {
            var t = L ("attitude_reference_ring_v1", ReferenceRing.Protocol, ReferenceRing.Schema,
                LayoutRegistry.KindReturn,
                "attitude_reference_ring_v1 (option 2048); sequence, count, channels (1 direction, 2 lateral, 4 throttle), " +
                "ut_first, ut_last: the ring now; served (channels), served_ut, segment, fraction, dir_*, a_*, u: the ring's " +
                "value at the last Earlyish (NaN where not served); a sequence not newer reads without change, an empty " +
                "list clears the ring");
            t.Once ("reply", "accepted", "protocol", "schema", "sequence", "count", "channels", "ut_first", "ut_last",
                "served", "served_ut", "segment", "fraction", "dir_x", "dir_y", "dir_z", "a_x", "a_y", "a_z", "u",
                "attitude_tick");
            LayoutRegistry.Expect (t, "reply", AttitudeV1.RingReplyLength);
        }

        // ------------------------------------------------------------------ gimbal authority per kN

        static void GimbalAuthorityLayout ()
        {
            var a = L ("gimbal_authority_v1", GimbalAuthorityRead.Protocol, GimbalAuthorityRead.Schema,
                LayoutRegistry.KindReturn, "gimbal_authority_v1(persistent_id); columns at the rest pose of the " +
                "pivots for 1 kN of the part's engines (split by maxThrust), body frame about the current centre " +
                "of mass, N.m/deg per kN; authority = sum |column| span per kN; per_kn_* of the header: per kN of " +
                "the summed maxThrust of every engine; pivot rows only for modules mapped per pivot (D42)");
            a.Once ("header", J ("protocol", "schema", "persistent_id", "found", "ut", "module_count",
                "module_stride", "pivot_count", "pivot_stride", "max_thrust_kn", "gimbal_max_thrust_kn",
                X ("authority_per_kn_"), X ("exact_authority_per_kn_"), X ("com_from_reference_m_"),
                "rest_pose"));
            a.Rows ("modules", "@module_count", "@module_stride", J ("part_flight_id", "part_persistent_id",
                "ordinal", "pivot_count", "mapped", "usable", "limiter", "x_min_deg", "x_max_deg", "y_min_deg",
                "y_max_deg", "max_thrust_kn", "response_speed", X ("column_x_per_kn_"), X ("column_y_per_kn_"),
                X ("authority_per_kn_"), X ("exact_authority_per_kn_"), X ("pivot_centroid_m_"), "rest_pose",
                "engine_count", "pivot_rows"));
            a.Rows ("pivots", "@pivot_count", "@pivot_stride", J ("module", "pivot", "map_xx", "map_xy", "map_yx",
                "map_yy", X ("column_x_per_kn_"), X ("column_y_per_kn_")));
            LayoutRegistry.Expect (a, "header", GimbalAuthorityRead.HeaderStride);
            LayoutRegistry.Expect (a, "modules", GimbalAuthorityRead.ModuleStride);
            LayoutRegistry.Expect (a, "pivots", GimbalAuthorityRead.PivotStride);
        }

        // ------------------------------------------------------------------ SurfaceModelV1 (lot 3, 3.1)

        internal static readonly string[] SurfaceHeader = J (
            "protocol", "schema", "enabled", "mode", "attitude_tick", "ut", "physics_dt",
            "vessel_persistent_id", "surface_count", "row_stride", "modeled", "engaged", "neutralized",
            "lag_same", "lag_previous", "lag_neither", "lag_established", "identity_max", "identity_judged",
            "sanctions", X ("scale_k_"), X ("scale_steps_"), X ("torque_realized_nm_"),
            X ("torque_free_nm_"), X ("residual_law_rad_s2_"), X ("residual_surfaces_rad_s2_"),
            X ("residual_surfaces_f_rad_s2_"), X ("authority_nm_"), "ctrl_pitch", "ctrl_roll", "ctrl_yaw",
            "earlyish_tick", "probe_same", "probe_previous");

        internal static readonly string[] SurfaceRow = J (
            "part_persistent_id", "module_index", "status", "reasons", "axes", "engaged", "neutralized",
            "shielded", "deflection_rad", "deflection_previous_rad", "action_rad", "target_same_rad",
            "target_previous_rad", "target_nominal_rad", "predicted_same_rad", "predicted_previous_rad",
            "lag_outcome", "rate_rad_s", "range_rad", "authority_limiter_pct", "deflection_direction",
            "q_lift_pa", "q_drag_pa", "mach", "rotated", X ("force_realized_n_"), X ("force_model_n_"),
            "identity", X ("torque_realized_nm_"), X ("column_nm_rad_"), "gain_pitch_rad", "gain_roll_rad",
            "gain_yaw_rad", "identity_strikes", "mach_source", "lift_coeff", "area", "drag_model",
            "exponential", "deploy_term_rad");

        static void Surfaces ()
        {
            var l = L ("surface_model_v1", SurfaceModel.Protocol, SurfaceModel.Schema, LayoutRegistry.KindStatus,
                "surface_model_v1 (attitude_surfaces_v1 mode 1 or 2); body frame; status 0 exact, 3 unmodeled; " +
                "reasons bits 1 subclass, 2 rotor, 4 adjusters, 8 reflection, 16 no transform, 32 node lift, " +
                "64 shielded, 128 identity, 256 not captured; sanctions bits 1/2/4 scale x/y/z, 8 lag, " +
                "16 identity, 32 not armed, 64/128/256 scale x/y/z not proven (armed only, D161), " +
                "512/1024/2048 kappa x/y/z not proven, 4096/8192/16384 kappa x/y/z out of tolerance (armed " +
                "with mode flag 8 only, D166), 32768/65536/131072 axis x/y/z not owned by the loop for this " +
                "step's engagement (armed only, D180); mode: base 0/1/2 ORed with 4 (N.m, D165) and 8 (kappa, D166); " +
                "lag_outcome -1 not told, 0 same step, 1 previous step, 2 neither; " +
                "mach_source 1 Earlyish, 2 after FlightIntegrator");
            l.Once ("header", SurfaceHeader);
            l.Rows ("rows", "@surface_count", "@row_stride", SurfaceRow);
            l.Once ("clock", "master_tick");
            LayoutRegistry.Expect (l, "header", SurfaceModel.HeaderStride);
            LayoutRegistry.Expect (l, "rows", SurfaceModel.RowStride);
            LayoutRegistry.Expect (l, "clock", SurfaceModel.ClockStride);

            var c = L ("attitude_surfaces_v1", SurfaceModel.Protocol, 1, LayoutRegistry.KindReturn,
                "attitude_surfaces_v1; mode 0 off, 1 shadow, 2 armed");
            c.Once ("reply", "accepted", "protocol", "mode", "surfaces_answering", "surfaces_modeled",
                "attitude_tick");
            LayoutRegistry.Expect (c, "reply", SurfaceModel.ConfigReplyLength);

            // the physical slope kappa (attitude_surfaces_v1 mode ORed with 8).
            var k = L ("surface_kappa_v1", SurfaceKappa.Protocol, SurfaceKappa.Schema, LayoutRegistry.KindStatus,
                "surface_kappa_v1 (attitude_surfaces_v1 mode ORed with 8, D166); body axes 0 x, 1 y, 2 z; state 0 " +
                "not proven, 1 proven (kappa +- t SE inside [band_min, band_max], the proof band), 2 out of " +
                "tolerance (out_low..out_high, t on the jackknife's own dof out_dof, entirely outside, for the " +
                "arming, D171); kappa: measured / law torque of the surfaces' deflections, the measured side with " +
                "the gyroscopic torque (D171); rms in N.m (rms_fins_nm in the law's unit: kN.m without mode flag 4); " +
                "schema 2 (D171/D172): band_min/band_max the proof band, out_* and rms_gyro_nm appended; schema 3 " +
                "(D178/D179): leak_gate (mode ORed with 16: a step counts on an axis only while the non-surface " +
                "realized increment is at most 1/2 of the surfaces'), leak_refused (steps the gate refused), " +
                "leak_beta (sum d m / sum d^2 of the counted steps, the collinear leak slope); schema 4 (D192): " +
                "inertia_physical (mode ORed with 32: the measured side on Vessel.MOI plus the parallel-axis terms " +
                "it leaves out), inertia_offset_kg_m2 (those terms, kg.m^2, whatever the flag)");
            k.Once ("header", SurfaceKappaHeader);
            k.Rows ("axes", "@axis_count", "@axis_stride", SurfaceKappaAxis);
            k.Once ("clock", "master_tick");
            LayoutRegistry.Expect (k, "header", SurfaceKappa.HeaderStride);
            LayoutRegistry.Expect (k, "axes", SurfaceKappa.AxisStride);
            LayoutRegistry.Expect (k, "clock", SurfaceKappa.ClockStride);

            // the client's window for the roll excitation, and its state (born closed).
            var x = L ("attitude_surfaces_roll_v1", SurfaceKappaRoll.Protocol, SurfaceKappaRoll.Schema,
                LayoutRegistry.KindReturn,
                "attitude_surfaces_roll_v1 (D176 a); state 0 off, 1 waiting, 2 exciting, 3 roll proven, 4 out of " +
                "tolerance; reasons bits 1 no window, 2 not shadow with kappa, 4 loop without its fly-by-wire link, " +
                "8 option 1 off, 16 the loop writes the roll or holds the surfaces, 32 roll proven, 64 an axis out of " +
                "tolerance, 128 roll not counted, 256 no surface answers the roll, 512 exponential actuator, 1024 leak " +
                "ratio above leak_max; torques in the law's unit (N.m with mode flag 4) except non_surface_per_unit_nm");
            x.Once ("reply", RollReply);
            LayoutRegistry.Expect (x, "reply", SurfaceKappaRoll.ReplyLength);
        }

        internal static readonly string[] RollReply = {
            "accepted", "protocol", "schema", "state", "reasons", "until", "ut", "attitude_tick", "command",
            "amplitude", "amplitude_torque_nm", "plan_torque_nm", "horizon_batches", "batch_information",
            "surface_per_unit_nm", "non_surface_per_unit_nm", "leak_ratio", "leak_max", "steps", "episodes",
            "plans", "writes", "last_written", "window_calls", "batch_seconds", "plan_seconds"
        };

        internal static readonly string[] SurfaceKappaHeader = {
            "protocol", "schema", "enabled", "mode", "attitude_tick", "ut", "vessel_persistent_id", "batch_steps",
            "batch_steps_base", "batch_capacity", "false_alarm", "band_min", "band_max", "steps_seen",
            "steps_counted", "restarts", "engine_quiet_steps", "aero_parts", "aero_lifting_modules",
            "aero_cost_us_last", "aero_cost_us_max", "axes_now", "proven_axes", "out_axes", "axis_count",
            "axis_stride"
        };

        internal static readonly string[] SurfaceKappaAxis = {
            "axis", "state", "proven", "out", "consistent_with_one", "kappa", "kappa_se", "t_quantile",
            "tolerance", "kappa_low", "kappa_high", "z_from_one", "nuisance_slope", "nuisance_se", "regressors",
            "steps", "batches", "effective_batches", "rms_y_nm", "rms_fins_nm", "rms_nuisance_nm",
            "rms_residual_nm", "out_dof", "out_t_quantile", "out_low", "out_high", "rms_gyro_nm", "leak_gate",
            "leak_refused", "leak_beta", "inertia_physical", "inertia_offset_kg_m2"
        };

        // ------------------------------------------------------------------ lateral force

        internal static readonly string[] TranslationReturnV1 = {
            "accepted", "lateral_authority_ms2", "diff_authority_nm", "common_fraction_realized",
            "layout_version", "state", "reasons", "wanted_ms2", "commanded_ms2", "realized_ms2",
            "full_common_torque_nm", "cancelled_torque_nm", "motif_engines", "main_throttle",
            "attitude_tick", "ut", "throttle_writes"
        };

        internal static readonly string[] TranslationReturnV2 = {
            "diff_gimbal_authority_ms2", "lever_authority_ms2", "rcs_x", "rcs_y", "rcs_capacity_n",
            "motif_lever_m", "residual_torque_nm", "frac_rcs"
        };

        internal static readonly string[] TranslationReturnV3 = {
            "model_authority_ms2", "measured_common_share", "common_share_evidence_deg2",
            "rcs_capacity_full_n", "rcs_weaker_sign_share", "rcs_held_rotation_max", "rcs_shared_flags"
        };

        internal static readonly string[] EvidenceHeader = { "block_protocol", "block_schema", "block_n" };

        // (option 256): the header of the free block, after the evidence block.
        internal static readonly string[] FreeHeader = { "free_block_protocol", "free_block_schema", "free_block_n" };

        static void Translation ()
        {
            var f = L ("attitude_translation_frames_v1", AttitudeV1.TranslationProtocolVersion,
                AttitudeV1.TranslationSchemaVersion, LayoutRegistry.KindFrame,
                "state 0 idle, 1 differential, 2 translation, 3 wind-down, 4 unavailable; reasons: " +
                "bits_v1(attitude_translation.reasons)");
            f.Once ("header", J ("protocol", "schema", "attitude_tick", "ut", "persistent_id", "state",
                "reasons", "mode", "sequence", "request_age_s", "validity_left_s", "thrust_n", "mass_kg",
                "main_throttle", X ("demand_body_"), X ("up_body_"), "wanted_x_ms2", "wanted_z_ms2",
                "commanded_x_ms2", "commanded_z_ms2", "phi_x_deg", "phi_z_deg", "limit_scale",
                "lateral_authority_ms2", "diff_authority_nm", "full_common_torque_nm",
                "fraction_commanded", "fraction_realized", X ("common_torque_realized_", "_nm"),
                "feed_forward_pitch", "feed_forward_yaw", "motif_realized_pitch", "motif_realized_yaw", "half_width_pitch",
                "half_width_yaw", "motif_written_pitch", "motif_written_yaw", "attitude_share", "fbw_tick",
                "fbw_main_throttle", "fbw_kappa", "throttle_leases", "wind_down_s", "realized_x_ms2",
                "realized_z_ms2", "gimbal_range_deg", "half_width_symmetric", "gimbal_count",
                "engine_count", "throttle_writes"));
            f.Rows ("gimbals", "@gimbal_count", AttitudeV1.TranslationGimbalRowStride.ToString (
                System.Globalization.CultureInfo.InvariantCulture), "flight_id", "ordinal", "common_x_deg",
                "common_y_deg", "common_realized_x_deg", "common_realized_y_deg");
            f.Rows ("engines", "@engine_count", AttitudeV1.TranslationEngineRowStride.ToString (
                System.Globalization.CultureInfo.InvariantCulture), "flight_id", "slope_n", "motif_pitch",
                "motif_yaw", "centroid_x_m", "centroid_z_m", "command_realized", "offset_decided",
                "pct_written_previous", "leased");
            // Schema 2 (refactor R3): appended at the end of the layout.
            f.Once ("clock", "master_tick");
            LayoutRegistry.Expect (f, "header", AttitudeV1.TranslationHeaderStride);
            LayoutRegistry.Expect (f, "clock", AttitudeV1.TranslationClockStride);
            LayoutRegistry.Expect (f, "gimbals", AttitudeV1.TranslationGimbalRowStride);
            LayoutRegistry.Expect (f, "engines", AttitudeV1.TranslationEngineRowStride);

            // The legacy return, whose length depends on the mode (layout_version at offset 4)
            // and on option 2 (the evidence block, after NaN padding to 32): kept as it flies. Schema 2
            //: the free block after the evidence block, with option 256 only.
            var r = L ("attitude_translation_v1", AttitudeTranslation.ReturnVersionHonest, 2, LayoutRegistry.KindReturn,
                "LEGACY, mode-dependent: section layout2 only when layout_version >= 2, layout3 when 3, " +
                "evidence only with option 2 or 256 (after NaN padding to 32; NaN values without option 2), " +
                "free only with option 256. Prefer attitude_translation_v2");
            r.Once ("layout1", TranslationReturnV1);
            r.Once ("layout2", TranslationReturnV2);
            r.Once ("layout3", TranslationReturnV3);
            r.Once ("evidence_header", EvidenceHeader);
            r.Once ("evidence", AttitudeTranslation.EvidenceFields);
            r.Once ("free_header", FreeHeader);
            r.Once ("free", AttitudeTranslation.FreeFields);
            LayoutRegistry.Expect (r, "layout1", 17);

            // The fixed return (R1): the same fields, every section always present.
            var v2 = L ("attitude_translation_v2", AttitudeV1.TranslationReturnV2Protocol, 1,
                LayoutRegistry.KindReturn, "attitude_translation_v2: one layout whatever the mode or the " +
                "options; a field the mode or option does not fill is NaN; layout_version says which " +
                "part the mode fills (1, 2 or 3)");
            v2.Once ("return", J (TranslationReturnV1, TranslationReturnV2, TranslationReturnV3));
            v2.Once ("evidence_header", EvidenceHeader);
            v2.Once ("evidence", AttitudeTranslation.EvidenceFields);
            LayoutRegistry.Expect (v2, "return", AttitudeTranslation.ReturnLengthHonest);

            var ev = L (AttitudeTranslation.EvidenceLayoutName, AttitudeTranslation.EvidenceBlockProtocol,
                AttitudeTranslation.EvidenceBlockSchema, LayoutRegistry.KindBlock,
                "appended to attitude_translation_v1 with option 2, always in attitude_translation_v2, " +
                "after block_protocol, block_schema, block_n");
            ev.Once ("evidence", AttitudeTranslation.EvidenceFields);

            // the free authority block (option 256), after the evidence block of attitude_translation_v1.
            var fr = L (AttitudeTranslation.FreeLayoutName, AttitudeTranslation.FreeBlockProtocol,
                AttitudeTranslation.FreeBlockSchema, LayoutRegistry.KindBlock,
                "appended to attitude_translation_v1 with option 256, after the evidence block, after " +
                "free_block_protocol, free_block_schema, free_block_n; source 0 not computed, 1 along the gap, " +
                "2 no gap (ratio 1), 3 a solve failed (ratio 1)");
            fr.Once ("free", AttitudeTranslation.FreeFields);

            var c = L ("attitude_translation_config_v1", 1, 1, LayoutRegistry.KindReturn);
            c.Once ("reply", "frac_gimbal", "frac_diff", "diff_priority", "min_lever_arm_m",
                "min_throttle_margin", "min_projection_det", "wind_down_done_deg", "wind_down_max_s",
                "throttle_lease_s");
        }

        // ------------------------------------------------------------------ the shadow allocation

        internal static readonly string[] AllocationBlockHeader = { "block_protocol", "block_schema", "block_n" };

        static void Allocation ()
        {
            string reasons = "reasons: bits_v1(attitude_allocation.reasons); state 0 not computed, 1 solved, 2 no law " +
                "target (authorities only), 3 no actuator";
            var b = L (AttitudeV1.AllocationLayoutName, AttitudeV1.AllocationBlockProtocol,
                AttitudeV1.AllocationBlockSchema, LayoutRegistry.KindBlock,
                "attitude_allocation_v1 (option 4096 or 8192): block_protocol, block_schema, block_n, then the fields; " + reasons +
                "; schema 2 (option 8192): the write's fields after ut, fallback_reasons: bits_v1(attitude_allocation.fallbacks), " +
                "write_state 0 off, 1 writing, 2 fallback");
            b.Once ("block", J (AttitudeV1.AllocationFields, "attitude_tick", "ut", AttitudeV1.AllocationWriteFields));
            var r = L ("attitude_allocation_v1", AttitudeV1.AllocationBlockProtocol, AttitudeV1.AllocationBlockSchema,
                LayoutRegistry.KindReturn, "option 4096 (D264); NaN values and state 0 without it");
            r.Once ("header", AllocationBlockHeader);
            r.Once ("block", J (AttitudeV1.AllocationFields, "attitude_tick", "ut", AttitudeV1.AllocationWriteFields));
            var f = L ("attitude_allocation_frames_v1", AttitudeV1.AllocationProtocolVersion,
                AttitudeV1.AllocationSchemaVersion, LayoutRegistry.KindFrame, "option 4096 (D264), one frame per step; " +
                reasons);
            f.Once ("header", J ("protocol", "schema", "attitude_tick", "ut", "persistent_id", AttitudeV1.AllocationFields,
                AttitudeV1.AllocationWriteFields));
            f.Rows ("engines", "@engine_count", AttitudeV1.AllocationEngineRowStride.ToString (
                System.Globalization.CultureInfo.InvariantCulture), "flight_id", "command_realized", "command",
                "realized_change", "phi", "slope_n");
            f.Rows ("gimbals", "@gimbal_count", AttitudeV1.AllocationGimbalRowStride.ToString (
                System.Globalization.CultureInfo.InvariantCulture), "flight_id", "ordinal", "realized_x_deg",
                "realized_y_deg", "command_x_deg", "command_y_deg", "phi", "force_per_deg_n");
            f.Once ("ctrl", "served_pitch", "served_roll", "served_yaw", "served_x", "served_y", "command_pitch",
                "command_roll", "command_yaw", "command_x", "command_y");
            f.Once ("clock", "master_tick");
            LayoutRegistry.Expect (f, "header", 5 + AttitudeV1.AllocationFields.Length + AttitudeV1.AllocationWriteFields.Length);
            LayoutRegistry.Expect (f, "engines", AttitudeV1.AllocationEngineRowStride);
            LayoutRegistry.Expect (f, "gimbals", AttitudeV1.AllocationGimbalRowStride);
            LayoutRegistry.Expect (f, "ctrl", AttitudeV1.AllocationCtrlStride);
            LayoutRegistry.Expect (f, "clock", AttitudeV1.AllocationClockStride);
        }
    }
}
