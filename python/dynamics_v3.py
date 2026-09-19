"""Decoder for KRPC.Bridge Actuators DynamicsSnapshotV3.

This module is intentionally dependency-free. PDG2 may later wrap the decoded values in
NumPy arrays, but the wire contract itself stays inspectable with plain Python.

REPÈRES ET UNITÉS (Q7, établi le 19/09/2026 — décompilation KSP 1.12.5 + 3 vols) :

* ``angular_velocity_world_*`` et ``angular_accel_derived_*`` sont en repère CORPS
  (``ReferenceTransform`` : x droite, y nez, z ventre), PAS en repère monde : KSP calcule
  ``Vessel.angularVelocity`` comme la moyenne massique de
  ``Inverse(ReferenceTransform.rotation) * rb.angularVelocity``. Les noms historiques sont
  gardés pour la compatibilité du fil et des journaux. Sur 3 vols témoins, la dérivée du
  quaternion publié donne un résidu médian de 0,3 % en repère corps contre > 100 % en
  repère monde.
* ``moi_native_*`` est la DIAGONALE du tenseur d'inertie autour du CdM, repère corps, en
  tonne·m² (kRPC ``moment_of_inertia`` = cette valeur × 1000, en kg·m²).

``canonical_rotation_state`` rend ces grandeurs sous des noms explicites ; les nouveaux
consommateurs doivent passer par elle. Les décodeurs ``decode_ext_*`` couvrent le
transport séparé DynamicsExtV1 (même tick, né fermé côté bridge) et ``decode_tracked_*``
l'état d'un vaisseau suivi par ``persistentId``.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence
import math

PROTOCOL_VERSION = 3
SCHEMA_VERSION = 2
SUPPORTED_SCHEMAS = (1, 2)
HEADER_STRIDE = 22
STATE_STRIDE_V1 = 67
STATE_STRIDE_V2 = 116
STATE_STRIDE = STATE_STRIDE_V2

STATE_FIELDS_V1 = (
    "position_world_x", "position_world_y", "position_world_z",
    "orbital_velocity_world_x", "orbital_velocity_world_y", "orbital_velocity_world_z",
    "surface_velocity_world_x", "surface_velocity_world_y", "surface_velocity_world_z",
    "surface_accel_derived_x", "surface_accel_derived_y", "surface_accel_derived_z",
    "orbital_accel_derived_x", "orbital_accel_derived_y", "orbital_accel_derived_z",
    "rotation_world_x", "rotation_world_y", "rotation_world_z", "rotation_world_w",
    "angular_velocity_world_x", "angular_velocity_world_y", "angular_velocity_world_z",
    "angular_accel_derived_x", "angular_accel_derived_y", "angular_accel_derived_z",
    "mass_tonnes",
    "com_world_x", "com_world_y", "com_world_z",
    "moi_native_x", "moi_native_y", "moi_native_z",
    "gravity_world_x", "gravity_world_y", "gravity_world_z",
    "atm_density", "static_pressure_kpa", "dynamic_pressure_kpa", "mach",
    "altitude_asl_m", "radar_altitude_m", "latitude_deg", "longitude_deg",
    "global_throttle",
    "aero_force_raw_x", "aero_force_raw_y", "aero_force_raw_z",
    "aero_torque_raw_x", "aero_torque_raw_y", "aero_torque_raw_z",
    "drag_vector_raw_x", "drag_vector_raw_y", "drag_vector_raw_z",
    "lift_vector_raw_x", "lift_vector_raw_y", "lift_vector_raw_z",
    "surface_speed_ms", "orbital_speed_ms", "gee_force_raw", "situation",
    "mission_time_s", "packed",
    "surface_velocity_reference_x", "surface_velocity_reference_y",
    "surface_velocity_reference_z", "aoa_raw", "sideslip_raw",
)

STATE_FIELDS_V2_EXTRA = (
    "engine_force_body_n_x", "engine_force_body_n_y", "engine_force_body_n_z",
    "engine_force_world_n_x", "engine_force_world_n_y", "engine_force_world_n_z",
    "engine_torque_body_nm_x", "engine_torque_body_nm_y", "engine_torque_body_nm_z",
    "engine_torque_world_nm_x", "engine_torque_world_nm_y", "engine_torque_world_nm_z",
    "aero_force_body_n_x", "aero_force_body_n_y", "aero_force_body_n_z",
    "aero_force_world_n_x", "aero_force_world_n_y", "aero_force_world_n_z",
    "aero_torque_body_nm_x", "aero_torque_body_nm_y", "aero_torque_body_nm_z",
    "aero_torque_world_nm_x", "aero_torque_world_nm_y", "aero_torque_world_nm_z",
    "aero_lift_body_n_x", "aero_lift_body_n_y", "aero_lift_body_n_z",
    "aero_drag_body_n_x", "aero_drag_body_n_y", "aero_drag_body_n_z",
    "aero_side_force_body_n_x", "aero_side_force_body_n_y", "aero_side_force_body_n_z",
    "dynamic_pressure_pa", "static_pressure_pa", "atmosphere_density_kg_m3",
    "speed_of_sound_ms", "true_air_speed_ms", "aoa_deg", "sideslip_deg",
    "external_force_residual_world_n_x", "external_force_residual_world_n_y",
    "external_force_residual_world_n_z",
    "external_force_residual_body_n_x", "external_force_residual_body_n_y",
    "external_force_residual_body_n_z",
    "unexplained_non_aero_force_world_n_x",
    "unexplained_non_aero_force_world_n_y",
    "unexplained_non_aero_force_world_n_z",
)

STATE_FIELDS_BY_SCHEMA = {
    1: STATE_FIELDS_V1,
    2: STATE_FIELDS_V1 + STATE_FIELDS_V2_EXTRA,
}
STATE_STRIDE_BY_SCHEMA = {k: len(v) for k, v in STATE_FIELDS_BY_SCHEMA.items()}
STATE_FIELDS = STATE_FIELDS_BY_SCHEMA[SCHEMA_VERSION]

CAPABILITIES = {
    0: "position",
    1: "orbital_velocity",
    2: "surface_velocity",
    3: "surface_acceleration_derived",
    4: "orbital_acceleration_derived",
    5: "rotation",
    6: "angular_velocity",
    7: "angular_acceleration_derived",
    8: "mass",
    9: "center_of_mass",
    10: "moi",
    11: "gravity",
    12: "atmosphere",
    13: "aero_force",
    14: "aero_torque",
    15: "drag_vector",
    16: "lift_vector",
    17: "body_surface_velocity",
    18: "aoa",
    19: "sideslip",
    20: "global_throttle",
    21: "actuators",
    22: "realized_engine_wrench",
    23: "krpc_live_aero_force",
    24: "krpc_live_aero_torque",
    25: "krpc_live_aero_components",
    26: "krpc_aero_angles",
    27: "krpc_aero_thermo",
    28: "external_force_residual",
    29: "unexplained_non_aero_force",
    30: "realized_engine_force_on_vessel",
}

ENGINE_FIELDS = (
    "flight_id", "ordinal", "ignited", "requested_throttle", "current_throttle",
    "final_thrust_kn", "max_thrust_kn", "thrust_percentage", "independent_throttle",
    "independent_percentage", "finite_response", "acceleration_speed",
    "deceleration_speed", "requested_mass_flow", "propellant_requirement_met",
    "real_isp_s", "thrust_transform_count", "leased", "flameout", "min_thrust_kn",
)

GIMBAL_FIELDS = (
    "flight_id", "ordinal", "locked", "stock_active", "limiter", "range",
    "range_x_negative", "range_x_positive", "range_y_negative", "range_y_positive",
    "finite_response", "response_speed", "actual_x_deg", "actual_y_deg", "actual_z_deg",
    "transform_count", "leased", "target_x_deg", "target_y_deg",
)

TRANSFORM_FIELDS = (
    "flight_id", "engine_ordinal", "transform_index", "multiplier",
    "lever_x", "lever_y", "lever_z", "direction_x", "direction_y", "direction_z",
)


def _rows(values: Sequence[float], start: int, count: int, stride: int, names: Sequence[str]):
    rows = []
    for i in range(count):
        row = values[start + i * stride : start + (i + 1) * stride]
        if len(row) != stride:
            raise ValueError("truncated DynamicsSnapshotV3 row")
        rows.append(dict(zip(names, row)))
    return rows, start + count * stride


def capability_names(mask: int) -> list[str]:
    """Return a deterministic JSON-safe capability list.

    A ``set`` here used to make every decoded DynamicsV3 snapshot fail JSON
    serialization in FlightRecorder, leaving ``dynamics_v3.jsonl`` empty even
    though the ring buffer drain itself was healthy.
    """
    return [name for bit, name in sorted(CAPABILITIES.items())
            if mask & (1 << bit)]


def decode_snapshot(values: Sequence[float]) -> dict:
    values = list(values)
    if not values:
        return {}
    if len(values) < HEADER_STRIDE:
        raise ValueError(f"DynamicsSnapshotV3 header truncated: {len(values)}")
    if int(values[0]) != PROTOCOL_VERSION:
        raise ValueError(f"unsupported Dynamics protocol {values[0]}")
    schema = int(values[1])
    if schema not in SUPPORTED_SCHEMAS:
        raise ValueError(f"unsupported Dynamics schema {values[1]}")

    state_stride = int(values[12])
    engine_count, engine_stride = int(values[13]), int(values[14])
    gimbal_count, gimbal_stride = int(values[15]), int(values[16])
    transform_count, transform_stride = int(values[17]), int(values[18])
    expected_state_stride = STATE_STRIDE_BY_SCHEMA[schema]
    if state_stride != expected_state_stride:
        raise ValueError(
            f"state stride {state_stride} != decoder schema {schema} stride "
            f"{expected_state_stride}")
    if engine_stride != len(ENGINE_FIELDS):
        raise ValueError(f"engine stride {engine_stride} != decoder {len(ENGINE_FIELDS)}")
    if gimbal_stride != len(GIMBAL_FIELDS):
        raise ValueError(f"gimbal stride {gimbal_stride} != decoder {len(GIMBAL_FIELDS)}")
    if transform_stride != len(TRANSFORM_FIELDS):
        raise ValueError(f"transform stride {transform_stride} != decoder {len(TRANSFORM_FIELDS)}")

    start = HEADER_STRIDE
    state_values = values[start : start + state_stride]
    if len(state_values) != state_stride:
        raise ValueError("truncated DynamicsSnapshotV3 state block")
    start += state_stride
    engines, start = _rows(values, start, engine_count, engine_stride, ENGINE_FIELDS)
    gimbals, start = _rows(values, start, gimbal_count, gimbal_stride, GIMBAL_FIELDS)
    transforms, start = _rows(values, start, transform_count, transform_stride, TRANSFORM_FIELDS)
    if start != len(values):
        raise ValueError(f"unexpected DynamicsSnapshotV3 tail: {len(values) - start} doubles")

    mask = int(values[11])
    return {
        "protocol": int(values[0]),
        "schema": schema,
        "physics_tick": int(values[2]),
        "ut": values[3],
        "physics_dt": values[4],
        "vessel_persistent_id": int(values[5]),
        "topology_generation": int(values[6]),
        "last_accepted_sequence": int(values[7]),
        "last_applied_sequence": int(values[8]),
        "last_applied_tick": int(values[9]),
        "last_result": int(values[10]),
        "capability_mask": mask,
        "capabilities": capability_names(mask),
        "history_capacity": int(values[19]),
        "history_count_before_capture": int(values[20]),
        "capture_phase": int(values[21]),
        "state": dict(zip(STATE_FIELDS_BY_SCHEMA[schema], state_values)),
        "engines": engines,
        "gimbals": gimbals,
        "thrust_transforms": transforms,
    }


def decode_history(values: Sequence[float]) -> dict:
    values = list(values)
    if not values:
        return {"frames": []}
    if len(values) < 6:
        raise ValueError("DynamicsFramesV3 header truncated")
    if int(values[0]) != PROTOCOL_VERSION or int(values[1]) not in SUPPORTED_SCHEMAS:
        raise ValueError("unsupported DynamicsFramesV3 protocol/schema")
    schema = int(values[1])
    frame_count = int(values[2])
    i = 6
    frames = []
    for _ in range(frame_count):
        if i >= len(values):
            raise ValueError("truncated DynamicsFramesV3 length prefix")
        length = int(values[i]); i += 1
        frame = values[i:i + length]; i += length
        if len(frame) != length:
            raise ValueError("truncated DynamicsFramesV3 frame")
        frames.append(decode_snapshot(frame))
    if i != len(values):
        raise ValueError(f"unexpected DynamicsFramesV3 tail: {len(values)-i} doubles")
    return {
        "protocol": int(values[0]),
        "schema": schema,
        "frame_count": frame_count,
        "oldest_tick": int(values[3]),
        "newest_tick": int(values[4]),
        "dropped_before": bool(values[5]),
        "frames": frames,
    }


# =====================================================================================
# Repères canoniques (Q7)
# =====================================================================================

#: Repère réel de ``angular_velocity_world_*`` / ``angular_accel_derived_*`` (voir en-tête).
ANGULAR_VELOCITY_FRAME = "body"
#: Unité réelle de ``moi_native_*``.
MOI_NATIVE_UNIT = "tonne_m2"
MOI_NATIVE_TO_KG_M2 = 1000.0


def _vec3(state: dict, prefix: str, suffixes=("x", "y", "z")):
    """Triplet fini lu dans ``state`` (clés ``prefix + suffixe``), ou None."""
    try:
        v = tuple(float(state[prefix + s]) for s in suffixes)
    except (KeyError, TypeError, ValueError):
        return None
    return v if all(math.isfinite(c) for c in v) else None


def quat_rotate(q_xyzw, v):
    """Tourne ``v`` par le quaternion Unity ``(x, y, z, w)`` : q·v·q⁻¹ (produit de Hamilton).

    Unity compose et applique ses quaternions avec les mêmes formules que le produit de
    Hamilton ; la main gauche du repère ne change pas l'algèbre. ``q`` est normalisé.
    """
    x, y, z, w = (float(c) for c in q_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not (n > 1e-12):
        raise ValueError("quaternion nul")
    x, y, z, w = x / n, y / n, z / n, w / n
    vx, vy, vz = (float(c) for c in v)
    # t = 2·(q_vec × v) ; v' = v + w·t + q_vec × t
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def canonical_rotation_state(snapshot: dict) -> dict:
    """État de rotation d'une trame V3 décodée, sous des noms qui disent la vérité.

    Rend ``omega_body_rad_s``, ``omega_dot_body_rad_s2`` (repère corps : x droite, y nez,
    z ventre), ``moi_body_kg_m2`` (diagonale, ×1000 depuis la tonne·m² native),
    ``rotation_world_xyzw`` et ``omega_world_rad_s`` = q·ω_corps (monde Unity, même
    tick). Toute grandeur absente vaut None ; rien n'est inventé. Si la trame porte une
    extension ``ext_v1`` (observer P5), ses valeurs canoniques sont préférées.
    """
    st = (snapshot or {}).get("state") or {}
    caps = set((snapshot or {}).get("capabilities") or ())
    omega = _vec3(st, "angular_velocity_world_") if "angular_velocity" in caps else None
    omega_dot = (_vec3(st, "angular_accel_derived_")
                 if "angular_acceleration_derived" in caps else None)
    moi = _vec3(st, "moi_native_") if "moi" in caps else None
    moi_kg = (tuple(c * MOI_NATIVE_TO_KG_M2 for c in moi)
              if moi is not None and min(moi) > 0.0 else None)
    source = "v3_champs_historiques"
    ext = (snapshot or {}).get("ext_v1")
    if isinstance(ext, dict) and ext.get("state"):
        est = ext["state"]
        omega = _vec3(est, "omega_body_rad_s_") or omega
        omega_dot = _vec3(est, "omega_dot_body_rad_s2_") or omega_dot
        moi_kg = _vec3(est, "moi_body_kg_m2_") or moi_kg
        source = "ext_v1"
    q = _vec3(st, "rotation_world_", ("x", "y", "z", "w")) if "rotation" in caps else None
    omega_world = None
    if q is not None and omega is not None:
        try:
            omega_world = quat_rotate(q, omega)
        except ValueError:
            omega_world = None
    return {
        "physics_tick": (snapshot or {}).get("physics_tick"),
        "omega_body_rad_s": omega,
        "omega_dot_body_rad_s2": omega_dot,
        "moi_body_kg_m2": moi_kg,
        "rotation_world_xyzw": q,
        "omega_world_rad_s": omega_world,
        "source": source,
    }


# =====================================================================================
# DynamicsExtV1 — transport séparé, même Pump et même tick que V3, né fermé
# =====================================================================================

EXT_PROTOCOL_VERSION = 1
EXT_SCHEMA_VERSION = 1
EXT_HEADER_STRIDE = 19


def _xyz(prefix: str) -> tuple:
    return (prefix + "x", prefix + "y", prefix + "z")


EXT_STATE_FIELDS = (
    *_xyz("omega_body_rad_s_"),
    *_xyz("omega_dot_body_rad_s2_"),
    *_xyz("moi_body_kg_m2_"),
    *_xyz("rcs_force_body_n_"),
    *_xyz("rcs_force_world_n_"),
    *_xyz("rcs_torque_body_nm_"),
    *_xyz("reaction_wheel_torque_body_nm_"),
    *_xyz("control_torque_body_nm_"),
    "engines_available_thrust_kn", "engines_max_thrust_kn",
    "engines_realized_mass_flow_kg_s", "engines_requested_mass_flow_kg_s",
    *_xyz("indi_delta_omega_dot_rad_s2_"),
    *_xyz("indi_pred_b_rad_s2_"),
    *_xyz("indi_pred_m_rad_s2_"),
    *_xyz("indi_pred_aero_rad_s2_"),
    *_xyz("indi_residual_b_rad_s2_"),
    *_xyz("indi_residual_m_rad_s2_"),
    *_xyz("indi_residual_b_filtered_rad_s2_"),
    "indi_filter_tau_s", "indi_lag_ticks", "indi_valid_streak",
)
EXT_STATE_STRIDE = len(EXT_STATE_FIELDS)

EXT_ENGINE_FIELDS = (
    "flight_id", "ordinal", "available_thrust_kn", "max_thrust_kn",
    "thrust_per_throttle_kn", "realized_mass_flow_kg_s", "requested_mass_flow_kg_s",
    "isp_at_pressure_s", "static_pressure_atm",
)
EXT_SURFACE_FIELDS = (
    "flight_id", "kind", "ordinal", "deflection_deg", "action_deg",
    "ctrl_surface_range_deg", "authority_limiter_pct", "deploy", "deploy_angle_deg",
    "current_deploy_angle_deg", "actuator_speed_deg_s", "use_exponential_speed",
    "ignore_mask",
)
EXT_RCS_FIELDS = (
    "flight_id", "ordinal", "module_enabled", "rcs_enabled", "rcs_active",
    "just_for_show", "nozzle_count", "thrust_sum_kn", "thruster_power_kn",
    "realized_isp_s", *_xyz("force_body_n_"), *_xyz("torque_body_nm_"),
)
EXT_B_FIELDS = (
    "flight_id", "ordinal", "kind", "axis", "b_body_x", "b_body_y", "b_body_z",
    "u", "du",
)
EXT_RESPONSE_FIELDS = (
    "kind", "flight_id", "ordinal", "sequence", "apply_tick", "first_response_tick",
    "baseline_x", "command_x", "realized_x", "baseline_y", "command_y", "realized_y",
)

#: Surfaces : 1 = ModuleAeroSurface (aérofrein), 2 = ModuleControlSurface ou dérivé.
EXT_SURFACE_KINDS = {1: "aero_surface", 2: "control_surface"}
#: Colonnes B : 1 = cardan (N·m/deg, u en deg), 2 = poussée moteur (N·m/kN, u en kN).
EXT_B_KINDS = {1: "gimbal", 2: "engine_thrust"}
#: Réponses : 1 = moteur (currentThrottle 0-1), 2 = cardan (actuationLocal x/y en deg).
EXT_RESPONSE_KINDS = {1: "engine", 2: "gimbal"}

EXT_CAPABILITIES = {
    0: "rotational_state_body",
    1: "moi_kg_m2",
    2: "rcs_realized",
    3: "reaction_wheel_realized",
    4: "control_torque",
    5: "engine_availability",
    6: "control_surfaces",
    7: "b_matrix",
    8: "indi_delta_omega_dot",
    9: "indi_prediction_b",
    10: "indi_prediction_m",
    11: "indi_prediction_aero",
    12: "indi_residual_b",
    13: "indi_residual_m",
    14: "response_tracking",
}

_EXT_SECTIONS = (
    ("engines", EXT_ENGINE_FIELDS),
    ("surfaces", EXT_SURFACE_FIELDS),
    ("rcs", EXT_RCS_FIELDS),
    ("b_columns", EXT_B_FIELDS),
    ("responses", EXT_RESPONSE_FIELDS),
)


def ext_capability_names(mask: int) -> list[str]:
    """Liste déterministe (JSON-sûre) des capacités DynamicsExtV1."""
    return [name for bit, name in sorted(EXT_CAPABILITIES.items()) if mask & (1 << bit)]


def decode_ext_snapshot(values: Sequence[float]) -> dict:
    """Décode une trame DynamicsExtV1 ; lève sur toute dérive de protocole ou de stride."""
    values = list(values)
    if not values:
        return {}
    if len(values) < EXT_HEADER_STRIDE:
        raise ValueError(f"DynamicsExtV1 header truncated: {len(values)}")
    if int(values[0]) != EXT_PROTOCOL_VERSION:
        raise ValueError(f"unsupported DynamicsExtV1 protocol {values[0]}")
    if int(values[1]) != EXT_SCHEMA_VERSION:
        raise ValueError(f"unsupported DynamicsExtV1 schema {values[1]}")
    state_stride = int(values[8])
    if state_stride != EXT_STATE_STRIDE:
        raise ValueError(
            f"DynamicsExtV1 state stride {state_stride} != decoder {EXT_STATE_STRIDE}")
    start = EXT_HEADER_STRIDE
    state_values = values[start:start + state_stride]
    if len(state_values) != state_stride:
        raise ValueError("truncated DynamicsExtV1 state block")
    start += state_stride
    sections = {}
    for index, (name, fields) in enumerate(_EXT_SECTIONS):
        count = int(values[9 + 2 * index])
        stride = int(values[10 + 2 * index])
        if stride != len(fields):
            raise ValueError(
                f"DynamicsExtV1 {name} stride {stride} != decoder {len(fields)}")
        if count < 0:
            raise ValueError(f"DynamicsExtV1 negative {name} count")
        rows, start = _rows(values, start, count, stride, fields)
        sections[name] = rows
    if start != len(values):
        raise ValueError(f"unexpected DynamicsExtV1 tail: {len(values) - start} doubles")
    mask = int(values[7])
    out = {
        "protocol": int(values[0]),
        "schema": int(values[1]),
        "physics_tick": int(values[2]),
        "ut": values[3],
        "physics_dt": values[4],
        "vessel_persistent_id": int(values[5]),
        "topology_generation": int(values[6]),
        "capability_mask": mask,
        "capabilities": ext_capability_names(mask),
        "state": dict(zip(EXT_STATE_FIELDS, state_values)),
    }
    out.update(sections)
    return out


def decode_ext_history(values: Sequence[float]) -> dict:
    """``DynamicsExtFramesV1`` / ``TrackedDynamicsExtFramesV1`` (en-tête à 6 valeurs)."""
    values = list(values)
    if not values:
        return {"frames": []}
    if len(values) < 6:
        raise ValueError("DynamicsExtFramesV1 header truncated")
    if int(values[0]) != EXT_PROTOCOL_VERSION or int(values[1]) != EXT_SCHEMA_VERSION:
        raise ValueError("unsupported DynamicsExtFramesV1 protocol/schema")
    frame_count = int(values[2])
    i = 6
    frames = []
    for _ in range(frame_count):
        if i >= len(values):
            raise ValueError("truncated DynamicsExtFramesV1 length prefix")
        length = int(values[i]); i += 1
        frame = values[i:i + length]; i += length
        if len(frame) != length:
            raise ValueError("truncated DynamicsExtFramesV1 frame")
        frames.append(decode_ext_snapshot(frame))
    if i != len(values):
        raise ValueError(f"unexpected DynamicsExtFramesV1 tail: {len(values) - i} doubles")
    return {
        "protocol": int(values[0]),
        "schema": int(values[1]),
        "frame_count": frame_count,
        "oldest_tick": int(values[3]),
        "newest_tick": int(values[4]),
        "dropped_before": bool(values[5]),
        "frames": frames,
    }


def decode_ext_status(values: Sequence[float]) -> dict:
    """``DynamicsExtStatusV1`` / ``DynamicsExtEnableV1`` (8 doubles)."""
    v = [float(x) for x in values]
    if len(v) != 8:
        raise ValueError(f"DynamicsExtStatusV1 attend 8 doubles, reçu {len(v)}")
    return {
        "protocol": int(v[0]),
        "schema": int(v[1]),
        "armed": bool(v[2]),
        "lease_remaining_s": v[3],
        "latest_tick": int(v[4]),
        "history_count": int(v[5]),
        "history_capacity": int(v[6]),
        "capability_mask": int(v[7]),
        "capabilities": ext_capability_names(int(v[7])),
    }


def attach_ext_frames(v3_frames, ext_frames) -> int:
    """Joint chaque trame ext à la trame V3 de même (tick, persistentId) : ``snap["ext_v1"]``.

    Rend le nombre de trames jointes. Une trame V3 sans jumelle ext reste intacte : la
    jointure ne fabrique rien.
    """
    index = {}
    for ext in ext_frames or ():
        if isinstance(ext, dict) and ext:
            index[(int(ext.get("physics_tick", -1)),
                   int(ext.get("vessel_persistent_id", -1)))] = ext
    joined = 0
    for snap in v3_frames or ():
        if not isinstance(snap, dict) or not snap:
            continue
        key = (int(snap.get("physics_tick", -2)), int(snap.get("vessel_persistent_id", -2)))
        ext = index.get(key)
        if ext is not None:
            snap["ext_v1"] = ext
            joined += 1
    return joined


# =====================================================================================
# Vaisseaux suivis par persistentId (vols doubles) — trames au format V3
# =====================================================================================

TRACKED_REGISTRY_FIELDS = (
    "persistent_id", "loaded", "lease_remaining_s", "topology_generation",
    "latest_tick", "history_count",
)


def persistent_id_from_ident(vessel_ids: str) -> str:
    """``conn.ident.vessel_ids(vessel)`` rend ``"persistentId\\tGuid"`` : garde le 1er champ."""
    champ = str(vessel_ids or "").split("\t", 1)[0].strip()
    if not champ.isdigit() or int(champ) > 0xFFFFFFFF:
        raise ValueError(f"persistentId illisible : {vessel_ids!r}")
    return champ


def decode_tracked_registry(values: Sequence[float]) -> dict:
    """``TrackedVesselsV3`` : en-tête (protocole, nombre, stride) puis une ligne par vaisseau."""
    v = [float(x) for x in values]
    if len(v) < 3:
        raise ValueError("TrackedVesselsV3 header truncated")
    if int(v[0]) != PROTOCOL_VERSION:
        raise ValueError(f"unsupported TrackedVesselsV3 protocol {v[0]}")
    count, stride = int(v[1]), int(v[2])
    if stride != len(TRACKED_REGISTRY_FIELDS):
        raise ValueError(f"TrackedVesselsV3 stride {stride} != {len(TRACKED_REGISTRY_FIELDS)}")
    rows, end = _rows(v, 3, count, stride, TRACKED_REGISTRY_FIELDS)
    if end != len(v):
        raise ValueError("unexpected TrackedVesselsV3 tail")
    for row in rows:
        row["persistent_id"] = str(int(row["persistent_id"]))
        row["loaded"] = bool(row["loaded"])
    return {"protocol": int(v[0]), "vessels": rows}


# =====================================================================================
# SimulateAerodynamicWrenchBatchV1 — N états en un RPC (lecture seule)
# =====================================================================================

AERO_BATCH_PROTOCOL_VERSION = 1
AERO_BATCH_INPUT_STRIDE = 14
AERO_BATCH_OUTPUT_STRIDE = 7
AERO_BATCH_MAX_STATES = 32


def encode_aero_wrench_states(states) -> list[float]:
    """Aplati des états ``(position3, vitesse3, rotation_xyzw4, omega3, ut)`` en 14n doubles.

    Chaque état est un mapping à clés ``position``, ``velocity``, ``rotation``,
    ``angular_velocity``, ``ut`` ou une séquence déjà plate de 14 nombres. Refuse avant le
    RPC plutôt que de laisser le bridge rendre une ligne en échec.
    """
    flat: list[float] = []
    for state in states:
        if isinstance(state, dict):
            row = [*state["position"], *state["velocity"], *state["rotation"],
                   *state.get("angular_velocity", (0.0, 0.0, 0.0)), state["ut"]]
        else:
            row = list(state)
        row = [float(x) for x in row]
        if len(row) != AERO_BATCH_INPUT_STRIDE:
            raise ValueError(f"état aéro de {len(row)} valeurs, 14 attendues")
        if not all(math.isfinite(x) for x in row):
            raise ValueError("état aéro non fini")
        flat.extend(row)
    if len(flat) // AERO_BATCH_INPUT_STRIDE > AERO_BATCH_MAX_STATES:
        raise ValueError(f"au plus {AERO_BATCH_MAX_STATES} états par appel")
    return flat


def decode_aero_wrench_batch(values: Sequence[float]) -> dict:
    """Réponse du lot : protocole, nombre, stride, repère, puis (ok, force N, couple N·m)."""
    v = [float(x) for x in values]
    if len(v) < 4:
        raise ValueError("SimulateAerodynamicWrenchBatchV1 header truncated")
    if int(v[0]) != AERO_BATCH_PROTOCOL_VERSION:
        raise ValueError(f"unsupported aero batch protocol {v[0]}")
    count, stride = int(v[1]), int(v[2])
    if stride != AERO_BATCH_OUTPUT_STRIDE:
        raise ValueError(f"aero batch stride {stride} != {AERO_BATCH_OUTPUT_STRIDE}")
    if len(v) != 4 + count * stride:
        raise ValueError("aero batch length mismatch")
    rows = []
    for i in range(count):
        r = v[4 + i * stride: 4 + (i + 1) * stride]
        ok = r[0] != 0.0
        rows.append({
            "ok": ok,
            "force_n": tuple(r[1:4]) if ok else None,
            "torque_nm": tuple(r[4:7]) if ok else None,
        })
    return {
        "protocol": int(v[0]),
        "frame": "body_non_rotating" if v[3] != 0.0 else "body_rotating",
        "results": rows,
    }
