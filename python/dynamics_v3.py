"""Decoder for KRPC.Bridge Actuators DynamicsSnapshotV3.

This module is intentionally dependency-free. PDG2 may later wrap the decoded values in
NumPy arrays, but the wire contract itself stays inspectable with plain Python.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence
import math

PROTOCOL_VERSION = 3
SCHEMA_VERSION = 1
HEADER_STRIDE = 22
STATE_STRIDE = 67

STATE_FIELDS = (
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


def capability_names(mask: int) -> set[str]:
    return {name for bit, name in CAPABILITIES.items() if mask & (1 << bit)}


def decode_snapshot(values: Sequence[float]) -> dict:
    values = list(values)
    if not values:
        return {}
    if len(values) < HEADER_STRIDE:
        raise ValueError(f"DynamicsSnapshotV3 header truncated: {len(values)}")
    if int(values[0]) != PROTOCOL_VERSION:
        raise ValueError(f"unsupported Dynamics protocol {values[0]}")
    if int(values[1]) != SCHEMA_VERSION:
        raise ValueError(f"unsupported Dynamics schema {values[1]}")

    state_stride = int(values[12])
    engine_count, engine_stride = int(values[13]), int(values[14])
    gimbal_count, gimbal_stride = int(values[15]), int(values[16])
    transform_count, transform_stride = int(values[17]), int(values[18])
    if state_stride != STATE_STRIDE:
        raise ValueError(f"state stride {state_stride} != decoder {STATE_STRIDE}")
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
        "schema": int(values[1]),
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
        "state": dict(zip(STATE_FIELDS, state_values)),
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
    if int(values[0]) != PROTOCOL_VERSION or int(values[1]) != SCHEMA_VERSION:
        raise ValueError("unsupported DynamicsFramesV3 protocol/schema")
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
        "schema": int(values[1]),
        "frame_count": frame_count,
        "oldest_tick": int(values[3]),
        "newest_tick": int(values[4]),
        "dropped_before": bool(values[5]),
        "frames": frames,
    }
