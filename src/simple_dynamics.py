from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .actuator import EL05_PUBLIC_ENVELOPE, PublicActuatorEnvelope
from .actuator_matching import STANDARD_GRAVITY_M_PER_S2
from .stroke import normal_vertical_posture

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class VerticalTrajectory:
    mode: str
    mass_kg: float
    ballistic_height_m: float
    stroke_m: float
    duration_s: float
    acceleration_m_per_s2: float
    total_axial_force_n: float
    ideal_mechanical_work_j: float
    time_s: FloatArray
    l0_m: FloatArray
    body_speed_m_per_s: FloatArray
    joint_speed_rad_per_s: FloatArray
    joint_speed_rpm: FloatArray
    required_joint_torque_nm: FloatArray
    available_joint_torque_nm: FloatArray
    torque_utilization: FloatArray
    total_mechanical_power_w: FloatArray
    conservative_overload_time_s: float
    within_public_magnitude_envelope: bool


def _validate_inputs(
    mass_kg: float,
    l0_start_m: float,
    l0_end_m: float,
    ballistic_height_m: float,
    resolution: int,
) -> None:
    if mass_kg <= 0.0:
        raise ValueError("mass must be positive")
    if l0_end_m <= l0_start_m:
        raise ValueError("end length must exceed start length")
    if ballistic_height_m <= 0.0:
        raise ValueError("ballistic height must be positive")
    if resolution < 2:
        raise ValueError("resolution must be at least 2")


def _actuator_mapping(
    l0_m: FloatArray,
    body_speed_m_per_s: FloatArray,
    force_total_n: float,
    actuator: PublicActuatorEnvelope,
) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray, FloatArray]:
    joint_speeds = []
    required_torques = []
    available_torques = []
    for length, body_speed in zip(l0_m, body_speed_m_per_s, strict=True):
        posture = normal_vertical_posture(float(length))
        joint_speed = float(np.max(np.abs(posture.dq_dl0))) * abs(body_speed)
        required_torque = (
            force_total_n / 2.0 / posture.metrics.max_axial_force
        )
        joint_speeds.append(joint_speed)
        required_torques.append(required_torque)
        available_torques.append(actuator.peak_torque_at_speed(joint_speed))
    joint_speed_array = np.asarray(joint_speeds)
    required_array = np.asarray(required_torques)
    available_array = np.asarray(available_torques)
    utilization = np.divide(
        required_array,
        available_array,
        out=np.full_like(required_array, np.inf),
        where=available_array > 0.0,
    )
    rpm = joint_speed_array * 60.0 / (2.0 * np.pi)
    return (
        joint_speed_array,
        rpm,
        required_array,
        available_array,
        utilization,
    )


def _build_trajectory(
    *,
    mode: str,
    mass_kg: float,
    ballistic_height_m: float,
    stroke_m: float,
    acceleration_m_per_s2: float,
    duration_s: float,
    force_total_n: float,
    time_s: FloatArray,
    l0_m: FloatArray,
    speed_m_per_s: FloatArray,
    actuator: PublicActuatorEnvelope,
) -> VerticalTrajectory:
    (
        joint_speed,
        joint_rpm,
        required_torque,
        available_torque,
        utilization,
    ) = _actuator_mapping(l0_m, speed_m_per_s, force_total_n, actuator)
    overload_time = actuator.conservative_overload_time_s(
        float(np.max(required_torque)), stalled=True
    )
    within_envelope = bool(
        np.max(utilization) <= 1.0
        and np.max(joint_speed) <= 50.0
        and (np.isinf(overload_time) or duration_s <= overload_time)
    )
    power_sign = 1.0 if mode == "push" else -1.0
    return VerticalTrajectory(
        mode=mode,
        mass_kg=mass_kg,
        ballistic_height_m=ballistic_height_m,
        stroke_m=stroke_m,
        duration_s=duration_s,
        acceleration_m_per_s2=acceleration_m_per_s2,
        total_axial_force_n=force_total_n,
        ideal_mechanical_work_j=force_total_n * stroke_m,
        time_s=time_s,
        l0_m=l0_m,
        body_speed_m_per_s=speed_m_per_s,
        joint_speed_rad_per_s=joint_speed,
        joint_speed_rpm=joint_rpm,
        required_joint_torque_nm=required_torque,
        available_joint_torque_nm=available_torque,
        torque_utilization=utilization,
        total_mechanical_power_w=power_sign * force_total_n * speed_m_per_s,
        conservative_overload_time_s=overload_time,
        within_public_magnitude_envelope=within_envelope,
    )


def constant_acceleration_push(
    mass_kg: float,
    ballistic_height_m: float,
    *,
    l0_start_m: float = 0.070,
    l0_end_m: float = 0.120,
    resolution: int = 501,
    actuator: PublicActuatorEnvelope = EL05_PUBLIC_ENVELOPE,
) -> VerticalTrajectory:
    """Accelerate over the leg stroke for a requested post-takeoff rise."""
    _validate_inputs(
        mass_kg, l0_start_m, l0_end_m, ballistic_height_m, resolution
    )
    stroke = l0_end_m - l0_start_m
    takeoff_speed = np.sqrt(2.0 * STANDARD_GRAVITY_M_PER_S2 * ballistic_height_m)
    acceleration = takeoff_speed**2 / (2.0 * stroke)
    duration = takeoff_speed / acceleration
    time_s = np.linspace(0.0, duration, resolution)
    displacement = 0.5 * acceleration * time_s**2
    l0_m = l0_start_m + displacement
    speed = acceleration * time_s
    force_total = mass_kg * (acceleration + STANDARD_GRAVITY_M_PER_S2)
    return _build_trajectory(
        mode="push",
        mass_kg=mass_kg,
        ballistic_height_m=ballistic_height_m,
        stroke_m=stroke,
        acceleration_m_per_s2=acceleration,
        duration_s=duration,
        force_total_n=force_total,
        time_s=time_s,
        l0_m=l0_m,
        speed_m_per_s=speed,
        actuator=actuator,
    )


def constant_deceleration_landing(
    mass_kg: float,
    drop_height_m: float,
    *,
    l0_start_m: float = 0.120,
    l0_end_m: float = 0.070,
    resolution: int = 501,
    actuator: PublicActuatorEnvelope = EL05_PUBLIC_ENVELOPE,
) -> VerticalTrajectory:
    """Stop a vertical landing over the available compression stroke."""
    _validate_inputs(
        mass_kg, l0_end_m, l0_start_m, drop_height_m, resolution
    )
    stroke = l0_start_m - l0_end_m
    impact_speed = np.sqrt(2.0 * STANDARD_GRAVITY_M_PER_S2 * drop_height_m)
    deceleration = impact_speed**2 / (2.0 * stroke)
    duration = impact_speed / deceleration
    time_s = np.linspace(0.0, duration, resolution)
    compression = impact_speed * time_s - 0.5 * deceleration * time_s**2
    l0_m = l0_start_m - compression
    speed = impact_speed - deceleration * time_s
    force_total = mass_kg * (deceleration + STANDARD_GRAVITY_M_PER_S2)
    return _build_trajectory(
        mode="landing",
        mass_kg=mass_kg,
        ballistic_height_m=drop_height_m,
        stroke_m=stroke,
        acceleration_m_per_s2=deceleration,
        duration_s=duration,
        force_total_n=force_total,
        time_s=time_s,
        l0_m=l0_m,
        speed_m_per_s=speed,
        actuator=actuator,
    )
