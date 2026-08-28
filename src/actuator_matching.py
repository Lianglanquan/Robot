from dataclasses import dataclass

import numpy as np

from .actuator import EL05_PUBLIC_ENVELOPE, PublicActuatorEnvelope
from .stroke import normal_vertical_posture

STANDARD_GRAVITY_M_PER_S2 = 9.80665


@dataclass(frozen=True)
class StaticSupportMatch:
    mass_kg: float
    l0_m: float
    axial_force_per_leg_n: float
    joint_torque_required_nm: float
    stall_continuous_margin: float


@dataclass(frozen=True)
class ExtensionOperatingPoint:
    l0_m: float
    extension_speed_m_per_s: float
    joint_speed_rad_per_s: float
    joint_speed_rpm: float
    available_joint_torque_nm: float
    axial_force_per_leg_n: float
    total_axial_force_n: float
    total_mechanical_power_w: float


def static_support_match(
    mass_kg: float,
    l0_m: float,
    actuator: PublicActuatorEnvelope = EL05_PUBLIC_ENVELOPE,
) -> StaticSupportMatch:
    """Match symmetric two-leg static support against the published stall rating."""
    if mass_kg <= 0.0:
        raise ValueError("mass must be positive")
    posture = normal_vertical_posture(l0_m)
    force_per_leg = mass_kg * STANDARD_GRAVITY_M_PER_S2 / 2.0
    torque = force_per_leg / posture.metrics.max_axial_force
    return StaticSupportMatch(
        mass_kg=mass_kg,
        l0_m=l0_m,
        axial_force_per_leg_n=force_per_leg,
        joint_torque_required_nm=torque,
        stall_continuous_margin=actuator.stall_continuous_torque_nm / torque,
    )


def extension_operating_point(
    l0_m: float,
    extension_speed_m_per_s: float,
    actuator: PublicActuatorEnvelope = EL05_PUBLIC_ENVELOPE,
) -> ExtensionOperatingPoint:
    """Map the public 48 V peak envelope through the centered vertical path."""
    posture = normal_vertical_posture(l0_m)
    speed = abs(extension_speed_m_per_s)
    joint_speed = float(np.max(np.abs(posture.dq_dl0))) * speed
    joint_speed_rpm = joint_speed * 60.0 / (2.0 * np.pi)
    joint_torque = actuator.peak_torque_at_speed(joint_speed)
    force_per_leg = posture.metrics.max_axial_force * joint_torque
    total_force = 2.0 * force_per_leg
    return ExtensionOperatingPoint(
        l0_m=l0_m,
        extension_speed_m_per_s=speed,
        joint_speed_rad_per_s=joint_speed,
        joint_speed_rpm=joint_speed_rpm,
        available_joint_torque_nm=joint_torque,
        axial_force_per_leg_n=force_per_leg,
        total_axial_force_n=total_force,
        total_mechanical_power_w=total_force * speed,
    )
