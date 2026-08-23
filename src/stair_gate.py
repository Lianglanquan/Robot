from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .parameters import DEFAULT_PARAMETERS, FiveBarParameters
from .stroke import normal_vertical_posture

FloatArray = NDArray[np.float64]
GRAVITY_M_PER_S2 = 9.80665
EVENT_COLUMNS = (
    "l0",
    "leg_displacement",
    "leg_speed",
    "joint_torque",
    "joint_speed",
    "joint_rpm",
    "total_joint_power",
)


@dataclass(frozen=True)
class AxialEvent:
    kind: str
    mass_kg: float
    start_length_m: float
    end_length_m: float
    external_height_m: float
    total_axial_force_n: float
    actuator_work_j: float
    duration_s: float
    values: FloatArray

    def column(self, name: str) -> FloatArray:
        try:
            index = EVENT_COLUMNS.index(name)
        except ValueError as error:
            raise KeyError(name) from error
        return self.values[:, index]

    @property
    def max_joint_torque_nm(self) -> float:
        return float(np.max(self.column("joint_torque")))

    @property
    def max_joint_speed_rpm(self) -> float:
        return float(np.max(self.column("joint_rpm")))

    @property
    def max_total_joint_power_w(self) -> float:
        return float(np.max(self.column("total_joint_power")))


def wheel_clearance_com_rises(
    step_height_m: float,
    wheel_radius_m: float,
    tuck_stroke_m: float,
    tuck_effectiveness: float,
) -> dict[str, float]:
    """Return task-space COM-rise benchmarks, not validated trajectories."""
    if step_height_m <= 0.0 or wheel_radius_m <= 0.0:
        raise ValueError("step height and wheel radius must be positive")
    if not 0.0 <= tuck_effectiveness <= 1.0 or tuck_stroke_m < 0.0:
        raise ValueError("tuck inputs are outside their physical ranges")
    tuck_lift = tuck_effectiveness * tuck_stroke_m
    edge_contact_rise = step_height_m - wheel_radius_m
    if tuck_lift >= edge_contact_rise:
        raise ValueError("tuck lift exceeds the screened step geometry")
    return {
        "full_clear_no_tuck": step_height_m,
        "full_clear_with_tuck": step_height_m - tuck_lift,
        "edge_contact_no_tuck": edge_contact_rise,
        "edge_contact_with_tuck": edge_contact_rise - tuck_lift,
    }


def static_edge_torque_per_wheel_nm(
    mass_kg: float, wheel_radius_m: float, wheel_count: int = 2
) -> float:
    """Return the worst gravity moment at initial upper-corner pivot contact."""
    if mass_kg <= 0.0 or wheel_radius_m <= 0.0 or wheel_count < 1:
        raise ValueError("edge-pivot inputs must be positive")
    return mass_kg * GRAVITY_M_PER_S2 * wheel_radius_m / wheel_count


def _event_values(
    lengths: FloatArray,
    displacement: FloatArray,
    leg_speed: FloatArray,
    total_axial_force_n: float,
    parameters: FiveBarParameters,
) -> FloatArray:
    rows = []
    for l0, travel, speed in zip(lengths, displacement, leg_speed, strict=True):
        posture = normal_vertical_posture(float(l0), parameters)
        joint_torque = (
            total_axial_force_n / 2.0 / posture.metrics.max_axial_force
        )
        joint_speed = float(np.max(np.abs(posture.dq_dl0))) * float(speed)
        rows.append(
            (
                l0,
                travel,
                speed,
                joint_torque,
                joint_speed,
                joint_speed * 60.0 / (2.0 * np.pi),
                4.0 * joint_torque * joint_speed,
            )
        )
    return np.asarray(rows, dtype=float)


def constant_force_jump(
    mass_kg: float,
    target_com_rise_m: float,
    start_length_m: float = 0.070,
    end_length_m: float = 0.120,
    *,
    resolution: int = 501,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> AxialEvent:
    """Screen an ideal symmetric jump using a constant-force point-mass model."""
    stroke = end_length_m - start_length_m
    if mass_kg <= 0.0 or stroke <= 0.0 or resolution < 2:
        raise ValueError("jump mass, stroke, and resolution must be positive")
    if target_com_rise_m <= stroke:
        raise ValueError("target rise must exceed stance stroke for a flight phase")

    force = mass_kg * GRAVITY_M_PER_S2 * target_com_rise_m / stroke
    acceleration = force / mass_kg - GRAVITY_M_PER_S2
    displacement = np.linspace(0.0, stroke, resolution)
    lengths = start_length_m + displacement
    leg_speed = np.sqrt(2.0 * acceleration * displacement)
    duration = float(np.sqrt(2.0 * stroke / acceleration))
    return AxialEvent(
        kind="jump",
        mass_kg=mass_kg,
        start_length_m=start_length_m,
        end_length_m=end_length_m,
        external_height_m=target_com_rise_m,
        total_axial_force_n=force,
        actuator_work_j=force * stroke,
        duration_s=duration,
        values=_event_values(
            lengths, displacement, leg_speed, force, parameters
        ),
    )


def constant_force_landing(
    mass_kg: float,
    drop_height_m: float,
    touchdown_length_m: float = 0.120,
    bottom_length_m: float = 0.070,
    *,
    resolution: int = 501,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> AxialEvent:
    """Screen an ideal vertical drop stopped by constant force over one stroke."""
    stroke = touchdown_length_m - bottom_length_m
    if mass_kg <= 0.0 or drop_height_m <= 0.0:
        raise ValueError("landing mass and drop height must be positive")
    if stroke <= 0.0 or resolution < 2:
        raise ValueError("landing stroke and resolution must be positive")

    force = mass_kg * GRAVITY_M_PER_S2 * (1.0 + drop_height_m / stroke)
    deceleration = force / mass_kg - GRAVITY_M_PER_S2
    displacement = np.linspace(0.0, stroke, resolution)
    lengths = touchdown_length_m - displacement
    impact_speed = np.sqrt(2.0 * GRAVITY_M_PER_S2 * drop_height_m)
    leg_speed = np.sqrt(
        np.maximum(0.0, impact_speed**2 - 2.0 * deceleration * displacement)
    )
    duration = float(2.0 * stroke / impact_speed)
    return AxialEvent(
        kind="landing",
        mass_kg=mass_kg,
        start_length_m=touchdown_length_m,
        end_length_m=bottom_length_m,
        external_height_m=drop_height_m,
        total_axial_force_n=force,
        actuator_work_j=force * stroke,
        duration_s=duration,
        values=_event_values(
            lengths, displacement, leg_speed, force, parameters
        ),
    )


def approximate_edulite05_envelope_nm(
    joint_rpm: FloatArray,
    bus_voltage_v: float = 48.0,
) -> FloatArray:
    """Approximate the manual's 48 V T-N plot; voltage scaling is a hypothesis."""
    if bus_voltage_v <= 0.0:
        raise ValueError("bus voltage must be positive")
    source_rpm = np.array([0.0, 100.0, 200.0, 250.0, 300.0, 350.0, 400.0, 430.0])
    source_torque = np.array([6.0, 5.6, 5.2, 4.7, 3.2, 2.3, 1.5, 1.0])
    scaled_rpm = source_rpm * bus_voltage_v / 48.0
    return np.interp(joint_rpm, scaled_rpm, source_torque, left=6.0, right=0.0)


def approximate_motor_utilization(
    event: AxialEvent, bus_voltage_v: float = 48.0
) -> float:
    if np.max(event.column("joint_rpm")) > 430.0 * bus_voltage_v / 48.0:
        return float("inf")
    available = approximate_edulite05_envelope_nm(
        event.column("joint_rpm"), bus_voltage_v
    )
    required = event.column("joint_torque")
    utilization = np.divide(
        required,
        available,
        out=np.full_like(required, np.inf),
        where=available > 0.0,
    )
    return float(np.max(utilization))
