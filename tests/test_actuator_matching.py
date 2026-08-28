import math

import numpy as np
import pytest

from src.actuator import EL05_PUBLIC_ENVELOPE
from src.actuator_matching import extension_operating_point, static_support_match


def test_public_peak_curve_is_symmetric_and_monotonic() -> None:
    actuator = EL05_PUBLIC_ENVELOPE
    speeds_rpm = np.linspace(0.0, actuator.no_load_speed_rpm, 101)
    torques = np.asarray(
        [
            actuator.peak_torque_at_speed(speed * 2.0 * np.pi / 60.0)
            for speed in speeds_rpm
        ]
    )

    assert torques[0] == pytest.approx(actuator.peak_torque_nm)
    assert torques[-1] == pytest.approx(0.0)
    assert np.all(np.diff(torques) <= 1e-12)
    assert actuator.peak_torque_at_speed(10.0) == pytest.approx(
        actuator.peak_torque_at_speed(-10.0)
    )


def test_overload_duration_uses_conservative_table_steps() -> None:
    actuator = EL05_PUBLIC_ENVELOPE

    assert math.isinf(actuator.conservative_overload_time_s(1.1, stalled=True))
    assert actuator.conservative_overload_time_s(1.2, stalled=True) == 175.0
    assert actuator.conservative_overload_time_s(3.5, stalled=True) == 6.0
    assert actuator.conservative_overload_time_s(5.5, stalled=False) == 5.0
    assert actuator.conservative_overload_time_s(6.1, stalled=False) == 0.0


def test_static_support_uses_two_equal_vertical_legs() -> None:
    match = static_support_match(mass_kg=2.5, l0_m=0.100)

    assert match.axial_force_per_leg_n == pytest.approx(2.5 * 9.80665 / 2.0)
    assert match.joint_torque_required_nm < 0.5
    assert match.stall_continuous_margin > 2.0


def test_extension_point_preserves_four_actuator_mechanical_power() -> None:
    point = extension_operating_point(l0_m=0.100, extension_speed_m_per_s=0.5)

    assert point.available_joint_torque_nm > 5.0
    assert point.total_axial_force_n > 300.0
    assert point.total_mechanical_power_w == pytest.approx(
        4.0 * point.available_joint_torque_nm * point.joint_speed_rad_per_s
    )


def test_available_axial_force_falls_at_high_extension_speed() -> None:
    slow = extension_operating_point(0.100, 0.25)
    fast = extension_operating_point(0.100, 2.0)

    assert fast.joint_speed_rpm > slow.joint_speed_rpm
    assert fast.available_joint_torque_nm < slow.available_joint_torque_nm
    assert fast.total_axial_force_n < slow.total_axial_force_n
