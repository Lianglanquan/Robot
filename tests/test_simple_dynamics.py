import numpy as np
import pytest

from src.actuator_matching import STANDARD_GRAVITY_M_PER_S2
from src.simple_dynamics import (
    constant_acceleration_push,
    constant_deceleration_landing,
)


def test_push_reaches_requested_ballistic_speed_over_stroke() -> None:
    trajectory = constant_acceleration_push(2.5, 0.150)
    takeoff_speed = np.sqrt(2.0 * STANDARD_GRAVITY_M_PER_S2 * 0.150)

    assert trajectory.l0_m[0] == pytest.approx(0.070)
    assert trajectory.l0_m[-1] == pytest.approx(0.120)
    assert trajectory.body_speed_m_per_s[-1] == pytest.approx(takeoff_speed)
    assert trajectory.acceleration_m_per_s2 == pytest.approx(
        takeoff_speed**2 / 0.100
    )


def test_push_work_splits_into_gravity_and_takeoff_energy() -> None:
    trajectory = constant_acceleration_push(2.5, 0.150)
    takeoff_speed = trajectory.body_speed_m_per_s[-1]
    expected = (
        2.5 * STANDARD_GRAVITY_M_PER_S2 * trajectory.stroke_m
        + 0.5 * 2.5 * takeoff_speed**2
    )

    assert trajectory.ideal_mechanical_work_j == pytest.approx(expected)
    assert trajectory.total_mechanical_power_w[-1] == pytest.approx(
        trajectory.total_axial_force_n * takeoff_speed
    )


def test_landing_absorbs_impact_and_gravity_over_compression() -> None:
    trajectory = constant_deceleration_landing(2.5, 0.150)
    impact_speed = trajectory.body_speed_m_per_s[0]
    expected = (
        0.5 * 2.5 * impact_speed**2
        + 2.5 * STANDARD_GRAVITY_M_PER_S2 * trajectory.stroke_m
    )

    assert trajectory.l0_m[0] == pytest.approx(0.120)
    assert trajectory.l0_m[-1] == pytest.approx(0.070)
    assert trajectory.body_speed_m_per_s[-1] == pytest.approx(0.0, abs=1e-14)
    assert trajectory.ideal_mechanical_work_j == pytest.approx(expected)
    assert np.all(trajectory.total_mechanical_power_w <= 1e-14)


def test_equal_push_and_drop_heights_have_equal_force_reference() -> None:
    push = constant_acceleration_push(2.3, 0.100)
    landing = constant_deceleration_landing(2.3, 0.100)

    assert push.duration_s == pytest.approx(landing.duration_s)
    assert push.total_axial_force_n == pytest.approx(landing.total_axial_force_n)
    assert push.ideal_mechanical_work_j == pytest.approx(
        landing.ideal_mechanical_work_j
    )


def test_reference_cases_fit_public_magnitude_envelope() -> None:
    for mass in (2.0, 2.3, 2.5):
        for height in (0.050, 0.100, 0.150):
            push = constant_acceleration_push(mass, height)
            landing = constant_deceleration_landing(mass, height)
            assert push.within_public_magnitude_envelope
            assert landing.within_public_magnitude_envelope
            assert np.max(push.torque_utilization) < 0.5
            assert np.max(landing.torque_utilization) < 0.5


@pytest.mark.parametrize(
    ("mass", "height"),
    [(0.0, 0.1), (2.5, 0.0), (-1.0, 0.1), (2.5, -0.1)],
)
def test_invalid_reference_inputs_are_rejected(mass: float, height: float) -> None:
    with pytest.raises(ValueError):
        constant_acceleration_push(mass, height)
