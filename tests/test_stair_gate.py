import numpy as np
import pytest

from src.stair_gate import (
    GRAVITY_M_PER_S2,
    approximate_motor_utilization,
    constant_force_jump,
    constant_force_landing,
    static_edge_torque_per_wheel_nm,
    wheel_clearance_com_rises,
)


def test_wheel_clearance_benchmarks_keep_strategy_assumptions_separate() -> None:
    rises = wheel_clearance_com_rises(0.150, 0.026, 0.050, 0.9)

    assert rises["full_clear_no_tuck"] == pytest.approx(0.150)
    assert rises["full_clear_with_tuck"] == pytest.approx(0.105)
    assert rises["edge_contact_no_tuck"] == pytest.approx(0.124)
    assert rises["edge_contact_with_tuck"] == pytest.approx(0.079)
    assert static_edge_torque_per_wheel_nm(2.5, 0.026) == pytest.approx(
        2.5 * GRAVITY_M_PER_S2 * 0.026 / 2.0
    )


def test_constant_force_jump_closes_energy_and_takeoff_height() -> None:
    event = constant_force_jump(2.5, 0.150, resolution=101)
    stroke = event.end_length_m - event.start_length_m
    takeoff_speed = event.column("leg_speed")[-1]

    assert event.total_axial_force_n == pytest.approx(
        2.5 * GRAVITY_M_PER_S2 * 0.150 / stroke
    )
    assert event.actuator_work_j == pytest.approx(
        2.5 * GRAVITY_M_PER_S2 * 0.150
    )
    assert stroke + takeoff_speed**2 / (2.0 * GRAVITY_M_PER_S2) == pytest.approx(
        0.150
    )
    assert event.duration_s < 0.1


def test_constant_force_landing_closes_drop_and_compression_energy() -> None:
    event = constant_force_landing(2.5, 0.150, resolution=101)
    stroke = event.start_length_m - event.end_length_m

    assert event.total_axial_force_n == pytest.approx(
        2.5 * GRAVITY_M_PER_S2 * (1.0 + 0.150 / stroke)
    )
    assert event.actuator_work_j == pytest.approx(
        2.5 * GRAVITY_M_PER_S2 * (0.150 + stroke)
    )
    assert event.column("leg_speed")[0] == pytest.approx(
        np.sqrt(2.0 * GRAVITY_M_PER_S2 * 0.150)
    )
    assert event.column("leg_speed")[-1] == pytest.approx(0.0, abs=3e-8)


def test_landing_stroke_is_a_decision_critical_variable() -> None:
    short = constant_force_landing(
        2.5, 0.150, touchdown_length_m=0.090, bottom_length_m=0.070
    )
    long = constant_force_landing(
        2.5, 0.150, touchdown_length_m=0.120, bottom_length_m=0.070
    )

    assert short.max_joint_torque_nm > 2.0 * long.max_joint_torque_nm
    assert short.max_total_joint_power_w > 2.0 * long.max_total_joint_power_w
    assert approximate_motor_utilization(short) > approximate_motor_utilization(long)
    assert np.isinf(approximate_motor_utilization(long, bus_voltage_v=24.0))


def test_demands_scale_with_mass_but_speeds_do_not() -> None:
    light = constant_force_jump(2.0, 0.105)
    heavy = constant_force_jump(2.5, 0.105)

    assert heavy.max_joint_torque_nm / light.max_joint_torque_nm == pytest.approx(
        1.25
    )
    assert heavy.actuator_work_j / light.actuator_work_j == pytest.approx(1.25)
    np.testing.assert_allclose(
        heavy.column("joint_speed"), light.column("joint_speed"), atol=1e-14
    )
