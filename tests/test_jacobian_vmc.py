import numpy as np
import pytest

from src.kinematics import analytic_jacobian, finite_difference_jacobian
from src.vmc import joint_torques, leg_velocity


@pytest.mark.parametrize(
    ("phi1", "phi4"),
    [(-2.5, -0.6), (-2.2, -0.9), (-1.5, -1.5), (1.5, 1.5), (2.5, 0.6)],
)
def test_sympy_jacobian_matches_central_finite_difference(
    phi1: float, phi4: float
) -> None:
    analytic = analytic_jacobian(phi1, phi4)
    numeric = finite_difference_jacobian(phi1, phi4, step=1e-7)

    assert analytic.shape == (2, 2)
    np.testing.assert_allclose(analytic, numeric, rtol=2e-6, atol=2e-8)


def test_leg_velocity_is_jacobian_times_joint_velocity() -> None:
    phi1, phi4 = -2.2, -0.9
    joint_velocity = np.array([1.3, -0.7])

    result = leg_velocity(phi1, phi4, *joint_velocity)
    expected = analytic_jacobian(phi1, phi4) @ joint_velocity

    np.testing.assert_allclose([result.dl0, result.dphi0], expected, atol=1e-15)


def test_joint_torque_is_transposed_jacobian_times_virtual_wrench() -> None:
    phi1, phi4 = -2.2, -0.9
    wrench = np.array([18.0, -0.35])

    result = joint_torques(wrench[0], wrench[1], phi1, phi4)
    expected = analytic_jacobian(phi1, phi4).T @ wrench

    np.testing.assert_allclose([result.t1, result.t2], expected, atol=1e-15)


def test_velocity_and_torque_mappings_preserve_virtual_work() -> None:
    phi1, phi4 = 2.2, 0.9
    joint_velocity = np.array([-0.8, 1.1])
    wrench = np.array([24.0, 0.42])

    velocity = leg_velocity(phi1, phi4, *joint_velocity)
    torque = joint_torques(wrench[0], wrench[1], phi1, phi4)

    joint_power = np.dot([torque.t1, torque.t2], joint_velocity)
    virtual_power = np.dot(wrench, [velocity.dl0, velocity.dphi0])
    assert joint_power == pytest.approx(virtual_power, abs=1e-14)
