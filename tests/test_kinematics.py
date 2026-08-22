import math

import numpy as np
import pytest

from src.kinematics import forward_kinematics, joint_positions
from src.parameters import DEFAULT_PARAMETERS, FiveBarParameters

LEGAL_ANGLES = [
    (-2.5, -0.6),
    (-2.2, -0.9),
    (-1.5, -1.5),
    (1.5, 1.5),
    (2.2, 0.9),
    (2.5, 0.6),
]


def test_upstream_default_parameters() -> None:
    assert DEFAULT_PARAMETERS == FiveBarParameters(
        l1=0.050,
        l2=0.105,
        l3=0.105,
        l4=0.050,
        l5=0.060,
    )


@pytest.mark.parametrize(("phi1", "phi4"), LEGAL_ANGLES)
def test_joint_positions_close_all_four_moving_links(
    phi1: float, phi4: float
) -> None:
    points = joint_positions(phi1, phi4)
    p = DEFAULT_PARAMETERS

    np.testing.assert_allclose(np.linalg.norm(points.b - points.a), p.l1, atol=1e-14)
    np.testing.assert_allclose(np.linalg.norm(points.c - points.b), p.l2, atol=1e-14)
    np.testing.assert_allclose(np.linalg.norm(points.c - points.d), p.l3, atol=1e-14)
    np.testing.assert_allclose(np.linalg.norm(points.d - points.e), p.l4, atol=1e-14)
    np.testing.assert_allclose(points.e - points.a, [p.l5, 0.0], atol=1e-14)


@pytest.mark.parametrize(("phi1", "phi4"), LEGAL_ANGLES)
def test_forward_pose_is_measured_from_fixed_pivot_midpoint(
    phi1: float, phi4: float
) -> None:
    points = joint_positions(phi1, phi4)
    pose = forward_kinematics(phi1, phi4)
    virtual_vector = points.c - np.array([DEFAULT_PARAMETERS.l5 / 2.0, 0.0])

    assert pose.xc == pytest.approx(points.c[0], abs=1e-15)
    assert pose.yc == pytest.approx(points.c[1], abs=1e-15)
    assert pose.l0 == pytest.approx(np.linalg.norm(virtual_vector), abs=1e-15)
    assert pose.phi0 == pytest.approx(
        math.atan2(virtual_vector[1], virtual_vector[0]), abs=1e-15
    )
