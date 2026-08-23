import numpy as np
import pytest

from src.kinematics import forward_kinematics
from src.parameters import DEFAULT_PARAMETERS
from src.stroke import (
    STROKE_COLUMNS,
    inverse_kinematics_candidates,
    normal_vertical_posture,
    scan_normal_vertical_stroke,
    vertical_stroke_limits,
)


def test_vertical_stroke_limits_are_the_two_serial_singularities() -> None:
    lower, upper = vertical_stroke_limits()
    p = DEFAULT_PARAMETERS

    assert lower == pytest.approx(
        np.sqrt((p.l2 - p.l1) ** 2 - (p.l5 / 2.0) ** 2)
    )
    assert upper == pytest.approx(
        np.sqrt((p.l2 + p.l1) ** 2 - (p.l5 / 2.0) ** 2)
    )
    assert upper - lower == pytest.approx(0.10597134097099105)


def test_inverse_kinematics_exposes_multiple_modes_for_one_foot_pose() -> None:
    candidates = inverse_kinematics_candidates(l0=0.070, phi0=np.pi / 2.0)
    normal = next(
        candidate
        for candidate in candidates
        if candidate.left_sign == 1 and candidate.right_sign == -1
    )

    assert len(candidates) == 4
    assert sum(candidate.matches_positive_root for candidate in candidates) == 3
    assert normal.matches_positive_root
    assert normal.b[0] < 0.0
    assert normal.d[0] > DEFAULT_PARAMETERS.l5
    assert normal.phi1 + normal.phi4 == pytest.approx(np.pi, abs=1e-14)


@pytest.mark.parametrize("l0", [0.050, 0.070, 0.090, 0.120, 0.140, 0.150])
def test_normal_vertical_branch_matches_verified_forward_model(l0: float) -> None:
    posture = normal_vertical_posture(l0)
    pose = forward_kinematics(posture.phi1, posture.phi4)

    assert pose.xc == pytest.approx(DEFAULT_PARAMETERS.l5 / 2.0, abs=2e-14)
    assert pose.yc == pytest.approx(l0, abs=2e-14)
    assert pose.l0 == pytest.approx(l0, abs=2e-14)
    assert pose.phi0 == pytest.approx(np.pi / 2.0, abs=2e-14)
    assert posture.phi1 + posture.phi4 == pytest.approx(np.pi, abs=2e-14)


def test_path_derivative_matches_inverse_kinematics_finite_difference() -> None:
    l0 = 0.100
    step = 1e-7
    posture = normal_vertical_posture(l0)
    plus = normal_vertical_posture(l0 + step)
    minus = normal_vertical_posture(l0 - step)
    finite_difference = np.array(
        [plus.phi1 - minus.phi1, plus.phi4 - minus.phi4]
    ) / (2.0 * step)

    np.testing.assert_allclose(posture.dq_dl0, finite_difference, rtol=2e-9)
    np.testing.assert_allclose(
        posture.metrics.jacobian @ posture.dq_dl0,
        [1.0, 0.0],
        atol=2e-14,
    )


@pytest.mark.parametrize("l0", [0.050, 0.070, 0.100, 0.120, 0.150])
def test_normalized_force_and_path_speed_are_reciprocal(l0: float) -> None:
    posture = normal_vertical_posture(l0)

    assert posture.path_extension_speed == pytest.approx(
        posture.metrics.max_extension_speed, rel=2e-14
    )
    assert posture.force_speed_product == pytest.approx(2.0, rel=2e-14)


def test_normal_stroke_scan_is_continuous_and_finite() -> None:
    scan = scan_normal_vertical_stroke(resolution=101)

    assert scan.values.shape == (101, len(STROKE_COLUMNS))
    assert np.all(np.isfinite(scan.values))
    assert np.all(np.diff(scan.column("l0")) > 0.0)
    np.testing.assert_allclose(
        scan.column("phi1") + scan.column("phi4"), np.pi, atol=2e-14
    )
    np.testing.assert_allclose(
        scan.column("dq1_dl0") + scan.column("dq4_dl0"), 0.0, atol=1e-10
    )
    assert scan.column("serial_sine_min")[0] < 0.01
    assert scan.column("serial_sine_min")[-1] < 0.01
    assert np.max(scan.column("serial_sine_min")) > 0.99
