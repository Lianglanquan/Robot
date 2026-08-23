import itertools

import numpy as np
import pytest

from src.analysis import (
    SCAN_COLUMNS,
    analyze_posture,
    classify_posture,
    scan_workspace,
    summarize_scan,
)
from src.kinematics import analytic_jacobian, forward_kinematics
from src.parameters import DEFAULT_PARAMETERS

REGULAR_POSTURES = [
    (-2.5, -0.6),
    (-2.2, -0.9),
    (-1.5, -1.5),
    (1.5, 1.5),
    (2.5, 0.6),
]


@pytest.mark.parametrize(("phi1", "phi4"), REGULAR_POSTURES)
def test_stable_posture_matches_verified_positive_root_branch(
    phi1: float, phi4: float
) -> None:
    metrics = analyze_posture(phi1, phi4)
    pose = forward_kinematics(phi1, phi4)
    p = DEFAULT_PARAMETERS

    assert metrics.xc == pytest.approx(pose.xc, abs=2e-14)
    assert metrics.yc == pytest.approx(pose.yc, abs=2e-14)
    assert metrics.l0 == pytest.approx(pose.l0, abs=2e-14)
    assert metrics.phi0 == pytest.approx(pose.phi0, abs=2e-14)
    assert np.linalg.norm(metrics.c - metrics.b) == pytest.approx(p.l2, abs=2e-14)
    assert np.linalg.norm(metrics.c - metrics.d) == pytest.approx(p.l3, abs=2e-14)


@pytest.mark.parametrize(("phi1", "phi4"), REGULAR_POSTURES)
def test_constraint_jacobian_matches_sympy_jacobian(
    phi1: float, phi4: float
) -> None:
    metrics = analyze_posture(phi1, phi4)

    np.testing.assert_allclose(
        metrics.jacobian,
        analytic_jacobian(phi1, phi4),
        rtol=2e-11,
        atol=2e-12,
    )
    np.testing.assert_allclose(
        metrics.physical_jacobian,
        np.diag([1.0, metrics.l0]) @ metrics.jacobian,
        atol=1e-14,
    )
    assert metrics.sigma_max >= metrics.sigma_min >= 0.0
    assert metrics.condition_number == pytest.approx(
        metrics.sigma_max / metrics.sigma_min
    )


def test_normalized_axial_force_saturates_a_joint_torque_limit() -> None:
    metrics = analyze_posture(-2.2, -0.9)
    torque = metrics.jacobian.T @ np.array([metrics.max_axial_force, 0.0])

    assert np.max(np.abs(torque)) == pytest.approx(1.0, abs=1e-14)
    assert np.max(np.abs(torque * 1.001)) > 1.0


def test_normalized_extension_speed_is_maximum_over_velocity_box() -> None:
    metrics = analyze_posture(-2.2, -0.9)
    corner_speeds = [
        abs((metrics.jacobian @ np.array(corner))[0])
        for corner in itertools.product((-1.0, 1.0), repeat=2)
    ]

    assert metrics.max_extension_speed == pytest.approx(max(corner_speeds))


@pytest.mark.parametrize(
    ("sigma_min", "condition_number", "expected"),
    [
        (0.010, 5.0, "recommended"),
        (0.009, 5.0, "usable"),
        (0.010, 5.1, "usable"),
        (0.002, 19.9, "usable"),
        (0.0019, 2.0, "near_singular"),
        (0.020, 20.0, "near_singular"),
        (0.020, float("inf"), "near_singular"),
    ],
)
def test_classification_boundaries(
    sigma_min: float, condition_number: float, expected: str
) -> None:
    assert classify_posture(sigma_min, condition_number) == expected


def test_workspace_scan_covers_unique_periodic_joint_grid() -> None:
    scan = scan_workspace(resolution=12)

    assert scan.resolution == 12
    assert scan.invalid_count == 0
    assert scan.values.shape == (144, len(SCAN_COLUMNS))
    assert len(np.unique(scan.values[:, :2], axis=0)) == 144
    assert np.min(scan.column("phi1")) == pytest.approx(-np.pi)
    assert np.max(scan.column("phi1")) < np.pi
    assert np.all(np.isfinite(scan.values))
    assert set(np.unique(scan.column("class_code"))) <= {0.0, 1.0, 2.0}


def test_workspace_scan_exposes_upright_band_and_summary() -> None:
    scan = scan_workspace(resolution=24)
    upright = scan.upright_mask(half_width_degrees=5.0)
    summary = summarize_scan(scan)

    assert upright.dtype == np.bool_
    assert 0 < np.count_nonzero(upright) < len(upright)
    assert summary["resolution"] == 24
    assert summary["total_samples"] == 24**2
    assert summary["valid_samples"] == len(scan.values)
    assert sum(summary["classification_counts"].values()) == len(scan.values)
    assert summary["l0_range_m"][0] < summary["l0_range_m"][1]
    assert summary["raw_sigma_min"]["min"] <= summary["raw_sigma_min"]["median"]
    assert (
        summary["raw_condition_number"]["median"]
        <= summary["raw_condition_number"]["max"]
    )
