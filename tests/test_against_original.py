from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest

from src.kinematics import analytic_jacobian, forward_kinematics
from src.vmc import joint_torques, leg_velocity
from tests.reference_c import CReference, build_reference_library


OUTPUT_NAMES = ("l0", "phi0", "dL", "dPhi", "T1", "T2")
PASS_THRESHOLDS = np.array([2e-7, 5e-6, 1e-4, 3e-3, 3e-3, 2e-3])
NOTICEABLE_THRESHOLDS = np.array([5e-8, 1e-6, 1e-5, 1e-4, 1e-4, 1e-4])


@dataclass(frozen=True)
class ValidationCase:
    phi1: float
    phi4: float
    dphi1: float
    dphi4: float
    force: float
    virtual_torque: float


def as_float32(value: float) -> float:
    return float(np.float32(value))


def closure_discriminant(phi1: float, phi4: float) -> float:
    xb = 0.050 * np.cos(phi1)
    yb = 0.050 * np.sin(phi1)
    xd = 0.060 + 0.050 * np.cos(phi4)
    yd = 0.050 * np.sin(phi4)
    a0 = 2.0 * 0.105 * (xd - xb)
    b0 = 2.0 * 0.105 * (yd - yb)
    c0 = (xd - xb) ** 2 + (yd - yb) ** 2
    return float(a0**2 + b0**2 - c0**2)


def validation_cases(random_count: int = 400) -> list[ValidationCase]:
    fixed_angles = [
        (-2.5, -0.6),
        (-2.2, -0.9),
        (-1.5, -1.5),
        (1.0864529609680176, -3.035454750061035),
        (2.2, 0.9),
        (2.5, 0.6),
    ]
    cases = [
        ValidationCase(
            as_float32(phi1),
            as_float32(phi4),
            as_float32(1.25 - index * 0.3),
            as_float32(-0.75 + index * 0.2),
            as_float32(12.0 + index * 7.0),
            as_float32(-0.5 + index * 0.2),
        )
        for index, (phi1, phi4) in enumerate(fixed_angles)
    ]

    rng = np.random.default_rng(616)
    while len(cases) < random_count + len(fixed_angles):
        phi1, phi4 = (as_float32(value) for value in rng.uniform(-np.pi, np.pi, 2))
        if closure_discriminant(phi1, phi4) < 2e-5:
            continue
        jacobian = analytic_jacobian(phi1, phi4)
        pose = forward_kinematics(phi1, phi4)
        if not np.all(np.isfinite(jacobian)) or np.max(np.abs(jacobian)) > 10.0:
            continue
        if not np.isfinite(pose.l0) or pose.l0 < 0.02:
            continue
        cases.append(
            ValidationCase(
                phi1,
                phi4,
                as_float32(rng.uniform(-8.0, 8.0)),
                as_float32(rng.uniform(-8.0, 8.0)),
                as_float32(rng.uniform(-80.0, 80.0)),
                as_float32(rng.uniform(-3.0, 3.0)),
            )
        )
    return cases


@pytest.fixture(scope="session")
def c_reference(tmp_path_factory: pytest.TempPathFactory) -> CReference:
    return build_reference_library(Path(tmp_path_factory.mktemp("reference-c")))


def error_matrix(c_reference: CReference) -> np.ndarray:
    rows = []
    for case in validation_cases():
        pose = forward_kinematics(case.phi1, case.phi4)
        speed = leg_velocity(case.phi1, case.phi4, case.dphi1, case.dphi4)
        torque = joint_torques(
            case.force, case.virtual_torque, case.phi1, case.phi4
        )

        python_output = np.array(
            [pose.l0, pose.phi0, speed.dl0, speed.dphi0, torque.t1, torque.t2]
        )
        c_output = np.concatenate(
            [
                c_reference.leg_pos(case.phi1, case.phi4),
                c_reference.leg_spd(
                    case.dphi1, case.dphi4, case.phi1, case.phi4
                ),
                c_reference.leg_conv(
                    case.force, case.virtual_torque, case.phi1, case.phi4
                ),
            ]
        )
        rows.append(np.abs(python_output - c_output))
    return np.array(rows)


def test_python_matches_original_c_with_float32_reference_tolerances(
    c_reference: CReference,
) -> None:
    errors = error_matrix(c_reference)

    assert np.all(np.isfinite(errors))
    np.testing.assert_array_less(np.max(errors, axis=0), PASS_THRESHOLDS)


def test_original_speed_and_torque_functions_represent_the_same_jacobian(
    c_reference: CReference,
) -> None:
    maximum_error = 0.0
    for case in validation_cases():
        speed_jacobian = np.column_stack(
            [
                c_reference.leg_spd(1.0, 0.0, case.phi1, case.phi4),
                c_reference.leg_spd(0.0, 1.0, case.phi1, case.phi4),
            ]
        )
        torque_jacobian = np.vstack(
            [
                c_reference.leg_conv(1.0, 0.0, case.phi1, case.phi4),
                c_reference.leg_conv(0.0, 1.0, case.phi1, case.phi4),
            ]
        )
        maximum_error = max(
            maximum_error, float(np.max(np.abs(speed_jacobian - torque_jacobian)))
        )

    assert maximum_error < 3e-4


def test_fixed_cases_cover_both_assembly_orientations(
    c_reference: CReference,
) -> None:
    poses = [
        c_reference.leg_pos(case.phi1, case.phi4)
        for case in validation_cases(random_count=0)
    ]

    assert any(pose[1] < 0.0 for pose in poses)
    assert any(pose[1] > 0.0 for pose in poses)
