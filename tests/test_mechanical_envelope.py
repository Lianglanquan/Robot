import numpy as np
import pytest

from src.mechanical_envelope import CAD_POSE_COLUMNS, cad_pose, cad_pose_schedule
from src.parameters import DEFAULT_PARAMETERS


def test_cad_schedule_covers_baseline_and_both_mathematical_end_regions() -> None:
    poses = cad_pose_schedule()

    assert len(poses) == 106
    assert poses[0].l0_mm == 47.0
    assert poses[-1].l0_mm == 152.0
    assert sum(pose.zone == "baseline_70_120" for pose in poses) == 51
    assert len(poses[0].row()) == len(CAD_POSE_COLUMNS)


@pytest.mark.parametrize("l0_mm", [47, 70, 90, 100, 120, 152])
def test_cad_pose_preserves_phase3_geometry(l0_mm: int) -> None:
    pose = cad_pose(l0_mm)

    assert pose.phi1_deg + pose.phi4_deg == pytest.approx(180.0, abs=1e-12)
    assert pose.a_mm == (0.0, 0.0)
    assert pose.e_mm == (60.0, 0.0)
    assert pose.c_mm == pytest.approx((30.0, float(l0_mm)), abs=1e-11)
    assert np.linalg.norm(np.subtract(pose.b_mm, pose.a_mm)) == pytest.approx(
        DEFAULT_PARAMETERS.l1 * 1000.0
    )
    assert np.linalg.norm(np.subtract(pose.c_mm, pose.b_mm)) == pytest.approx(
        DEFAULT_PARAMETERS.l2 * 1000.0
    )
    assert np.linalg.norm(np.subtract(pose.c_mm, pose.d_mm)) == pytest.approx(
        DEFAULT_PARAMETERS.l3 * 1000.0
    )
    assert np.linalg.norm(np.subtract(pose.d_mm, pose.e_mm)) == pytest.approx(
        DEFAULT_PARAMETERS.l4 * 1000.0
    )


def test_cad_pose_rejects_singular_or_outside_lengths() -> None:
    with pytest.raises(ValueError):
        cad_pose(46)
    with pytest.raises(ValueError):
        cad_pose(153)
