from pathlib import Path

import numpy as np
import pytest

import mujoco
from src.mujoco_robot import MODEL_ROOT_Z_M, joint_configuration, set_leg_length

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = PROJECT_ROOT / "mujoco" / "robot.xml"


@pytest.fixture(scope="module")
def model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(MODEL_PATH))


def test_model_contains_real_closed_loop_structure(model: mujoco.MjModel) -> None:
    assert model.nmesh == 22
    assert model.njnt == 10
    assert model.neq == 2
    assert model.nsite == 4


@pytest.mark.parametrize("l0_mm", [70.0, 90.0, 120.0])
def test_mujoco_pose_matches_phase3_and_closes(
    model: mujoco.MjModel, l0_mm: float
) -> None:
    data = mujoco.MjData(model)
    closure_error_mm = set_leg_length(model, data, l0_mm)
    configuration = joint_configuration(l0_mm)

    assert closure_error_mm < 1e-6
    for side in ("left", "right"):
        wheel_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_wheel_center"
        )
        np.testing.assert_allclose(
            data.site_xpos[wheel_id], configuration.expected_axle_m, atol=2e-11
        )


def test_leg_length_changes_axle_height_by_full_stroke(
    model: mujoco.MjModel,
) -> None:
    data = mujoco.MjData(model)
    set_leg_length(model, data, 70.0)
    short_height = data.site("left_wheel_center").xpos[2]
    set_leg_length(model, data, 120.0)
    long_height = data.site("left_wheel_center").xpos[2]

    assert short_height == pytest.approx(MODEL_ROOT_Z_M - 0.036)
    assert long_height == pytest.approx(MODEL_ROOT_Z_M - 0.086)
    assert short_height - long_height == pytest.approx(0.050)
