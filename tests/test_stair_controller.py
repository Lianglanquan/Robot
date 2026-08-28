from pathlib import Path

import numpy as np

import mujoco
from src.mujoco_dynamics import initialize_dynamic_state, set_step_height
from src.stair_controller import StairController, StairControllerConfig, StairPhase

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def dynamic_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(
        str(PROJECT_ROOT / "mujoco" / "robot_dynamic_2p5kg.xml")
    )


def test_stair_controller_starts_in_approach_and_validates_height() -> None:
    model = dynamic_model()
    controller = StairController(model)

    assert controller.phase is StairPhase.APPROACH
    assert not controller.terminal

    try:
        StairController(model, StairControllerConfig(step_height_m=0.075))
    except ValueError as error:
        assert "supported stair heights" in str(error)
    else:
        raise AssertionError("unsupported stair height was accepted")


def test_five_centimetre_state_machine_is_finite_and_reports_result() -> None:
    model = dynamic_model()
    set_step_height(model, 0.05)
    data = mujoco.MjData(model)
    initialize_dynamic_state(model, data, l0_mm=90.0)
    controller = StairController(model)
    phases: list[StairPhase] = []

    for _ in range(round(8.0 / model.opt.timestep)):
        telemetry = controller.step(data)
        mujoco.mj_step(model, data)
        phases.append(telemetry.phase)
        assert np.all(np.isfinite(data.qpos))
        assert np.all(np.isfinite(data.qvel))
        assert telemetry.max_hip_torque_nm <= 6.0 + 1e-12
        assert telemetry.max_wheel_torque_nm <= 0.3 + 1e-12
        if controller.terminal:
            break

    assert controller.terminal
    assert StairPhase.APPROACH in phases
    assert StairPhase.CROUCH in phases
    assert StairPhase.PUSH in phases
    assert controller.phase in (StairPhase.SUCCESS, StairPhase.FAILED)
    if controller.phase is StairPhase.FAILED:
        assert controller.failure_reason
