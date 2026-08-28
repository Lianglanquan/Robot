from pathlib import Path

import numpy as np
import pytest

import mujoco
from src.mujoco_dynamics import (
    EL05_UNIT_MASS_KG,
    QD4310_UNIT_MASS_KG,
    apply_standing_controller,
    chassis_pitch,
    initialize_dynamic_state,
    leg_length_mm,
    mass_budget,
    set_step_height,
    whole_robot_mass_properties,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def dynamic_model(mass_kg: float = 2.5) -> mujoco.MjModel:
    label = str(mass_kg).replace(".", "p")
    path = PROJECT_ROOT / "mujoco" / f"robot_dynamic_{label}kg.xml"
    return mujoco.MjModel.from_xml_path(str(path))


@pytest.mark.parametrize("total_mass_kg", [2.0, 2.3, 2.5])
def test_mass_budget_is_complete(total_mass_kg: float) -> None:
    budget = mass_budget(total_mass_kg)

    assert budget.accounted_mass_kg == pytest.approx(total_mass_kg)
    assert budget.unresolved_fixed_mass_kg > 0.0
    assert EL05_UNIT_MASS_KG == pytest.approx(0.242)
    assert QD4310_UNIT_MASS_KG == pytest.approx(0.127)


@pytest.mark.parametrize("total_mass_kg", [2.0, 2.3, 2.5])
def test_dynamic_model_has_free_base_contacts_and_actuators(
    total_mass_kg: float,
) -> None:
    model = dynamic_model(total_mass_kg)

    assert model.body_mass.sum() == pytest.approx(total_mass_kg)
    assert model.nq == 17
    assert model.nv == 16
    assert model.nu == 6
    assert model.neq == 2
    assert model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    np.testing.assert_allclose(
        model.actuator_ctrlrange[:4], np.tile((-6.0, 6.0), (4, 1))
    )
    np.testing.assert_allclose(
        model.actuator_ctrlrange[4:], np.tile((-0.3, 0.3), (2, 1))
    )

    data = mujoco.MjData(model)
    initialize_dynamic_state(model, data, l0_mm=90.0)
    properties = whole_robot_mass_properties(model, data)
    assert properties.mass_kg == pytest.approx(total_mass_kg)
    assert abs(properties.com_m[0]) < 1e-12
    assert abs(properties.com_m[1]) < 1e-10
    assert np.all(np.linalg.eigvalsh(properties.inertia_at_com_kg_m2) > 0.0)


@pytest.mark.parametrize("l0_mm", [70.0, 90.0, 120.0])
def test_leg_length_hold_on_flat_ground(l0_mm: float) -> None:
    model = dynamic_model()
    data = mujoco.MjData(model)
    initialize_dynamic_state(model, data, l0_mm=l0_mm)

    for _ in range(3000):
        apply_standing_controller(model, data, target_l0_mm=l0_mm)
        mujoco.mj_step(model, data)

    assert data.ncon >= 2
    assert abs(leg_length_mm(model, data, "left") - l0_mm) < 0.8
    assert abs(leg_length_mm(model, data, "right") - l0_mm) < 0.8
    assert abs(np.rad2deg(chassis_pitch(model, data))) < 0.25


def test_balance_recovers_from_three_degree_pitch() -> None:
    model = dynamic_model()
    data = mujoco.MjData(model)
    initialize_dynamic_state(
        model,
        data,
        l0_mm=90.0,
        pitch_rad=np.deg2rad(3.0),
    )

    for _ in range(5000):
        apply_standing_controller(model, data, target_l0_mm=90.0)
        mujoco.mj_step(model, data)

    assert abs(np.rad2deg(chassis_pitch(model, data))) < 0.2
    assert abs(data.qpos[1]) < 0.01
    assert data.ncon >= 2


def test_five_centimetre_drop_returns_to_standing() -> None:
    model = dynamic_model()
    data = mujoco.MjData(model)
    initialize_dynamic_state(
        model,
        data,
        l0_mm=90.0,
        drop_height_m=0.05,
        pitch_rad=np.deg2rad(2.0),
    )

    made_contact = False
    for _ in range(4000):
        apply_standing_controller(model, data, target_l0_mm=90.0)
        mujoco.mj_step(model, data)
        made_contact |= data.ncon > 0

    assert made_contact
    assert np.all(np.isfinite(data.qpos))
    assert abs(np.rad2deg(chassis_pitch(model, data))) < 0.2
    assert abs(leg_length_mm(model, data, "left") - 90.0) < 0.8


@pytest.mark.parametrize("height_m", [0.0, 0.05, 0.10, 0.15])
def test_step_height_selection(height_m: float) -> None:
    model = dynamic_model()
    step_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "step")

    set_step_height(model, height_m)

    if height_m == 0.0:
        assert model.geom_contype[step_id] == 1
        assert model.geom_pos[step_id, 2] == pytest.approx(-1.0)
        assert model.geom_rgba[step_id, 3] == pytest.approx(0.0)
    else:
        assert model.geom_contype[step_id] == 1
        assert model.geom_size[step_id, 2] == pytest.approx(height_m / 2.0)
        assert model.geom_pos[step_id, 2] == pytest.approx(height_m / 2.0)


@pytest.mark.parametrize("height_m", [0.05, 0.10, 0.15])
def test_wheels_physically_contact_each_step(height_m: float) -> None:
    model = dynamic_model()
    set_step_height(model, height_m)
    data = mujoco.MjData(model)
    initialize_dynamic_state(model, data, l0_mm=90.0)
    step_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "step")
    touched_step = False

    for _ in range(1600):
        apply_standing_controller(
            model,
            data,
            target_l0_mm=90.0,
            target_y_m=-0.3,
        )
        mujoco.mj_step(model, data)
        touched_step |= any(
            contact.geom1 == step_id or contact.geom2 == step_id
            for contact in data.contact[: data.ncon]
        )

    assert touched_step
