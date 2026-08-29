#!/usr/bin/env python3
"""Test symmetric wheel-contact stair actions without lateral asymmetry."""

import json
import sys
from pathlib import Path
from typing import cast

import numpy as np

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mujoco_dynamics import (  # noqa: E402
    ControllerGains,
    apply_standing_controller,
    chassis_pitch,
    initialize_dynamic_state,
    set_wheel_torque_limit,
)
from src.stair_controller import _wheel_top_flags  # noqa: E402


def run_case(initial_l0_mm: float, extension_l0_mm: float) -> dict[str, object]:
    model = mujoco.MjModel.from_xml_path(
        str(PROJECT_ROOT / "mujoco" / "robot_dynamic_2p5kg_step_5cm.xml")
    )
    set_wheel_torque_limit(model, 1.0)
    data = mujoco.MjData(model)
    initialize_dynamic_state(model, data, l0_mm=initial_l0_mm)
    gains = ControllerGains(wheel_torque_limit_nm=1.0)
    step_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "step")
    contacted = False
    max_pitch = 0.0
    max_vertical_speed = 0.0
    top_seen = False
    contact_state: dict[str, float] | None = None

    for _ in range(2500):
        target_l0 = extension_l0_mm if contacted else initial_l0_mm
        apply_standing_controller(
            model, data, target_l0_mm=target_l0, target_y_m=0.0, gains=gains
        )
        data.ctrl[4] = 0.20 if not contacted else 0.02
        data.ctrl[5] = data.ctrl[4]
        mujoco.mj_step(model, data)

        pitch_deg = abs(float(np.rad2deg(chassis_pitch(model, data))))
        max_pitch = max(max_pitch, pitch_deg)
        max_vertical_speed = max(max_vertical_speed, abs(float(data.qvel[2])))
        step_contact = any(
            step_id in (contact.geom1, contact.geom2)
            for contact in data.contact[: data.ncon]
        )
        if step_contact and not contacted:
            contacted = True
            contact_state = {
                "time_s": float(data.time),
                "chassis_y_m": float(data.qpos[1]),
                "chassis_z_m": float(data.qpos[2]),
                "pitch_deg": pitch_deg,
            }
        top_seen |= _wheel_top_flags(model, data, 0.05)
        if top_seen or max_pitch > 90.0:
            break

    return {
        "initial_l0_mm": initial_l0_mm,
        "extension_l0_mm": extension_l0_mm,
        "symmetric_commands": True,
        "contact_observed": contacted,
        "contact_state": contact_state,
        "top_contact_observed": top_seen,
        "max_abs_pitch_deg": max_pitch,
        "max_abs_vertical_speed_m_s": max_vertical_speed,
        "terminal_chassis_y_m": float(data.qpos[1]),
        "terminal_chassis_z_m": float(data.qpos[2]),
        "safe_symmetric_success": bool(
            top_seen and max_pitch <= 75.0 and max_vertical_speed <= 1.2
        ),
    }


def main() -> int:
    cases = [
        run_case(initial_l0, extension_l0)
        for initial_l0 in (70.0, 80.0, 90.0)
        for extension_l0 in (100.0, 120.0)
    ]
    output = (
        PROJECT_ROOT
        / "artifacts"
        / "stair_controller"
        / "symmetric_action_validation_2p5kg_5cm.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(cases, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"evaluated {len(cases)} symmetric cases")
    success_count = sum(
        cast(bool, case["safe_symmetric_success"]) for case in cases
    )
    print(f"safe symmetric successes: {success_count}")
    print(f"wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
