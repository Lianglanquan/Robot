#!/usr/bin/env python3
"""Calibrate wheel/step contact geometry and approach torque in MuJoCo."""

import argparse
import json
import sys
from pathlib import Path

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


def step_contact(model: mujoco.MjModel, data: mujoco.MjData) -> bool:
    step_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "step")
    wheel_ids = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_wheel_collision")
        for side in ("left", "right")
    }
    return any(
        step_id in (contact.geom1, contact.geom2)
        and bool({contact.geom1, contact.geom2} & wheel_ids)
        for contact in data.contact[: data.ncon]
    )


def run_case(torque_nm: float, duration_s: float) -> dict[str, object]:
    model = mujoco.MjModel.from_xml_path(
        str(PROJECT_ROOT / "mujoco" / "robot_dynamic_2p5kg_step_5cm.xml")
    )
    set_wheel_torque_limit(model, 1.0)
    data = mujoco.MjData(model)
    initialize_dynamic_state(model, data, l0_mm=90.0)
    gains = ControllerGains(wheel_torque_limit_nm=1.0)
    wheel_site = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SITE, "left_wheel_center"
    )
    first_contact: dict[str, float] | None = None
    max_pitch = 0.0
    max_speed = 0.0
    for _ in range(round(duration_s / model.opt.timestep)):
        apply_standing_controller(
            model, data, target_l0_mm=90.0, target_y_m=0.0, gains=gains
        )
        data.ctrl[4] = torque_nm
        data.ctrl[5] = torque_nm
        mujoco.mj_step(model, data)
        pitch_deg = float(np.rad2deg(chassis_pitch(model, data)))
        max_pitch = max(max_pitch, abs(pitch_deg))
        max_speed = max(max_speed, abs(float(data.qvel[2])))
        if first_contact is None and step_contact(model, data):
            first_contact = {
                "time_s": float(data.time),
                "chassis_y_m": float(data.qpos[1]),
                "chassis_z_m": float(data.qpos[2]),
                "wheel_center_y_m": float(data.site_xpos[wheel_site, 1]),
                "wheel_center_z_m": float(data.site_xpos[wheel_site, 2]),
                "pitch_deg": pitch_deg,
                "vertical_speed_m_s": float(data.qvel[2]),
            }
            break
    return {
        "torque_nm": torque_nm,
        "duration_s": duration_s,
        "contact_observed": first_contact is not None,
        "first_contact": first_contact,
        "max_abs_pitch_deg_before_contact": max_pitch,
        "max_abs_vertical_speed_m_s_before_contact": max_speed,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument(
        "--torques", type=float, nargs="+", default=[0.02, 0.05, 0.10, 0.15, 0.20, 0.30]
    )
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args(argv)
    if args.duration <= 0 or any(torque <= 0 for torque in args.torques):
        parser.error("duration and torques must be positive")

    results = [run_case(float(torque), args.duration) for torque in args.torques]
    output = args.output or (
        PROJECT_ROOT
        / "artifacts"
        / "stair_controller"
        / "contact_calibration_2p5kg_5cm.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(results, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    for result in results:
        contact = result["first_contact"]
        if isinstance(contact, dict):
            print(
                f"{result['torque_nm']:.2f} N·m: contact at "
                f"chassis y={contact['chassis_y_m']:.4f} m, "
                f"pitch={contact['pitch_deg']:.1f} deg"
            )
        else:
            print(f"{result['torque_nm']:.2f} N·m: no step contact")
    print(f"wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

