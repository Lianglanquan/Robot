#!/usr/bin/env python3
"""Run the first stair-controller matrix across mass and stair height."""

import csv
import json
import sys
from pathlib import Path

import numpy as np

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mujoco_dynamics import initialize_dynamic_state, set_step_height  # noqa: E402
from src.stair_controller import StairController, StairControllerConfig  # noqa: E402


def run_case(mass_kg: float, height_m: float) -> dict[str, object]:
    label = str(mass_kg).replace(".", "p")
    model = mujoco.MjModel.from_xml_path(
        str(PROJECT_ROOT / "mujoco" / f"robot_dynamic_{label}kg.xml")
    )
    set_step_height(model, height_m)
    data = mujoco.MjData(model)
    initialize_dynamic_state(model, data, l0_mm=90.0)
    controller = StairController(
        model,
        StairControllerConfig(step_height_m=height_m),
    )
    phases: list[str] = []
    max_hip = 0.0
    max_wheel = 0.0
    step_contact = False
    for _ in range(round(8.0 / model.opt.timestep)):
        telemetry = controller.step(data)
        mujoco.mj_step(model, data)
        phases.append(telemetry.phase.value)
        max_hip = max(max_hip, telemetry.max_hip_torque_nm)
        max_wheel = max(max_wheel, telemetry.max_wheel_torque_nm)
        step_contact |= telemetry.step_wheel_contacts > 0
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            controller.fail("non-finite state")
        if controller.terminal:
            break
    return {
        "mass_kg": mass_kg,
        "height_cm": height_m * 100.0,
        "final_phase": controller.phase.value,
        "failure_reason": controller.failure_reason,
        "terminal_time_s": float(data.time),
        "phases_observed": list(dict.fromkeys(phases)),
        "step_contact_observed": step_contact,
        "max_hip_torque_nm": max_hip,
        "max_wheel_torque_nm": max_wheel,
    }


def main() -> int:
    output_dir = PROJECT_ROOT / "artifacts" / "stair_controller"
    output_dir.mkdir(parents=True, exist_ok=True)
    results = [
        run_case(mass, height)
        for mass in (2.0, 2.3, 2.5)
        for height in (0.05, 0.10, 0.15)
    ]
    (output_dir / "sweep_summary.json").write_text(
        json.dumps(results, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    with (output_dir / "sweep_summary.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    for result in results:
        print(
            f"{result['mass_kg']:g} kg / {result['height_cm']:g} cm: "
            f"{result['final_phase']} — {result['failure_reason'] or 'success'}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
