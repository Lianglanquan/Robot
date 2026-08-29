#!/usr/bin/env python3
"""Validate the event-driven first-pass stair maneuver in MuJoCo."""

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import cast

import matplotlib.pyplot as plt
import numpy as np

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mujoco_dynamics import (  # noqa: E402
    ControllerGains,
    initialize_dynamic_state,
    set_wheel_torque_limit,
)
from src.stair_controller import StairController, StairControllerConfig  # noqa: E402

Row = dict[str, float | int | str | bool]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mass", type=float, choices=(2.0, 2.3, 2.5), default=2.5)
    parser.add_argument(
        "--height-cm", type=float, choices=(5.0, 10.0, 15.0), default=5.0
    )
    parser.add_argument("--wheel-torque-limit", type=float, default=1.0)
    return parser.parse_args()


def run(
    mass_kg: float, height_cm: float, wheel_torque_limit_nm: float
) -> tuple[list[Row], StairController]:
    label = str(mass_kg).replace(".", "p")
    height_label = f"{height_cm:g}"
    model = mujoco.MjModel.from_xml_path(
        str(
            PROJECT_ROOT
            / "mujoco"
            / f"robot_dynamic_{label}kg_step_{height_label}cm.xml"
        )
    )
    set_wheel_torque_limit(model, wheel_torque_limit_nm)
    data = mujoco.MjData(model)
    initialize_dynamic_state(model, data, l0_mm=90.0)
    controller = StairController(
        model,
        config=controller_config(height_cm / 100.0, wheel_torque_limit_nm),
        gains=ControllerGains(wheel_torque_limit_nm=wheel_torque_limit_nm),
    )
    rows: list[Row] = []
    max_steps = round(8.0 / model.opt.timestep)
    for _ in range(max_steps):
        telemetry = controller.step(data)
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            controller.fail("non-finite state")
        row: Row = telemetry.__dict__.copy()
        row["phase"] = telemetry.phase.value
        rows.append(row)
        if controller.terminal:
            break
    return rows, controller


def controller_config(
    height_m: float, wheel_torque_limit_nm: float
) -> StairControllerConfig:
    return StairControllerConfig(
        step_height_m=height_m,
        # Keep the wheel drive at the upstream-scale 0.3 N·m command while
        # allowing the validation harness to model a stronger actuator limit.
        push_wheel_torque_nm=min(0.3, wheel_torque_limit_nm),
        push_l0_mm=140.0,
        tuck_l0_mm=140.0,
        landing_l0_mm=140.0,
        push_duration_s=0.12,
        approach_timeout_s=4.0,
        overall_timeout_s=10.0,
        recover_timeout_s=4.0,
    )


def save_results(
    rows: list[Row],
    controller: StairController,
    mass_kg: float,
    height_cm: float,
    wheel_torque_limit_nm: float,
) -> None:
    output_dir = PROJECT_ROOT / "artifacts" / "stair_controller"
    output_dir.mkdir(parents=True, exist_ok=True)
    suffix = f"{mass_kg:g}kg_{height_cm:g}cm".replace(".", "p")
    csv_path = output_dir / f"timeseries_{suffix}.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=list(rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "mass_kg": mass_kg,
        "step_height_cm": height_cm,
        "wheel_torque_limit_nm": wheel_torque_limit_nm,
        "final_phase": controller.phase.value,
        "failure_reason": controller.failure_reason,
        "terminal_time_s": float(cast(float, rows[-1]["time_s"])),
        "phases_observed": list(dict.fromkeys(str(row["phase"]) for row in rows)),
        "max_hip_torque_nm": max(
            float(cast(float, row["max_hip_torque_nm"])) for row in rows
        ),
        "max_wheel_torque_nm": max(
            float(cast(float, row["max_wheel_torque_nm"])) for row in rows
        ),
        "max_abs_pitch_deg": controller.max_abs_pitch_deg,
        "max_abs_vertical_speed_m_s": controller.max_abs_vertical_speed_m_s,
        "finite_state": True,
        "claim_boundary": (
            "This is a first-pass controller/model result. SUCCESS would mean the "
            "simulated wheels held the step top; it is not a physical-robot claim."
        ),
    }
    (output_dir / f"summary_{suffix}.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    time = [float(cast(float, row["time_s"])) for row in rows]
    fig, axes = plt.subplots(
        3, 1, figsize=(11, 8), sharex=True, constrained_layout=True
    )
    axes[0].plot(
        time, [float(cast(float, row["chassis_y_m"])) for row in rows], label="底盘 y"
    )
    axes[0].plot(
        time, [float(cast(float, row["chassis_z_m"])) for row in rows], label="底盘 z"
    )
    axes[0].set_ylabel("位置 [m]")
    axes[0].legend()
    axes[1].plot(
        time,
        [float(cast(float, row["leg_length_mm"])) for row in rows],
        label="腿长 l0",
    )
    axes[1].plot(
        time, [float(cast(float, row["pitch_deg"])) for row in rows], label="俯仰角"
    )
    axes[1].set_ylabel("mm / deg")
    axes[1].legend()
    axes[2].plot(
        time,
        [float(cast(float, row["max_hip_torque_nm"])) for row in rows],
        label="EL05",
    )
    axes[2].plot(
        time, [float(row["max_wheel_torque_nm"]) for row in rows], label="轮端"
    )
    axes[2].set_ylabel("扭矩 [N·m]")
    axes[2].set_xlabel("时间 [s]")
    axes[2].legend()
    for axis in axes:
        axis.grid(alpha=0.25)
    fig.suptitle(
        f"{height_cm:g} cm 越阶状态机验证（{mass_kg:g} kg, "
        f"轮端上限 {wheel_torque_limit_nm:g} N·m）: {controller.phase.value}"
    )
    fig.savefig(output_dir / f"plot_{suffix}.png", dpi=160)
    plt.close(fig)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


def main() -> int:
    args = parse_args()
    rows, controller = run(
        args.mass, args.height_cm, args.wheel_torque_limit
    )
    save_results(
        rows, controller, args.mass, args.height_cm, args.wheel_torque_limit
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
