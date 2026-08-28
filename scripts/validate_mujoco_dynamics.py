#!/usr/bin/env python3
"""Run the first flat-ground and landing checks for the dynamic MuJoCo model."""

import csv
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mujoco_dynamics import (  # noqa: E402
    apply_standing_controller,
    chassis_pitch,
    initialize_dynamic_state,
    leg_length_mm,
    mass_budget,
    set_step_height,
    whole_robot_mass_properties,
)

MODEL_PATH = PROJECT_ROOT / "mujoco" / "robot_dynamic_2p5kg.xml"
OUTPUT_DIR = PROJECT_ROOT / "artifacts" / "mujoco_dynamics"


def run_case(
    model: mujoco.MjModel,
    *,
    name: str,
    target_l0_mm: float,
    duration_s: float,
    pitch_deg: float = 0.0,
    drop_height_m: float = 0.0,
) -> list[dict[str, float | int | str]]:
    data = mujoco.MjData(model)
    initialize_dynamic_state(
        model,
        data,
        l0_mm=target_l0_mm,
        pitch_rad=np.deg2rad(pitch_deg),
        drop_height_m=drop_height_m,
    )
    free_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "chassis_free")
    qpos_address = model.jnt_qposadr[free_id]
    rows: list[dict[str, float | int | str]] = []
    steps = round(duration_s / model.opt.timestep)
    sample_every = max(1, round(0.005 / model.opt.timestep))
    for index in range(steps):
        apply_standing_controller(model, data, target_l0_mm=target_l0_mm)
        mujoco.mj_step(model, data)
        if index % sample_every == 0 or index == steps - 1:
            rows.append(
                {
                    "case": name,
                    "time_s": float(data.time),
                    "target_l0_mm": target_l0_mm,
                    "actual_l0_mm": leg_length_mm(model, data, "left"),
                    "chassis_y_m": float(data.qpos[qpos_address + 1]),
                    "chassis_z_m": float(data.qpos[qpos_address + 2]),
                    "pitch_deg": float(np.rad2deg(chassis_pitch(model, data))),
                    "contact_count": int(data.ncon),
                    "max_hip_torque_nm": float(np.max(np.abs(data.ctrl[:4]))),
                    "max_wheel_torque_nm": float(np.max(np.abs(data.ctrl[4:]))),
                }
            )
    return rows


def case_summary(rows: list[dict[str, float | int | str]]) -> dict[str, float | bool]:
    final = rows[-1]
    return {
        "final_pitch_deg": float(final["pitch_deg"]),
        "final_position_y_m": float(final["chassis_y_m"]),
        "final_leg_length_mm": float(final["actual_l0_mm"]),
        "max_abs_pitch_deg": max(abs(float(row["pitch_deg"])) for row in rows),
        "max_abs_hip_torque_nm": max(float(row["max_hip_torque_nm"]) for row in rows),
        "max_abs_wheel_torque_nm": max(
            float(row["max_wheel_torque_nm"]) for row in rows
        ),
        "contact_observed": any(int(row["contact_count"]) > 0 for row in rows),
    }


def mass_properties_at(
    model: mujoco.MjModel, l0_mm: float
) -> dict[str, float | tuple[float, ...] | tuple[tuple[float, ...], ...]]:
    data = mujoco.MjData(model)
    initialize_dynamic_state(model, data, l0_mm=l0_mm)
    properties = whole_robot_mass_properties(model, data)
    return {
        "mass_kg": properties.mass_kg,
        "com_m": properties.com_m,
        "inertia_at_com_kg_m2": properties.inertia_at_com_kg_m2,
    }


def step_contact_check(
    model: mujoco.MjModel, height_m: float
) -> dict[str, float | bool]:
    set_step_height(model, height_m)
    data = mujoco.MjData(model)
    initialize_dynamic_state(model, data, l0_mm=90.0)
    step_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "step")
    first_contact_s: float | None = None
    for _ in range(1600):
        apply_standing_controller(
            model,
            data,
            target_l0_mm=90.0,
            target_y_m=-0.3,
        )
        mujoco.mj_step(model, data)
        if first_contact_s is None and any(
            contact.geom1 == step_id or contact.geom2 == step_id
            for contact in data.contact[: data.ncon]
        ):
            first_contact_s = float(data.time)
    return {
        "wheel_step_contact_observed": first_contact_s is not None,
        "first_contact_time_s": first_contact_s
        if first_contact_s is not None
        else -1.0,
    }


def plot_results(
    stand: list[dict[str, float | int | str]],
    holds: dict[float, list[dict[str, float | int | str]]],
    landing: list[dict[str, float | int | str]],
    output_path: Path,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    stand_time = [float(row["time_s"]) for row in stand]
    axes[0, 0].plot(stand_time, [float(row["pitch_deg"]) for row in stand])
    axes[0, 0].axhline(0.0, color="black", linewidth=0.7)
    axes[0, 0].set(title="Balance recovery from 3° pitch", xlabel="Time [s]")
    axes[0, 0].set_ylabel("Chassis pitch [deg]")
    axes[0, 0].grid(alpha=0.25)

    targets = sorted(holds)
    final_lengths = [float(holds[target][-1]["actual_l0_mm"]) for target in targets]
    axes[0, 1].plot(targets, targets, "k--", label="command = measured")
    axes[0, 1].scatter(
        targets, final_lengths, s=55, color="#d95f02", label="simulation"
    )
    axes[0, 1].set(
        title="Static leg-length hold (2.5 kg)",
        xlabel="Commanded l0 [mm]",
        ylabel="Measured l0 [mm]",
    )
    axes[0, 1].legend()
    axes[0, 1].grid(alpha=0.25)

    landing_time = [float(row["time_s"]) for row in landing]
    axes[1, 0].plot(
        landing_time,
        [1000.0 * float(row["chassis_z_m"]) for row in landing],
        label="chassis root z",
    )
    axes[1, 0].plot(
        landing_time,
        [float(row["actual_l0_mm"]) for row in landing],
        label="measured l0",
    )
    axes[1, 0].set(
        title="5 cm vertical drop and recovery",
        xlabel="Time [s]",
        ylabel="Distance [mm]",
    )
    axes[1, 0].legend()
    axes[1, 0].grid(alpha=0.25)

    axes[1, 1].plot(
        landing_time,
        [float(row["max_hip_torque_nm"]) for row in landing],
        label="max |EL05 torque|",
    )
    axes[1, 1].plot(
        landing_time,
        [float(row["max_wheel_torque_nm"]) for row in landing],
        label="max |wheel torque|",
    )
    axes[1, 1].axhline(6.0, color="#d95f02", linestyle="--", label="EL05 peak limit")
    axes[1, 1].axhline(0.3, color="#1b9e77", linestyle=":", label="QD4310 peak limit")
    axes[1, 1].set(
        title="Actuator command during drop",
        xlabel="Time [s]",
        ylabel="Torque command [N·m]",
    )
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].grid(alpha=0.25)
    fig.suptitle(
        "MuJoCo dynamics checkpoint — estimated inertias, ideal torque sources"
    )
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    stand = run_case(
        model,
        name="balance_3deg",
        target_l0_mm=90.0,
        duration_s=5.0,
        pitch_deg=3.0,
    )
    holds = {
        length: run_case(
            model,
            name=f"hold_{length:g}mm",
            target_l0_mm=length,
            duration_s=3.0,
        )
        for length in (70.0, 90.0, 120.0)
    }
    landing = run_case(
        model,
        name="drop_50mm",
        target_l0_mm=90.0,
        duration_s=4.0,
        pitch_deg=2.0,
        drop_height_m=0.05,
    )
    all_rows = stand + [row for rows in holds.values() for row in rows] + landing
    with (OUTPUT_DIR / "validation_timeseries.csv").open(
        "w", newline="", encoding="utf-8"
    ) as output:
        writer = csv.DictWriter(output, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)

    budgets = {str(mass): mass_budget(mass).__dict__ for mass in (2.0, 2.3, 2.5)}
    mass_properties = {}
    for mass in (2.0, 2.3, 2.5):
        label = str(mass).replace(".", "p")
        mass_model = mujoco.MjModel.from_xml_path(
            str(PROJECT_ROOT / "mujoco" / f"robot_dynamic_{label}kg.xml")
        )
        mass_properties[str(mass)] = mass_properties_at(mass_model, 90.0)
    step_contacts = {
        f"{100.0 * height:g}_cm": step_contact_check(model, height)
        for height in (0.05, 0.10, 0.15)
    }
    set_step_height(model, 0.0)
    summary = {
        "evidence_scope": {
            "manufacturer_values": {
                "EL05_unit_mass_kg": 0.242,
                "EL05_peak_torque_nm": 6.0,
                "QD4310_unit_mass_kg": 0.127,
                "QD4310_peak_torque_nm": 0.3,
            },
            "model_assumptions": [
                "total robot mass is parameterized at 2.0, 2.3 and 2.5 kg",
                "unmeasured fixed mass is lumped into the chassis",
                "link, wheel, COM and inertia values are engineering estimates",
                "actuators are ideal torque sources within peak torque clamps",
            ],
        },
        "mass_budgets": budgets,
        "whole_robot_mass_properties_at_90mm": mass_properties,
        "2p5kg_posture_mass_properties": {
            f"{length:g}_mm": mass_properties_at(model, length)
            for length in (70.0, 90.0, 120.0)
        },
        "balance_3deg": case_summary(stand),
        "leg_length_hold": {
            f"{length:g}_mm": case_summary(rows) for length, rows in holds.items()
        },
        "drop_50mm": case_summary(landing),
        "step_contact_scenes": step_contacts,
        "claim_boundary": (
            "flat standing and a 50 mm vertical drop are controller/model checks; "
            "step-climbing capability has not yet been demonstrated"
        ),
    }
    (OUTPUT_DIR / "validation_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    plot_results(stand, holds, landing, OUTPUT_DIR / "dynamics_validation.png")
    print(f"wrote {OUTPUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
