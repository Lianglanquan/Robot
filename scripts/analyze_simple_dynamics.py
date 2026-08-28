#!/usr/bin/env python3
"""Generate bounded 1-D push and landing references for the verified leg stroke."""

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.simple_dynamics import (  # noqa: E402
    VerticalTrajectory,
    constant_acceleration_push,
    constant_deceleration_landing,
)

DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "artifacts" / "simple_dynamics"
MASSES_KG = (2.0, 2.3, 2.5)
HEIGHTS_M = (0.050, 0.100, 0.150)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args()


def trajectory_summary(trajectory: VerticalTrajectory) -> dict[str, Any]:
    overload = trajectory.conservative_overload_time_s
    return {
        "duration_s": trajectory.duration_s,
        "takeoff_or_impact_speed_m_per_s": float(
            np.max(trajectory.body_speed_m_per_s)
        ),
        "acceleration_m_per_s2": trajectory.acceleration_m_per_s2,
        "total_axial_force_n": trajectory.total_axial_force_n,
        "force_to_weight_ratio": trajectory.total_axial_force_n
        / (trajectory.mass_kg * 9.80665),
        "ideal_mechanical_work_j": trajectory.ideal_mechanical_work_j,
        "peak_mechanical_power_magnitude_w": float(
            np.max(np.abs(trajectory.total_mechanical_power_w))
        ),
        "peak_joint_speed_rad_per_s": float(
            np.max(trajectory.joint_speed_rad_per_s)
        ),
        "peak_joint_speed_rpm": float(np.max(trajectory.joint_speed_rpm)),
        "peak_required_joint_torque_nm": float(
            np.max(trajectory.required_joint_torque_nm)
        ),
        "peak_torque_utilization": float(np.max(trajectory.torque_utilization)),
        "conservative_overload_time_s": (
            "continuous" if np.isinf(overload) else overload
        ),
        "within_public_magnitude_envelope": (
            trajectory.within_public_magnitude_envelope
        ),
    }


def all_trajectories() -> list[VerticalTrajectory]:
    result = []
    for mass in MASSES_KG:
        for height in HEIGHTS_M:
            result.append(constant_acceleration_push(mass, height))
            result.append(constant_deceleration_landing(mass, height))
    return result


def write_trajectory_csv(
    trajectories: list[VerticalTrajectory], path: Path
) -> None:
    fields = (
        "mode",
        "mass_kg",
        "ballistic_height_m",
        "time_s",
        "l0_m",
        "body_speed_m_per_s",
        "joint_speed_rad_per_s",
        "joint_speed_rpm",
        "required_joint_torque_nm",
        "available_joint_torque_nm",
        "torque_utilization",
        "total_mechanical_power_w",
    )
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for trajectory in trajectories:
            for index in range(len(trajectory.time_s)):
                writer.writerow(
                    {
                        "mode": trajectory.mode,
                        "mass_kg": trajectory.mass_kg,
                        "ballistic_height_m": trajectory.ballistic_height_m,
                        "time_s": trajectory.time_s[index],
                        "l0_m": trajectory.l0_m[index],
                        "body_speed_m_per_s": (
                            trajectory.body_speed_m_per_s[index]
                        ),
                        "joint_speed_rad_per_s": (
                            trajectory.joint_speed_rad_per_s[index]
                        ),
                        "joint_speed_rpm": trajectory.joint_speed_rpm[index],
                        "required_joint_torque_nm": (
                            trajectory.required_joint_torque_nm[index]
                        ),
                        "available_joint_torque_nm": (
                            trajectory.available_joint_torque_nm[index]
                        ),
                        "torque_utilization": trajectory.torque_utilization[index],
                        "total_mechanical_power_w": (
                            trajectory.total_mechanical_power_w[index]
                        ),
                    }
                )


def plot_results(path: Path) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(13.0, 9.0))
    reference = constant_acceleration_push(2.5, 0.150)
    time_ms = reference.time_s * 1000.0

    length_axis = axes[0, 0]
    speed_axis = length_axis.twinx()
    length_axis.plot(
        time_ms, reference.l0_m * 1000.0, color="#2563eb", label="leg length"
    )
    speed_axis.plot(
        time_ms,
        reference.body_speed_m_per_s,
        color="#d97706",
        label="vertical speed",
    )
    length_axis.set(
        xlabel="time [ms]",
        ylabel="virtual leg length [mm]",
        title="2.5 kg / 15 cm pure-vertical reference",
    )
    speed_axis.set_ylabel("body speed [m/s]")
    length_axis.grid(alpha=0.25)
    lines = length_axis.lines + speed_axis.lines
    length_axis.legend(lines, [line.get_label() for line in lines], loc="upper left")

    axes[0, 1].plot(
        reference.l0_m * 1000.0,
        reference.required_joint_torque_nm,
        label="required",
        color="#dc2626",
        linewidth=2.2,
    )
    axes[0, 1].plot(
        reference.l0_m * 1000.0,
        reference.available_joint_torque_nm,
        label="public 48 V peak envelope",
        color="#059669",
        linewidth=2.2,
    )
    axes[0, 1].set(
        xlabel="virtual leg length [mm]",
        ylabel="joint torque magnitude [N m]",
        title="Torque-speed check along the push",
    )
    axes[0, 1].legend()
    axes[0, 1].grid(alpha=0.25)

    height_cm = np.asarray(HEIGHTS_M) * 100.0
    for mass in MASSES_KG:
        forces = [
            constant_acceleration_push(mass, height).total_axial_force_n
            for height in HEIGHTS_M
        ]
        axes[1, 0].plot(height_cm, forces, marker="o", label=f"{mass:g} kg")
    axes[1, 0].set(
        xlabel="post-takeoff ballistic rise reference [cm]",
        ylabel="constant two-leg axial force [N]",
        title="Push-force reference over 50 mm stroke",
    )
    axes[1, 0].legend()
    axes[1, 0].grid(alpha=0.25)

    for mass in MASSES_KG:
        powers = [
            np.max(
                np.abs(
                    constant_deceleration_landing(
                        mass, height
                    ).total_mechanical_power_w
                )
            )
            for height in HEIGHTS_M
        ]
        axes[1, 1].plot(height_cm, powers, marker="o", label=f"{mass:g} kg")
    axes[1, 1].set(
        xlabel="vertical drop reference [cm]",
        ylabel="peak ideal absorption power magnitude [W]",
        title="Landing reference over 50 mm compression",
    )
    axes[1, 1].legend()
    axes[1, 1].grid(alpha=0.25)

    figure.suptitle(
        "Bounded 1-D dynamics — excludes pitch, wheel-edge contact and friction"
    )
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def build_summary(trajectories: list[VerticalTrajectory]) -> dict[str, Any]:
    scenarios: dict[str, Any] = {}
    for trajectory in trajectories:
        mass_key = f"{trajectory.mass_kg:g}_kg"
        height_key = f"{trajectory.ballistic_height_m * 100:g}_cm"
        scenarios.setdefault(mass_key, {}).setdefault(height_key, {})[
            trajectory.mode
        ] = trajectory_summary(trajectory)
    maximum_utilization = max(
        float(np.max(trajectory.torque_utilization))
        for trajectory in trajectories
    )
    return {
        "schema_version": 1,
        "status": "IDEAL_1D_SCREENING_NOT_STEP_VALIDATION",
        "model": {
            "stroke_m": [0.070, 0.120],
            "masses_kg": MASSES_KG,
            "post_takeoff_or_drop_heights_m": HEIGHTS_M,
            "push": "constant net acceleration over the full 50 mm stroke",
            "flight": "ballistic rise after takeoff",
            "landing": "constant deceleration over the full 50 mm compression",
            "actuator": "approximate public EL05 48 V peak T-N envelope",
        },
        "scenarios": scenarios,
        "cross_scenario": {
            "all_within_public_magnitude_envelope": all(
                trajectory.within_public_magnitude_envelope
                for trajectory in trajectories
            ),
            "maximum_peak_torque_utilization": maximum_utilization,
        },
        "evidence_limits": [
            "Ballistic rise is not the same as step height when the wheel contacts "
            "the edge or the body pitches.",
            "The whole robot is treated as one vertical point mass; wheel and link "
            "inertia are omitted.",
            "Both legs share load perfectly and remain centered and vertical.",
            "Ground friction, forward speed and wheel-edge contact are omitted.",
            "The EL05 T-N curve is approximate visual digitization at 48 V.",
            "Landing checks torque-speed magnitude only; the public motoring curve "
            "does not prove braking energy handling or impact control bandwidth.",
            "A result inside the envelope supports further work but does not prove "
            "a 150 mm stair maneuver.",
        ],
    }


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    trajectories = all_trajectories()
    write_trajectory_csv(trajectories, args.output_dir / "trajectories.csv")
    plot_results(args.output_dir / "simplified_dynamics.png")
    summary = build_summary(trajectories)
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"wrote {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
