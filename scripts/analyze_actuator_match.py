#!/usr/bin/env python3
"""Match the public EL05 envelope to the verified centered vertical stroke."""

import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.actuator import EL05_PUBLIC_ENVELOPE  # noqa: E402
from src.actuator_matching import (  # noqa: E402
    extension_operating_point,
    static_support_match,
)

DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "artifacts" / "actuator_match"
DEFAULT_MANUAL = (
    PROJECT_ROOT
    / ".worktrees"
    / "edulite-reference"
    / "产品资料"
    / "EL05"
    / "EL05使用说明书260713.pdf"
)
MANUAL_SHA256 = "a1c258af2b907ff81cb410302bbbc20b2e1f7c6fe1c0b78b02ac7584f27d1cdc"
MASSES_KG = (2.0, 2.3, 2.5)
LANDMARK_LENGTHS_M = (0.070, 0.090, 0.100, 0.120)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manual", type=Path, default=DEFAULT_MANUAL)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_force_speed_csv(path: Path) -> None:
    speeds = np.linspace(0.0, 2.5, 251)
    with path.open("w", newline="", encoding="utf-8") as stream:
        fieldnames = tuple(extension_operating_point(0.100, 0.0).__dict__)
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for l0_m in LANDMARK_LENGTHS_M:
            for speed in speeds:
                writer.writerow(extension_operating_point(l0_m, float(speed)).__dict__)


def static_summary() -> dict[str, Any]:
    lengths = np.linspace(0.070, 0.120, 501)
    result = {}
    for mass in MASSES_KG:
        rows = [static_support_match(mass, float(l0)) for l0 in lengths]
        worst = max(rows, key=lambda row: row.joint_torque_required_nm)
        result[f"{mass:g}_kg"] = {
            "maximum_joint_torque_required_nm": worst.joint_torque_required_nm,
            "worst_l0_m": worst.l0_m,
            "minimum_stall_continuous_margin": worst.stall_continuous_margin,
        }
    return result


def write_static_csv(path: Path) -> None:
    lengths = np.linspace(0.070, 0.120, 501)
    with path.open("w", newline="", encoding="utf-8") as stream:
        fieldnames = tuple(static_support_match(2.0, 0.070).__dict__)
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for mass in MASSES_KG:
            for l0_m in lengths:
                writer.writerow(static_support_match(mass, float(l0_m)).__dict__)


def plot_match(path: Path) -> None:
    actuator = EL05_PUBLIC_ENVELOPE
    figure, axes = plt.subplots(1, 2, figsize=(13.0, 5.2))

    curve = np.asarray(actuator.peak_curve_rpm_nm)
    axes[0].plot(curve[:, 0], curve[:, 1], color="#d97706", linewidth=2.3)
    axes[0].scatter(
        [actuator.rotating_rated_speed_rpm],
        [actuator.rotating_rated_torque_nm],
        color="#2563eb",
        zorder=3,
        label="published rotating rating",
    )
    axes[0].scatter(
        [0.0],
        [actuator.stall_continuous_torque_nm],
        color="#059669",
        zorder=3,
        label="published stall continuous",
    )
    axes[0].set(
        xlabel="joint speed [rpm]",
        ylabel="joint torque [N m]",
        title="EL05 public 48 V envelope",
        xlim=(0.0, 440.0),
        ylim=(0.0, 6.4),
    )
    axes[0].legend(loc="upper right")
    axes[0].grid(alpha=0.25)

    speeds = np.linspace(0.0, 2.5, 251)
    for l0_m in LANDMARK_LENGTHS_M:
        total_force = [
            extension_operating_point(l0_m, float(speed)).total_axial_force_n
            for speed in speeds
        ]
        axes[1].plot(speeds, total_force, label=f"l0={l0_m * 1000:.0f} mm")
    for mass, style in zip(MASSES_KG, (":", "--", "-."), strict=True):
        axes[1].axhline(
            mass * 9.80665,
            color="#4b5563",
            linestyle=style,
            linewidth=1.0,
            label=f"{mass:g} kg weight",
        )
    axes[1].set(
        xlabel="centered vertical extension speed [m/s]",
        ylabel="two-leg ideal axial force [N]",
        title="Mechanism-mapped peak screening envelope",
        xlim=(0.0, 2.5),
        ylim=(0.0, 500.0),
    )
    axes[1].legend(ncol=2, fontsize=8)
    axes[1].grid(alpha=0.25)
    figure.suptitle(
        "Public actuator capability match — not a jump or landing prediction"
    )
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def build_summary(manual: Path) -> dict[str, Any]:
    actuator = EL05_PUBLIC_ENVELOPE
    snapshots = {}
    for l0_m in LANDMARK_LENGTHS_M:
        snapshots[f"{l0_m * 1000:.0f}_mm"] = {
            f"{speed:g}_m_per_s": extension_operating_point(l0_m, speed).__dict__
            for speed in (0.0, 0.25, 0.5, 1.0, 1.5, 2.0)
        }
    motor_mass = 4.0 * actuator.mass_kg
    return {
        "schema_version": 1,
        "status": "PUBLIC_CAPABILITY_SCREENING_NOT_DYNAMIC_VALIDATION",
        "source": {
            "manual": manual.name,
            "sha256": MANUAL_SHA256,
            "pdf_pages": [6, 7, 8, 30, 35],
        },
        "published_exact": {
            "rated_voltage_v": actuator.rated_voltage_v,
            "voltage_range_v": actuator.voltage_range_v,
            "no_load_speed_rpm": actuator.no_load_speed_rpm,
            "rotating_rating": {
                "torque_nm": actuator.rotating_rated_torque_nm,
                "speed_rpm": actuator.rotating_rated_speed_rpm,
                "test_heat_sink_mm": [70.0, 70.0],
                "ambient_c": 25.0,
            },
            "stall_continuous_torque_nm": actuator.stall_continuous_torque_nm,
            "peak_torque_nm": actuator.peak_torque_nm,
            "mass_per_actuator_kg": actuator.mass_kg,
            "rotating_overload_s": actuator.rotating_overload_s,
            "stall_overload_s": actuator.stall_overload_s,
            "control": {
                "bus": "CAN 2.0B extended frame, 1 Mbps",
                "modes": ["motion", "Iq current", "speed", "PP", "CSP"],
                "motion_command_torque_nm": [-6.0, 6.0],
                "motion_command_speed_rad_per_s": [-50.0, 50.0],
            },
        },
        "approximate_48v_torque_speed_curve_rpm_nm": actuator.peak_curve_rpm_nm,
        "four_leg_actuator_mass": {
            "kg": motor_mass,
            "fraction_of_2kg_robot": motor_mass / 2.0,
            "fraction_of_2_5kg_robot": motor_mass / 2.5,
        },
        "static_two_leg_support": static_summary(),
        "peak_extension_snapshots": snapshots,
        "evidence_limits": [
            "The T-N anchors are approximate visual digitization of the manual plot.",
            "The 430 rpm zero-torque closure combines the plot with the textual "
            "no-load rating and is an inference.",
            "The rotating rating requires the manufacturer's 70 x 70 mm heat sink "
            "test condition; the current robot bracket has not been thermally equated.",
            "Peak force is an ideal kinematic upper envelope, not available impulse, "
            "jump height, traction, landing absorption or controller bandwidth.",
            "The manual does not publish torque tracking bandwidth or impact accuracy.",
            "The 48 V T-N curve cannot be applied unchanged to an unspecified bus.",
        ],
    }


def main() -> int:
    args = parse_args()
    if sha256(args.manual) != MANUAL_SHA256:
        raise ValueError("unexpected EL05 manual SHA-256")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_force_speed_csv(args.output_dir / "force_speed_envelope.csv")
    write_static_csv(args.output_dir / "static_support.csv")
    plot_match(args.output_dir / "public_capability_match.png")
    summary = build_summary(args.manual)
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"wrote {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
