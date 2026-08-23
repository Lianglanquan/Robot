#!/usr/bin/env python3
import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from src.stair_gate import (  # noqa: E402
    AxialEvent,
    approximate_edulite05_envelope_nm,
    approximate_motor_utilization,
    constant_force_jump,
    constant_force_landing,
    static_edge_torque_per_wheel_nm,
    wheel_clearance_com_rises,
)

STEP_HEIGHT_M = 0.150
WHEEL_RADIUS_M = 0.026
FULL_STROKE_M = 0.050
TUCK_EFFECTIVENESS = 0.90
MASSES_KG = (2.0, 2.3, 2.5)
BASELINE_MASS_KG = 2.5
EDULITE_COUNT = 4
EDULITE_MASS_KG = 0.242
QD4310_COUNT = 2
QD4310_MASS_KG = 0.127


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a task-level decision gate for a 150 mm stair"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "artifacts" / "stair_decision_gate",
    )
    return parser.parse_args()


def _event_summary(event: AxialEvent) -> dict[str, Any]:
    takeoff_or_impact_speed = float(np.max(event.column("leg_speed")))
    utilization_24v = approximate_motor_utilization(event, 24.0)
    summary = {
        "mass_kg": event.mass_kg,
        "length_range_m": [event.start_length_m, event.end_length_m],
        "external_height_m": event.external_height_m,
        "total_axial_force_n": event.total_axial_force_n,
        "actuator_work_j": event.actuator_work_j,
        "duration_s": event.duration_s,
        "max_leg_speed_m_per_s": takeoff_or_impact_speed,
        "max_joint_torque_nm": event.max_joint_torque_nm,
        "max_joint_speed_rpm": event.max_joint_speed_rpm,
        "max_total_joint_power_w": event.max_total_joint_power_w,
        "approximate_el05_utilization_48v": approximate_motor_utilization(
            event, 48.0
        ),
        "hypothetical_el05_utilization_24v": (
            utilization_24v if np.isfinite(utilization_24v) else None
        ),
    }
    if event.kind == "jump":
        ascent_time = takeoff_or_impact_speed / 9.80665
        summary["flight_ascent_time_s"] = ascent_time
        summary["horizontal_speed_for_50mm_standoff_m_per_s"] = 0.050 / ascent_time
        summary["horizontal_speed_for_100mm_standoff_m_per_s"] = (
            0.100 / ascent_time
        )
    return summary


def build_summary() -> dict[str, Any]:
    rises = wheel_clearance_com_rises(
        STEP_HEIGHT_M,
        WHEEL_RADIUS_M,
        FULL_STROKE_M,
        TUCK_EFFECTIVENESS,
    )
    jump_scenarios = {
        "full_clear_no_tuck": rises["full_clear_no_tuck"],
        "full_clear_with_90pct_tuck": rises["full_clear_with_tuck"],
        "edge_contact_with_90pct_tuck_optimistic": rises[
            "edge_contact_with_tuck"
        ],
    }
    events: dict[str, Any] = {}
    for mass in MASSES_KG:
        mass_key = f"{mass:.1f}_kg"
        events[mass_key] = {
            "jump": {
                name: _event_summary(constant_force_jump(mass, height))
                for name, height in jump_scenarios.items()
            },
            "landing": {
                "20_mm_compression": _event_summary(
                    constant_force_landing(
                        mass,
                        STEP_HEIGHT_M,
                        touchdown_length_m=0.090,
                        bottom_length_m=0.070,
                    )
                ),
                "50_mm_compression": _event_summary(
                    constant_force_landing(mass, STEP_HEIGHT_M)
                ),
            },
        }

    actuator_mass = EDULITE_COUNT * EDULITE_MASS_KG
    wheel_motor_mass = QD4310_COUNT * QD4310_MASS_KG
    return {
        "evidence_class": "screening model; not a feasibility proof",
        "task": {
            "step_height_m": STEP_HEIGHT_M,
            "wheel_radius_m": WHEEL_RADIUS_M,
            "full_candidate_stroke_m": FULL_STROKE_M,
            "tuck_effectiveness_assumption": TUCK_EFFECTIVENESS,
            "com_rise_benchmarks_m": rises,
        },
        "model_assumptions": [
            "One-dimensional point mass with symmetric 50/50 leg loading.",
            "Constant axial force, rigid ground, no losses, no pitch, and no "
            "impact peak.",
            "Wheel tuck changes wheel height relative to COM by 90% of retraction.",
            "Edge contact is only an optimistic geometric lower bound, not a "
            "stable maneuver.",
            "The 48 V EL05 curve is visually digitized; 24 V speed scaling is "
            "hypothetical.",
        ],
        "candidate_hardware": {
            "edulite05": {
                "count": EDULITE_COUNT,
                "unit_mass_kg": EDULITE_MASS_KG,
                "combined_mass_kg": actuator_mass,
                "rated_torque_nm_at_100rpm": 1.8,
                "peak_torque_nm": 6.0,
                "rated_output_power_w": 19.0,
                "no_load_speed_rpm_at_48v": 430.0,
                "rated_voltage_v": 48.0,
                "voltage_range_v": [15.0, 60.0],
            },
            "qd4310": {
                "count": QD4310_COUNT,
                "unit_mass_kg": QD4310_MASS_KG,
                "combined_mass_kg": wheel_motor_mass,
                "rated_torque_nm": 0.2,
                "peak_torque_nm": 0.3,
                "rated_speed_rpm": 500.0,
                "peak_speed_rpm": 800.0,
                "voltage_range_v": [7.0, 26.0],
            },
            "six_motor_mass_kg": actuator_mass + wheel_motor_mass,
            "mass_fraction": {
                f"at_{mass:.1f}_kg": (actuator_mass + wheel_motor_mass) / mass
                for mass in MASSES_KG
            },
            "static_edge_pivot_screen": {
                f"{mass:.1f}_kg_required_nm_per_wheel": (
                    static_edge_torque_per_wheel_nm(mass, WHEEL_RADIUS_M)
                )
                for mass in MASSES_KG
            },
            "power_architecture_conflict": (
                "EL05 performance is specified at 48 V while QD4310 is limited to 26 V."
            ),
            "landing_regeneration_requirement": (
                "EL05 documentation warns that externally driven damping feeds "
                "the bus; overvoltage protection or an energy sink is required."
            ),
        },
        "events": events,
        "decision_readout": {
            "original_links": "KEEP as the geometric baseline",
            "edulite05": "KEEP as a candidate; not approved for bulk purchase",
            "stair_strategy": (
                "Prioritize forward jump plus in-flight wheel tuck; treat edge capture "
                "as optional and unproven"
            ),
            "full_hardware_purchase": "NOT YET",
            "mujoco": "NOT YET",
            "next_physical_evidence": (
                "EduLite STEP packaging plus CAD motion/collision envelope, "
                "followed by a one-actuator 48 V torque-speed/regeneration bench test"
            ),
        },
    }


def plot_summary(summary: dict[str, Any], output: Path) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(14.5, 10.0))
    rises = summary["task"]["com_rise_benchmarks_m"]

    rise_names = ("Full clear", "Full clear + tuck", "Edge + tuck\n(optimistic)")
    rise_values = np.array(
        [
            rises["full_clear_no_tuck"],
            rises["full_clear_with_tuck"],
            rises["edge_contact_with_tuck"],
        ]
    )
    axes[0, 0].bar(
        rise_names,
        rise_values * 1000.0,
        color=("#636363", "#3182bd", "#f16913"),
    )
    axes[0, 0].axhline(
        50.0, color="#238b45", linestyle="--", label="50 mm stance stroke"
    )
    axes[0, 0].set_ylabel("required COM rise benchmark [mm]")
    axes[0, 0].set_title("Wheel clearance is not the same as leg stroke")
    axes[0, 0].legend(fontsize=8)

    for label, color, path in (
        ("Jump: full clear", "#636363", ("jump", "full_clear_no_tuck")),
        (
            "Jump: full clear + tuck",
            "#3182bd",
            ("jump", "full_clear_with_90pct_tuck"),
        ),
        ("Landing: 50 mm", "#31a354", ("landing", "50_mm_compression")),
        ("Landing: 20 mm", "#de2d26", ("landing", "20_mm_compression")),
    ):
        energies = [
            summary["events"][f"{mass:.1f}_kg"][path[0]][path[1]][
                "actuator_work_j"
            ]
            for mass in MASSES_KG
        ]
        axes[0, 1].plot(MASSES_KG, energies, marker="o", label=label, color=color)
    axes[0, 1].set_xlabel("robot mass [kg]")
    axes[0, 1].set_ylabel("ideal mechanical work / absorbed energy [J]")
    axes[0, 1].set_title("Energy scales directly with final robot mass")
    axes[0, 1].legend(fontsize=8)

    rpm = np.linspace(0.0, 430.0, 500)
    axes[1, 0].plot(
        rpm,
        approximate_edulite05_envelope_nm(rpm, 48.0),
        color="black",
        linewidth=2,
        label="EL05 48 V manual curve (approx.)",
    )
    axes[1, 0].plot(
        rpm,
        approximate_edulite05_envelope_nm(rpm, 24.0),
        color="#969696",
        linestyle="--",
        label="24 V speed scaling (hypothesis)",
    )
    baseline_events = {
        "Jump full clear": constant_force_jump(BASELINE_MASS_KG, 0.150),
        "Jump + tuck": constant_force_jump(BASELINE_MASS_KG, 0.105),
        "Landing 50 mm": constant_force_landing(BASELINE_MASS_KG, 0.150),
        "Landing 20 mm": constant_force_landing(
            BASELINE_MASS_KG,
            0.150,
            touchdown_length_m=0.090,
            bottom_length_m=0.070,
        ),
    }
    colors = ("#636363", "#3182bd", "#31a354", "#de2d26")
    for (label, event), color in zip(baseline_events.items(), colors, strict=True):
        axes[1, 0].plot(
            event.column("joint_rpm"),
            event.column("joint_torque"),
            color=color,
            label=label,
        )
    axes[1, 0].axhline(1.8, color="#756bb1", linestyle=":", label="1.8 N m rated")
    axes[1, 0].set_xlim(0.0, 450.0)
    axes[1, 0].set_ylim(0.0, 6.4)
    axes[1, 0].set_xlabel("joint speed [rpm]")
    axes[1, 0].set_ylabel("joint torque [N m]")
    axes[1, 0].set_title("Ideal 2.5 kg events versus candidate actuator")
    axes[1, 0].legend(fontsize=7, loc="upper right")

    strokes = np.linspace(0.015, 0.070, 80)
    landing_events = [
        constant_force_landing(
            BASELINE_MASS_KG,
            STEP_HEIGHT_M,
            touchdown_length_m=0.070 + float(stroke),
            bottom_length_m=0.070,
            resolution=151,
        )
        for stroke in strokes
    ]
    torque_axis = axes[1, 1]
    power_axis = torque_axis.twinx()
    torque_axis.plot(
        strokes * 1000.0,
        [event.max_joint_torque_nm for event in landing_events],
        color="#de2d26",
        label="max joint torque",
    )
    power_axis.plot(
        strokes * 1000.0,
        [event.max_total_joint_power_w for event in landing_events],
        color="#3182bd",
        label="peak regenerative power",
    )
    torque_axis.axvline(20.0, color="#636363", linestyle=":")
    torque_axis.axvline(50.0, color="#238b45", linestyle="--")
    torque_axis.set_xlabel("available landing compression [mm]")
    torque_axis.set_ylabel("max ideal joint torque [N m]", color="#de2d26")
    power_axis.set_ylabel("peak ideal regenerative power [W]", color="#3182bd")
    torque_axis.set_title("CAD-verified compression stroke changes landing risk")
    handles = torque_axis.lines[:1] + power_axis.lines[:1]
    torque_axis.legend(handles, [line.get_label() for line in handles], fontsize=8)

    for axis in axes.flat:
        axis.grid(True, linestyle=":", linewidth=0.5, alpha=0.5)
    figure.suptitle(
        "150 mm stair decision gate: ideal bounds, not a feasibility proof",
        fontsize=14,
    )
    figure.tight_layout()
    figure.savefig(output, dpi=170, facecolor="white")
    plt.close(figure)


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary = build_summary()
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    plot_summary(summary, args.output_dir / "stair_decision_gate.png")
    print(f"Saved {args.output_dir / 'summary.json'}")
    print(f"Saved {args.output_dir / 'stair_decision_gate.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
