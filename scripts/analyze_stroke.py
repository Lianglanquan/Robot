#!/usr/bin/env python3
import argparse
import csv
import gzip
import io
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
from matplotlib.patches import Circle  # noqa: E402

from src.parameters import DEFAULT_PARAMETERS  # noqa: E402
from src.stroke import (  # noqa: E402
    NORMAL_LEFT_SIGN,
    NORMAL_RIGHT_SIGN,
    STROKE_COLUMNS,
    StrokeScan,
    normal_vertical_posture,
    scan_normal_vertical_stroke,
)

FIGURE_NAMES = (
    "continuous_stroke_geometry.png",
    "joint_continuation.png",
    "kinematic_quality.png",
    "force_speed_tradeoff.png",
)
UPSTREAM_NORMAL_MIN_M = 0.070
UPSTREAM_NORMAL_MAX_M = 0.090
UPSTREAM_AIRBORNE_TARGET_M = 0.120
UPSTREAM_SIMSCAPE_SCOPE_M = (0.05431, 0.12523)
WHEEL_RADIUS_M = 0.026
LANDMARK_LENGTHS_M = (0.050, 0.05431, 0.070, 0.090, 0.100, 0.120, 0.12523, 0.140, 0.150)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze the original five-bar's continuous normal stroke"
    )
    parser.add_argument("--resolution", type=int, default=1201)
    parser.add_argument(
        "--output-dir", type=Path, default=PROJECT_ROOT / "artifacts" / "phase3"
    )
    return parser.parse_args()


def _save(figure: plt.Figure, output: Path) -> None:
    figure.tight_layout()
    figure.savefig(output, dpi=170, facecolor="white")
    plt.close(figure)


def _shade_upstream_evidence(axis: plt.Axes) -> None:
    axis.axvspan(
        UPSTREAM_NORMAL_MIN_M * 1000.0,
        UPSTREAM_NORMAL_MAX_M * 1000.0,
        color="#2ca25f",
        alpha=0.13,
        label="Upstream normal command 70-90 mm",
    )
    axis.axvline(
        UPSTREAM_AIRBORNE_TARGET_M * 1000.0,
        color="#756bb1",
        linestyle="--",
        linewidth=1.4,
        label="Upstream airborne target / soft guard 120 mm",
    )


def _mark_singular_limits(axis: plt.Axes, scan: StrokeScan) -> None:
    for limit in (scan.lower_singularity_m, scan.upper_singularity_m):
        axis.axvline(limit * 1000.0, color="#c43c39", linestyle=":", linewidth=1.3)


def plot_continuous_stroke_geometry(output: Path) -> None:
    lengths = (0.050, 0.070, 0.090, 0.120, 0.145)
    figure, axes = plt.subplots(1, len(lengths), figsize=(16.0, 4.4), sharey=True)
    p = DEFAULT_PARAMETERS
    a = np.array([0.0, 0.0])
    e = np.array([p.l5, 0.0])

    for axis, length in zip(axes, lengths, strict=True):
        posture = normal_vertical_posture(length)
        m = posture.metrics
        points = (a, m.b, m.c, m.d, e)
        for start, end, color, width in (
            (points[0], points[1], "#2f5597", 3.0),
            (points[1], points[2], "#d97706", 3.0),
            (points[2], points[3], "#0f766e", 3.0),
            (points[3], points[4], "#b91c1c", 3.0),
        ):
            axis.plot(
                np.array([start[0], end[0]]) * 1000.0,
                np.array([start[1], end[1]]) * 1000.0,
                color=color,
                linewidth=width,
                marker="o",
                markersize=3.5,
            )
        axis.plot([0.0, p.l5 * 1000.0], [0.0, 0.0], color="#4b5563", linewidth=2)
        axis.add_patch(
            Circle(
                (m.xc * 1000.0, m.yc * 1000.0),
                WHEEL_RADIUS_M * 1000.0,
                fill=False,
                color="#666666",
                linestyle="--",
                linewidth=1.2,
            )
        )
        axis.set_title(
            f"l0 = {length * 1000:.0f} mm\n"
            f"phi1={np.rad2deg(posture.phi1):.1f} deg, "
            f"phi4={np.rad2deg(posture.phi4):.1f} deg",
            fontsize=9,
        )
        axis.set_xlim(-62.0, 122.0)
        axis.set_ylim(-20.0, 180.0)
        axis.invert_yaxis()
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, linestyle=":", linewidth=0.5, alpha=0.45)
        axis.set_xlabel("x [mm]")
    axes[0].set_ylabel("physical downward direction [mm]")
    figure.suptitle(
        "Continuous outward-elbow mode (wheel circle shown only for scale)",
        fontsize=13,
    )
    _save(figure, output)


def plot_joint_continuation(scan: StrokeScan, output: Path) -> None:
    length_mm = scan.column("l0") * 1000.0
    figure, axes = plt.subplots(2, 1, figsize=(10.2, 8.0), sharex=True)

    axes[0].plot(length_mm, np.rad2deg(scan.column("phi1")), label="phi1 (unwrapped)")
    axes[0].plot(length_mm, np.rad2deg(scan.column("phi4")), label="phi4 (unwrapped)")
    axes[0].set_ylabel("active-joint angle [deg]")
    axes[0].set_title("Joint continuation along a vertical stroke")
    axes[0].legend(loc="best")

    gradient = np.rad2deg(np.abs(scan.column("dq1_dl0"))) / 1000.0
    axes[1].plot(length_mm, gradient, color="#6a3d9a")
    axes[1].set_yscale("log")
    axes[1].set_xlabel("virtual leg length l0 [mm]")
    axes[1].set_ylabel("required joint rotation [deg/mm]")
    axes[1].set_title("Joint motion cost for one millimetre of extension")

    for axis in axes:
        _shade_upstream_evidence(axis)
        _mark_singular_limits(axis, scan)
        axis.grid(True, which="both", linestyle=":", linewidth=0.5, alpha=0.5)
    handles, labels = axes[1].get_legend_handles_labels()
    axes[1].legend(handles, labels, loc="best", fontsize=8)
    _save(figure, output)


def plot_kinematic_quality(scan: StrokeScan, output: Path) -> None:
    length_mm = scan.column("l0") * 1000.0
    figure, axes = plt.subplots(1, 3, figsize=(16.0, 5.2))

    axes[0].plot(length_mm, scan.column("sigma_min"), label="sigma min")
    axes[0].plot(length_mm, scan.column("sigma_max"), label="sigma max")
    axes[0].axhline(0.010, color="#e69f00", linestyle="--", label="Phase 2 0.010")
    axes[0].axhline(0.030, color="#238b45", linestyle="--", label="0.030 screen")
    axes[0].set_ylabel("physical J singular value [m/rad]")
    axes[0].set_title("Motion authority, not just anisotropy")
    axes[0].legend(loc="best", fontsize=8)

    axes[1].plot(length_mm, scan.column("condition_number"), color="#8c2d04")
    axes[1].set_yscale("log")
    axes[1].set_ylabel("physical J condition number")
    axes[1].set_title("Condition remains finite at serial limits")

    axes[2].plot(
        length_mm,
        scan.column("serial_sine_min"),
        label="serial-singularity sine margin",
    )
    axes[2].plot(
        length_mm,
        scan.column("parallel_sine"),
        label="parallel-singularity sine margin",
    )
    axes[2].set_ylim(-0.03, 1.05)
    axes[2].set_ylabel("dimensionless geometric margin")
    axes[2].set_title("Which singularity is approached?")
    axes[2].legend(loc="best", fontsize=8)

    for axis in axes:
        _shade_upstream_evidence(axis)
        _mark_singular_limits(axis, scan)
        axis.set_xlabel("virtual leg length l0 [mm]")
        axis.grid(True, which="both", linestyle=":", linewidth=0.5, alpha=0.5)
    _save(figure, output)


def plot_force_speed_tradeoff(scan: StrokeScan, output: Path) -> None:
    length_mm = scan.column("l0") * 1000.0
    figure, axes = plt.subplots(1, 2, figsize=(12.8, 5.3))

    axes[0].plot(length_mm, scan.column("max_axial_force"), color="#4d4d4d")
    axes[0].set_yscale("log")
    axes[0].set_ylabel("max pure axial force [N] at 1 N m per joint")
    axes[0].set_title("Ideal force gain diverges at serial singularities")

    axes[1].plot(
        length_mm,
        scan.column("path_extension_speed") * 1000.0,
        color="#2b8cbe",
    )
    axes[1].set_ylabel("max fixed-angle extension speed [mm/s]\nat 1 rad/s per joint")
    axes[1].set_title("Extension speed collapses at the same endpoints")

    for axis in axes:
        _shade_upstream_evidence(axis)
        _mark_singular_limits(axis, scan)
        axis.set_xlabel("virtual leg length l0 [mm]")
        axis.grid(True, which="both", linestyle=":", linewidth=0.5, alpha=0.5)
    axes[1].text(
        0.98,
        0.04,
        "Along this symmetric path:\nFmax x vmax = 2 W (ideal)",
        transform=axes[1].transAxes,
        ha="right",
        va="bottom",
        bbox={"facecolor": "white", "edgecolor": "#999999", "alpha": 0.9},
    )
    _save(figure, output)


def write_csv(scan: StrokeScan, output: Path) -> None:
    with output.open("wb") as raw_file:
        with gzip.GzipFile(
            filename="", fileobj=raw_file, mode="wb", mtime=0
        ) as compressed_file:
            with io.TextIOWrapper(
                compressed_file, encoding="ascii", newline=""
            ) as file:
                writer = csv.writer(file)
                writer.writerow(STROKE_COLUMNS)
                for row in scan.values:
                    writer.writerow(f"{value:.12g}" for value in row)


def _threshold_interval(
    scan: StrokeScan, column_name: str, threshold: float
) -> list[float] | None:
    values = scan.column(column_name)
    selected = values >= threshold
    indices = np.flatnonzero(selected)
    if not len(indices):
        return None
    return [
        float(scan.column("l0")[indices[0]]),
        float(scan.column("l0")[indices[-1]]),
    ]


def _posture_snapshot(l0: float) -> dict[str, float]:
    posture = normal_vertical_posture(l0)
    metrics = posture.metrics
    return {
        "phi1_deg_unwrapped": float(np.rad2deg(posture.phi1)),
        "phi4_deg_unwrapped": float(np.rad2deg(posture.phi4)),
        "sigma_min_m_per_rad": metrics.sigma_min,
        "sigma_max_m_per_rad": metrics.sigma_max,
        "condition_number": metrics.condition_number,
        "serial_sine_margin": min(
            posture.serial_sine_left, posture.serial_sine_right
        ),
        "parallel_sine_margin": posture.parallel_sine,
        "max_axial_force_n_at_1_nm": metrics.max_axial_force,
        "max_extension_speed_m_per_s_at_1_rad_per_s": (
            posture.path_extension_speed
        ),
        "joint_rotation_deg_per_mm": float(
            np.rad2deg(np.max(np.abs(posture.dq_dl0))) / 1000.0
        ),
    }


def _band_summary(scan: StrokeScan, lower_m: float, upper_m: float) -> dict[str, Any]:
    selected = (scan.column("l0") >= lower_m) & (scan.column("l0") <= upper_m)
    start = normal_vertical_posture(lower_m)
    end = normal_vertical_posture(upper_m)
    endpoints = {
        "sigma_min": (start.metrics.sigma_min, end.metrics.sigma_min),
        "condition_number": (
            start.metrics.condition_number,
            end.metrics.condition_number,
        ),
        "serial_sine_min": (
            min(start.serial_sine_left, start.serial_sine_right),
            min(end.serial_sine_left, end.serial_sine_right),
        ),
        "parallel_sine": (start.parallel_sine, end.parallel_sine),
        "max_axial_force": (
            start.metrics.max_axial_force,
            end.metrics.max_axial_force,
        ),
        "path_extension_speed": (
            start.path_extension_speed,
            end.path_extension_speed,
        ),
    }

    def band_column(name: str) -> np.ndarray:
        return np.append(scan.column(name)[selected], endpoints[name])

    return {
        "range_m": [lower_m, upper_m],
        "stroke_m": upper_m - lower_m,
        "joint_excursion_deg_each": float(
            abs(np.rad2deg(end.phi1 - start.phi1))
        ),
        "sigma_min_m_per_rad": {
            "min": float(np.min(band_column("sigma_min"))),
            "max": float(np.max(band_column("sigma_min"))),
        },
        "condition_number_max": float(np.max(band_column("condition_number"))),
        "serial_sine_margin_min": float(np.min(band_column("serial_sine_min"))),
        "parallel_sine_margin_min": float(np.min(band_column("parallel_sine"))),
        "max_axial_force_n_at_1_nm": {
            "min": float(np.min(band_column("max_axial_force"))),
            "max": float(np.max(band_column("max_axial_force"))),
        },
        "max_extension_speed_m_per_s_at_1_rad_per_s": {
            "min": float(np.min(band_column("path_extension_speed"))),
            "max": float(np.max(band_column("path_extension_speed"))),
        },
    }


def build_summary(scan: StrokeScan) -> dict[str, Any]:
    sigma_min = scan.column("sigma_min")
    condition = scan.column("condition_number")
    speed = scan.column("path_extension_speed")
    force = scan.column("max_axial_force")
    l0 = scan.column("l0")
    best_sigma_index = int(np.argmax(sigma_min))
    best_condition_index = int(np.argmin(condition))
    threshold_intervals = {}
    for column_name, thresholds in (
        ("sigma_min", (0.010, 0.020, 0.030, 0.040)),
        ("serial_sine_min", (0.5, 0.7, 0.9)),
    ):
        threshold_intervals[column_name] = {
            f"ge_{threshold:g}": _threshold_interval(scan, column_name, threshold)
            for threshold in thresholds
        }

    return {
        "resolution": scan.resolution,
        "normal_phi0_deg": 90.0,
        "normal_mode_signs": [NORMAL_LEFT_SIGN, NORMAL_RIGHT_SIGN],
        "lower_serial_singularity_m": scan.lower_singularity_m,
        "upper_serial_singularity_m": scan.upper_singularity_m,
        "mathematical_stroke_m": (
            scan.upper_singularity_m - scan.lower_singularity_m
        ),
        "sample_endpoint_margin_m": scan.endpoint_margin_m,
        "best_motion_authority": {
            "l0_m": float(l0[best_sigma_index]),
            "sigma_min_m_per_rad": float(sigma_min[best_sigma_index]),
            "condition_number": float(condition[best_sigma_index]),
            "max_axial_force_n_at_1_nm": float(force[best_sigma_index]),
            "max_extension_speed_m_per_s_at_1_rad_per_s": float(
                speed[best_sigma_index]
            ),
        },
        "best_isotropy": {
            "l0_m": float(l0[best_condition_index]),
            "condition_number": float(condition[best_condition_index]),
            "sigma_min_m_per_rad": float(sigma_min[best_condition_index]),
        },
        "threshold_intervals": threshold_intervals,
        "operating_bands": {
            "upstream_normal_command_70_90_mm": _band_summary(
                scan, UPSTREAM_NORMAL_MIN_M, UPSTREAM_NORMAL_MAX_M
            ),
            "upstream_normal_to_airborne_target_70_120_mm": _band_summary(
                scan, UPSTREAM_NORMAL_MIN_M, UPSTREAM_AIRBORNE_TARGET_M
            ),
        },
        "landmarks": {
            f"{length:.3f}_m": _posture_snapshot(length)
            for length in LANDMARK_LENGTHS_M
        },
        "upstream_evidence": {
            "normal_command_range_m": [
                UPSTREAM_NORMAL_MIN_M,
                UPSTREAM_NORMAL_MAX_M,
            ],
            "airborne_target_m": UPSTREAM_AIRBORNE_TARGET_M,
            "soft_extension_guard_m": UPSTREAM_AIRBORNE_TARGET_M,
            "simscape_saved_scope_display_range_m": list(
                UPSTREAM_SIMSCAPE_SCOPE_M
            ),
            "wheel_radius_m": WHEEL_RADIUS_M,
        },
        "ideal_force_speed_product_w": {
            "min": float(np.min(scan.column("force_speed_product"))),
            "max": float(np.max(scan.column("force_speed_product"))),
        },
        "evidence_limits": [
            "No hard active-joint limits are specified upstream.",
            "The SolidWorks assembly is available but does not provide a checked "
            "collision envelope here.",
            "Force and speed values are ideal normalized kinematics, not actuator "
            "performance.",
        ],
    }


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    scan = scan_normal_vertical_stroke(args.resolution)
    write_csv(scan, args.output_dir / "normal_vertical_stroke.csv.gz")
    summary = build_summary(scan)
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    plot_continuous_stroke_geometry(args.output_dir / FIGURE_NAMES[0])
    plot_joint_continuation(scan, args.output_dir / FIGURE_NAMES[1])
    plot_kinematic_quality(scan, args.output_dir / FIGURE_NAMES[2])
    plot_force_speed_tradeoff(scan, args.output_dir / FIGURE_NAMES[3])
    print(
        f"Scanned {scan.resolution} normal-mode postures over "
        f"{summary['mathematical_stroke_m'] * 1000.0:.3f} mm theoretical stroke"
    )
    for name in FIGURE_NAMES:
        print(f"Saved {args.output_dir / name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
