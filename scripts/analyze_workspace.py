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
from matplotlib.colors import BoundaryNorm, ListedColormap, LogNorm  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

from src.analysis import (  # noqa: E402
    CLASS_CODES,
    RECOMMENDED_CONDITION_MAX,
    RECOMMENDED_SIGMA_MIN,
    SCAN_COLUMNS,
    USABLE_CONDITION_MAX,
    USABLE_SIGMA_MIN,
    WorkspaceScan,
    best_class_xy_grid,
    scan_workspace,
    summarize_scan,
)

FIGURE_NAMES = (
    "workspace_pose.png",
    "jacobian_singularity.png",
    "upright_condition.png",
    "force_speed_transmission.png",
    "workspace_classification.png",
)
CLASS_LABELS = ("Recommended", "Usable", "Near singular / avoid")
CLASS_COLORS = ("#238b45", "#e69f00", "#c43c39")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze the original five-bar workspace and transmission"
    )
    parser.add_argument("--resolution", type=int, default=360)
    parser.add_argument(
        "--output-dir", type=Path, default=PROJECT_ROOT / "artifacts" / "phase2"
    )
    return parser.parse_args()


def _equal_xy(axis: plt.Axes) -> None:
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("Foot x [m]")
    axis.set_ylabel("Foot y [m]")
    axis.grid(True, linestyle=":", linewidth=0.5, alpha=0.4)


def _joint_axes(axis: plt.Axes) -> None:
    axis.set_xlabel("phi1 [deg]")
    axis.set_ylabel("phi4 [deg]")
    axis.set_xlim(-180.0, 180.0)
    axis.set_ylim(-180.0, 180.0)
    axis.grid(True, linestyle=":", linewidth=0.5, alpha=0.35)


def _save(figure: plt.Figure, path: Path) -> None:
    figure.tight_layout()
    figure.savefig(path, dpi=170, facecolor="white")
    plt.close(figure)


def _scatter_size(scan: WorkspaceScan) -> float:
    return max(0.5, min(8.0, 1800.0 / scan.resolution))


def plot_workspace_pose(scan: WorkspaceScan, output: Path) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(12.8, 5.6))
    size = _scatter_size(scan)
    x = scan.column("xc")
    y = scan.column("yc")

    length_plot = axes[0].scatter(
        x,
        y,
        c=scan.column("l0") * 1000.0,
        s=size,
        cmap="viridis",
        linewidths=0,
        rasterized=True,
    )
    figure.colorbar(length_plot, ax=axes[0], label="Virtual leg length l0 [mm]")
    axes[0].set_title("Reachable foot positions colored by leg length")
    _equal_xy(axes[0])

    angle_plot = axes[1].scatter(
        x,
        y,
        c=np.rad2deg(scan.column("phi0")),
        s=size,
        cmap="twilight_shifted",
        vmin=-180.0,
        vmax=180.0,
        linewidths=0,
        rasterized=True,
    )
    figure.colorbar(angle_plot, ax=axes[1], label="Virtual leg angle phi0 [deg]")
    axes[1].set_title("Same workspace colored by virtual-leg angle")
    _equal_xy(axes[1])
    figure.suptitle("Original five-bar workspace: every legal joint posture")
    _save(figure, output)


def plot_jacobian_singularity(scan: WorkspaceScan, output: Path) -> None:
    figure, axes = plt.subplots(1, 3, figsize=(17.0, 5.3))
    size = _scatter_size(scan)
    q1 = np.rad2deg(scan.column("phi1"))
    q4 = np.rad2deg(scan.column("phi4"))
    condition = scan.column("condition_number")
    sigma_min = scan.column("sigma_min")
    finite_condition = condition[np.isfinite(condition)]
    condition_max = float(np.percentile(finite_condition, 99.5))

    condition_plot = axes[0].scatter(
        q1,
        q4,
        c=np.clip(condition, 1.0, condition_max),
        s=size,
        cmap="magma",
        norm=LogNorm(vmin=1.0, vmax=condition_max),
        linewidths=0,
        rasterized=True,
    )
    figure.colorbar(condition_plot, ax=axes[0], label="Physical J condition number")
    axes[0].set_title("Joint-space conditioning (clipped at p99.5)")
    _joint_axes(axes[0])

    positive_sigma = sigma_min[sigma_min > 0.0]
    sigma_floor = max(float(np.percentile(positive_sigma, 0.5)), 1e-7)
    sigma_plot = axes[1].scatter(
        q1,
        q4,
        c=np.clip(sigma_min, sigma_floor, float(np.max(sigma_min))),
        s=size,
        cmap="viridis",
        norm=LogNorm(vmin=sigma_floor, vmax=float(np.max(sigma_min))),
        linewidths=0,
        rasterized=True,
    )
    figure.colorbar(sigma_plot, ax=axes[1], label="Minimum singular value [m/rad]")
    axes[1].set_title("Loss of motion authority (clipped below p0.5)")
    _joint_axes(axes[1])

    bad = scan.column("class_code") == CLASS_CODES["near_singular"]
    axes[2].scatter(
        scan.column("xc"),
        scan.column("yc"),
        s=size,
        color="#c9c9c9",
        linewidths=0,
        rasterized=True,
        label="Other legal postures",
    )
    axes[2].scatter(
        scan.column("xc")[bad],
        scan.column("yc")[bad],
        s=max(size, 2.0),
        color=CLASS_COLORS[2],
        marker="x",
        linewidths=0.35,
        rasterized=True,
        label="Near singular / avoid",
    )
    axes[2].set_title("Where near-singular postures place the foot")
    axes[2].legend(loc="best", fontsize=8)
    _equal_xy(axes[2])
    figure.suptitle("Physical Jacobian singularity map")
    _save(figure, output)


def _upright_binned_statistics(
    length: np.ndarray, condition: np.ndarray, bins: int = 28
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    edges = np.linspace(float(np.min(length)), float(np.max(length)), bins + 1)
    centers = (edges[:-1] + edges[1:]) / 2.0
    median = np.full(bins, np.nan)
    low = np.full(bins, np.nan)
    high = np.full(bins, np.nan)
    for index in range(bins):
        selected = (length >= edges[index]) & (length < edges[index + 1])
        if index == bins - 1:
            selected |= length == edges[index + 1]
        if np.any(selected):
            median[index] = np.median(condition[selected])
            low[index], high[index] = np.percentile(condition[selected], [10, 90])
    valid = np.isfinite(median)
    return centers[valid], median[valid], low[valid], high[valid]


def plot_upright_condition(scan: WorkspaceScan, output: Path) -> None:
    upright = scan.upright_mask()
    length = scan.column("l0")[upright]
    condition = scan.column("condition_number")[upright]
    centers, median, low, high = _upright_binned_statistics(length, condition)

    figure, axis = plt.subplots(figsize=(9.4, 6.0))
    axis.scatter(
        length * 1000.0,
        condition,
        s=5,
        color="#7f8c8d",
        alpha=0.22,
        linewidths=0,
        rasterized=True,
        label="All postures within +/-5 deg of vertical",
    )
    axis.fill_between(
        centers * 1000.0,
        low,
        high,
        color="#4c78a8",
        alpha=0.2,
        label="Binned 10-90% range",
    )
    axis.plot(
        centers * 1000.0,
        median,
        color="#1f5a94",
        linewidth=2.2,
        label="Binned median",
    )
    axis.axhline(5.0, color=CLASS_COLORS[0], linestyle="--", label="Recommended limit")
    axis.axhline(20.0, color=CLASS_COLORS[2], linestyle="--", label="Avoid limit")
    axis.set_yscale("log")
    axis.set_xlabel("Virtual leg length l0 [mm]")
    axis.set_ylabel("Physical J condition number")
    axis.set_title("Conditioning along the near-vertical operating band")
    axis.grid(True, which="both", linestyle=":", linewidth=0.5, alpha=0.5)
    axis.legend(loc="best", fontsize=8)
    _save(figure, output)


def plot_force_speed_transmission(scan: WorkspaceScan, output: Path) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(12.8, 5.6))
    size = _scatter_size(scan)
    q1 = np.rad2deg(scan.column("phi1"))
    q4 = np.rad2deg(scan.column("phi4"))
    force = scan.column("max_axial_force")
    speed = scan.column("max_extension_speed")

    force_low, force_high = np.percentile(force[np.isfinite(force)], [1, 99])
    force_plot = axes[0].scatter(
        q1,
        q4,
        c=np.clip(force, force_low, force_high),
        s=size,
        cmap="cividis",
        norm=LogNorm(vmin=float(force_low), vmax=float(force_high)),
        linewidths=0,
        rasterized=True,
    )
    figure.colorbar(force_plot, ax=axes[0], label="Max pure axial force [N] at 1 N m")
    axes[0].set_title("Axial force by joint posture (clipped p1-p99)")
    _joint_axes(axes[0])

    positive_speed = speed[speed > 0.0]
    speed_low, speed_high = np.percentile(positive_speed, [1, 99])
    speed_plot = axes[1].scatter(
        q1,
        q4,
        c=np.clip(speed, speed_low, speed_high),
        s=size,
        cmap="plasma",
        norm=LogNorm(vmin=float(speed_low), vmax=float(speed_high)),
        linewidths=0,
        rasterized=True,
    )
    figure.colorbar(
        speed_plot, ax=axes[1], label="Max |dL| [m/s] at 1 rad/s per joint"
    )
    axes[1].set_title("Extension speed by joint posture (clipped p1-p99)")
    _joint_axes(axes[1])
    figure.suptitle("Normalized force and speed transmission")
    _save(figure, output)


def plot_workspace_classification(scan: WorkspaceScan, output: Path) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(12.8, 5.6))
    size = _scatter_size(scan)
    codes = scan.column("class_code")
    cmap = ListedColormap(CLASS_COLORS)
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)

    axes[0].scatter(
        np.rad2deg(scan.column("phi1")),
        np.rad2deg(scan.column("phi4")),
        c=codes,
        s=size,
        cmap=cmap,
        norm=norm,
        linewidths=0,
        rasterized=True,
    )
    axes[0].set_title("Joint-space classification")
    _joint_axes(axes[0])

    x_edges, y_edges, best_class = best_class_xy_grid(scan)
    axes[1].pcolormesh(
        x_edges,
        y_edges,
        np.ma.masked_invalid(best_class),
        cmap=cmap,
        norm=norm,
        shading="flat",
        rasterized=True,
    )
    axes[1].set_title("Foot workspace: best class in each XY bin")
    _equal_xy(axes[1])

    handles = [
        Line2D(
            [0],
            [0],
            marker="o",
            linestyle="none",
            markerfacecolor=color,
            markeredgecolor="none",
            label=label,
        )
        for label, color in zip(CLASS_LABELS, CLASS_COLORS, strict=True)
    ]
    figure.legend(handles=handles, loc="lower center", ncol=3, frameon=False)
    figure.suptitle("Kinematic operating-region summary")
    figure.subplots_adjust(bottom=0.14)
    _save(figure, output)


def write_csv(scan: WorkspaceScan, output: Path) -> None:
    with output.open("wb") as raw_file:
        with gzip.GzipFile(
            filename="", fileobj=raw_file, mode="wb", mtime=0
        ) as compressed_file:
            with io.TextIOWrapper(
                compressed_file, encoding="ascii", newline=""
            ) as file:
                writer = csv.writer(file)
                writer.writerow(SCAN_COLUMNS)
                for row in scan.values:
                    writer.writerow(f"{value:.12g}" for value in row)


def _posture_snapshot(scan: WorkspaceScan, index: int) -> dict[str, Any]:
    return {
        "row_index": index,
        "phi1_rad": float(scan.column("phi1")[index]),
        "phi4_rad": float(scan.column("phi4")[index]),
        "phi1_deg": float(np.rad2deg(scan.column("phi1")[index])),
        "phi4_deg": float(np.rad2deg(scan.column("phi4")[index])),
        "xc_m": float(scan.column("xc")[index]),
        "yc_m": float(scan.column("yc")[index]),
        "l0_m": float(scan.column("l0")[index]),
        "phi0_deg": float(np.rad2deg(scan.column("phi0")[index])),
        "sigma_min_m_per_rad": float(scan.column("sigma_min")[index]),
        "condition_number": float(scan.column("condition_number")[index]),
        "max_axial_force_n": float(scan.column("max_axial_force")[index]),
        "max_extension_speed_m_per_s": float(
            scan.column("max_extension_speed")[index]
        ),
        "class_code": int(scan.column("class_code")[index]),
    }


def _upright_length_bins(scan: WorkspaceScan) -> list[dict[str, Any]]:
    upright = scan.upright_mask()
    length = scan.column("l0")
    condition = scan.column("condition_number")
    class_code = scan.column("class_code")
    force = scan.column("max_axial_force")
    speed = scan.column("max_extension_speed")
    edges_mm = (45.0, 60.0, 80.0, 100.0, 120.0, 140.0, 153.0)
    result: list[dict[str, Any]] = []
    for low_mm, high_mm in zip(edges_mm[:-1], edges_mm[1:], strict=True):
        selected = upright & (length >= low_mm / 1000.0) & (
            length < high_mm / 1000.0
        )
        if high_mm == edges_mm[-1]:
            selected |= upright & (length == high_mm / 1000.0)
        count = int(np.count_nonzero(selected))
        if count == 0:
            continue
        result.append(
            {
                "l0_low_mm": low_mm,
                "l0_high_mm": high_mm,
                "samples": count,
                "condition_median": float(np.median(condition[selected])),
                "condition_p90": float(np.percentile(condition[selected], 90)),
                "condition_max": float(np.max(condition[selected])),
                "recommended_percent": float(
                    np.count_nonzero(
                        class_code[selected] == CLASS_CODES["recommended"]
                    )
                    / count
                    * 100.0
                ),
                "axial_force_median_n": float(np.median(force[selected])),
                "extension_speed_median_m_per_s": float(
                    np.median(speed[selected])
                ),
            }
        )
    return result


def _classification_metrics(scan: WorkspaceScan) -> dict[str, Any]:
    class_code = scan.column("class_code")
    result: dict[str, Any] = {}
    for name, code in CLASS_CODES.items():
        selected = class_code == code
        metrics: dict[str, Any] = {"samples": int(np.count_nonzero(selected))}
        for output_name, column_name in (
            ("condition_number", "condition_number"),
            ("sigma_min_m_per_rad", "sigma_min"),
            ("max_axial_force_n", "max_axial_force"),
            ("max_extension_speed_m_per_s", "max_extension_speed"),
        ):
            values = scan.column(column_name)[selected]
            metrics[output_name] = {
                "min": float(np.min(values)),
                "median": float(np.median(values)),
                "p95": float(np.percentile(values, 95)),
                "max": float(np.max(values)),
            }
        result[name] = metrics
    return result


def _extended_summary(scan: WorkspaceScan) -> dict[str, Any]:
    summary: dict[str, Any] = dict(summarize_scan(scan))
    upright = scan.upright_mask()
    class_code = scan.column("class_code")
    extreme_columns = {
        "maximum_condition_number": ("condition_number", np.argmax),
        "minimum_sigma_min": ("sigma_min", np.argmin),
        "minimum_axial_force": ("max_axial_force", np.argmin),
        "maximum_axial_force": ("max_axial_force", np.argmax),
        "minimum_extension_speed": ("max_extension_speed", np.argmin),
        "maximum_extension_speed": ("max_extension_speed", np.argmax),
    }
    summary.update(
        {
            "joint_scan_range_rad": [-float(np.pi), float(np.pi)],
            "joint_scan_endpoint_included": False,
            "upright_half_width_deg": 5.0,
            "xy_classification_bins": max(24, scan.resolution // 3),
            "upright_l0_range_m": [
                float(np.min(scan.column("l0")[upright])),
                float(np.max(scan.column("l0")[upright])),
            ],
            "upright_condition_number": {
                "min": float(np.min(scan.column("condition_number")[upright])),
                "median": float(np.median(scan.column("condition_number")[upright])),
                "p90": float(
                    np.percentile(scan.column("condition_number")[upright], 90)
                ),
                "max": float(np.max(scan.column("condition_number")[upright])),
            },
            "upright_length_bins": _upright_length_bins(scan),
            "vertical_classification_counts": {
                name: int(np.count_nonzero(upright & (class_code == code)))
                for name, code in CLASS_CODES.items()
            },
            "classification_metrics": _classification_metrics(scan),
            "extreme_postures": {
                name: _posture_snapshot(
                    scan, int(selector(scan.column(column_name)))
                )
                for name, (column_name, selector) in extreme_columns.items()
            },
            "classification_thresholds": {
                "recommended_condition_max": RECOMMENDED_CONDITION_MAX,
                "recommended_sigma_min_m_per_rad": RECOMMENDED_SIGMA_MIN,
                "usable_condition_max_exclusive": USABLE_CONDITION_MAX,
                "usable_sigma_min_m_per_rad": USABLE_SIGMA_MIN,
            },
            "normalization": {
                "joint_torque_limit_nm": 1.0,
                "joint_speed_limit_rad_per_s": 1.0,
            },
            "class_code_mapping": {
                str(int(code)): name for name, code in CLASS_CODES.items()
            },
        }
    )
    return summary


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    scan = scan_workspace(args.resolution)

    write_csv(scan, args.output_dir / "workspace_scan.csv.gz")
    summary = _extended_summary(scan)
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    plot_workspace_pose(scan, args.output_dir / FIGURE_NAMES[0])
    plot_jacobian_singularity(scan, args.output_dir / FIGURE_NAMES[1])
    plot_upright_condition(scan, args.output_dir / FIGURE_NAMES[2])
    plot_force_speed_transmission(scan, args.output_dir / FIGURE_NAMES[3])
    plot_workspace_classification(scan, args.output_dir / FIGURE_NAMES[4])

    print(
        f"Scanned {summary['valid_samples']} valid postures at "
        f"{args.resolution}x{args.resolution}; invalid={summary['invalid_samples']}"
    )
    for name in FIGURE_NAMES:
        print(f"Saved {args.output_dir / name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
