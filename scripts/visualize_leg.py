#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.kinematics import forward_kinematics, joint_positions  # noqa: E402
from src.parameters import DEFAULT_PARAMETERS  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot the original five-bar leg")
    parser.add_argument("--phi1", type=float, default=-2.2, help="phi1 in radians")
    parser.add_argument("--phi4", type=float, default=-0.9, help="phi4 in radians")
    parser.add_argument("--output", type=Path, help="save PNG instead of showing it")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.output:
        import matplotlib

        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    points = joint_positions(args.phi1, args.phi4)
    pose = forward_kinematics(args.phi1, args.phi4)
    origin = np.array([DEFAULT_PARAMETERS.l5 / 2.0, 0.0])

    figure, axis = plt.subplots(figsize=(7.2, 7.2))
    links = [
        (points.a, points.b, "AB (l1)", "#2f5597"),
        (points.b, points.c, "BC (l2)", "#d97706"),
        (points.c, points.d, "CD (l3)", "#0f766e"),
        (points.d, points.e, "DE (l4)", "#b91c1c"),
    ]
    for start, end, label, color in links:
        axis.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color=color,
            linewidth=3,
            marker="o",
            markersize=5,
            label=label,
        )

    axis.plot(
        [points.a[0], points.e[0]],
        [points.a[1], points.e[1]],
        color="#4b5563",
        linewidth=2,
        linestyle=":",
        label="AE (l5, fixed base)",
    )
    axis.plot(
        [origin[0], points.c[0]],
        [origin[1], points.c[1]],
        color="#15803d",
        linewidth=2.5,
        linestyle="--",
        label="OC (virtual leg)",
    )

    axis.scatter(
        [points.a[0], points.e[0]],
        [points.a[1], points.e[1]],
        s=100,
        color="#111827",
        zorder=5,
        label="Active joints",
    )
    axis.scatter(
        points.c[0], points.c[1], s=130, color="#facc15", edgecolor="#111827", zorder=6
    )
    axis.scatter(origin[0], origin[1], s=55, color="#15803d", zorder=6)

    for name, point in (
        ("A", points.a),
        ("B", points.b),
        ("C (foot)", points.c),
        ("D", points.d),
        ("E", points.e),
        ("O", origin),
    ):
        axis.annotate(name, point, xytext=(7, 7), textcoords="offset points")

    all_points = np.vstack([points.a, points.b, points.c, points.d, points.e])
    center = np.mean(all_points, axis=0)
    span = max(float(np.ptp(all_points[:, 0])), float(np.ptp(all_points[:, 1])), 0.12)
    half_span = span * 0.65
    axis.set_xlim(center[0] - half_span, center[0] + half_span)
    axis.set_ylim(center[1] - half_span, center[1] + half_span)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title("Five-bar leg geometry")
    axis.grid(True, linestyle=":", alpha=0.55)
    axis.legend(loc="best", fontsize=8)
    axis.text(
        0.02,
        0.98,
        f"phi1 = {args.phi1:.4f} rad\n"
        f"phi4 = {args.phi4:.4f} rad\n"
        f"l0 = {pose.l0:.6f} m\n"
        f"phi0 = {pose.phi0:.6f} rad",
        transform=axis.transAxes,
        va="top",
        fontsize=9,
        bbox={"facecolor": "white", "edgecolor": "#9ca3af", "alpha": 0.9},
    )
    figure.tight_layout()

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.output, dpi=160)
        print(f"Saved {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
