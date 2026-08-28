#!/usr/bin/env python3
"""Render the three verified leg-length poses from the MuJoCo model."""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mujoco_robot import set_leg_length  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", type=Path, default=PROJECT_ROOT / "mujoco" / "robot.xml"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT
        / "artifacts"
        / "mujoco"
        / "visual_kinematic_checkpoint.png",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=540, width=720)
    figure, axes = plt.subplots(1, 3, figsize=(14.4, 4.6))
    for axis, l0_mm in zip(axes, (70.0, 90.0, 120.0), strict=True):
        closure_error_mm = set_leg_length(model, data, l0_mm)
        renderer.update_scene(data, camera="overview")
        axis.imshow(renderer.render())
        axis.set_title(f"l0 = {l0_mm:g} mm\nclosure {closure_error_mm:.2e} mm")
        axis.axis("off")
    renderer.close()
    figure.suptitle(
        "Real CAD visuals · constrained MuJoCo five-bar mechanism", y=0.98
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.88))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180, bbox_inches="tight")
    plt.close(figure)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
