#!/usr/bin/env python3
"""Stage snap-visible CAD inputs, export meshes, and optimize them for MuJoCo."""

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_EDULITE_STEP = (
    PROJECT_ROOT
    / ".worktrees"
    / "edulite-reference"
    / "Product Literature"
    / "EL05"
    / "el05.stp"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--edulite-step", type=Path, default=DEFAULT_EDULITE_STEP)
    parser.add_argument("--pose-mm", type=float, default=90.0)
    parser.add_argument("--deflection-mm", type=float, default=0.30)
    return parser.parse_args()


def copy_inputs(stage: Path, edulite_step: Path) -> None:
    ignore = shutil.ignore_patterns("__pycache__", "*.pyc")
    shutil.copytree(PROJECT_ROOT / "scripts", stage / "scripts", ignore=ignore)
    shutil.copytree(PROJECT_ROOT / "src", stage / "src", ignore=ignore)
    cad_dir = stage / "reference" / "cad"
    cad_dir.mkdir(parents=True)
    shutil.copy2(
        PROJECT_ROOT
        / "reference"
        / "cad"
        / "foc-wheel-legged-robot_full_assembly_AP214.step",
        cad_dir,
    )
    product_dir = (
        stage
        / ".worktrees"
        / "edulite-reference"
        / "Product Literature"
        / "EL05"
    )
    product_dir.mkdir(parents=True)
    shutil.copy2(edulite_step, product_dir / "el05.stp")


def main() -> int:
    args = parse_args()
    freecad = shutil.which("freecad.cmd")
    if freecad is None:
        raise FileNotFoundError("freecad.cmd is required")
    if not args.edulite_step.exists():
        raise FileNotFoundError(args.edulite_step)
    stage = Path(tempfile.mkdtemp(prefix="wheel-leg-freecad-", dir=Path.home()))
    try:
        copy_inputs(stage, args.edulite_step)
        macro = stage / "scripts" / "export_mujoco_assets_freecad.FCMacro"
        subprocess.run(
            [
                freecad,
                str(macro),
                "--",
                "--pose-mm",
                f"{args.pose_mm:g}",
                "--deflection-mm",
                f"{args.deflection_mm:g}",
            ],
            cwd=stage,
            check=True,
        )
        subprocess.run(
            [
                sys.executable,
                str(PROJECT_ROOT / "scripts" / "optimize_mujoco_meshes.py"),
                str(stage / "mujoco" / "assets"),
                "--output-dir",
                str(PROJECT_ROOT / "mujoco" / "assets"),
            ],
            cwd=PROJECT_ROOT,
            check=True,
        )
    finally:
        shutil.rmtree(stage)
    subprocess.run(
        [sys.executable, str(PROJECT_ROOT / "scripts" / "build_mujoco_model.py")],
        cwd=PROJECT_ROOT,
        check=True,
    )
    print("MuJoCo CAD assets and MJCF are ready")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
