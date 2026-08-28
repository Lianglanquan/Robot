#!/usr/bin/env python3
"""Build the MJCF vehicle model from the committed visual assets."""

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mujoco_dynamics import build_dynamic_model_xml  # noqa: E402
from src.mujoco_robot import build_model_xml  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "mujoco" / "robot.xml",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    xml = build_model_xml(args.output.parent / "assets")
    args.output.write_text(xml, encoding="utf-8")
    print(f"wrote {args.output}")
    for mass in (2.0, 2.3, 2.5):
        mass_label = str(mass).replace(".", "p")
        dynamic_path = args.output.parent / f"robot_dynamic_{mass_label}kg.xml"
        dynamic_path.write_text(
            build_dynamic_model_xml(args.output.parent / "assets", mass),
            encoding="utf-8",
        )
        print(f"wrote {dynamic_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
