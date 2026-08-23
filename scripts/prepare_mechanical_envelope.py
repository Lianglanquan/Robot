#!/usr/bin/env python3
import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mechanical_envelope import CAD_POSE_COLUMNS, cad_pose_schedule  # noqa: E402
from src.stroke import vertical_stroke_limits  # noqa: E402

UPSTREAM_COMMIT = "e2444395dd3a76c20b0683fbb1e123c21186a502"
UPSTREAM_ASSEMBLY = "solidworks/总装.SLDASM"
UPSTREAM_ASSEMBLY_SHA256 = (
    "24fdfd78dad6243235ad09fe32328f37442979fbae6191bfd7d7b444b13a299d"
)
EDULITE_COMMIT = "6ad12f50006273b7ea4eea88980f927d97c22f0d"
EDULITE_STEP = "产品资料/EL05/el05.stp"
EDULITE_STEP_SHA256 = (
    "3c970be58644420e97a332473e9b1d806601125c9e95c95743ddeb0c99e27ee3"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Prepare the exact Phase 3 posture schedule for CAD validation"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "artifacts" / "mechanical_envelope",
    )
    parser.add_argument(
        "--upstream-dir",
        type=Path,
        help="Optional checkout at the fixed upstream commit; validates 总装.SLDASM",
    )
    parser.add_argument(
        "--edulite-step",
        type=Path,
        help="Optional official EL05 STEP path; validates its immutable file hash",
    )
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify_file(path: Path, expected_sha256: str) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    observed = sha256(path)
    if observed != expected_sha256:
        raise ValueError(
            f"unexpected SHA-256 for {path}: {observed}; expected {expected_sha256}"
        )
    return {"verified": True, "bytes": path.stat().st_size, "sha256": observed}


def write_schedule(output: Path) -> None:
    poses = cad_pose_schedule()
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(CAD_POSE_COLUMNS)
        writer.writerows(pose.row() for pose in poses)


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_schedule(args.output_dir / "cad_postures.csv")

    lower, upper = vertical_stroke_limits()
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "purpose": "immutable inputs and pose schedule for real mechanical validation",
        "mathematical_vertical_stroke_mm": {
            "lower_open": lower * 1000.0,
            "upper_open": upper * 1000.0,
        },
        "scan": {
            "step_mm": 1,
            "short_end_exploration_mm": [47, 69],
            "baseline_mm": [70, 120],
            "long_end_exploration_mm": [121, 152],
            "pose_count": 106,
        },
        "original_cad": {
            "repository": "https://github.com/Skythinker616/foc-wheel-legged-robot",
            "commit": UPSTREAM_COMMIT,
            "path": UPSTREAM_ASSEMBLY,
            "sha256": UPSTREAM_ASSEMBLY_SHA256,
            "verification": None,
        },
        "edulite_05": {
            "repository": "https://github.com/RobStride/Product_Information",
            "commit": EDULITE_COMMIT,
            "path": EDULITE_STEP,
            "sha256": EDULITE_STEP_SHA256,
            "step_standard": "ISO 10303-203 CONFIG_CONTROL_DESIGN",
            "solid_count": 18,
            "stepcontrol_compound_bbox_mm": [
                46.0000002,
                46.0000002,
                47.0000001,
            ],
            "verification": None,
        },
    }

    if args.upstream_dir is not None:
        assembly = args.upstream_dir / UPSTREAM_ASSEMBLY
        manifest["original_cad"]["verification"] = verify_file(
            assembly, UPSTREAM_ASSEMBLY_SHA256
        )
    if args.edulite_step is not None:
        manifest["edulite_05"]["verification"] = verify_file(
            args.edulite_step, EDULITE_STEP_SHA256
        )

    manifest_path = args.output_dir / "source_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"wrote {args.output_dir / 'cad_postures.csv'}")
    print(f"wrote {manifest_path}")


if __name__ == "__main__":
    main()
