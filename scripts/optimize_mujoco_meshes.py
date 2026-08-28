#!/usr/bin/env python3
"""Reduce CAD visual meshes to real-time Viewer assets."""

import argparse
import json
from pathlib import Path
from typing import Any

import trimesh

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "mujoco" / "assets"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_dir", type=Path)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args()


def target_faces(name: str, original: int) -> int:
    if "stator" in name:
        target = 20_000
    elif "rotor" in name or "fasteners" in name:
        target = 8_000
    elif "wheel" in name:
        target = 20_000
    elif "distal" in name:
        target = 12_000
    elif "proximal" in name:
        target = 8_000
    else:
        target = 10_000
    return min(original, target)


def optimize_mesh(source: Path, target: int) -> trimesh.Trimesh:
    loaded = trimesh.load_mesh(source, process=False)
    if not isinstance(loaded, trimesh.Trimesh):
        raise ValueError(f"expected one mesh in {source}")
    mesh = loaded.copy()
    if len(mesh.faces) > target:
        mesh = mesh.simplify_quadric_decimation(face_count=target)
    mesh.remove_unreferenced_vertices()
    mesh.remove_infinite_values()
    if len(mesh.vertices) == 0 or len(mesh.faces) == 0:
        raise ValueError(f"optimization removed all geometry from {source}")
    return mesh


def main() -> int:
    args = parse_args()
    raw_manifest_path = args.raw_dir / "manifest.json"
    raw_manifest: dict[str, Any] = json.loads(
        raw_manifest_path.read_text(encoding="utf-8")
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    optimized_manifest = {
        **raw_manifest,
        "status": "REALTIME_CAD_VISUAL_ASSETS",
        "raw_asset_source": "temporary FreeCAD tessellation (not committed)",
        "meshes": {},
    }
    for name, row in raw_manifest["meshes"].items():
        source = args.raw_dir / row["file"]
        target = target_faces(name, int(row["triangles"]))
        mesh = optimize_mesh(source, target)
        output = args.output_dir / row["file"]
        mesh.export(output, file_type="obj", include_color=False)
        output.write_text(
            output.read_text(encoding="utf-8").rstrip() + "\n",
            encoding="utf-8",
        )
        optimized_manifest["meshes"][name] = {
            **row,
            "raw_vertices": row["vertices"],
            "raw_triangles": row["triangles"],
            "vertices": int(len(mesh.vertices)),
            "triangles": int(len(mesh.faces)),
        }
        print(f"{name}: {row['triangles']} -> {len(mesh.faces)} triangles")
    (args.output_dir / "manifest.json").write_text(
        json.dumps(optimized_manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
