#!/usr/bin/env python3
"""Export the qualified EduLite vehicle geometry as MuJoCo visual meshes.

This script runs inside FreeCAD.  It rebuilds the same 90 mm structural
assembly used by the mechanical-envelope checkpoint, then writes each moving
rigid group in its own local joint frame.  The coordinate conversion is:

    MuJoCo (x, y, z) = CAD (x, z, y)

so the CAD baseplate becomes horizontal and MuJoCo z points upward.
"""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import Part  # type: ignore[import-not-found]

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import analyze_mechanical_envelope_freecad as envelope  # noqa: E402
import build_edulite_single_leg_freecad as design  # noqa: E402
import build_edulite_vehicle_freecad as vehicle  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--assembly-step",
        type=Path,
        default=PROJECT_ROOT
        / "reference"
        / "cad"
        / "foc-wheel-legged-robot_full_assembly_AP214.step",
    )
    parser.add_argument(
        "--edulite-step",
        type=Path,
        default=PROJECT_ROOT
        / ".worktrees"
        / "edulite-reference"
        / "Product Literature"
        / "EL05"
        / "el05.stp",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "mujoco" / "assets",
    )
    parser.add_argument("--pose-mm", type=float, default=90.0)
    parser.add_argument("--deflection-mm", type=float, default=0.30)
    return parser.parse_args()


def cad_to_mujoco(point: Any) -> tuple[float, float, float]:
    return (float(point.x) / 1000.0, float(point.z) / 1000.0, float(point.y) / 1000.0)


def local_vertex(
    point: Any,
    origin_cad_mm: tuple[float, float, float],
    frame_angle_mj_deg: float,
) -> tuple[float, float, float]:
    world = cad_to_mujoco(point)
    origin = (
        origin_cad_mm[0] / 1000.0,
        origin_cad_mm[2] / 1000.0,
        origin_cad_mm[1] / 1000.0,
    )
    relative = tuple(world[index] - origin[index] for index in range(3))
    angle = math.radians(-frame_angle_mj_deg)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        relative[0],
        cosine * relative[1] - sine * relative[2],
        sine * relative[1] + cosine * relative[2],
    )


def write_obj(
    path: Path,
    shapes: list[Any],
    origin_cad_mm: tuple[float, float, float],
    frame_angle_mj_deg: float,
    deflection_mm: float,
) -> dict[str, int]:
    vertices: list[tuple[float, float, float]] = []
    triangles: list[tuple[int, int, int]] = []
    for shape in shapes:
        points, facets = shape.tessellate(deflection_mm)
        offset = len(vertices)
        vertices.extend(
            local_vertex(point, origin_cad_mm, frame_angle_mj_deg)
            for point in points
        )
        # Swapping CAD y/z changes handedness, so reverse each triangle winding.
        triangles.extend(
            (offset + first, offset + third, offset + second)
            for first, second, third in facets
        )
    if not vertices or not triangles:
        raise ValueError(f"no tessellated geometry for {path.name}")
    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write(f"o {path.stem}\n")
        for x, y, z in vertices:
            stream.write(f"v {x:.9g} {y:.9g} {z:.9g}\n")
        for first, second, third in triangles:
            stream.write(f"f {first + 1} {second + 1} {third + 1}\n")
    return {"vertices": len(vertices), "triangles": len(triangles)}


def compound_entities(entities: dict[str, Any]) -> Any:
    return Part.makeCompound(list(entities.values()))


def main() -> None:
    args = parse_args()
    if args.deflection_mm <= 0.0:
        raise ValueError("deflection must be positive")
    design.require_hash(args.assembly_step, design.ORIGINAL_STEP_SHA256)
    design.require_hash(args.edulite_step, design.EDULITE_STEP_SHA256)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    features = envelope.load_features(args.assembly_step, "mujoco_source", 80)
    reference = envelope.recover_reference_geometry(features)
    edulite_features = envelope.load_features(args.edulite_step, "mujoco_el05", 18)
    motor_groups = envelope.place_edulite_groups(
        edulite_features, design.OUTPUT_FACE_LOCAL_MM, 0.0
    )

    rebuilt = {
        "大腿_EduLite": design.rebuild_output_hub(
            features["大腿"], reference["pivots_yz_mm"]["a"]
        ),
        "大腿垫高_EduLite": design.rebuild_output_hub(
            features["大腿垫高"], reference["pivots_yz_mm"]["e"]
        ),
        "大腿001_EduLite": design.rebuild_output_hub(
            features["大腿001"], reference["pivots_yz_mm"]["e"]
        ),
        "大腿002_EduLite": design.rebuild_output_hub(
            features["大腿002"], reference["pivots_yz_mm"]["e"]
        ),
        "大腿垫高001_EduLite": design.rebuild_output_hub(
            features["大腿垫高001"], reference["pivots_yz_mm"]["a"]
        ),
        "大腿003_EduLite": design.rebuild_output_hub(
            features["大腿003"], reference["pivots_yz_mm"]["a"]
        ),
    }
    source_groups = {
        name: {label: features[label] for label in labels}
        for name, labels in envelope.MOVING_LABELS.items()
    }
    source_groups.update(
        {
            "left_proximal_negative": {"大腿_EduLite": rebuilt["大腿_EduLite"]},
            "left_proximal_positive": {
                "大腿垫高_EduLite": rebuilt["大腿垫高_EduLite"],
                "大腿001_EduLite": rebuilt["大腿001_EduLite"],
            },
            "right_proximal_positive": {
                "大腿002_EduLite": rebuilt["大腿002_EduLite"]
            },
            "right_proximal_negative": {
                "大腿垫高001_EduLite": rebuilt["大腿垫高001_EduLite"],
                "大腿003_EduLite": rebuilt["大腿003_EduLite"],
            },
        }
    )
    placed = envelope.place_groups(source_groups, reference, args.pose_mm)
    target = envelope.target_geometry(
        args.pose_mm, reference["pivots_yz_mm"]["a"][0]
    )
    pose = target["pose"]

    link_for_motor = {
        "edulite_left_negative": compound_entities(
            placed["left_proximal_negative"]
        ),
        "edulite_left_positive": compound_entities(
            placed["left_proximal_positive"]
        ),
        "edulite_right_negative": compound_entities(
            placed["right_proximal_negative"]
        ),
        "edulite_right_positive": compound_entities(
            placed["right_proximal_positive"]
        ),
    }
    motor_pivots = {
        "edulite_left_negative": target["a"],
        "edulite_left_positive": target["e"],
        "edulite_right_negative": target["a"],
        "edulite_right_positive": target["e"],
    }
    for name, shapes in motor_groups.items():
        design.align_output_rotor(shapes, link_for_motor[name], motor_pivots[name])

    frames = {
        "identity": ((0.0, 0.0, 0.0), 0.0),
        "proximal_negative": (
            (0.0, target["a"][0], target["a"][1]),
            -pose.phi1_deg,
        ),
        "proximal_positive": (
            (0.0, target["e"][0], target["e"][1]),
            -pose.phi4_deg,
        ),
        "distal_negative": (
            (0.0, target["b"][0], target["b"][1]),
            -pose.theta_bc_deg,
        ),
        "distal_positive": (
            (0.0, target["d"][0], target["d"][1]),
            -pose.theta_dc_deg,
        ),
        "wheel": ((0.0, target["c"][0], target["c"][1]), 0.0),
    }

    left_bracket = design.build_bracket()
    right_bracket = vehicle.mirror_x(
        design.build_bracket(rear_pattern_mirror_y=True)
    )
    fixed_screws = {
        **{
            f"left_{name}": shape
            for name, shape in vehicle.base_screws("left").items()
        },
        **{
            f"right_{name}": shape
            for name, shape in vehicle.base_screws("right").items()
        },
        **{
            f"left_{name}": shape
            for name, shape in vehicle.rear_motor_screws("left").items()
        },
        **{
            f"right_{name}": shape
            for name, shape in vehicle.rear_motor_screws("right").items()
        },
    }

    mesh_specs: dict[str, tuple[list[Any], str]] = {
        "baseplate": ([features["底板"]], "identity"),
        "left_bracket": ([left_bracket], "identity"),
        "right_bracket": ([right_bracket], "identity"),
        "fixed_fasteners": (list(fixed_screws.values()), "identity"),
    }
    for motor_name, shapes in motor_groups.items():
        stator = [
            shape
            for label, shape in shapes.items()
            if label not in design.OUTPUT_ROTOR_LABELS
        ]
        rotor = [
            shape
            for label, shape in shapes.items()
            if label in design.OUTPUT_ROTOR_LABELS
        ]
        link_kind = "negative" if motor_name.endswith("negative") else "positive"
        mesh_specs[f"{motor_name}_stator"] = (stator, "identity")
        mesh_specs[f"{motor_name}_rotor"] = (rotor, f"proximal_{link_kind}")

    for side in ("left", "right"):
        for kind in ("negative", "positive"):
            proximal_name = f"{side}_proximal_{kind}"
            link_shapes = list(placed[proximal_name].values())
            output_length = (
                vehicle.DIRECT_OUTPUT_M4_SCREW_LENGTH_MM
                if (side, kind) in (("left", "negative"), ("right", "positive"))
                else vehicle.OFFSET_OUTPUT_M4_SCREW_LENGTH_MM
            )
            link_shapes.extend(
                vehicle.output_screws(
                    compound_entities(placed[proximal_name]),
                    target["a"] if kind == "negative" else target["e"],
                    side,
                    output_length,
                ).values()
            )
            mesh_specs[proximal_name] = (link_shapes, f"proximal_{kind}")
            distal_name = f"{side}_distal_{kind}"
            mesh_specs[distal_name] = (
                list(placed[distal_name].values()),
                f"distal_{kind}",
            )
        wheel_name = f"{side}_wheel"
        mesh_specs[wheel_name] = (list(placed[wheel_name].values()), "wheel")

    manifest: dict[str, Any] = {
        "schema_version": 1,
        "status": "CAD_VISUAL_ASSETS_AT_REFERENCE_POSE",
        "reference_pose_mm": args.pose_mm,
        "coordinate_map": "mujoco_xyz_m = cad_xzy_mm / 1000",
        "deflection_mm": args.deflection_mm,
        "source": {
            "assembly_step": str(args.assembly_step.relative_to(PROJECT_ROOT)),
            "assembly_sha256": design.sha256(args.assembly_step),
            "edulite_step": str(args.edulite_step.relative_to(PROJECT_ROOT)),
            "edulite_sha256": design.sha256(args.edulite_step),
        },
        "frames": {
            name: {"origin_cad_mm": origin, "angle_mj_deg": angle}
            for name, (origin, angle) in frames.items()
        },
        "meshes": {},
    }
    for name, (mesh_shapes, frame_name) in mesh_specs.items():
        origin, angle = frames[frame_name]
        output_path = args.output_dir / f"{name}.obj"
        counts = write_obj(
            output_path, mesh_shapes, origin, angle, args.deflection_mm
        )
        manifest["meshes"][name] = {
            "file": output_path.name,
            "frame": frame_name,
            **counts,
        }
        print(f"{name}: {counts['triangles']} triangles")

    (args.output_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"wrote {args.output_dir}")


if __name__ == "__main__":
    main()
