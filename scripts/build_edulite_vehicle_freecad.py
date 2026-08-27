#!/usr/bin/env python3
"""Build the two-sided EduLite structural replacement and connection audit."""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import FreeCAD  # type: ignore[import-not-found]
import Import  # type: ignore[import-not-found]
import Part  # type: ignore[import-not-found]

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import analyze_mechanical_envelope_freecad as envelope  # noqa: E402
import build_edulite_single_leg_freecad as design  # noqa: E402

from src.edulite_joint import (  # noqa: E402
    ACTIVE_AXIS_Z_MM,
    BASE_M3_SCREW_LENGTH_MM,
    BASE_M3_THREAD_ENGAGEMENT_MM,
    DIRECT_OUTPUT_M4_SCREW_LENGTH_MM,
    EDULITE_OUTPUT_PATTERN_COUNT,
    EDULITE_OUTPUT_PATTERN_PCD_MM,
    EDULITE_REAR_PATTERN_CLOCK_DEG,
    EDULITE_REAR_PATTERN_COUNT,
    EDULITE_REAR_PATTERN_PCD_MM,
    HIP_Y_MM,
    MOTOR_M3_SCREW_LENGTH_MM,
    MOTOR_M3_THREAD_ENGAGEMENT_MM,
    OFFSET_OUTPUT_M4_SCREW_LENGTH_MM,
    REUSED_BASE_HOLES_XZ_MM,
    circular_pattern_centers,
)

M3_HEAD_DIAMETER_MM = 5.5
M3_HEAD_HEIGHT_MM = 3.0
M4_HEAD_DIAMETER_MM = 7.0
M4_HEAD_HEIGHT_MM = 4.0


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
    parser.add_argument("--edulite-step", type=Path, required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "artifacts" / "edulite_vehicle",
    )
    parser.add_argument("--pose-mm", type=float, default=90.0)
    return parser.parse_args()


def mirror_x(shape: Any) -> Any:
    result = shape.mirror(FreeCAD.Vector(), FreeCAD.Vector(1.0, 0.0, 0.0))
    if not result.isValid():
        raise ValueError("x-mirrored part is invalid")
    return result


def screw(
    shank_start: Any,
    direction: Any,
    shank_diameter_mm: float,
    shank_length_mm: float,
    head_diameter_mm: float,
    head_height_mm: float,
) -> Any:
    shank = Part.makeCylinder(
        shank_diameter_mm / 2.0,
        shank_length_mm,
        shank_start,
        direction,
    )
    head_start = shank_start - direction * head_height_mm
    head = Part.makeCylinder(
        head_diameter_mm / 2.0,
        head_height_mm,
        head_start,
        direction,
    )
    result = shank.fuse(head).removeSplitter()
    if not result.isValid() or len(result.Solids) != 1:
        raise ValueError("simplified screw is not one valid solid")
    return result


def output_screws(
    link: Any,
    pivot_yz: tuple[float, float],
    side: str,
    length_mm: float,
) -> dict[str, Any]:
    box = link.BoundBox
    if side == "left":
        outer_x = float(box.XMin)
        direction = FreeCAD.Vector(1.0, 0.0, 0.0)
    else:
        outer_x = float(box.XMax)
        direction = FreeCAD.Vector(-1.0, 0.0, 0.0)
    pivot_y, pivot_z = pivot_yz
    centers = circular_pattern_centers(
        EDULITE_OUTPUT_PATTERN_COUNT,
        EDULITE_OUTPUT_PATTERN_PCD_MM,
    )
    return {
        f"M4_output_{index + 1}": screw(
            FreeCAD.Vector(outer_x, pivot_y + offset_y, pivot_z + offset_z),
            direction,
            4.0,
            length_mm,
            M4_HEAD_DIAMETER_MM,
            M4_HEAD_HEIGHT_MM,
        )
        for index, (offset_y, offset_z) in enumerate(centers)
    }


def base_screws(side: str) -> dict[str, Any]:
    sign = -1.0 if side == "left" else 1.0
    return {
        f"M3_base_{index + 1}": screw(
            FreeCAD.Vector(sign * abs(x), 0.0, z),
            FreeCAD.Vector(0.0, 1.0, 0.0),
            3.0,
            BASE_M3_SCREW_LENGTH_MM,
            M3_HEAD_DIAMETER_MM,
            M3_HEAD_HEIGHT_MM,
        )
        for index, (x, z) in enumerate(REUSED_BASE_HOLES_XZ_MM)
    }


def rear_pattern_centers(axis_z_mm: float) -> tuple[tuple[float, float], ...]:
    offsets = circular_pattern_centers(
        EDULITE_REAR_PATTERN_COUNT,
        EDULITE_REAR_PATTERN_PCD_MM,
        EDULITE_REAR_PATTERN_CLOCK_DEG,
    )
    return tuple(
        (HIP_Y_MM + offset_y, axis_z_mm - offset_x)
        for offset_x, offset_y in offsets
    )


def rear_motor_screws(side: str) -> dict[str, Any]:
    if side == "left":
        start_x = design.BRACKET_REAR_FACE_X_MM + design.BRACKET_PLATE_THICKNESS_MM
        direction = FreeCAD.Vector(-1.0, 0.0, 0.0)
    else:
        start_x = -(design.BRACKET_REAR_FACE_X_MM + design.BRACKET_PLATE_THICKNESS_MM)
        direction = FreeCAD.Vector(1.0, 0.0, 0.0)
    screws = {}
    for axis_name, axis_z in zip(("lower", "upper"), ACTIVE_AXIS_Z_MM, strict=True):
        centers = rear_pattern_centers(axis_z)
        if side == "right":
            centers = tuple(
                (2.0 * HIP_Y_MM - center_y, center_z)
                for center_y, center_z in centers
            )
        for index, (center_y, center_z) in enumerate(centers):
            screws[f"M3_{axis_name}_rear_{index + 1}"] = screw(
                FreeCAD.Vector(start_x, center_y, center_z),
                direction,
                3.0,
                MOTOR_M3_SCREW_LENGTH_MM,
                M3_HEAD_DIAMETER_MM,
                M3_HEAD_HEIGHT_MM,
            )
    return screws


def nominal_motor(side: str, axis_z_mm: float) -> Any:
    if side == "left":
        start_x = -66.5
        direction = FreeCAD.Vector(1.0, 0.0, 0.0)
    else:
        start_x = 66.5
        direction = FreeCAD.Vector(-1.0, 0.0, 0.0)
    return Part.makeCylinder(
        23.0,
        44.0,
        FreeCAD.Vector(start_x, HIP_Y_MM, axis_z_mm),
        direction,
    )


def y_axis_centers(shape: Any, radius_mm: float) -> set[tuple[float, float]]:
    centers = set()
    for face in shape.Faces:
        surface = face.Surface
        if not all(hasattr(surface, name) for name in ("Radius", "Axis", "Center")):
            continue
        if abs(float(surface.Radius) - radius_mm) > 1e-7:
            continue
        axis = surface.Axis
        if abs(abs(float(axis.y)) - 1.0) > 1e-9:
            continue
        centers.add(
            (
                round(float(surface.Center.x), 6),
                round(float(surface.Center.z), 6),
            )
        )
    return centers


def maximum_center_error(
    expected: set[tuple[float, float]], observed: set[tuple[float, float]]
) -> float:
    if not expected or not observed or len(expected) != len(observed):
        return math.inf
    return max(
        min(math.dist(point, candidate) for candidate in observed)
        for point in expected
    )


def export_part(shape: Any, label: str, path: Path) -> None:
    document = FreeCAD.newDocument(f"export_{label}")
    feature = design.add_feature(document, label, shape)
    document.recompute()
    Import.export([feature], str(path))
    FreeCAD.closeDocument(document.Name)


def main() -> None:
    args = parse_args()
    design.require_hash(args.assembly_step, design.ORIGINAL_STEP_SHA256)
    design.require_hash(args.edulite_step, design.EDULITE_STEP_SHA256)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    features = envelope.load_features(args.assembly_step, "vehicle_source", 80)
    reference = envelope.recover_reference_geometry(features)
    edulite_features = envelope.load_features(args.edulite_step, "vehicle_el05", 18)
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
    source_groups["left_proximal_negative"].update(
        output_screws(
            rebuilt["大腿_EduLite"],
            reference["pivots_yz_mm"]["a"],
            "left",
            DIRECT_OUTPUT_M4_SCREW_LENGTH_MM,
        )
    )
    source_groups["left_proximal_positive"].update(
        output_screws(
            rebuilt["大腿001_EduLite"],
            reference["pivots_yz_mm"]["e"],
            "left",
            OFFSET_OUTPUT_M4_SCREW_LENGTH_MM,
        )
    )
    source_groups["right_proximal_positive"].update(
        output_screws(
            rebuilt["大腿002_EduLite"],
            reference["pivots_yz_mm"]["e"],
            "right",
            DIRECT_OUTPUT_M4_SCREW_LENGTH_MM,
        )
    )
    source_groups["right_proximal_negative"].update(
        output_screws(
            rebuilt["大腿003_EduLite"],
            reference["pivots_yz_mm"]["a"],
            "right",
            OFFSET_OUTPUT_M4_SCREW_LENGTH_MM,
        )
    )
    placed = envelope.place_groups(source_groups, reference, args.pose_mm)

    left_bracket = design.build_bracket()
    # The right EL05 is turned through -90 deg about y, which reflects its rear
    # hole clock in global y.  Mirror the bracket body in x, but cut that
    # reflected four-hole pattern explicitly.
    right_bracket = mirror_x(design.build_bracket(rear_pattern_mirror_y=True))
    if left_bracket.BoundBox.XMax >= 0.0 or right_bracket.BoundBox.XMin <= 0.0:
        raise ValueError("left and right brackets are not on opposite chassis sides")
    brackets = {"left": left_bracket, "right": right_bracket}
    bracket_feet = {
        "left": design.build_bracket_foot(),
        "right": mirror_x(design.build_bracket_foot()),
    }
    fixed_screws = {
        **{f"left_{key}": value for key, value in base_screws("left").items()},
        **{f"right_{key}": value for key, value in base_screws("right").items()},
        **{
            f"left_{key}": value
            for key, value in rear_motor_screws("left").items()
        },
        **{
            f"right_{key}": value
            for key, value in rear_motor_screws("right").items()
        },
    }

    target = envelope.target_geometry(
        args.pose_mm, reference["pivots_yz_mm"]["a"][0]
    )
    motor_to_link = {
        "edulite_left_negative": (
            placed["left_proximal_negative"]["大腿_EduLite"],
            target["a"],
        ),
        "edulite_left_positive": (
            Part.makeCompound(
                [
                    placed["left_proximal_positive"]["大腿垫高_EduLite"],
                    placed["left_proximal_positive"]["大腿001_EduLite"],
                ]
            ),
            target["e"],
        ),
        "edulite_right_positive": (
            placed["right_proximal_positive"]["大腿002_EduLite"],
            target["e"],
        ),
        "edulite_right_negative": (
            Part.makeCompound(
                [
                    placed["right_proximal_negative"]["大腿垫高001_EduLite"],
                    placed["right_proximal_negative"]["大腿003_EduLite"],
                ]
            ),
            target["a"],
        ),
    }
    rotor_alignment = {}
    output_common = {}
    for group_name, (link_stack, pivot) in motor_to_link.items():
        rotor_alignment[group_name] = design.align_output_rotor(
            motor_groups[group_name], link_stack, pivot
        )
        output_common[group_name] = design.common_volume(
            Part.makeCompound(list(motor_groups[group_name].values())),
            link_stack,
        )

    bracket_rear_centers = {
        "left": envelope.yz_cylinder_centers(left_bracket, 1.7),
        "right": envelope.yz_cylinder_centers(right_bracket, 1.7),
    }
    motor_rear_errors = {}
    for group_name, group in motor_groups.items():
        side = "left" if "left" in group_name else "right"
        axis_z = (
            ACTIVE_AXIS_Z_MM[0]
            if "negative" in group_name
            else ACTIVE_AXIS_Z_MM[1]
        )
        observed = envelope.yz_cylinder_centers(
            group["1000_1-541385_1_1"], 1.6
        )
        expected = {
            point
            for point in bracket_rear_centers[side]
            if abs(point[1] - axis_z) < EDULITE_REAR_PATTERN_PCD_MM / 2.0 + 1.0
        }
        motor_rear_errors[group_name] = maximum_center_error(expected, observed)

    motor_foot_clearances = {
        group_name: min(
            float(bracket_feet[side].distToShape(shape)[0])
            for shape in group.values()
        )
        for group_name, group in motor_groups.items()
        for side in ("left" if "left" in group_name else "right",)
    }
    motor_rear_face_x_errors = {
        group_name: abs(
            (
                max(float(shape.BoundBox.XMax) for shape in group.values())
                if side == "left"
                else min(float(shape.BoundBox.XMin) for shape in group.values())
            )
            - (
                design.BRACKET_REAR_FACE_X_MM
                if side == "left"
                else -design.BRACKET_REAR_FACE_X_MM
            )
        )
        for group_name, group in motor_groups.items()
        for side in ("left" if "left" in group_name else "right",)
    }

    output_paths = {
        "left_bracket": args.output_dir / "edulite_left_shared_bracket.step",
        "right_bracket": args.output_dir / "edulite_right_shared_bracket.step",
        "vehicle": args.output_dir
        / f"edulite_vehicle_structural_{args.pose_mm:g}mm.step",
    }
    export_part(
        left_bracket,
        "EduLite_left_shared_bracket",
        output_paths["left_bracket"],
    )
    export_part(
        right_bracket, "EduLite_right_shared_bracket", output_paths["right_bracket"]
    )

    assembly = FreeCAD.newDocument("edulite_vehicle_structural")
    objects = [design.add_feature(assembly, "Original_baseplate", features["底板"])]
    for side, bracket in brackets.items():
        objects.append(
            design.add_feature(assembly, f"EduLite_{side}_shared_bracket", bracket)
        )
        for axis_name, axis_z in zip(("lower", "upper"), ACTIVE_AXIS_Z_MM, strict=True):
            objects.append(
                design.add_feature(
                    assembly,
                    f"EduLite_{side}_{axis_name}_nominal_body",
                    nominal_motor(side, axis_z),
                )
            )
    for name, shape in fixed_screws.items():
        objects.append(design.add_feature(assembly, name, shape))
    for group_name, entities in placed.items():
        for label, shape in entities.items():
            objects.append(
                design.add_feature(assembly, f"{group_name}__{label}", shape)
            )
    assembly.recompute()
    Import.export(objects, str(output_paths["vehicle"]))

    reimport_document = FreeCAD.newDocument("edulite_vehicle_reimport")
    Import.insert(str(output_paths["vehicle"]), reimport_document.Name)
    reimport_features = [
        obj
        for obj in reimport_document.Objects
        if obj.TypeId == "Part::Feature"
    ]
    reimport_invalid = [
        obj.Label for obj in reimport_features if not obj.Shape.isValid()
    ]
    reimport_non_single = [
        obj.Label for obj in reimport_features if len(obj.Shape.Solids) != 1
    ]

    base_centers = y_axis_centers(features["底板"], 1.6)
    expected_left = {(x, z) for x, z in REUSED_BASE_HOLES_XZ_MM}
    expected_right = {(-x, z) for x, z in REUSED_BASE_HOLES_XZ_MM}
    base_errors = {
        "left_mm": maximum_center_error(expected_left, expected_left & base_centers),
        "right_mm": maximum_center_error(expected_right, expected_right & base_centers),
    }
    custom_shapes = [
        *rebuilt.values(),
        left_bracket,
        right_bracket,
        *fixed_screws.values(),
    ]
    audit = {
        "schema_version": 1,
        "status": "STRUCTURAL_CONNECTION_BASELINE",
        "scope": (
            "baseplate, both original five-bar legs and wheels, four EduLite "
            "active joints, two shared brackets and explicit simplified fasteners"
        ),
        "pose_mm": args.pose_mm,
        "part_quality": {
            "all_custom_parts_valid": all(shape.isValid() for shape in custom_shapes),
            "all_custom_parts_single_solid": all(
                len(shape.Solids) == 1 for shape in custom_shapes
            ),
            "export_reimport": {
                "feature_count": len(reimport_features),
                "solid_count": sum(
                    len(obj.Shape.Solids) for obj in reimport_features
                ),
                "invalid_features": reimport_invalid,
                "non_single_solid_features": reimport_non_single,
            },
            "bracket_x_bounds_mm": {
                "left": [
                    float(left_bracket.BoundBox.XMin),
                    float(left_bracket.BoundBox.XMax),
                ],
                "right": [
                    float(right_bracket.BoundBox.XMin),
                    float(right_bracket.BoundBox.XMax),
                ],
            },
        },
        "connections": {
            "bracket_to_baseplate": {
                "method": "16 x M3x10 from below into tapped bracket feet",
                "hole_center_max_error_mm": base_errors,
                "thread_engagement_mm": BASE_M3_THREAD_ENGAGEMENT_MM,
                "state": "DEFINED",
            },
            "motor_stator_to_bracket": {
                "method": "4 x M3x10 per motor through bracket into EL05 rear M3",
                "fastener_count": 16,
                "thread_engagement_mm": MOTOR_M3_THREAD_ENGAGEMENT_MM,
                "hole_center_max_error_mm": motor_rear_errors,
                "mating_face_x_max_error_mm": motor_rear_face_x_errors,
                "state": "DEFINED",
            },
            "motor_output_to_direct_link": {
                "method": "6 x M4x8 plus the EL05 three output dowels",
                "motor_count": 2,
                "thread_engagement_mm": 3.0,
                "state": "DEFINED",
            },
            "motor_output_to_offset_stack": {
                "method": "6 x M4x14 clamp spacer and link; EL05 dowels locate spacer",
                "motor_count": 2,
                "thread_engagement_mm": 3.0,
                "state": "DEFINED",
            },
            "output_interface_common_volume_mm3": output_common,
            "output_rotor_alignment_deg": rotor_alignment,
            "new_part_connection_graph_reaches_baseplate": True,
        },
        "assembly_readiness": {
            "motor_body_to_foot_minimum_clearance_mm": motor_foot_clearances,
            "base_tapped_hole_minimum_edge_ligament_mm": 3.0,
            "assembly_sequence": [
                "fasten each EduLite rear face to its side bracket",
                "fasten each bracket to the original baseplate from below",
                "fasten the direct and offset active-link stacks from outboard",
                "connect the unchanged distal links and wheel assemblies",
            ],
            "tool_access_basis": (
                "rear screws are installed before the bracket reaches the baseplate; "
                "base screws remain accessible from below; output screws remain "
                "accessible from outboard"
            ),
            "output_screw_bottoming_margin_mm": {
                "M4x8_direct_nominal": 0.0,
                "M4x14_offset_nominal": 0.0,
            },
            "output_screw_release_gate": (
                "measure actual under-head screw length and usable EL05 thread depth "
                "on first hardware; add a thin shim washer if needed"
            ),
            "state": "GEOMETRICALLY_ASSEMBLABLE_NOT_PRODUCTION_RELEASED",
        },
        "fastener_bill": {
            "M3x10_base": 16,
            "M3x10_motor_rear": 16,
            "M4x8_direct_output": 12,
            "M4x14_offset_output": 12,
        },
        "deliberately_omitted_for_relayout": [
            "battery and battery bracket/standoffs",
            "NanoPi and NanoPi bracket",
            "main controller plate and standoffs",
            "camera",
            "the four original 4010 motors, brackets and separate leg driver boards",
        ],
        "exports": {
            name: {
                "path": str(path.relative_to(PROJECT_ROOT)),
                "sha256": design.sha256(path),
                "bytes": path.stat().st_size,
            }
            for name, path in output_paths.items()
        },
        "evidence_limit": (
            "The committed vehicle STEP uses nominal 46 x 44 mm motor bodies. "
            "All interface alignment and collision evidence uses the hash-qualified "
            "official EL05 STEP; strength is not claimed."
        ),
    }
    audit_path = args.output_dir / "connection_audit.json"
    audit_path.write_text(
        json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(f"wrote {output_paths['vehicle']}")
    print(f"wrote {audit_path}")


if __name__ == "__main__":
    main()
