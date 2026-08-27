#!/usr/bin/env python3
"""Build the first minimal-change EduLite single-leg interface CAD."""

import argparse
import hashlib
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

from src.edulite_joint import (  # noqa: E402
    ACTIVE_AXIS_Z_MM,
    BASE_M3_TAPPED_DIAMETER_MM,
    BRACKET_FOOT_X_MM,
    BRACKET_FOOT_Y_MM,
    BRACKET_PLATE_THICKNESS_MM,
    BRACKET_PLATE_Y_MM,
    BRACKET_REAR_FACE_X_MM,
    BRACKET_Z_MM,
    EDULITE_OUTPUT_DOWEL_CLOCK_DEG,
    EDULITE_OUTPUT_DOWEL_COUNT,
    EDULITE_OUTPUT_DOWEL_PCD_MM,
    EDULITE_OUTPUT_PATTERN_COUNT,
    EDULITE_OUTPUT_PATTERN_PCD_MM,
    EDULITE_REAR_PATTERN_CLOCK_DEG,
    EDULITE_REAR_PATTERN_COUNT,
    EDULITE_REAR_PATTERN_PCD_MM,
    HIP_Y_MM,
    OUTPUT_BOLT_CLEARANCE_DIAMETER_MM,
    OUTPUT_CENTER_RELIEF_DIAMETER_MM,
    OUTPUT_DOWEL_CLEARANCE_DIAMETER_MM,
    PROXIMAL_HUB_RADIUS_MM,
    REUSED_BASE_HOLES_XZ_MM,
    circular_pattern_centers,
)

ORIGINAL_STEP_SHA256 = (
    "cdca79d8ed21ebf3462d4d65d81c4d7696d988069e280445a09181b2856e37c4"
)
EDULITE_STEP_SHA256 = (
    "3c970be58644420e97a332473e9b1d806601125c9e95c95743ddeb0c99e27ee3"
)
OUTPUT_FACE_LOCAL_MM = 39.75
REAR_BOLT_CLEARANCE_DIAMETER_MM = 3.4
GUSSET_Z_MM = ((-58.0, -54.0), (-2.0, 2.0), (54.0, 58.0))
OUTPUT_ROTOR_LABELS = {
    "000_1-4238_1_1",
    "00_1-4459_1_1",
    "00_1-4459_1_001",
    "00_1-4459_1_002",
}
OUTPUT_DOWEL_LABELS = {
    "00_1-4459_1_1",
    "00_1-4459_1_001",
    "00_1-4459_1_002",
}


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
        default=PROJECT_ROOT / "artifacts" / "edulite_joint_module",
    )
    parser.add_argument("--pose-mm", type=float, default=90.0)
    parser.add_argument("--edulite-clock-deg", type=float, default=0.0)
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require_hash(path: Path, expected: str) -> None:
    observed = sha256(path)
    if observed != expected:
        raise ValueError(f"unexpected SHA-256 for {path}: {observed}")


def cylinder_cutters_x(
    centers_yz: tuple[tuple[float, float], ...],
    radius_mm: float,
    x_min_mm: float,
    length_mm: float,
) -> Any:
    return Part.makeCompound(
        [
            Part.makeCylinder(
                radius_mm,
                length_mm,
                FreeCAD.Vector(x_min_mm, center_y, center_z),
                FreeCAD.Vector(1.0, 0.0, 0.0),
            )
            for center_y, center_z in centers_yz
        ]
    )


def rebuild_output_hub(shape: Any, pivot_yz: tuple[float, float]) -> Any:
    box = shape.BoundBox
    x_min = float(box.XMin)
    thickness = float(box.XLength)
    pivot_y, pivot_z = pivot_yz
    filled_hub = Part.makeCylinder(
        PROXIMAL_HUB_RADIUS_MM,
        thickness,
        FreeCAD.Vector(x_min, pivot_y, pivot_z),
        FreeCAD.Vector(1.0, 0.0, 0.0),
    )
    rebuilt = shape.fuse(filled_hub)
    holes = [(pivot_y, pivot_z)]
    output_offsets = circular_pattern_centers(
        EDULITE_OUTPUT_PATTERN_COUNT,
        EDULITE_OUTPUT_PATTERN_PCD_MM,
    )
    output_holes = tuple(
        (pivot_y + offset_y, pivot_z + offset_z)
        for offset_y, offset_z in output_offsets
    )
    dowel_offsets = circular_pattern_centers(
        EDULITE_OUTPUT_DOWEL_COUNT,
        EDULITE_OUTPUT_DOWEL_PCD_MM,
        EDULITE_OUTPUT_DOWEL_CLOCK_DEG,
    )
    dowel_holes = tuple(
        (pivot_y + offset_y, pivot_z + offset_z)
        for offset_y, offset_z in dowel_offsets
    )
    center_cutter = cylinder_cutters_x(
        tuple(holes),
        OUTPUT_CENTER_RELIEF_DIAMETER_MM / 2.0,
        x_min - 1.0,
        thickness + 2.0,
    )
    bolt_cutters = cylinder_cutters_x(
        output_holes,
        OUTPUT_BOLT_CLEARANCE_DIAMETER_MM / 2.0,
        x_min - 1.0,
        thickness + 2.0,
    )
    dowel_cutters = cylinder_cutters_x(
        dowel_holes,
        OUTPUT_DOWEL_CLEARANCE_DIAMETER_MM / 2.0,
        x_min - 1.0,
        thickness + 2.0,
    )
    result = rebuilt.cut(
        Part.makeCompound([center_cutter, bolt_cutters, dowel_cutters])
    )
    if not result.isValid() or len(result.Solids) != 1:
        raise ValueError("rebuilt proximal output hub is not one valid solid")
    return result.removeSplitter()


def triangular_gusset(z_min: float, z_max: float) -> Any:
    plate_inner_x = BRACKET_REAR_FACE_X_MM + BRACKET_PLATE_THICKNESS_MM
    wire = Part.makePolygon(
        [
            FreeCAD.Vector(plate_inner_x, BRACKET_FOOT_Y_MM[1], z_min),
            FreeCAD.Vector(
                BRACKET_FOOT_X_MM[1], BRACKET_FOOT_Y_MM[1], z_min
            ),
            FreeCAD.Vector(plate_inner_x, 24.0, z_min),
            FreeCAD.Vector(plate_inner_x, BRACKET_FOOT_Y_MM[1], z_min),
        ]
    )
    return Part.Face(wire).extrude(FreeCAD.Vector(0.0, 0.0, z_max - z_min))


def build_bracket_foot() -> Any:
    return Part.makeBox(
        BRACKET_FOOT_X_MM[1] - BRACKET_FOOT_X_MM[0],
        BRACKET_FOOT_Y_MM[1] - BRACKET_FOOT_Y_MM[0],
        BRACKET_Z_MM[1] - BRACKET_Z_MM[0],
        FreeCAD.Vector(
            BRACKET_FOOT_X_MM[0], BRACKET_FOOT_Y_MM[0], BRACKET_Z_MM[0]
        ),
    )


def build_bracket(*, rear_pattern_mirror_y: bool = False) -> Any:
    plate = Part.makeBox(
        BRACKET_PLATE_THICKNESS_MM,
        BRACKET_PLATE_Y_MM[1] - BRACKET_PLATE_Y_MM[0],
        BRACKET_Z_MM[1] - BRACKET_Z_MM[0],
        FreeCAD.Vector(
            BRACKET_REAR_FACE_X_MM,
            BRACKET_PLATE_Y_MM[0],
            BRACKET_Z_MM[0],
        ),
    )
    foot = build_bracket_foot()
    bracket = plate.fuse(foot)
    for z_min, z_max in GUSSET_Z_MM:
        bracket = bracket.fuse(triangular_gusset(z_min, z_max))

    rear_offsets = circular_pattern_centers(
        EDULITE_REAR_PATTERN_COUNT,
        EDULITE_REAR_PATTERN_PCD_MM,
        EDULITE_REAR_PATTERN_CLOCK_DEG,
    )
    rear_holes = tuple(
        (
            HIP_Y_MM + (-offset_y if rear_pattern_mirror_y else offset_y),
            axis_z - offset_x,
        )
        for axis_z in ACTIVE_AXIS_Z_MM
        for offset_x, offset_y in rear_offsets
    )
    rear_cutters = cylinder_cutters_x(
        rear_holes,
        REAR_BOLT_CLEARANCE_DIAMETER_MM / 2.0,
        BRACKET_REAR_FACE_X_MM - 1.0,
        BRACKET_PLATE_THICKNESS_MM + 2.0,
    )
    base_cutters = Part.makeCompound(
        [
            Part.makeCylinder(
                BASE_M3_TAPPED_DIAMETER_MM / 2.0,
                BRACKET_FOOT_Y_MM[1] - BRACKET_FOOT_Y_MM[0] + 2.0,
                FreeCAD.Vector(x, BRACKET_FOOT_Y_MM[0] - 1.0, z),
                FreeCAD.Vector(0.0, 1.0, 0.0),
            )
            for x, z in REUSED_BASE_HOLES_XZ_MM
        ]
    )
    result = bracket.cut(Part.makeCompound([rear_cutters, base_cutters]))
    if not result.isValid() or len(result.Solids) != 1:
        raise ValueError("EduLite bracket is not one valid solid")
    return result.removeSplitter()


def nominal_motor_envelope(axis_z_mm: float) -> Any:
    """Return a display-only nominal body; exact scans use the official STEP."""
    return Part.makeCylinder(
        23.0,
        44.0,
        FreeCAD.Vector(-66.5, HIP_Y_MM, axis_z_mm),
        FreeCAD.Vector(1.0, 0.0, 0.0),
    )


def add_feature(document: Any, label: str, shape: Any) -> Any:
    feature = document.addObject("Part::Feature", label)
    feature.Label = label
    feature.Shape = shape
    return feature


def compound(shapes: list[Any]) -> Any:
    return Part.makeCompound(shapes)


def pattern_clock_deg(
    centers_yz: set[tuple[float, float]],
    pivot_yz: tuple[float, float],
    count: int,
) -> float:
    pivot_y, pivot_z = pivot_yz
    phases = [
        math.radians(count * math.degrees(math.atan2(z - pivot_z, y - pivot_y)))
        for y, z in centers_yz
    ]
    phase = math.atan2(
        sum(math.sin(value) for value in phases),
        sum(math.cos(value) for value in phases),
    )
    return math.degrees(phase) / count % (360.0 / count)


def align_output_rotor(
    motor_features: dict[str, Any],
    proximal_link: Any,
    pivot_yz: tuple[float, float],
) -> float:
    flange = motor_features["000_1-4238_1_1"]
    product_centers = envelope.yz_cylinder_centers(flange, 1.6)
    link_centers = envelope.yz_cylinder_centers(
        proximal_link, OUTPUT_BOLT_CLEARANCE_DIAMETER_MM / 2.0
    )
    if len(product_centers) != EDULITE_OUTPUT_PATTERN_COUNT:
        raise ValueError("official output flange does not expose six M4 axes")
    if len(link_centers) != EDULITE_OUTPUT_PATTERN_COUNT:
        raise ValueError("rebuilt proximal link does not expose six M4 holes")
    product_clock = pattern_clock_deg(
        product_centers, pivot_yz, EDULITE_OUTPUT_PATTERN_COUNT
    )
    link_clock = pattern_clock_deg(
        link_centers, pivot_yz, EDULITE_OUTPUT_PATTERN_COUNT
    )
    pitch = 360.0 / EDULITE_OUTPUT_PATTERN_COUNT
    base_rotation = (
        (link_clock - product_clock + pitch / 2.0) % pitch - pitch / 2.0
    )
    candidates = []
    for rotation in (base_rotation, base_rotation + pitch):
        pins = []
        for label in OUTPUT_DOWEL_LABELS:
            pin = motor_features[label].copy()
            pin.rotate(
                FreeCAD.Vector(0.0, pivot_yz[0], pivot_yz[1]),
                FreeCAD.Vector(1.0, 0.0, 0.0),
                rotation,
            )
            pins.append(pin)
        candidates.append(
            (common_volume(Part.makeCompound(pins), proximal_link), rotation)
        )
    _volume, rotation = min(candidates)
    for label in OUTPUT_ROTOR_LABELS:
        shape = motor_features[label].copy()
        shape.rotate(
            FreeCAD.Vector(0.0, pivot_yz[0], pivot_yz[1]),
            FreeCAD.Vector(1.0, 0.0, 0.0),
            rotation,
        )
        motor_features[label] = shape
    return rotation


def common_volume(left: Any, right: Any) -> float:
    return float(left.common(right).Volume)


def common_volume_details(
    features: dict[str, Any], target: Any
) -> list[dict[str, Any]]:
    details = []
    for label, shape in features.items():
        overlap = shape.common(target)
        volume = float(overlap.Volume)
        if volume <= 1e-6:
            continue
        box = overlap.BoundBox
        details.append(
            {
                "feature": label,
                "volume_mm3": volume,
                "bbox_mm": [
                    float(box.XMin),
                    float(box.YMin),
                    float(box.ZMin),
                    float(box.XMax),
                    float(box.YMax),
                    float(box.ZMax),
                ],
            }
        )
    return sorted(details, key=lambda row: float(row["volume_mm3"]), reverse=True)


def minimum_distance(left: Any, shapes: list[Any]) -> float:
    return min(float(left.distToShape(shape)[0]) for shape in shapes)


def export_feature(shape: Any, label: str, path: Path) -> None:
    document = FreeCAD.newDocument(f"export_{label}")
    feature = add_feature(document, label, shape)
    document.recompute()
    Import.export([feature], str(path))
    FreeCAD.closeDocument(document.Name)


def main() -> None:
    args = parse_args()
    require_hash(args.assembly_step, ORIGINAL_STEP_SHA256)
    require_hash(args.edulite_step, EDULITE_STEP_SHA256)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    features = envelope.load_features(args.assembly_step, "original_assembly", 80)
    reference = envelope.recover_reference_geometry(features)
    edulite_features = envelope.load_features(args.edulite_step, "edulite", 18)
    edulite_groups = envelope.place_edulite_groups(
        edulite_features,
        OUTPUT_FACE_LOCAL_MM,
        args.edulite_clock_deg,
    )

    rebuilt = {
        "大腿_EduLite": rebuild_output_hub(
            features["大腿"], reference["pivots_yz_mm"]["a"]
        ),
        "大腿垫高_EduLite": rebuild_output_hub(
            features["大腿垫高"], reference["pivots_yz_mm"]["e"]
        ),
        "大腿001_EduLite": rebuild_output_hub(
            features["大腿001"], reference["pivots_yz_mm"]["e"]
        ),
    }
    source_groups = {
        name: {label: features[label] for label in labels}
        for name, labels in envelope.MOVING_LABELS.items()
    }
    source_groups["left_proximal_negative"] = {
        "大腿_EduLite": rebuilt["大腿_EduLite"]
    }
    source_groups["left_proximal_positive"] = {
        "大腿垫高_EduLite": rebuilt["大腿垫高_EduLite"],
        "大腿001_EduLite": rebuilt["大腿001_EduLite"],
    }
    placed = envelope.place_groups(source_groups, reference, args.pose_mm)
    bracket = build_bracket()
    bracket_foot = build_bracket_foot()
    baseplate = features["底板"]

    direct_link = placed["left_proximal_negative"]["大腿_EduLite"]
    offset_spacer = placed["left_proximal_positive"]["大腿垫高_EduLite"]
    offset_link = placed["left_proximal_positive"]["大腿001_EduLite"]
    target = envelope.target_geometry(
        args.pose_mm, reference["pivots_yz_mm"]["a"][0]
    )
    rotor_alignment_deg = {
        "direct": align_output_rotor(
            edulite_groups["edulite_left_negative"], direct_link, target["a"]
        ),
        "offset": align_output_rotor(
            edulite_groups["edulite_left_positive"], offset_spacer, target["e"]
        ),
    }
    lower_motor = compound(list(edulite_groups["edulite_left_negative"].values()))
    upper_motor = compound(list(edulite_groups["edulite_left_positive"].values()))
    distal_and_wheel = [
        shape
        for group in (
            "left_distal_negative",
            "left_distal_positive",
            "left_wheel",
        )
        for shape in placed[group].values()
    ]

    output_paths = {
        "shared_bracket": args.output_dir / "edulite_left_shared_bracket.step",
        "direct_proximal_link": args.output_dir
        / "edulite_direct_proximal_link.step",
        "offset_spacer": args.output_dir / "edulite_offset_spacer.step",
        "offset_proximal_link": args.output_dir
        / "edulite_offset_proximal_link.step",
        "single_leg_reference": args.output_dir
        / f"edulite_single_leg_{args.pose_mm:g}mm.step",
    }
    export_feature(
        bracket, "EduLite_left_shared_bracket", output_paths["shared_bracket"]
    )
    export_feature(
        rebuilt["大腿_EduLite"],
        "EduLite_direct_proximal_link",
        output_paths["direct_proximal_link"],
    )
    export_feature(
        rebuilt["大腿垫高_EduLite"],
        "EduLite_offset_spacer",
        output_paths["offset_spacer"],
    )
    export_feature(
        rebuilt["大腿001_EduLite"],
        "EduLite_offset_proximal_link",
        output_paths["offset_proximal_link"],
    )

    assembly = FreeCAD.newDocument("edulite_single_leg_reference")
    assembly_objects = [
        add_feature(assembly, "Original_baseplate", baseplate),
        add_feature(assembly, "EduLite_left_shared_bracket", bracket),
        add_feature(
            assembly,
            "EduLite_lower_nominal_body_display_only",
            nominal_motor_envelope(ACTIVE_AXIS_Z_MM[0]),
        ),
        add_feature(
            assembly,
            "EduLite_upper_nominal_body_display_only",
            nominal_motor_envelope(ACTIVE_AXIS_Z_MM[1]),
        ),
        add_feature(assembly, "EduLite_direct_proximal_link", direct_link),
        add_feature(assembly, "EduLite_offset_spacer", offset_spacer),
        add_feature(assembly, "EduLite_offset_proximal_link", offset_link),
    ]
    for group in ("left_distal_negative", "left_distal_positive", "left_wheel"):
        for label, shape in placed[group].items():
            assembly_objects.append(add_feature(assembly, label, shape))
    assembly.recompute()
    Import.export(assembly_objects, str(output_paths["single_leg_reference"]))

    motor_link_volume = {
        "direct_mm3": common_volume(lower_motor, direct_link),
        "offset_stack_mm3": common_volume(
            upper_motor, compound([offset_spacer, offset_link])
        ),
    }
    bracket_motor_volume = {
        "lower_mm3": common_volume(bracket, lower_motor),
        "upper_mm3": common_volume(bracket, upper_motor),
    }
    motor_link_details = {
        "direct": common_volume_details(
            edulite_groups["edulite_left_negative"], direct_link
        ),
        "offset_stack": common_volume_details(
            edulite_groups["edulite_left_positive"],
            compound([offset_spacer, offset_link]),
        ),
    }
    moving_nonadjacent = distal_and_wheel + [direct_link, offset_spacer, offset_link]
    audit = {
        "schema_version": 1,
        "status": "CONCEPT_GEOMETRY_NOT_PRODUCTION_RELEASE",
        "pose_mm": args.pose_mm,
        "inputs": {
            "original_assembly": {
                "path": str(args.assembly_step.relative_to(PROJECT_ROOT)),
                "sha256": ORIGINAL_STEP_SHA256,
            },
            "edulite_step": {
                "path": args.edulite_step.name,
                "sha256": EDULITE_STEP_SHA256,
                "clock_deg": args.edulite_clock_deg,
                "output_face_local_mm": OUTPUT_FACE_LOCAL_MM,
                "output_rotor_alignment_deg": rotor_alignment_deg,
            },
        },
        "frozen_interfaces": {
            "five_bar_lengths_mm": list(reference["lengths"].values()),
            "active_axes_yz_mm": [
                [HIP_Y_MM, ACTIVE_AXIS_Z_MM[0]],
                [HIP_Y_MM, ACTIVE_AXIS_Z_MM[1]],
            ],
            "output_center_relief_diameter_mm": OUTPUT_CENTER_RELIEF_DIAMETER_MM,
            "output_bolt_pattern": {
                "count": EDULITE_OUTPUT_PATTERN_COUNT,
                "pcd_mm": EDULITE_OUTPUT_PATTERN_PCD_MM,
                "clearance_diameter_mm": OUTPUT_BOLT_CLEARANCE_DIAMETER_MM,
            },
            "output_dowel_pattern": {
                "count": EDULITE_OUTPUT_DOWEL_COUNT,
                "pcd_mm": EDULITE_OUTPUT_DOWEL_PCD_MM,
                "clearance_diameter_mm": OUTPUT_DOWEL_CLEARANCE_DIAMETER_MM,
                "clock_from_m4_pattern_deg": EDULITE_OUTPUT_DOWEL_CLOCK_DEG,
            },
            "rear_mount_pattern": {
                "count": EDULITE_REAR_PATTERN_COUNT,
                "pcd_mm": EDULITE_REAR_PATTERN_PCD_MM,
                "clock_deg": EDULITE_REAR_PATTERN_CLOCK_DEG,
                "clearance_diameter_mm": REAR_BOLT_CLEARANCE_DIAMETER_MM,
            },
            "reused_base_holes_xz_mm": [
                list(point) for point in REUSED_BASE_HOLES_XZ_MM
            ],
            "base_mount": {
                "thread": "M3 tapped in bracket foot",
                "modeled_tap_drill_diameter_mm": BASE_M3_TAPPED_DIAMETER_MM,
                "foot_thickness_mm": BRACKET_FOOT_Y_MM[1]
                - BRACKET_FOOT_Y_MM[0],
            },
        },
        "checks": {
            "all_custom_shapes_valid": all(
                shape.isValid()
                for shape in [bracket, *rebuilt.values(), direct_link, offset_link]
            ),
            "motor_to_link_common_volume_mm3": motor_link_volume,
            "motor_to_link_overlap_details": motor_link_details,
            "bracket_to_motor_common_volume_mm3": bracket_motor_volume,
            "motor_to_foot_minimum_clearance_mm": {
                "lower": minimum_distance(
                    bracket_foot,
                    list(edulite_groups["edulite_left_negative"].values()),
                ),
                "upper": minimum_distance(
                    bracket_foot,
                    list(edulite_groups["edulite_left_positive"].values()),
                ),
            },
            "bracket_to_baseplate_common_volume_mm3": common_volume(
                bracket, baseplate
            ),
            "bracket_to_moving_minimum_distance_mm": minimum_distance(
                bracket, moving_nonadjacent
            ),
        },
        "exports": {
            name: {
                "path": str(path.relative_to(PROJECT_ROOT)),
                "sha256": sha256(path),
                "bytes": path.stat().st_size,
            }
            for name, path in output_paths.items()
        },
        "single_leg_reference_note": (
            "The assembly STEP contains nominal 46 x 44 mm motor cylinders for "
            "display only; all fit checks use the hash-qualified official "
            "18-solid STEP."
        ),
        "open_items": [
            "fastener strength, bracket stiffness and fatigue are not yet validated",
            "connector clock and cable keep-out are not frozen",
            "internal battery and compute layout is intentionally excluded",
        ],
    }
    audit_path = args.output_dir / "interface_audit.json"
    audit_path.write_text(
        json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(f"wrote {audit_path}")


if __name__ == "__main__":
    main()
