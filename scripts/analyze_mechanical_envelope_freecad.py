#!/usr/bin/env python3
"""Run the exact-B-Rep mechanical-envelope scan inside FreeCAD's Python."""

import argparse
import csv
import json
import math
import os
import sys
from collections import Counter
from itertools import combinations
from pathlib import Path
from typing import Any

import FreeCAD  # type: ignore[import-not-found]
import Import  # type: ignore[import-not-found]

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mechanical_envelope import cad_pose  # noqa: E402

FIXED_LABELS = (
    "底板",
    "关节电机支架",
    "4010电机",
    "驱动板",
    "驱动板001",
    "关节电机支架001",
    "4010电机001",
    "驱动板002",
    "关节电机支架002",
    "关节电机支架003",
    "4010电机002",
    "驱动板003",
    "4010电机003",
    "主控板PCB",
    "电池架",
    "电池支撑铜柱",
    "电池支撑铜柱001",
    "电池支撑铜柱002",
    "电池支撑铜柱003",
    "动力锂电池",
    "主控支撑铜柱",
    "主控支撑铜柱001",
    "主控支撑铜柱002",
    "主控支撑铜柱003",
    "主控支架",
    "NanoPi支架",
    "NanoPi",
    "电池支撑铜柱004",
    "电池支撑铜柱005",
    "电池支撑铜柱006",
    "电池支撑铜柱007",
    "OV5640镜头",
)

MOVING_LABELS = {
    "left_proximal_negative": ("大腿",),
    "left_distal_negative": (
        "推力轴承",
        "小腿",
        "小腿001",
        "MISUMI_C-SE604ZZ_不锈钢小径滚珠轴承001",
        "cross recessed pan head screws gb_GB_CROSS_SCREWS_TYPE1 M4X16-16  H type-C",
        "hex nuts, style 1-grades ab gb_GB_FASTENER_NUT_SNAB1 M4-C",
    ),
    "left_proximal_positive": ("大腿垫高", "大腿001"),
    "left_distal_positive": (
        "小腿003",
        "小腿004",
        "推力轴承001",
        "MISUMI_C-SE604ZZ_不锈钢小径滚珠轴承002",
        "cross recessed pan head screws gb_GB_CROSS_SCREWS_TYPE1 M4X16-16  H type-C001",
        "hex nuts, style 1-grades ab gb_GB_FASTENER_NUT_SNAB1 M4-C001",
    ),
    "left_wheel": (
        "推力轴承002",
        "驱动板004",
        "2804电机",
        "轮胎",
        "MISUMI_C-SE604ZZ_不锈钢小径滚珠轴承",
        "hex nuts, style 1-grades ab gb_GB_FASTENER_NUT_SNAB1 M4-C002",
        "编码器磁铁",
        "MISUMI_CBSTSR4-16_内六角螺栓（盖螺栓）",
        "车轮",
    ),
    "right_proximal_positive": ("大腿002",),
    "right_distal_positive": (
        "推力轴承003",
        "小腿006",
        "小腿007",
        "MISUMI_C-SE604ZZ_不锈钢小径滚珠轴承003",
        "cross recessed pan head screws gb_GB_CROSS_SCREWS_TYPE1 M4X16-16  H type-C002",
        "hex nuts, style 1-grades ab gb_GB_FASTENER_NUT_SNAB1 M4-C003",
    ),
    "right_proximal_negative": ("大腿垫高001", "大腿003"),
    "right_distal_negative": (
        "推力轴承004",
        "小腿009",
        "小腿010",
        "MISUMI_C-SE604ZZ_不锈钢小径滚珠轴承005",
        "hex nuts, style 1-grades ab gb_GB_FASTENER_NUT_SNAB1 M4-C005",
        "cross recessed pan head screws gb_GB_CROSS_SCREWS_TYPE1 M4X16-16  H type-C003",
    ),
    "right_wheel": (
        "推力轴承005",
        "驱动板005",
        "轮胎001",
        "2804电机001",
        "编码器磁铁001",
        "MISUMI_C-SE604ZZ_不锈钢小径滚珠轴承004",
        "hex nuts, style 1-grades ab gb_GB_FASTENER_NUT_SNAB1 M4-C004",
        "MISUMI_CBSTSR4-16_内六角螺栓（盖螺栓）001",
        "车轮001",
    ),
}

ADJACENT_PAIRS = {
    frozenset(("fixed", "left_proximal_negative")),
    frozenset(("fixed", "left_proximal_positive")),
    frozenset(("fixed", "right_proximal_negative")),
    frozenset(("fixed", "right_proximal_positive")),
    frozenset(("left_proximal_negative", "left_distal_negative")),
    frozenset(("left_proximal_positive", "left_distal_positive")),
    frozenset(("right_proximal_negative", "right_distal_negative")),
    frozenset(("right_proximal_positive", "right_distal_positive")),
    frozenset(("left_distal_negative", "left_distal_positive")),
    frozenset(("right_distal_negative", "right_distal_positive")),
    frozenset(("left_distal_negative", "left_wheel")),
    frozenset(("left_distal_positive", "left_wheel")),
    frozenset(("right_distal_negative", "right_wheel")),
    frozenset(("right_distal_positive", "right_wheel")),
}

REPLACED_FIXED_LABELS = {
    "关节电机支架",
    "4010电机",
    "驱动板",
    "关节电机支架001",
    "4010电机001",
    "驱动板001",
    "关节电机支架002",
    "4010电机002",
    "驱动板002",
    "关节电机支架003",
    "4010电机003",
    "驱动板003",
}

EDULITE_ADJACENT_LINKS = {
    "edulite_left_negative": "left_proximal_negative",
    "edulite_left_positive": "left_proximal_positive",
    "edulite_right_negative": "right_proximal_negative",
    "edulite_right_positive": "right_proximal_positive",
}

PAIR_COLUMNS = (
    "l0_mm",
    "group_a",
    "entity_a",
    "group_b",
    "entity_b",
    "adjacent",
    "bbox_lower_bound_mm",
    "distance_mm",
    "common_volume_mm3",
    "reference_common_volume_mm3",
    "excess_common_volume_mm3",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=Path, default=os.environ.get("CAD_STEP_PATH"))
    parser.add_argument(
        "--output-dir", type=Path, default=os.environ.get("CAD_OUTPUT_DIR")
    )
    parser.add_argument(
        "--start-mm", type=float, default=float(os.environ.get("CAD_START_MM", 70.0))
    )
    parser.add_argument(
        "--stop-mm", type=float, default=float(os.environ.get("CAD_STOP_MM", 120.0))
    )
    parser.add_argument(
        "--step-mm", type=float, default=float(os.environ.get("CAD_STEP_MM", 1.0))
    )
    parser.add_argument(
        "--near-mm", type=float, default=float(os.environ.get("CAD_NEAR_MM", 2.0))
    )
    parser.add_argument(
        "--volume-tolerance-mm3",
        type=float,
        default=float(os.environ.get("CAD_VOLUME_TOLERANCE_MM3", 1e-5)),
    )
    parser.add_argument(
        "--compute-common-volume",
        action="store_true",
        default=os.environ.get("CAD_COMPUTE_COMMON_VOLUME") == "1",
    )
    parser.add_argument(
        "--classify-contact-volume",
        action="store_true",
        default=os.environ.get("CAD_CLASSIFY_CONTACT_VOLUME") == "1",
        help="run a Boolean common only for nonadjacent pairs at zero distance",
    )
    parser.add_argument(
        "--check-adjacent-excess",
        action="store_true",
        default=os.environ.get("CAD_CHECK_ADJACENT_EXCESS") == "1",
        help="subtract saved-pose joint overlap and detect new adjacent clashes",
    )
    parser.add_argument(
        "--edulite-step",
        type=Path,
        default=os.environ.get("CAD_EDULITE_STEP_PATH"),
        help="replace the four 4010 motors/brackets with official EL05 geometry",
    )
    parser.add_argument(
        "--edulite-clock-deg",
        type=float,
        default=float(os.environ.get("CAD_EDULITE_CLOCK_DEG", 0.0)),
    )
    parser.add_argument(
        "--edulite-output-face-mm",
        type=float,
        default=float(os.environ.get("CAD_EDULITE_OUTPUT_FACE_MM", 39.75)),
    )
    parser.add_argument(
        "--edulite-linkage-envelope-only",
        action="store_true",
        default=os.environ.get("CAD_EDULITE_LINKAGE_ENVELOPE_ONLY") == "1",
        help="retain only the baseplate from the reconfigurable fixed packaging",
    )
    args, _unknown = parser.parse_known_args()
    if args.step is None or args.output_dir is None:
        parser.error("set CAD_STEP_PATH and CAD_OUTPUT_DIR or pass both arguments")
    return args


def load_features(
    step_path: Path, document_name: str, expected_count: int
) -> dict[str, Any]:
    document = FreeCAD.newDocument(document_name)
    Import.insert(str(step_path), document.Name)
    features = {
        obj.Label: obj.Shape
        for obj in document.Objects
        if obj.TypeId == "Part::Feature"
    }
    if len(features) != expected_count:
        raise ValueError(
            f"expected {expected_count} named solids, found {len(features)}"
        )
    if any(not shape.isValid() for shape in features.values()):
        raise ValueError("the STEP assembly contains an invalid Part::Feature")
    return features


def is_adjacent(group_a: str, group_b: str) -> bool:
    pair = frozenset((group_a, group_b))
    if pair in ADJACENT_PAIRS:
        return True
    return any(
        pair == frozenset((actuator, proximal))
        for actuator, proximal in EDULITE_ADJACENT_LINKS.items()
    )


def validate_group_partition(features: dict[str, Any]) -> None:
    labels = set(FIXED_LABELS)
    for group_labels in MOVING_LABELS.values():
        overlap = labels.intersection(group_labels)
        if overlap:
            raise ValueError(f"labels assigned more than once: {sorted(overlap)}")
        labels.update(group_labels)
    expected = set(features)
    if labels != expected:
        raise ValueError(
            f"group partition mismatch; missing={sorted(expected - labels)}, "
            f"extra={sorted(labels - expected)}"
        )


def yz_cylinder_centers(shape: Any, radius_mm: float) -> set[tuple[float, float]]:
    centers = set()
    for face in shape.Faces:
        surface = face.Surface
        if not all(
            hasattr(surface, attribute)
            for attribute in ("Radius", "Axis", "Center")
        ):
            continue
        if abs(float(surface.Radius) - radius_mm) > 1e-7:
            continue
        axis = surface.Axis
        if abs(abs(float(axis.x)) - 1.0) > 1e-9:
            continue
        centers.add(
            (
                round(float(surface.Center.y), 9),
                round(float(surface.Center.z), 9),
            )
        )
    return centers


def yz_cylinder_center(shape: Any, radius_mm: float) -> tuple[float, float]:
    centers = yz_cylinder_centers(shape, radius_mm)
    if len(centers) != 1:
        raise ValueError(
            f"expected one radius-{radius_mm:g} mm x-axis center, found {centers}"
        )
    return centers.pop()


def distance_yz(left: tuple[float, float], right: tuple[float, float]) -> float:
    return math.hypot(right[0] - left[0], right[1] - left[1])


def model_angle(
    start: tuple[float, float], end: tuple[float, float]
) -> float:
    """Return the Phase 3 angle from global (y,z), with downward-positive y."""
    return math.degrees(math.atan2(start[0] - end[0], end[1] - start[1]))


def recover_reference_geometry(features: dict[str, Any]) -> dict[str, Any]:
    a = yz_cylinder_center(features["大腿"], 8.0)
    b = yz_cylinder_center(features["大腿"], 10.0)
    e = yz_cylinder_center(features["大腿001"], 8.0)
    d = yz_cylinder_center(features["大腿001"], 10.0)
    c = yz_cylinder_center(features["小腿001"], 15.0)

    mirror_checks = (
        (a, yz_cylinder_center(features["大腿003"], 8.0)),
        (b, yz_cylinder_center(features["大腿003"], 10.0)),
        (e, yz_cylinder_center(features["大腿002"], 8.0)),
        (d, yz_cylinder_center(features["大腿002"], 10.0)),
        (c, yz_cylinder_center(features["小腿010"], 15.0)),
    )
    if any(distance_yz(left, right) > 2e-6 for left, right in mirror_checks):
        raise ValueError(
            "left/right linkage axes are not mirrored in the saved assembly"
        )

    lengths = {
        "l1_mm": distance_yz(a, b),
        "l2_mm": distance_yz(b, c),
        "l3_mm": distance_yz(d, c),
        "l4_mm": distance_yz(e, d),
        "l5_mm": distance_yz(a, e),
    }
    expected_lengths = (50.0, 105.0, 105.0, 50.0, 60.0)
    if any(
        abs(value - expected) > 2e-6
        for value, expected in zip(lengths.values(), expected_lengths, strict=True)
    ):
        raise ValueError(f"CAD pivot lengths do not match Phase 3: {lengths}")

    origin = ((a[0] + e[0]) / 2.0, (a[1] + e[1]) / 2.0)
    return {
        "pivots_yz_mm": {"a": a, "b": b, "c": c, "d": d, "e": e},
        "lengths": lengths,
        "saved_pose": {
            "l0_mm": distance_yz(origin, c),
            "phi0_deg": model_angle(origin, c),
            "phi1_deg": model_angle(a, b),
            "phi4_deg": model_angle(e, d),
            "theta_bc_deg": model_angle(b, c),
            "theta_dc_deg": model_angle(d, c),
        },
    }


def rotate_translate_shape(
    source: Any,
    pivot_ref: tuple[float, float],
    pivot_target: tuple[float, float],
    angle_delta_deg: float,
) -> Any:
    result = source.copy()
    result.rotate(
        FreeCAD.Vector(0.0, pivot_ref[0], pivot_ref[1]),
        FreeCAD.Vector(1.0, 0.0, 0.0),
        angle_delta_deg,
    )
    result.translate(
        FreeCAD.Vector(
            0.0,
            pivot_target[0] - pivot_ref[0],
            pivot_target[1] - pivot_ref[1],
        )
    )
    return result


def recover_edulite_axis(features: dict[str, Any]) -> tuple[float, float]:
    centers: Counter[tuple[float, float]] = Counter()
    for shape in features.values():
        for face in shape.Faces:
            surface = face.Surface
            if not all(
                hasattr(surface, attribute)
                for attribute in ("Radius", "Axis", "Center")
            ):
                continue
            axis = surface.Axis
            if abs(abs(float(axis.z)) - 1.0) > 1e-9:
                continue
            if float(surface.Radius) < 5.0:
                continue
            centers[
                (
                    round(float(surface.Center.x), 8),
                    round(float(surface.Center.y), 8),
                )
            ] += 1
    if not centers:
        raise ValueError("could not recover the EL05 output axis")
    center, count = centers.most_common(1)[0]
    if count < 10:
        raise ValueError(f"EL05 output-axis evidence is too weak: {center}, {count}")
    return center


def place_edulite_groups(
    features: dict[str, Any], output_face_mm: float, clock_deg: float
) -> dict[str, dict[str, Any]]:
    center_x, center_y = recover_edulite_axis(features)
    local_origin = FreeCAD.Vector(center_x, center_y, output_face_mm)
    placements = {
        "edulite_left_negative": (-66.5, 34.0, -30.0, 90.0),
        "edulite_left_positive": (-66.5, 34.0, 30.0, 90.0),
        "edulite_right_negative": (66.5, 34.0, -30.0, -90.0),
        "edulite_right_positive": (66.5, 34.0, 30.0, -90.0),
    }
    groups = {}
    for group, (target_x, target_y, target_z, tilt_deg) in placements.items():
        target = FreeCAD.Vector(target_x, target_y, target_z)
        placed = {}
        for label, source in features.items():
            shape = source.copy()
            shape.rotate(
                local_origin,
                FreeCAD.Vector(0.0, 0.0, 1.0),
                clock_deg,
            )
            shape.rotate(
                local_origin,
                FreeCAD.Vector(0.0, 1.0, 0.0),
                tilt_deg,
            )
            shape.translate(target - local_origin)
            placed[label] = shape
        groups[group] = placed
    return groups


def target_geometry(l0_mm: float, hip_y_mm: float) -> dict[str, Any]:
    pose = cad_pose(l0_mm)

    def global_yz(point: tuple[float, float]) -> tuple[float, float]:
        return hip_y_mm - point[1], point[0] - 30.0

    return {
        "pose": pose,
        "a": global_yz(pose.a_mm),
        "b": global_yz(pose.b_mm),
        "c": global_yz(pose.c_mm),
        "d": global_yz(pose.d_mm),
        "e": global_yz(pose.e_mm),
    }


def place_groups(
    source_groups: dict[str, dict[str, Any]],
    reference: dict[str, Any],
    l0_mm: float,
) -> dict[str, dict[str, Any]]:
    pivots = reference["pivots_yz_mm"]
    saved = reference["saved_pose"]
    target = target_geometry(l0_mm, pivots["a"][0])
    pose = target["pose"]

    placed = {
        name: entities
        for name, entities in source_groups.items()
        if name == "fixed" or name.startswith("edulite_")
    }
    placements = {
        "proximal_negative": (
            pivots["a"],
            target["a"],
            pose.phi1_deg - saved["phi1_deg"],
        ),
        "proximal_positive": (
            pivots["e"],
            target["e"],
            pose.phi4_deg - saved["phi4_deg"],
        ),
        "distal_negative": (
            pivots["b"],
            target["b"],
            pose.theta_bc_deg - saved["theta_bc_deg"],
        ),
        "distal_positive": (
            pivots["d"],
            target["d"],
            pose.theta_dc_deg - saved["theta_dc_deg"],
        ),
        "wheel": (pivots["c"], target["c"], 0.0),
    }
    for side in ("left", "right"):
        for linkage_group, transform in placements.items():
            name = f"{side}_{linkage_group}"
            placed[name] = {
                label: rotate_translate_shape(
                    shape, transform[0], transform[1], transform[2]
                )
                for label, shape in source_groups[name].items()
            }
    return placed


def validate_kinematic_placement(
    features: dict[str, Any], reference: dict[str, Any], l0_mm: float
) -> None:
    pivots = reference["pivots_yz_mm"]
    saved = reference["saved_pose"]
    target = target_geometry(l0_mm, pivots["a"][0])
    pose = target["pose"]
    checks = (
        (
            "大腿",
            pivots["a"],
            target["a"],
            pose.phi1_deg - saved["phi1_deg"],
            ((8.0, target["a"]), (10.0, target["b"])),
        ),
        (
            "大腿001",
            pivots["e"],
            target["e"],
            pose.phi4_deg - saved["phi4_deg"],
            ((8.0, target["e"]), (10.0, target["d"])),
        ),
        (
            "小腿001",
            pivots["b"],
            target["b"],
            pose.theta_bc_deg - saved["theta_bc_deg"],
            ((10.0, target["b"]), (15.0, target["c"])),
        ),
        (
            "小腿004",
            pivots["d"],
            target["d"],
            pose.theta_dc_deg - saved["theta_dc_deg"],
            ((10.0, target["d"]), (15.0, target["c"])),
        ),
    )
    for label, pivot_ref, pivot_target, angle_delta, expected_centers in checks:
        placed = rotate_translate_shape(
            features[label], pivot_ref, pivot_target, angle_delta
        )
        for radius, expected in expected_centers:
            candidates = yz_cylinder_centers(placed, radius)
            if not candidates:
                raise ValueError(f"{label} has no radius-{radius:g} cylinder")
            actual = min(candidates, key=lambda center: distance_yz(center, expected))
            if distance_yz(actual, expected) > 2e-6:
                raise ValueError(
                    f"{label} radius-{radius:g} axis misses its target at "
                    f"l0={l0_mm:g} mm: actual={actual}, expected={expected}"
                )


def bbox_distance(left: Any, right: Any) -> float:
    a = left.BoundBox
    b = right.BoundBox
    gaps = (
        max(float(a.XMin) - float(b.XMax), float(b.XMin) - float(a.XMax), 0.0),
        max(float(a.YMin) - float(b.YMax), float(b.YMin) - float(a.YMax), 0.0),
        max(float(a.ZMin) - float(b.ZMax), float(b.ZMin) - float(a.ZMax), 0.0),
    )
    return math.sqrt(sum(gap * gap for gap in gaps))


def common_volume(
    left: Any, right: Any, distance_mm: float | None = None
) -> float:
    lower_bound = bbox_distance(left, right)
    if lower_bound > 1e-7:
        return 0.0
    if distance_mm is None:
        distance_mm = float(left.distToShape(right)[0])
    if distance_mm > 1e-7:
        return 0.0
    return float(left.common(right).Volume)


def component_pairs(
    groups: dict[str, dict[str, Any]],
) -> list[tuple[str, str, str, str]]:
    pairs = []
    for group_a, group_b in combinations(groups, 2):
        for entity_a in groups[group_a]:
            for entity_b in groups[group_b]:
                pairs.append((group_a, entity_a, group_b, entity_b))
    return pairs


def component_pair_key(
    group_a: str, entity_a: str, group_b: str, entity_b: str
) -> tuple[tuple[str, str], tuple[str, str]]:
    left = (group_a, entity_a)
    right = (group_b, entity_b)
    return (left, right) if left <= right else (right, left)


def exact_minimum_pair(
    groups: dict[str, dict[str, Any]],
    pairs: list[tuple[str, str, str, str]],
) -> tuple[float, tuple[str, str, str, str]]:
    ordered = sorted(
        (
            bbox_distance(groups[group_a][entity_a], groups[group_b][entity_b]),
            group_a,
            entity_a,
            group_b,
            entity_b,
        )
        for group_a, entity_a, group_b, entity_b in pairs
        if not is_adjacent(group_a, group_b)
    )
    best_distance = math.inf
    best_pair = ("", "", "", "")
    for lower_bound, group_a, entity_a, group_b, entity_b in ordered:
        if lower_bound >= best_distance:
            break
        distance = float(
            groups[group_a][entity_a].distToShape(groups[group_b][entity_b])[0]
        )
        if distance < best_distance:
            best_distance = distance
            best_pair = (group_a, entity_a, group_b, entity_b)
    return best_distance, best_pair


def scan_lengths(start_mm: float, stop_mm: float, step_mm: float) -> list[float]:
    if step_mm <= 0.0 or start_mm > stop_mm:
        raise ValueError("invalid scan interval")
    count = int(math.floor((stop_mm - start_mm) / step_mm + 1e-9))
    values = [start_mm + index * step_mm for index in range(count + 1)]
    if values[-1] < stop_mm - 1e-9:
        values.append(stop_mm)
    return values


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    features = load_features(args.step, "mechanical_envelope", 80)
    validate_group_partition(features)
    reference = recover_reference_geometry(features)

    fixed_labels = tuple(
        label
        for label in FIXED_LABELS
        if args.edulite_step is None or label not in REPLACED_FIXED_LABELS
    )
    if args.edulite_linkage_envelope_only:
        if args.edulite_step is None:
            raise ValueError("linkage-envelope mode requires an EL05 STEP")
        fixed_labels = ("底板",)
    source_groups = {
        "fixed": {label: features[label] for label in fixed_labels}
    }
    source_groups.update(
        {
            name: {label: features[label] for label in labels}
            for name, labels in MOVING_LABELS.items()
        }
    )
    edulite_axis = None
    if args.edulite_step is not None:
        edulite_features = load_features(
            args.edulite_step, "edulite_replacement", 18
        )
        edulite_axis = recover_edulite_axis(edulite_features)
        source_groups.update(
            place_edulite_groups(
                edulite_features,
                args.edulite_output_face_mm,
                args.edulite_clock_deg,
            )
        )
    reference_pairs = component_pairs(source_groups)
    reference_common = {}
    for group_a, entity_a, group_b, entity_b in reference_pairs:
        left = source_groups[group_a][entity_a]
        right = source_groups[group_b][entity_b]
        key = component_pair_key(group_a, entity_a, group_b, entity_b)
        adjacent = is_adjacent(group_a, group_b)
        reference_common[key] = (
            common_volume(left, right)
            if (args.compute_common_volume and not adjacent)
            or (args.check_adjacent_excess and adjacent)
            else 0.0
        )

    summaries = []
    pair_events = []
    for l0_mm in scan_lengths(args.start_mm, args.stop_mm, args.step_mm):
        print(f"starting l0={l0_mm:.3f} mm", flush=True)
        validate_kinematic_placement(features, reference, l0_mm)
        groups = place_groups(source_groups, reference, l0_mm)
        pairs = component_pairs(groups)
        minimum, minimum_pair = exact_minimum_pair(groups, pairs)
        maximum_excess = 0.0
        maximum_pair = ("", "", "", "")
        for group_a, entity_a, group_b, entity_b in pairs:
            left = groups[group_a][entity_a]
            right = groups[group_b][entity_b]
            key = component_pair_key(group_a, entity_a, group_b, entity_b)
            adjacent = is_adjacent(group_a, group_b)
            lower_bound = bbox_distance(left, right)
            distance = math.inf
            volume = 0.0
            if lower_bound <= args.near_mm:
                distance = float(left.distToShape(right)[0])
                if args.compute_common_volume and not adjacent:
                    volume = common_volume(left, right, distance)
                elif (
                    args.classify_contact_volume
                    and not adjacent
                    and distance <= 1e-7
                ):
                    volume = common_volume(left, right, distance)
                elif (
                    args.check_adjacent_excess
                    and adjacent
                    and distance <= 1e-7
                ):
                    volume = common_volume(left, right, distance)
            excess = max(
                0.0,
                volume - reference_common[key] - args.volume_tolerance_mm3,
            )
            if excess > maximum_excess:
                maximum_excess = excess
                maximum_pair = (group_a, entity_a, group_b, entity_b)
            if lower_bound <= args.near_mm or excess > 0.0:
                pair_events.append(
                    (
                        l0_mm,
                        group_a,
                        entity_a,
                        group_b,
                        entity_b,
                        adjacent,
                        lower_bound,
                        distance,
                        volume,
                        reference_common[key],
                        excess,
                    )
                )
        minimum_description = (
            f"{minimum_pair[0]}:{minimum_pair[1]} / "
            f"{minimum_pair[2]}:{minimum_pair[3]}"
        )
        maximum_description = (
            f"{maximum_pair[0]}:{maximum_pair[1]} / "
            f"{maximum_pair[2]}:{maximum_pair[3]}"
            if maximum_pair[0]
            else ""
        )
        summaries.append(
            {
                "l0_mm": l0_mm,
                "state": (
                    "INTERFERENCE"
                    if maximum_excess > 0.0
                    else "CONTACT"
                    if minimum <= 1e-7 and args.classify_contact_volume
                    else "CONTACT_OR_INTERFERENCE"
                    if minimum <= 1e-7
                    else "CLEAR"
                ),
                "minimum_nonadjacent_clearance_mm": minimum,
                "minimum_clearance_pair": minimum_description,
                "maximum_excess_common_volume_mm3": maximum_excess,
                "maximum_excess_pair": maximum_description,
            }
        )
        print(
            f"l0={l0_mm:.3f} mm state={summaries[-1]['state']} "
            f"clearance={minimum:.6g} mm pair={minimum_description} "
            f"excess_volume={maximum_excess:.6g} mm^3",
            flush=True,
        )

    with (args.output_dir / "pose_summary.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(
            stream, fieldnames=tuple(summaries[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(summaries)
    with (args.output_dir / "pair_events.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        pair_writer = csv.writer(stream, lineterminator="\n")
        pair_writer.writerow(PAIR_COLUMNS)
        pair_writer.writerows(pair_events)

    audit = {
        "schema_version": 1,
        "input": {
            "path": args.step.name,
            "part_features": len(features),
            "all_features_valid": True,
            "edulite_replacement": (
                {
                    "path": args.edulite_step.name,
                    "part_features": 18,
                    "axis_xy_mm": edulite_axis,
                    "output_face_mm": args.edulite_output_face_mm,
                    "clock_deg": args.edulite_clock_deg,
                    "removed_original_entities": sorted(REPLACED_FIXED_LABELS),
                    "adapter_geometry_included": False,
                    "packaging_mode": (
                        "linkage_envelope_only"
                        if args.edulite_linkage_envelope_only
                        else "retained_original_packaging"
                    ),
                }
                if args.edulite_step is not None
                else None
            ),
        },
        "reference_geometry": reference,
        "groups": {name: list(entities) for name, entities in source_groups.items()},
        "reference_common_volume_mm3": {
            (
                f"{pair[0][0]}:{pair[0][1]} / {pair[1][0]}:{pair[1][1]}"
            ): volume
            for pair, volume in reference_common.items()
            if volume > args.volume_tolerance_mm3
        },
        "scan": {
            "start_mm": args.start_mm,
            "stop_mm": args.stop_mm,
            "step_mm": args.step_mm,
            "pose_count": len(summaries),
            "clear_count": sum(row["state"] == "CLEAR" for row in summaries),
            "contact_or_interference_count": sum(
                row["state"] == "CONTACT_OR_INTERFERENCE" for row in summaries
            ),
            "contact_count": sum(row["state"] == "CONTACT" for row in summaries),
            "interference_count": sum(
                row["state"] == "INTERFERENCE" for row in summaries
            ),
            "common_volume_enabled": args.compute_common_volume,
            "contact_volume_classification_enabled": (
                args.classify_contact_volume
            ),
            "adjacent_excess_check_enabled": args.check_adjacent_excess,
        },
    }
    (args.output_dir / "cad_audit.json").write_text(
        json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
