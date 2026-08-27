#!/usr/bin/env python3
"""Scan the two-sided EduLite structure over the 70--120 mm path."""

import argparse
import csv
import json
import math
import sys
from itertools import combinations
from pathlib import Path
from typing import Any

import FreeCAD  # type: ignore[import-not-found]  # noqa: F401
import Part  # type: ignore[import-not-found]

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import analyze_mechanical_envelope_freecad as envelope  # noqa: E402
import build_edulite_single_leg_freecad as design  # noqa: E402
import build_edulite_vehicle_freecad as vehicle  # noqa: E402

from src.edulite_joint import (  # noqa: E402
    DIRECT_OUTPUT_M4_SCREW_LENGTH_MM,
    OFFSET_OUTPUT_M4_SCREW_LENGTH_MM,
)

STANDARD_ADJACENT_GROUPS = {
    pair
    for side in ("left", "right")
    for pair in (
        frozenset((f"{side}_proximal_negative", f"{side}_distal_negative")),
        frozenset((f"{side}_proximal_positive", f"{side}_distal_positive")),
        frozenset((f"{side}_distal_negative", f"{side}_distal_positive")),
        frozenset((f"{side}_distal_negative", f"{side}_wheel")),
        frozenset((f"{side}_distal_positive", f"{side}_wheel")),
    )
}
OWN_MOTOR_GROUP = {
    "EduLite_left_lower_product": "left_proximal_negative",
    "EduLite_left_upper_product": "left_proximal_positive",
    "EduLite_right_lower_product": "right_proximal_negative",
    "EduLite_right_upper_product": "right_proximal_positive",
}
PAIR_FIELDS = (
    "l0_mm",
    "group_a",
    "entity_a",
    "group_b",
    "entity_b",
    "bbox_lower_bound_mm",
    "distance_mm",
    "common_volume_mm3",
)


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
        default=PROJECT_ROOT / "artifacts" / "edulite_vehicle" / "scan_70_120",
    )
    parser.add_argument("--start-mm", type=float, default=70.0)
    parser.add_argument("--stop-mm", type=float, default=120.0)
    parser.add_argument("--step-mm", type=float, default=1.0)
    parser.add_argument("--near-mm", type=float, default=2.0)
    return parser.parse_args()


def copy_group(group: dict[str, Any]) -> dict[str, Any]:
    return {label: shape.copy() for label, shape in group.items()}


def is_designed_adjacent(
    group_a: str, entity_a: str, group_b: str, entity_b: str
) -> bool:
    if frozenset((group_a, group_b)) in STANDARD_ADJACENT_GROUPS:
        return True
    return (group_a == "fixed" and OWN_MOTOR_GROUP.get(entity_a) == group_b) or (
        group_b == "fixed" and OWN_MOTOR_GROUP.get(entity_b) == group_a
    )


def component_pairs(
    groups: dict[str, dict[str, Any]],
) -> list[tuple[str, str, str, str]]:
    pairs = []
    for group_a, group_b in combinations(groups, 2):
        for entity_a in groups[group_a]:
            for entity_b in groups[group_b]:
                if is_designed_adjacent(group_a, entity_a, group_b, entity_b):
                    continue
                pairs.append((group_a, entity_a, group_b, entity_b))
    return pairs


def scan_pose(
    groups: dict[str, dict[str, Any]], near_mm: float
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    ordered = sorted(
        (
            envelope.bbox_distance(
                groups[group_a][entity_a], groups[group_b][entity_b]
            ),
            group_a,
            entity_a,
            group_b,
            entity_b,
        )
        for group_a, entity_a, group_b, entity_b in component_pairs(groups)
    )
    best_distance = math.inf
    best_pair = ("", "", "", "")
    maximum_volume = 0.0
    maximum_pair = ("", "", "", "")
    events = []
    for lower_bound, group_a, entity_a, group_b, entity_b in ordered:
        if lower_bound > near_mm and lower_bound >= best_distance:
            break
        left = groups[group_a][entity_a]
        right = groups[group_b][entity_b]
        distance = float(left.distToShape(right)[0])
        if distance < best_distance:
            best_distance = distance
            best_pair = (group_a, entity_a, group_b, entity_b)
        volume = float(left.common(right).Volume) if distance <= 1e-7 else 0.0
        if volume > maximum_volume:
            maximum_volume = volume
            maximum_pair = (group_a, entity_a, group_b, entity_b)
        if distance <= near_mm:
            events.append(
                {
                    "group_a": group_a,
                    "entity_a": entity_a,
                    "group_b": group_b,
                    "entity_b": entity_b,
                    "bbox_lower_bound_mm": lower_bound,
                    "distance_mm": distance,
                    "common_volume_mm3": volume,
                }
            )
    state = (
        "INTERFERENCE"
        if maximum_volume > 1e-5
        else "CONTACT"
        if best_distance <= 1e-7
        else "CLEAR"
    )
    return (
        {
            "state": state,
            "minimum_clearance_mm": best_distance,
            "minimum_pair": " / ".join(best_pair),
            "maximum_common_volume_mm3": maximum_volume,
            "maximum_volume_pair": " / ".join(maximum_pair),
        },
        events,
    )


def main() -> None:
    args = parse_args()
    design.require_hash(args.assembly_step, design.ORIGINAL_STEP_SHA256)
    design.require_hash(args.edulite_step, design.EDULITE_STEP_SHA256)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    features = envelope.load_features(args.assembly_step, "single_leg_scan", 80)
    reference = envelope.recover_reference_geometry(features)
    edulite_features = envelope.load_features(args.edulite_step, "edulite_scan", 18)
    base_motors = envelope.place_edulite_groups(
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
    source_groups["left_proximal_negative"] = {"大腿_EduLite": rebuilt["大腿_EduLite"]}
    source_groups["left_proximal_positive"] = {
        "大腿垫高_EduLite": rebuilt["大腿垫高_EduLite"],
        "大腿001_EduLite": rebuilt["大腿001_EduLite"],
    }
    source_groups["right_proximal_positive"] = {
        "大腿002_EduLite": rebuilt["大腿002_EduLite"]
    }
    source_groups["right_proximal_negative"] = {
        "大腿垫高001_EduLite": rebuilt["大腿垫高001_EduLite"],
        "大腿003_EduLite": rebuilt["大腿003_EduLite"],
    }
    direct_screws = vehicle.output_screws(
        rebuilt["大腿_EduLite"],
        reference["pivots_yz_mm"]["a"],
        "left",
        DIRECT_OUTPUT_M4_SCREW_LENGTH_MM,
    )
    offset_screws = vehicle.output_screws(
        rebuilt["大腿001_EduLite"],
        reference["pivots_yz_mm"]["e"],
        "left",
        OFFSET_OUTPUT_M4_SCREW_LENGTH_MM,
    )
    source_groups["left_proximal_negative"]["M4_output_screws"] = Part.makeCompound(
        list(direct_screws.values())
    )
    source_groups["left_proximal_positive"]["M4_output_screws"] = Part.makeCompound(
        list(offset_screws.values())
    )
    right_direct_screws = vehicle.output_screws(
        rebuilt["大腿002_EduLite"],
        reference["pivots_yz_mm"]["e"],
        "right",
        DIRECT_OUTPUT_M4_SCREW_LENGTH_MM,
    )
    right_offset_screws = vehicle.output_screws(
        rebuilt["大腿003_EduLite"],
        reference["pivots_yz_mm"]["a"],
        "right",
        OFFSET_OUTPUT_M4_SCREW_LENGTH_MM,
    )
    source_groups["right_proximal_positive"]["M4_output_screws"] = (
        Part.makeCompound(list(right_direct_screws.values()))
    )
    source_groups["right_proximal_negative"]["M4_output_screws"] = (
        Part.makeCompound(list(right_offset_screws.values()))
    )
    brackets = {
        "left": design.build_bracket(),
        "right": vehicle.mirror_x(
            design.build_bracket(rear_pattern_mirror_y=True)
        ),
    }
    lengths = envelope.scan_lengths(args.start_mm, args.stop_mm, args.step_mm)
    summaries: list[dict[str, Any]] = []
    pair_events: list[dict[str, Any]] = []
    output_fit: list[dict[str, Any]] = []

    for l0_mm in lengths:
        envelope.validate_kinematic_placement(features, reference, l0_mm)
        placed = envelope.place_groups(source_groups, reference, l0_mm)
        target = envelope.target_geometry(l0_mm, reference["pivots_yz_mm"]["a"][0])
        motor_links = {
            "left_lower": (
                "edulite_left_negative",
                placed["left_proximal_negative"]["大腿_EduLite"],
                target["a"],
            ),
            "left_upper": (
                "edulite_left_positive",
                Part.makeCompound(
                    [
                        placed["left_proximal_positive"]["大腿垫高_EduLite"],
                        placed["left_proximal_positive"]["大腿001_EduLite"],
                    ]
                ),
                target["e"],
            ),
            "right_lower": (
                "edulite_right_negative",
                Part.makeCompound(
                    [
                        placed["right_proximal_negative"]["大腿垫高001_EduLite"],
                        placed["right_proximal_negative"]["大腿003_EduLite"],
                    ]
                ),
                target["a"],
            ),
            "right_upper": (
                "edulite_right_positive",
                placed["right_proximal_positive"]["大腿002_EduLite"],
                target["e"],
            ),
        }
        fit: dict[str, Any] = {"l0_mm": l0_mm}
        motors = {}
        for name, (motor_group_name, link_stack, pivot) in motor_links.items():
            motor_group = copy_group(base_motors[motor_group_name])
            fit[f"{name}_rotor_deg"] = design.align_output_rotor(
                motor_group, link_stack, pivot
            )
            motor = Part.makeCompound(list(motor_group.values()))
            motors[name] = motor
            fit[f"{name}_common_volume_mm3"] = design.common_volume(
                motor, link_stack
            )
        output_fit.append(fit)
        groups = {
            "fixed": {
                "Original_baseplate": features["底板"],
                "EduLite_left_shared_bracket": brackets["left"],
                "EduLite_right_shared_bracket": brackets["right"],
                **{
                    f"EduLite_{name}_product": motor
                    for name, motor in motors.items()
                },
            },
            **placed,
        }
        summary, events = scan_pose(groups, args.near_mm)
        summary["l0_mm"] = l0_mm
        summaries.append(summary)
        pair_events.extend({"l0_mm": l0_mm, **event} for event in events)
        print(
            f"l0={l0_mm:.1f} mm {summary['state']} "
            f"clearance={summary['minimum_clearance_mm']:.6g} mm",
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
        writer = csv.DictWriter(stream, fieldnames=PAIR_FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(pair_events)
    with (args.output_dir / "output_fit.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(
            stream, fieldnames=tuple(output_fit[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(output_fit)

    minimum_row = min(summaries, key=lambda row: row["minimum_clearance_mm"])
    audit = {
        "schema_version": 1,
        "status": (
            "CLEAR"
            if all(row["state"] == "CLEAR" for row in summaries)
            and all(
                all(
                    value <= 1e-5
                    for key, value in row.items()
                    if key.endswith("_common_volume_mm3")
                )
                for row in output_fit
            )
            else "NOT_CLEAR"
        ),
        "scope": (
            "both five-bar legs and four EduLite joints; internal packaging excluded"
        ),
        "scan": {
            "start_mm": args.start_mm,
            "stop_mm": args.stop_mm,
            "step_mm": args.step_mm,
            "pose_count": len(summaries),
            "clear_count": sum(row["state"] == "CLEAR" for row in summaries),
            "minimum_clearance_mm": minimum_row["minimum_clearance_mm"],
            "minimum_pose_mm": minimum_row["l0_mm"],
            "minimum_pair": minimum_row["minimum_pair"],
            "maximum_output_fit_common_volume_mm3": max(
                max(
                    value
                    for key, value in row.items()
                    if key.endswith("_common_volume_mm3")
                )
                for row in output_fit
            ),
        },
        "method_note": (
            "Original designed revolute-joint contact pairs are excluded; all new "
            "bracket, motor, modified-hub and moving M4 output-screw-head pairs "
            "are checked explicitly."
        ),
        "open_items": [
            "bracket stiffness and fastener loads",
            "connector orientation and cable keep-out",
        ],
    }
    audit_path = args.output_dir / "scan_audit.json"
    audit_path.write_text(
        json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(f"wrote {audit_path}")


if __name__ == "__main__":
    main()
