#!/usr/bin/env python3
"""Audit exact actuator STEP geometry inside FreeCAD."""

import json
import os
from pathlib import Path
from typing import Any

import FreeCAD  # type: ignore[import-not-found]
import Import  # type: ignore[import-not-found]


def vector(value: Any) -> list[float]:
    return [float(value.x), float(value.y), float(value.z)]


def bbox(shape: Any) -> list[float]:
    box = shape.BoundBox
    return [
        float(box.XMin),
        float(box.YMin),
        float(box.ZMin),
        float(box.XMax),
        float(box.YMax),
        float(box.ZMax),
    ]


def cylinders(shape: Any) -> list[dict[str, Any]]:
    result = {}
    for face in shape.Faces:
        surface = face.Surface
        if not all(
            hasattr(surface, attribute)
            for attribute in ("Radius", "Axis", "Center")
        ):
            continue
        axis = vector(surface.Axis)
        if next((value for value in axis if abs(value) > 1e-9), 1.0) < 0.0:
            axis = [-value for value in axis]
        radius = float(surface.Radius)
        center = vector(surface.Center)
        row = {
            "radius_mm": radius,
            "axis": axis,
            "center_mm": center,
        }
        values = [radius, *axis, *center]
        key = tuple(round(value, 8) for value in values)
        result[key] = row
    return list(result.values())


def main() -> None:
    step_path = Path(os.environ["ACTUATOR_STEP_PATH"])
    output_path = Path(os.environ["ACTUATOR_AUDIT_PATH"])
    document = FreeCAD.newDocument("actuator_audit")
    Import.insert(str(step_path), document.Name)
    features = [obj for obj in document.Objects if obj.TypeId == "Part::Feature"]
    label_contains = os.environ.get("ACTUATOR_LABEL_CONTAINS")
    if label_contains:
        features = [obj for obj in features if label_contains in obj.Label]
    if not features or any(not obj.Shape.isValid() for obj in features):
        raise ValueError("actuator STEP did not import as valid Part::Feature geometry")
    compound = features[0].Shape.copy()
    if len(features) > 1:
        import Part  # type: ignore[import-not-found]

        compound = Part.makeCompound([obj.Shape for obj in features])
    audit = {
        "input": step_path.name,
        "feature_count": len(features),
        "solid_count": sum(len(obj.Shape.Solids) for obj in features),
        "all_features_valid": True,
        "bbox_mm": bbox(compound),
        "features": [
            {
                "label": obj.Label,
                "solid_count": len(obj.Shape.Solids),
                "bbox_mm": bbox(obj.Shape),
                "cylinders": cylinders(obj.Shape),
            }
            for obj in features
        ],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
