#!/usr/bin/env python3
"""Plot the qualified original-CAD and EL05 mechanical-envelope evidence."""

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ARTIFACTS = PROJECT_ROOT / "artifacts" / "mechanical_envelope"


def read_rows(directory: str) -> list[dict[str, str]]:
    path = ARTIFACTS / directory / "pose_summary.csv"
    with path.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main() -> None:
    original_rows = [
        *read_rows("original_cad_short_verified"),
        *read_rows("original_cad_70_120_verified"),
        *read_rows("original_cad_long_verified"),
    ]
    original = {float(row["l0_mm"]): row for row in original_rows}
    x = np.array(sorted(original))
    clearance = np.array(
        [float(original[value]["minimum_nonadjacent_clearance_mm"]) for value in x]
    )
    excess = np.array(
        [float(original[value]["maximum_excess_common_volume_mm3"]) for value in x]
    )
    states = np.array([original[value]["state"] for value in x])

    edulite_rows = read_rows("edulite_linkage_envelope_70_120")
    ex = np.array([float(row["l0_mm"]) for row in edulite_rows])
    ec = np.array(
        [float(row["minimum_nonadjacent_clearance_mm"]) for row in edulite_rows]
    )

    figure, axes = plt.subplots(2, 1, figsize=(10.5, 7.2), height_ratios=[2.2, 1.0])
    clearance_axis, interval_axis = axes
    clearance_axis.axvspan(
        70, 120, color="#dff0df", alpha=0.75, label="70–120 baseline"
    )
    clear = states == "CLEAR"
    contact = states == "CONTACT"
    interference = states == "INTERFERENCE"
    clearance_axis.plot(
        x[clear],
        clearance[clear],
        color="#1f77b4",
        lw=2,
        label="Original CAD clearance",
    )
    clearance_axis.scatter(
        x[contact],
        np.zeros(contact.sum()),
        color="#e69f00",
        s=28,
        label="Nominal contact",
    )
    clearance_axis.scatter(
        x[interference],
        np.zeros(interference.sum()),
        color="#d62728",
        marker="x",
        s=36,
        label="Interference",
    )
    clearance_axis.plot(
        ex,
        ec,
        color="#7b3294",
        lw=1.7,
        ls="--",
        label="EL05 linkage-only envelope",
    )
    clearance_axis.set_xlim(45, 153)
    clearance_axis.set_ylim(bottom=-0.35)
    clearance_axis.set_ylabel("Minimum nonadjacent clearance (mm)")
    clearance_axis.grid(alpha=0.25)
    clearance_axis.legend(ncol=2, fontsize=9)

    volume_axis = clearance_axis.twinx()
    volume_axis.fill_between(
        x, 0, excess, where=excess > 0, color="#d62728", alpha=0.18
    )
    volume_axis.set_ylabel("Added adjacent overlap (mm³)", color="#a51f1f")
    volume_axis.tick_params(axis="y", colors="#a51f1f")

    bars = (
        ("Mathematical non-singular", 46.0977222865, 152.0690632575, "#969696"),
        ("Original CAD: positive clearance", 67.71, 152.0, "#1f77b4"),
        ("Candidate for dynamics", 70.0, 120.0, "#2ca02c"),
        ("EL05 linkage-only necessary check", 70.0, 120.0, "#7b3294"),
    )
    for index, (label, start, stop, color) in enumerate(bars):
        interval_axis.plot(
            [start, stop],
            [index, index],
            lw=10,
            color=color,
            solid_capstyle="butt",
        )
        interval_axis.text(45.5, index, label, ha="left", va="center", fontsize=9)
    interval_axis.text(
        95,
        4.0,
        "EL05 with retained original packaging: INTERFERENCE",
        color="#d62728",
        ha="center",
        va="center",
        fontsize=9,
    )
    interval_axis.set_xlim(45, 153)
    interval_axis.set_ylim(-0.7, 4.55)
    interval_axis.set_yticks([])
    interval_axis.set_xlabel("Virtual leg length l0 (mm)")
    interval_axis.grid(axis="x", alpha=0.25)
    for spine in ("left", "right", "top"):
        interval_axis.spines[spine].set_visible(False)

    figure.suptitle("Real mechanical envelope evidence")
    figure.tight_layout()
    output = ARTIFACTS / "mechanical_envelope.png"
    figure.savefig(output, dpi=180)
    plt.close(figure)
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
