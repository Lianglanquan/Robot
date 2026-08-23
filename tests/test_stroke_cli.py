import gzip
import json
import subprocess
import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIGURE_NAMES = (
    "continuous_stroke_geometry.png",
    "joint_continuation.png",
    "kinematic_quality.png",
    "force_speed_tradeoff.png",
)


def test_stroke_analysis_cli_writes_reproducible_research_artifacts(
    tmp_path: Path,
) -> None:
    subprocess.run(
        [
            sys.executable,
            str(PROJECT_ROOT / "scripts" / "analyze_stroke.py"),
            "--resolution",
            "101",
            "--output-dir",
            str(tmp_path),
        ],
        cwd=PROJECT_ROOT,
        check=True,
    )

    csv_path = tmp_path / "normal_vertical_stroke.csv.gz"
    with gzip.open(csv_path, "rt", encoding="ascii") as file:
        rows = file.readlines()
    assert len(rows) == 102
    assert rows[0].startswith("l0,phi0,phi1,phi4,xc,yc,")
    assert csv_path.read_bytes()[4:8] == b"\x00" * 4

    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["resolution"] == 101
    assert summary["mathematical_stroke_m"] > 0.1
    assert summary["normal_mode_signs"] == [1, -1]
    assert summary["upstream_evidence"]["normal_command_range_m"] == [0.07, 0.09]
    assert summary["landmarks"]["0.070_m"]["condition_number"] < 1.3
    assert summary["landmarks"]["0.120_m"]["condition_number"] < 1.4
    assert (
        summary["operating_bands"][
            "upstream_normal_to_airborne_target_70_120_mm"
        ]["joint_excursion_deg_each"]
        > 40.0
    )
    assert summary["operating_bands"][
        "upstream_normal_to_airborne_target_70_120_mm"
    ]["condition_number_max"] == pytest.approx(
        summary["landmarks"]["0.120_m"]["condition_number"]
    )
    assert summary["operating_bands"]["upstream_normal_command_70_90_mm"][
        "max_axial_force_n_at_1_nm"
    ]["max"] == pytest.approx(
        summary["landmarks"]["0.070_m"]["max_axial_force_n_at_1_nm"]
    )
    assert summary["threshold_intervals"]

    for name in FIGURE_NAMES:
        figure = tmp_path / name
        assert figure.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
        assert figure.stat().st_size > 20_000
