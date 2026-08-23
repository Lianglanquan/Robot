import gzip
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

from src.analysis import best_class_xy_grid, scan_workspace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIGURE_NAMES = (
    "workspace_pose.png",
    "jacobian_singularity.png",
    "upright_condition.png",
    "force_speed_transmission.png",
    "workspace_classification.png",
)


def test_best_class_xy_grid_aggregates_all_internal_postures() -> None:
    scan = scan_workspace(resolution=12)
    x_edges, y_edges, grid = best_class_xy_grid(scan)
    expected = np.full_like(grid, np.nan)
    x_index = np.clip(
        np.searchsorted(x_edges, scan.column("xc"), side="right") - 1,
        0,
        grid.shape[1] - 1,
    )
    y_index = np.clip(
        np.searchsorted(y_edges, scan.column("yc"), side="right") - 1,
        0,
        grid.shape[0] - 1,
    )
    for row, column, code in zip(
        y_index, x_index, scan.column("class_code"), strict=True
    ):
        previous = expected[row, column]
        expected[row, column] = code if np.isnan(previous) else min(previous, code)

    np.testing.assert_array_equal(np.isnan(grid), np.isnan(expected))
    np.testing.assert_array_equal(grid[~np.isnan(grid)], expected[~np.isnan(expected)])


def test_workspace_analysis_cli_writes_data_and_five_figures(tmp_path: Path) -> None:
    subprocess.run(
        [
            sys.executable,
            str(PROJECT_ROOT / "scripts" / "analyze_workspace.py"),
            "--resolution",
            "24",
            "--output-dir",
            str(tmp_path),
        ],
        cwd=PROJECT_ROOT,
        check=True,
    )

    with gzip.open(tmp_path / "workspace_scan.csv.gz", "rt", encoding="ascii") as file:
        rows = file.readlines()
    assert len(rows) == 24**2 + 1
    assert rows[0].startswith("phi1,phi4,xc,yc,l0,phi0,")

    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["resolution"] == 24
    assert summary["valid_samples"] == 24**2
    assert summary["upright_length_bins"]
    assert "maximum_condition_number" in summary["extreme_postures"]
    assert summary["vertical_classification_counts"]["recommended"] > 0
    assert summary["classification_metrics"]["recommended"]["samples"] > 0

    for name in FIGURE_NAMES:
        figure = tmp_path / name
        assert figure.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
        assert figure.stat().st_size > 20_000
