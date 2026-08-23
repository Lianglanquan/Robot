import json
import subprocess
import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_stair_gate_cli_writes_decision_artifacts(tmp_path: Path) -> None:
    subprocess.run(
        [
            sys.executable,
            str(PROJECT_ROOT / "scripts" / "analyze_stair_gate.py"),
            "--output-dir",
            str(tmp_path),
        ],
        cwd=PROJECT_ROOT,
        check=True,
    )

    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["task"]["step_height_m"] == 0.15
    assert summary["task"]["com_rise_benchmarks_m"][
        "full_clear_with_tuck"
    ] == pytest.approx(0.105)
    assert summary["candidate_hardware"]["six_motor_mass_kg"] > 1.2
    assert summary["events"]["2.5_kg"]["landing"][
        "20_mm_compression"
    ]["max_joint_torque_nm"] > 3.0
    assert summary["events"]["2.5_kg"]["landing"][
        "50_mm_compression"
    ]["max_joint_torque_nm"] < 1.8

    figure = tmp_path / "stair_decision_gate.png"
    assert figure.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert figure.stat().st_size > 100_000
