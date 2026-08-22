import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_visualization_cli_writes_png(tmp_path: Path) -> None:
    output = tmp_path / "five-bar.png"

    subprocess.run(
        [
            sys.executable,
            str(PROJECT_ROOT / "scripts" / "visualize_leg.py"),
            "--phi1",
            "-2.2",
            "--phi4",
            "-0.9",
            "--output",
            str(output),
        ],
        cwd=PROJECT_ROOT,
        check=True,
    )

    assert output.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert output.stat().st_size > 10_000
