# Workspace And Transmission Analysis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Quantify the original five-bar leg's workspace, singularities, and normalized force/speed transmission and publish reproducible plots and data.

**Architecture:** Add a focused analysis module that evaluates the verified positive-root branch through stable circle geometry and constraint differentiation, then expose a deterministic grid scanner. A single CLI owns CSV/JSON export and five static Matplotlib figures; README explains measured implications.

**Tech Stack:** Python 3, NumPy, Matplotlib, pytest, gzip/CSV/JSON standard libraries

---

### Task 1: Stable Posture Metrics

**Files:**
- Create: `src/analysis.py`
- Create: `tests/test_analysis.py`

- [ ] Write failing tests for four-link closure, positive-root C agreement, and
  `J` agreement with `analytic_jacobian` at regular postures.
- [ ] Run `python3 -m pytest tests/test_analysis.py -v`; expect import failure for
  `src.analysis`.
- [ ] Implement `PostureMetrics` and `analyze_posture` with circle intersection,
  differentiated constraints, raw `J`, `J_phys`, SVD metrics, force, and speed.
- [ ] Add failing boundary tests for classification and box-constrained force and
  speed formulas, then implement `classify_posture`.
- [ ] Run the focused tests and require all pass.
- [ ] Commit `src/analysis.py` and `tests/test_analysis.py`.

### Task 2: Deterministic Grid Scan

**Files:**
- Modify: `src/analysis.py`
- Modify: `tests/test_analysis.py`

- [ ] Write failing tests that `scan_workspace(resolution=12)` returns 144 unique
  `[-pi,pi)` input pairs, finite legal rows, and stable named columns.
- [ ] Run the focused grid tests and confirm missing API failure.
- [ ] Implement `WorkspaceScan`, `scan_workspace`, upright-mask calculation, and
  aggregate summary values without adding pandas.
- [ ] Run focused tests and commit the scanner.

### Task 3: Artifact And Plot Generator

**Files:**
- Create: `scripts/analyze_workspace.py`
- Create: `tests/test_workspace_cli.py`

- [ ] Write a CLI smoke test using resolution 24 that requires CSV gzip, summary
  JSON, and five valid nonempty PNG files.
- [ ] Run the smoke test and confirm the script is missing.
- [ ] Implement one-pass data export and five figures with equal XY axes,
  log-normalized wide-range quantities, threshold annotations, and labeled
  three-class colors.
- [ ] Run the smoke test and inspect all five small-run images for nonblank output.
- [ ] Commit the generator and CLI test.

### Task 4: Full Resolution Results And README

**Files:**
- Create: `artifacts/phase2/workspace_scan.csv.gz`
- Create: `artifacts/phase2/summary.json`
- Create: `artifacts/phase2/workspace_pose.png`
- Create: `artifacts/phase2/jacobian_singularity.png`
- Create: `artifacts/phase2/upright_condition.png`
- Create: `artifacts/phase2/force_speed_transmission.png`
- Create: `artifacts/phase2/workspace_classification.png`
- Modify: `README.md`

- [ ] Run `python3 scripts/analyze_workspace.py --resolution 360` and require
  129600 rows and five generated figures.
- [ ] Inspect every full-resolution figure for correct labels, nonblank marks,
  readable scales, and coherent category colors.
- [ ] Add exact summary results, methods, classification caveats, commands, and a
  practical interpretation directly below every embedded figure in README.
- [ ] Check README numbers against `summary.json` and commit artifacts/docs.

### Task 5: Final Verification

**Files:**
- Modify only files whose own verification fails.

- [ ] Run `ruff check .` and require no findings.
- [ ] Run `mypy .` and require no type errors.
- [ ] Run `python3 -m pytest -x -v` and require zero failures.
- [ ] Regenerate the 360-grid artifacts and require exit code zero.
- [ ] Verify compressed CSV row count, JSON fields, PNG dimensions, repository
  status, and all original phase-one tests.
