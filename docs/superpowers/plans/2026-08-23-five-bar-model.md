# Five-Bar Leg Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and verify a Python reproduction of the upstream five-bar leg mathematics.

**Architecture:** Define the upstream equations once in SymPy, cache NumPy callables for pose and Jacobian evaluation, and keep the two mappings as direct matrix products. Compile the unmodified MATLAB Coder output into a test-only shared library and compare deterministic legal samples at the original float32 API boundary.

**Tech Stack:** Python 3, NumPy, SymPy, Matplotlib, pytest, GCC/ctypes

---

### Task 1: Reference Baseline

**Files:**
- Create: `reference/README.md`
- Create: `reference/LICENSE.upstream`
- Create: `reference/matlab/leg_func_calc.m`
- Create: `reference/matlab/README.md`
- Create: `reference/c/leg_pos.c`
- Create: `reference/c/leg_pos.h`
- Create: `reference/c/leg_spd.c`
- Create: `reference/c/leg_spd.h`
- Create: `reference/c/leg_conv.c`
- Create: `reference/c/leg_conv.h`

- [ ] Copy only the approved upstream reference files without editing their generated expressions.
- [ ] Record repository URL, exact commit, retrieval date, and GPL-3.0 provenance.
- [ ] Compile the three C sources with `gcc -shared -fPIC -O2 ... -lm` and expect exit code 0.

### Task 2: Forward Kinematics With TDD

**Files:**
- Create: `src/__init__.py`
- Create: `src/parameters.py`
- Create: `src/kinematics.py`
- Create: `tests/test_kinematics.py`

- [ ] Write tests that assert the default parameters, four link lengths, C-point closure, and pose reconstruction from C.
- [ ] Run `pytest tests/test_kinematics.py -v` and confirm failure because `src` is absent.
- [ ] Implement `FiveBarParameters`, `joint_positions`, and `forward_kinematics` from the approved equations.
- [ ] Run the focused test and expect all tests to pass.

### Task 3: Analytic Jacobian And Mappings With TDD

**Files:**
- Modify: `src/kinematics.py`
- Create: `src/vmc.py`
- Create: `tests/test_jacobian_vmc.py`

- [ ] Write tests for Jacobian shape, central finite-difference agreement, velocity mapping, torque mapping, and virtual-work equality.
- [ ] Run `pytest tests/test_jacobian_vmc.py -v` and confirm missing-function failures.
- [ ] Build the pose expressions with SymPy, differentiate them with `diff`, cache lambdified evaluators, and implement the two matrix products.
- [ ] Run both focused test modules and expect all tests to pass.

### Task 4: Original C Comparison With TDD

**Files:**
- Create: `tests/reference_c.py`
- Create: `tests/test_against_original.py`
- Create: `scripts/validate_against_original.py`

- [ ] Write a ctypes wrapper that compiles the original C sources in a pytest temporary directory and exposes all three typed APIs.
- [ ] Write fixed-point and seeded-random comparisons for `leg_pos`, `leg_spd`, and `leg_conv`, initially with deliberately strict zero tolerance to prove the tests detect float differences.
- [ ] Run the focused comparisons, confirm nonzero float32/float64 differences, and replace zero tolerance with evidence-based per-function thresholds.
- [ ] Implement a validation script that uses the same sample generator and prints count, max error, mean error, and threshold failures per scalar output.
- [ ] Run the focused suite and validation script; preserve the measured output for README reporting.

### Task 5: Visualization And Documentation

**Files:**
- Create: `scripts/visualize_leg.py`
- Create: `README.md`
- Create: `requirements.txt`
- Create: `pyproject.toml`
- Create: `artifacts/five_bar_example.png`

- [ ] Write a CLI smoke test that runs the plotter with a known legal pose and checks for a nonempty PNG.
- [ ] Run the smoke test and confirm failure because the script is absent.
- [ ] Implement the plot using `joint_positions`, equal axes, labeled points and links, the virtual leg, and displayed `l0`/`phi0`.
- [ ] Generate `artifacts/five_bar_example.png` and visually inspect it for correct closure and labels.
- [ ] Document formulas, conventions, reference provenance, measured errors, branch limitations, commands, and next-step suitability.

### Task 6: Full Verification

**Files:**
- Modify only files whose own checks fail.

- [ ] Run `ruff check .` and fix only issues introduced by this project.
- [ ] Run `mypy .` and fix only project type errors.
- [ ] Run `pytest -x` and require zero failures.
- [ ] Run `python scripts/validate_against_original.py` and require no threshold failures.
- [ ] Run `python scripts/visualize_leg.py --output artifacts/five_bar_example.png` and require a nonempty image.
- [ ] Re-read the approved design and task requirements, then record any residual limitation honestly in README.
