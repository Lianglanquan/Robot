#!/usr/bin/env python3
"""Parallel, deterministic search for safe 5 cm stair actions."""

import argparse
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import cast

import numpy as np

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mujoco_dynamics import (  # noqa: E402
    ControllerGains,
    initialize_dynamic_state,
    set_wheel_torque_limit,
)
from src.stair_controller import StairController, StairControllerConfig  # noqa: E402


@dataclass(frozen=True)
class ActionCandidate:
    crouch_l0_mm: float
    push_l0_mm: float
    push_force_n: float
    push_duration_s: float
    push_wheel_torque_nm: float
    landing_l0_mm: float
    landing_compression_l0_mm: float


def sample_candidates(count: int, seed: int) -> list[ActionCandidate]:
    if count < 1:
        raise ValueError("count must be positive")
    rng = np.random.default_rng(seed)
    candidates: list[ActionCandidate] = []
    for _ in range(count):
        crouch = float(rng.uniform(62.0, 82.0))
        push = float(rng.uniform(max(105.0, crouch + 15.0), 145.0))
        candidates.append(
            ActionCandidate(
                crouch_l0_mm=crouch,
                push_l0_mm=push,
                push_force_n=float(rng.uniform(20.0, 140.0)),
                push_duration_s=float(rng.uniform(0.10, 0.50)),
                push_wheel_torque_nm=float(rng.uniform(0.0, 1.0)),
                landing_l0_mm=float(rng.uniform(105.0, 125.0)),
                landing_compression_l0_mm=float(rng.uniform(92.0, 115.0)),
            )
        )
    return candidates


def evaluate_candidate(
    task: tuple[float, float, ActionCandidate]
) -> dict[str, object]:
    mass_kg, height_m, candidate = task
    label = str(mass_kg).replace(".", "p")
    height_label = f"{height_m * 100:g}"
    model = mujoco.MjModel.from_xml_path(
        str(
            PROJECT_ROOT
            / "mujoco"
            / f"robot_dynamic_{label}kg_step_{height_label}cm.xml"
        )
    )
    wheel_limit = 1.0
    set_wheel_torque_limit(model, wheel_limit)
    data = mujoco.MjData(model)
    initialize_dynamic_state(model, data, l0_mm=90.0)
    controller = StairController(
        model,
        StairControllerConfig(
            step_height_m=height_m,
            crouch_l0_mm=candidate.crouch_l0_mm,
            push_l0_mm=candidate.push_l0_mm,
            push_force_n=candidate.push_force_n,
            push_duration_s=candidate.push_duration_s,
            push_wheel_torque_nm=candidate.push_wheel_torque_nm,
            landing_l0_mm=candidate.landing_l0_mm,
            landing_compression_l0_mm=candidate.landing_compression_l0_mm,
        ),
        gains=ControllerGains(wheel_torque_limit_nm=wheel_limit),
    )
    phases: list[str] = []
    max_landing_speed = 0.0
    step_contact = False
    for _ in range(round(8.0 / model.opt.timestep)):
        telemetry = controller.step(data)
        mujoco.mj_step(model, data)
        phases.append(telemetry.phase.value)
        step_contact |= telemetry.step_wheel_contacts > 0
        if telemetry.phase.value == "LANDING":
            max_landing_speed = max(
                max_landing_speed, abs(telemetry.vertical_velocity_m_s)
            )
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            controller.fail("non-finite state")
        if controller.terminal:
            break

    max_pitch = controller.max_abs_pitch_deg
    pitch_margin = StairControllerConfig.max_abs_pitch_deg - max_pitch
    landing_margin = StairControllerConfig.max_landing_speed_m_s - max_landing_speed
    score = 10000.0 if controller.phase.value == "SUCCESS" else 0.0
    score += 12.0 * min(pitch_margin, 0.0)
    score += 8.0 * min(landing_margin, 0.0)
    score += 20.0 if step_contact else 0.0
    score -= 0.5 * float(data.time)
    return {
        **asdict(candidate),
        "mass_kg": mass_kg,
        "height_cm": height_m * 100.0,
        "final_phase": controller.phase.value,
        "failure_reason": controller.failure_reason,
        "score": score,
        "max_abs_pitch_deg": max_pitch,
        "max_landing_speed_m_s": max_landing_speed,
        "pitch_margin_deg": pitch_margin,
        "landing_speed_margin_m_s": landing_margin,
        "step_contact_observed": step_contact,
        "terminal_time_s": float(data.time),
        "phases_observed": list(dict.fromkeys(phases)),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Search 5 cm stair actions with independent MuJoCo workers."
    )
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--mass", type=float, default=2.5)
    parser.add_argument("--height-cm", type=float, default=5.0)
    parser.add_argument("--seed", type=int, default=20260829)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args(argv)
    if args.workers < 1 or args.samples < 1:
        parser.error("--workers and --samples must be >= 1")
    if args.height_cm not in (5.0, 10.0, 15.0):
        parser.error("--height-cm must be 5, 10 or 15")

    candidates = sample_candidates(args.samples, args.seed)
    tasks = [(args.mass, args.height_cm / 100.0, candidate) for candidate in candidates]
    worker_count = min(args.workers, len(tasks))
    if worker_count == 1:
        results = [evaluate_candidate(task) for task in tasks]
    else:
        with ProcessPoolExecutor(max_workers=worker_count) as executor:
            results = list(executor.map(evaluate_candidate, tasks))
    results.sort(
        key=lambda result: float(cast(float, result["score"])), reverse=True
    )

    output = args.output or (
        PROJECT_ROOT / "artifacts" / "stair_controller" / "action_search.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(results, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    successes = sum(result["final_phase"] == "SUCCESS" for result in results)
    print(
        f"evaluated {len(results)} candidates with {worker_count} worker(s); "
        f"safe successes: {successes}; host CPUs: {os.cpu_count() or 1}"
    )
    for result in results[:10]:
        print(
            f"{result['score']:8.1f} {result['final_phase']:6s} "
            f"pitch={result['max_abs_pitch_deg']:.1f} deg "
            f"push={result['push_l0_mm']:.1f} mm/{result['push_duration_s']:.3f} s "
            f"force={result['push_force_n']:.1f} N"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
