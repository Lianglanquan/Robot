#!/usr/bin/env python3
"""Open the live MuJoCo model and a visible leg-length control panel."""

import argparse
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from functools import partial
from pathlib import Path

import mujoco.viewer
import numpy as np

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.mechanical_envelope import cad_pose  # noqa: E402
from src.mujoco_robot import set_leg_length  # noqa: E402


@dataclass
class SharedState:
    target_l0_mm: float = 90.0
    auto_cycle: bool = False
    camera_name: str = "overview"
    show_collision: bool = False
    show_joints: bool = False
    closure_error_mm: float = 0.0
    viewer_running: bool = False
    error: str = ""
    stop: threading.Event = field(default_factory=threading.Event)
    lock: threading.Lock = field(default_factory=threading.Lock)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=Path,
        default=PROJECT_ROOT / "mujoco" / "robot.xml",
    )
    return parser.parse_args()


def viewer_loop(model_path: Path, state: SharedState) -> None:
    try:
        model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
        data = mujoco.MjData(model)
        camera_ids = {
            name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
            for name in ("overview", "front", "side")
        }
        start = time.monotonic()
        with mujoco.viewer.launch_passive(
            model,
            data,
            show_left_ui=True,
            show_right_ui=True,
        ) as viewer:
            with state.lock:
                state.viewer_running = True
            while viewer.is_running() and not state.stop.is_set():
                with state.lock:
                    if state.auto_cycle:
                        phase = (time.monotonic() - start) * np.pi / 2.5
                        state.target_l0_mm = 95.0 + 25.0 * float(np.sin(phase))
                    l0_mm = state.target_l0_mm
                    camera_name = state.camera_name
                    show_collision = state.show_collision
                    show_joints = state.show_joints
                with viewer.lock():
                    error_mm = set_leg_length(model, data, l0_mm)
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    viewer.cam.fixedcamid = camera_ids[camera_name]
                    viewer.opt.geomgroup[3] = int(show_collision)
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = int(show_joints)
                with state.lock:
                    state.closure_error_mm = error_mm
                viewer.sync()
                time.sleep(1.0 / 60.0)
    except Exception as error:  # GUI must surface worker failures to the user.
        with state.lock:
            state.error = f"{type(error).__name__}: {error}"
    finally:
        with state.lock:
            state.viewer_running = False
        state.stop.set()


class ControlPanel:
    def __init__(self, root: tk.Tk, state: SharedState):
        self.root = root
        self.state = state
        self.slider_value = tk.DoubleVar(value=90.0)
        self.status = tk.StringVar(value="正在启动 MuJoCo Viewer…")
        self.auto_text = tk.StringVar(value="开始自动伸缩")
        self.collision_value = tk.BooleanVar(value=False)
        self.joint_value = tk.BooleanVar(value=False)
        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(100, self.refresh)

    def _build(self) -> None:
        self.root.title("EduLite 五连杆 · MuJoCo 检查面板")
        self.root.geometry("500x390+10+330")
        self.root.resizable(False, False)
        frame = tk.Frame(self.root, padx=18, pady=16)
        frame.pack(fill="both", expand=True)
        tk.Label(
            frame,
            text="EduLite 五连杆整机 · 运动学检查",
            font=("Sans", 15, "bold"),
        ).pack(anchor="w")
        tk.Label(
            frame,
            text="真实 CAD 外观 + MuJoCo 闭环约束（当前未启用动力学动作）",
            fg="#555555",
        ).pack(anchor="w", pady=(2, 14))

        tk.Label(frame, text="虚拟腿长 l0 [mm]").pack(anchor="w")
        slider = tk.Scale(
            frame,
            from_=70.0,
            to=120.0,
            resolution=0.1,
            orient="horizontal",
            length=465,
            variable=self.slider_value,
            command=self.change_length,
        )
        slider.pack(anchor="w")

        preset_row = tk.Frame(frame)
        preset_row.pack(fill="x", pady=(4, 10))
        for value in (70.0, 90.0, 120.0):
            tk.Button(
                preset_row,
                text=f"{value:g} mm",
                width=9,
                command=partial(self.set_preset, value),
            ).pack(side="left", padx=(0, 8))
        tk.Button(
            preset_row,
            textvariable=self.auto_text,
            width=14,
            command=self.toggle_auto,
        ).pack(side="left")

        camera_row = tk.LabelFrame(frame, text="相机", padx=8, pady=7)
        camera_row.pack(fill="x", pady=(0, 10))
        for name, label in (
            ("overview", "立体"),
            ("front", "正面"),
            ("side", "侧面"),
        ):
            tk.Button(
                camera_row,
                text=label,
                width=9,
                command=partial(self.set_camera, name),
            ).pack(side="left", padx=(0, 8))

        option_row = tk.Frame(frame)
        option_row.pack(fill="x", pady=(0, 10))
        tk.Checkbutton(
            option_row,
            text="显示简化碰撞体",
            variable=self.collision_value,
            command=self.set_options,
        ).pack(side="left")
        tk.Checkbutton(
            option_row,
            text="显示关节轴",
            variable=self.joint_value,
            command=self.set_options,
        ).pack(side="left", padx=(18, 0))

        tk.Label(
            frame,
            textvariable=self.status,
            justify="left",
            anchor="nw",
            relief="groove",
            padx=10,
            pady=8,
            font=("Monospace", 10),
        ).pack(fill="both", expand=True)

    def change_length(self, value: str) -> None:
        with self.state.lock:
            if not self.state.auto_cycle:
                self.state.target_l0_mm = float(value)

    def set_preset(self, value: float) -> None:
        with self.state.lock:
            self.state.auto_cycle = False
            self.state.target_l0_mm = value
        self.slider_value.set(value)
        self.auto_text.set("开始自动伸缩")

    def toggle_auto(self) -> None:
        with self.state.lock:
            self.state.auto_cycle = not self.state.auto_cycle
            enabled = self.state.auto_cycle
        self.auto_text.set("停止自动伸缩" if enabled else "开始自动伸缩")

    def set_camera(self, name: str) -> None:
        with self.state.lock:
            self.state.camera_name = name

    def set_options(self) -> None:
        with self.state.lock:
            self.state.show_collision = self.collision_value.get()
            self.state.show_joints = self.joint_value.get()

    def refresh(self) -> None:
        with self.state.lock:
            l0_mm = self.state.target_l0_mm
            error_mm = self.state.closure_error_mm
            running = self.state.viewer_running
            error = self.state.error
            auto = self.state.auto_cycle
        if auto:
            self.slider_value.set(l0_mm)
        pose = cad_pose(l0_mm)
        if error:
            message = f"Viewer 启动失败\n{error}"
        else:
            message = (
                f"Viewer: {'RUNNING' if running else 'STARTING'}\n"
                f"l0 = {l0_mm:6.2f} mm\n"
                f"phi1 = {pose.phi1_deg:7.3f} deg\n"
                f"phi4 = {pose.phi4_deg:7.3f} deg\n"
                f"左右闭环最大误差 = {error_mm:.6g} mm"
            )
        self.status.set(message)
        if self.state.stop.is_set():
            self.root.after(500, self.close)
        else:
            self.root.after(100, self.refresh)

    def close(self) -> None:
        self.state.stop.set()
        self.root.destroy()


def main() -> int:
    args = parse_args()
    if not args.model.exists():
        raise FileNotFoundError(f"MuJoCo model not found: {args.model}")
    state = SharedState()
    thread = threading.Thread(
        target=viewer_loop,
        args=(args.model, state),
        name="mujoco-viewer",
        daemon=True,
    )
    thread.start()
    root = tk.Tk()
    ControlPanel(root, state)
    root.mainloop()
    state.stop.set()
    thread.join(timeout=3.0)
    return 1 if state.error else 0


if __name__ == "__main__":
    raise SystemExit(main())
