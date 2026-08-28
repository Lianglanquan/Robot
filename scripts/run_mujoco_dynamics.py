#!/usr/bin/env python3
"""Run the free-base dynamic robot with a visible Chinese control panel."""

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

from src.mujoco_dynamics import (  # noqa: E402
    apply_standing_controller,
    chassis_pitch,
    initialize_dynamic_state,
    leg_length_mm,
    set_step_height,
)


@dataclass
class SharedState:
    target_l0_mm: float = 90.0
    target_y_m: float = 0.0
    step_height_m: float = 0.0
    running: bool = True
    balance_enabled: bool = True
    camera_name: str = "overview"
    show_collision: bool = False
    reset_request: tuple[float, float] | None = (0.0, 0.0)
    viewer_running: bool = False
    simulation_time_s: float = 0.0
    actual_l0_mm: float = 90.0
    pitch_deg: float = 0.0
    chassis_y_m: float = 0.0
    chassis_z_m: float = 0.0
    contact_count: int = 0
    hip_torque_nm: float = 0.0
    wheel_torque_nm: float = 0.0
    error: str = ""
    stop: threading.Event = field(default_factory=threading.Event)
    lock: threading.Lock = field(default_factory=threading.Lock)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mass", type=float, choices=(2.0, 2.3, 2.5), default=2.5)
    return parser.parse_args()


def viewer_loop(model_path: Path, state: SharedState) -> None:
    try:
        model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
        data = mujoco.MjData(model)
        free_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "chassis_free")
        qpos_address = model.jnt_qposadr[free_id]
        camera_ids = {
            name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
            for name in ("overview", "side")
        }
        accumulator = 0.0
        last_time = time.monotonic()
        applied_step_height = -1.0
        with mujoco.viewer.launch_passive(
            model,
            data,
            show_left_ui=True,
            show_right_ui=True,
        ) as viewer:
            with state.lock:
                state.viewer_running = True
            while viewer.is_running() and not state.stop.is_set():
                now = time.monotonic()
                accumulator += min(now - last_time, 0.05)
                last_time = now
                with state.lock:
                    target_l0_mm = state.target_l0_mm
                    target_y_m = state.target_y_m
                    step_height_m = state.step_height_m
                    running = state.running
                    balance_enabled = state.balance_enabled
                    camera_name = state.camera_name
                    show_collision = state.show_collision
                    reset_request = state.reset_request
                    state.reset_request = None
                if reset_request is not None:
                    drop_height_m, pitch_deg = reset_request
                    initialize_dynamic_state(
                        model,
                        data,
                        l0_mm=target_l0_mm,
                        drop_height_m=drop_height_m,
                        pitch_rad=np.deg2rad(pitch_deg),
                    )
                    accumulator = 0.0
                if step_height_m != applied_step_height:
                    set_step_height(model, step_height_m)
                    applied_step_height = step_height_m
                if running:
                    while accumulator >= model.opt.timestep:
                        if balance_enabled:
                            apply_standing_controller(
                                model,
                                data,
                                target_l0_mm=target_l0_mm,
                                target_y_m=target_y_m,
                            )
                        else:
                            data.ctrl[:] = 0.0
                        mujoco.mj_step(model, data)
                        accumulator -= model.opt.timestep
                else:
                    accumulator = 0.0
                with viewer.lock():
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    viewer.cam.fixedcamid = camera_ids[camera_name]
                    viewer.opt.geomgroup[3] = int(show_collision)
                with state.lock:
                    state.simulation_time_s = float(data.time)
                    state.actual_l0_mm = leg_length_mm(model, data, "left")
                    state.pitch_deg = float(np.rad2deg(chassis_pitch(model, data)))
                    state.chassis_y_m = float(data.qpos[qpos_address + 1])
                    state.chassis_z_m = float(data.qpos[qpos_address + 2])
                    state.contact_count = int(data.ncon)
                    state.hip_torque_nm = float(np.max(np.abs(data.ctrl[:4])))
                    state.wheel_torque_nm = float(np.max(np.abs(data.ctrl[4:])))
                viewer.sync()
                time.sleep(1.0 / 120.0)
    except Exception as error:  # GUI must surface worker failures to the user.
        with state.lock:
            state.error = f"{type(error).__name__}: {error}"
    finally:
        with state.lock:
            state.viewer_running = False
        state.stop.set()


class ControlPanel:
    def __init__(self, root: tk.Tk, state: SharedState, mass_kg: float):
        self.root = root
        self.state = state
        self.mass_kg = mass_kg
        self.length_value = tk.DoubleVar(value=90.0)
        self.position_value = tk.DoubleVar(value=0.0)
        self.balance_value = tk.BooleanVar(value=True)
        self.collision_value = tk.BooleanVar(value=False)
        self.run_text = tk.StringVar(value="暂停仿真")
        self.status = tk.StringVar(value="正在启动 MuJoCo Viewer…")
        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(100, self.refresh)

    def _build(self) -> None:
        self.root.title("轮足机器人 · MuJoCo 动力学控制台")
        self.root.geometry("540x640+10+120")
        self.root.resizable(False, False)
        frame = tk.Frame(self.root, padx=18, pady=14)
        frame.pack(fill="both", expand=True)
        tk.Label(
            frame,
            text="EduLite 轮足整机 · 动力学验证",
            font=("Sans", 15, "bold"),
        ).pack(anchor="w")
        tk.Label(
            frame,
            text=f"自由底盘 · 轮地摩擦 · {self.mass_kg:g} kg 参数模型",
            fg="#555555",
        ).pack(anchor="w", pady=(2, 12))

        tk.Label(frame, text="腿长目标 l0 [mm]").pack(anchor="w")
        tk.Scale(
            frame,
            from_=70.0,
            to=120.0,
            resolution=0.5,
            orient="horizontal",
            length=500,
            variable=self.length_value,
            command=self.change_length,
        ).pack(anchor="w")
        row = tk.Frame(frame)
        row.pack(fill="x", pady=(2, 9))
        for value in (70.0, 90.0, 120.0):
            tk.Button(
                row,
                text=f"{value:g} mm",
                width=10,
                command=partial(self.set_length, value),
            ).pack(side="left", padx=(0, 7))

        tk.Label(frame, text="底盘纵向位置目标 y [m]（负方向朝台阶）").pack(anchor="w")
        tk.Scale(
            frame,
            from_=0.08,
            to=-0.45,
            resolution=0.01,
            orient="horizontal",
            length=500,
            variable=self.position_value,
            command=self.change_position,
        ).pack(anchor="w")

        step_frame = tk.LabelFrame(frame, text="场景台阶", padx=8, pady=7)
        step_frame.pack(fill="x", pady=(7, 9))
        for height_cm in (0, 5, 10, 15):
            tk.Button(
                step_frame,
                text="平地" if height_cm == 0 else f"{height_cm} cm",
                width=9,
                command=partial(self.set_step, height_cm / 100.0),
            ).pack(side="left", padx=(0, 7))

        action_row = tk.Frame(frame)
        action_row.pack(fill="x", pady=(0, 9))
        tk.Button(
            action_row,
            textvariable=self.run_text,
            width=12,
            command=self.toggle_running,
        ).pack(side="left", padx=(0, 7))
        tk.Button(
            action_row,
            text="复位站立",
            width=12,
            command=partial(self.request_reset, 0.0, 0.0),
        ).pack(side="left", padx=(0, 7))
        tk.Button(
            action_row,
            text="5 cm 落地测试",
            width=14,
            command=partial(self.request_reset, 0.05, 2.0),
        ).pack(side="left")

        option_row = tk.Frame(frame)
        option_row.pack(fill="x", pady=(0, 9))
        tk.Checkbutton(
            option_row,
            text="启用站立控制",
            variable=self.balance_value,
            command=self.set_options,
        ).pack(side="left")
        tk.Checkbutton(
            option_row,
            text="显示碰撞体",
            variable=self.collision_value,
            command=self.set_options,
        ).pack(side="left", padx=(18, 0))
        for name, label in (("overview", "立体相机"), ("side", "侧面相机")):
            tk.Button(
                option_row,
                text=label,
                command=partial(self.set_camera, name),
            ).pack(side="right", padx=(7, 0))

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
            self.state.target_l0_mm = float(value)

    def set_length(self, value: float) -> None:
        self.length_value.set(value)
        self.change_length(str(value))

    def change_position(self, value: str) -> None:
        with self.state.lock:
            self.state.target_y_m = float(value)

    def set_step(self, height_m: float) -> None:
        with self.state.lock:
            self.state.step_height_m = height_m

    def toggle_running(self) -> None:
        with self.state.lock:
            self.state.running = not self.state.running
            running = self.state.running
        self.run_text.set("暂停仿真" if running else "继续仿真")

    def request_reset(self, drop_height_m: float, pitch_deg: float) -> None:
        self.position_value.set(0.0)
        with self.state.lock:
            self.state.target_y_m = 0.0
            self.state.reset_request = (drop_height_m, pitch_deg)
            self.state.running = True
        self.run_text.set("暂停仿真")

    def set_options(self) -> None:
        with self.state.lock:
            self.state.balance_enabled = self.balance_value.get()
            self.state.show_collision = self.collision_value.get()

    def set_camera(self, name: str) -> None:
        with self.state.lock:
            self.state.camera_name = name

    def refresh(self) -> None:
        with self.state.lock:
            values = {
                "viewer": "RUNNING" if self.state.viewer_running else "STARTING",
                "time": self.state.simulation_time_s,
                "target_l0": self.state.target_l0_mm,
                "actual_l0": self.state.actual_l0_mm,
                "pitch": self.state.pitch_deg,
                "y": self.state.chassis_y_m,
                "z": self.state.chassis_z_m,
                "contacts": self.state.contact_count,
                "hip": self.state.hip_torque_nm,
                "wheel": self.state.wheel_torque_nm,
                "step": self.state.step_height_m,
                "error": self.state.error,
            }
        if values["error"]:
            message = f"Viewer 启动失败\n{values['error']}"
        else:
            message = (
                f"Viewer: {values['viewer']}   t = {values['time']:7.3f} s\n"
                f"腿长: target {values['target_l0']:6.2f} / "
                f"actual {values['actual_l0']:6.2f} mm\n"
                f"底盘: pitch {values['pitch']:7.3f} deg, "
                f"y {values['y']:7.3f} m, z {values['z']:7.3f} m\n"
                f"接触点: {values['contacts']:2d}   台阶: "
                f"{100.0 * float(values['step']):4.0f} cm\n"
                f"执行器命令: EL05 max {values['hip']:5.3f} N·m / 6.0, "
                f"轮端 {values['wheel']:5.3f} N·m / 0.3\n\n"
                "注意：台阶按钮只切换真实接触场景；尚未加入越阶动作轨迹，\n"
                "因此不能把驶向台阶后的结果解释为已验证越阶能力。"
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
    label = str(args.mass).replace(".", "p")
    model_path = PROJECT_ROOT / "mujoco" / f"robot_dynamic_{label}kg.xml"
    if not model_path.exists():
        raise FileNotFoundError(f"MuJoCo model not found: {model_path}")
    state = SharedState()
    thread = threading.Thread(
        target=viewer_loop,
        args=(model_path, state),
        name="mujoco-dynamics-viewer",
        daemon=True,
    )
    thread.start()
    root = tk.Tk()
    ControlPanel(root, state, args.mass)
    root.mainloop()
    state.stop.set()
    thread.join(timeout=3.0)
    return 1 if state.error else 0


if __name__ == "__main__":
    raise SystemExit(main())
