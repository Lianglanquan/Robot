from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

import mujoco

from .mechanical_envelope import CadPose, cad_pose

FloatArray = NDArray[np.float64]
REFERENCE_LEG_MM = 90.0
HIP_CAD_Y_MM = 34.0
MODEL_ROOT_Z_M = 0.120
WHEEL_RADIUS_M = 0.0257
WHEEL_CENTER_X_M = 0.083


@dataclass(frozen=True)
class Frame:
    origin_m: FloatArray
    angle_x_rad: float


@dataclass(frozen=True)
class MuJoCoConfiguration:
    l0_mm: float
    joint_positions: dict[str, float]
    expected_axle_m: FloatArray


def _point_mj(point_mm: tuple[float, float]) -> FloatArray:
    math_x, math_y = point_mm
    return np.array([0.0, (math_x - 30.0) / 1000.0, (HIP_CAD_Y_MM - math_y) / 1000.0])


def _angle_mj(angle_deg: float) -> float:
    return -float(np.deg2rad(angle_deg))


def posture_frames(pose: CadPose) -> dict[str, Frame]:
    return {
        "proximal_negative": Frame(_point_mj(pose.a_mm), _angle_mj(pose.phi1_deg)),
        "proximal_positive": Frame(_point_mj(pose.e_mm), _angle_mj(pose.phi4_deg)),
        "distal_negative": Frame(_point_mj(pose.b_mm), _angle_mj(pose.theta_bc_deg)),
        "distal_positive": Frame(_point_mj(pose.d_mm), _angle_mj(pose.theta_dc_deg)),
        "wheel": Frame(_point_mj(pose.c_mm), 0.0),
    }


def rotate_x(vector: FloatArray, angle_rad: float) -> FloatArray:
    cosine = np.cos(angle_rad)
    sine = np.sin(angle_rad)
    x, y, z = vector
    return np.array([x, cosine * y - sine * z, sine * y + cosine * z], dtype=float)


def relative_frame(child: Frame, parent: Frame) -> Frame:
    return Frame(
        rotate_x(child.origin_m - parent.origin_m, -parent.angle_x_rad),
        child.angle_x_rad - parent.angle_x_rad,
    )


def joint_configuration(l0_mm: float) -> MuJoCoConfiguration:
    reference = posture_frames(cad_pose(REFERENCE_LEG_MM))
    current_pose = cad_pose(l0_mm)
    current = posture_frames(current_pose)
    negative_relative = (
        current["distal_negative"].angle_x_rad
        - current["proximal_negative"].angle_x_rad
    )
    negative_reference = (
        reference["distal_negative"].angle_x_rad
        - reference["proximal_negative"].angle_x_rad
    )
    positive_relative = (
        current["distal_positive"].angle_x_rad
        - current["proximal_positive"].angle_x_rad
    )
    positive_reference = (
        reference["distal_positive"].angle_x_rad
        - reference["proximal_positive"].angle_x_rad
    )
    values = {
        "hip_negative": (
            current["proximal_negative"].angle_x_rad
            - reference["proximal_negative"].angle_x_rad
        ),
        "hip_positive": (
            current["proximal_positive"].angle_x_rad
            - reference["proximal_positive"].angle_x_rad
        ),
        "knee_negative": negative_relative - negative_reference,
        "knee_positive": positive_relative - positive_reference,
    }
    joint_positions = {
        f"{side}_{joint}": value
        for side in ("left", "right")
        for joint, value in values.items()
    }
    return MuJoCoConfiguration(
        l0_mm=l0_mm,
        joint_positions=joint_positions,
        expected_axle_m=current["wheel"].origin_m
        + np.array([0.0, 0.0, MODEL_ROOT_Z_M]),
    )


def set_leg_length(model: mujoco.MjModel, data: mujoco.MjData, l0_mm: float) -> float:
    """Apply one exact Phase 3 pose and return maximum closure error in mm."""
    configuration = joint_configuration(l0_mm)
    for name, value in configuration.joint_positions.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint_id]] = value
    mujoco.mj_forward(model, data)
    errors = []
    for side in ("left", "right"):
        closure_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_closure"
        )
        wheel_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_wheel_center"
        )
        errors.append(
            float(np.linalg.norm(data.site_xpos[closure_id] - data.site_xpos[wheel_id]))
        )
    return max(errors) * 1000.0


def _numbers(values: FloatArray | tuple[float, ...]) -> str:
    return " ".join(f"{float(value):.10g}" for value in values)


def _quaternion_x(angle_rad: float) -> str:
    return _numbers((np.cos(angle_rad / 2.0), np.sin(angle_rad / 2.0), 0.0, 0.0))


def _mesh_assets(asset_names: list[str]) -> str:
    return "\n".join(
        f'    <mesh name="{name}" file="assets/{name}.obj"/>' for name in asset_names
    )


def _fixed_visual_geoms() -> str:
    entries = (
        ("baseplate", "carbon"),
        ("left_bracket", "bracket"),
        ("right_bracket", "bracket"),
        ("fixed_fasteners", "steel"),
        ("edulite_left_negative_stator", "motor"),
        ("edulite_left_positive_stator", "motor"),
        ("edulite_right_negative_stator", "motor"),
        ("edulite_right_positive_stator", "motor"),
    )
    return "\n".join(
        f'      <geom class="visual" mesh="{mesh}" material="{material}"/>'
        for mesh, material in entries
    )


def _leg_xml(side: str, reference: dict[str, Frame]) -> str:
    side_x = -WHEEL_CENTER_X_M if side == "left" else WHEEL_CENTER_X_M
    link_x = -0.070 if side == "left" else 0.070
    proximal_negative = reference["proximal_negative"]
    proximal_positive = reference["proximal_positive"]
    distal_negative = relative_frame(reference["distal_negative"], proximal_negative)
    distal_positive = relative_frame(reference["distal_positive"], proximal_positive)
    wheel = relative_frame(reference["wheel"], reference["distal_negative"])
    wheel_relative_angle = (
        reference["wheel"].angle_x_rad - reference["distal_negative"].angle_x_rad
    )
    return f"""
      <body name="{side}_proximal_negative" pos="{_numbers(proximal_negative.origin_m)}"
            quat="{_quaternion_x(proximal_negative.angle_x_rad)}">
        <inertial pos="{link_x:g} 0.025 0" mass="0.05" diaginertia="2e-5 1e-5 2e-5"/>
        <joint name="{side}_hip_negative" type="hinge" axis="1 0 0"
               range="-0.8 0.8" damping="0.05"/>
        <geom class="visual" mesh="{side}_proximal_negative" material="active_link"/>
        <geom class="visual" mesh="edulite_{side}_negative_rotor" material="rotor"/>
        <geom class="collision" type="capsule"
              fromto="{link_x:g} 0 0 {link_x:g} 0.05 0" size="0.009"/>
        <body name="{side}_distal_negative" pos="{_numbers(distal_negative.origin_m)}"
              quat="{_quaternion_x(distal_negative.angle_x_rad)}">
          <inertial pos="{link_x:g} 0.0525 0" mass="0.04"
                    diaginertia="4e-5 5e-6 4e-5"/>
          <joint name="{side}_knee_negative" type="hinge" axis="1 0 0"
                 range="-0.8 0.8" damping="0.03"/>
          <geom class="visual" mesh="{side}_distal_negative" material="passive_link"/>
          <geom class="collision" type="capsule"
                fromto="{link_x:g} 0 0 {link_x:g} 0.105 0" size="0.007"/>
          <body name="{side}_wheel" pos="{_numbers(wheel.origin_m)}"
                quat="{_quaternion_x(wheel_relative_angle)}">
            <inertial pos="{side_x:g} 0 0" mass="0.12"
                      diaginertia="4e-5 7e-5 7e-5"/>
            <joint name="{side}_wheel_spin" type="hinge" axis="1 0 0" damping="0.01"/>
            <geom class="visual" mesh="{side}_wheel" material="tire"/>
            <geom class="collision" type="cylinder" pos="{side_x:g} 0 0"
                  size="0.0257 0.023" quat="0.70710678 0 0.70710678 0"/>
            <site name="{side}_wheel_center" size="0.003" rgba="0.1 0.8 1 0.8"/>
          </body>
        </body>
      </body>
      <body name="{side}_proximal_positive" pos="{_numbers(proximal_positive.origin_m)}"
            quat="{_quaternion_x(proximal_positive.angle_x_rad)}">
        <inertial pos="{link_x:g} 0.025 0" mass="0.05" diaginertia="2e-5 1e-5 2e-5"/>
        <joint name="{side}_hip_positive" type="hinge" axis="1 0 0"
               range="-0.8 0.8" damping="0.05"/>
        <geom class="visual" mesh="{side}_proximal_positive" material="active_link"/>
        <geom class="visual" mesh="edulite_{side}_positive_rotor" material="rotor"/>
        <geom class="collision" type="capsule"
              fromto="{link_x:g} 0 0 {link_x:g} 0.05 0" size="0.009"/>
        <body name="{side}_distal_positive" pos="{_numbers(distal_positive.origin_m)}"
              quat="{_quaternion_x(distal_positive.angle_x_rad)}">
          <inertial pos="{link_x:g} 0.0525 0" mass="0.04"
                    diaginertia="4e-5 5e-6 4e-5"/>
          <joint name="{side}_knee_positive" type="hinge" axis="1 0 0"
                 range="-0.8 0.8" damping="0.03"/>
          <geom class="visual" mesh="{side}_distal_positive" material="passive_link"/>
          <geom class="collision" type="capsule"
                fromto="{link_x:g} 0 0 {link_x:g} 0.105 0" size="0.007"/>
          <site name="{side}_closure" pos="0 0.105 0" size="0.003"
                rgba="1 0.45 0.1 0.8"/>
        </body>
      </body>"""


def build_model_xml(asset_dir: Path) -> str:
    manifest_path = asset_dir / "manifest.json"
    if not manifest_path.exists():
        raise FileNotFoundError(f"missing visual-asset manifest: {manifest_path}")
    import json

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    asset_names = sorted(manifest["meshes"])
    reference = posture_frames(cad_pose(REFERENCE_LEG_MM))
    return f"""<mujoco model="EduLite wheel-legged robot visual checkpoint">
  <compiler angle="radian" autolimits="true" balanceinertia="true"/>
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.80665"/>
  <size njmax="1000" nconmax="400"/>
  <visual>
    <global azimuth="135" elevation="-18" offwidth="720" offheight="540"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.75 0.75 0.75"
               specular="0.25 0.25 0.25"/>
    <quality shadowsize="4096" offsamples="4"/>
  </visual>
  <default>
    <default class="visual">
      <geom type="mesh" contype="0" conaffinity="0" group="1" density="0"/>
    </default>
    <default class="collision">
      <geom contype="0" conaffinity="0" group="3" density="0" rgba="0.2 0.7 1 0.12"/>
    </default>
  </default>
  <asset>
{_mesh_assets(asset_names)}
    <material name="carbon" rgba="0.055 0.065 0.08 1" metallic="0.45" roughness="0.38"/>
    <material name="bracket" rgba="0.88 0.22 0.055 1" metallic="0.35" roughness="0.3"/>
    <material name="motor" rgba="0.10 0.115 0.14 1" metallic="0.7" roughness="0.24"/>
    <material name="rotor" rgba="0.33 0.36 0.41 1" metallic="0.8" roughness="0.2"/>
    <material name="active_link" rgba="0.86 0.12 0.08 1"
              metallic="0.65" roughness="0.25"/>
    <material name="passive_link" rgba="0.66 0.69 0.74 1"
              metallic="0.8" roughness="0.22"/>
    <material name="steel" rgba="0.62 0.65 0.70 1" metallic="0.9" roughness="0.16"/>
    <material name="tire" rgba="0.025 0.03 0.04 1" metallic="0.05" roughness="0.78"/>
    <texture name="ground_tex" type="2d" builtin="checker" rgb1="0.16 0.18 0.22"
             rgb2="0.08 0.09 0.11" width="512" height="512"/>
    <material name="ground" texture="ground_tex" texrepeat="4 4"
              reflectance="0.12" roughness="0.72"/>
  </asset>
  <worldbody>
    <light pos="0 -0.5 1.2" dir="0 0.4 -1" diffuse="0.9 0.88 0.84" castshadow="true"/>
    <light pos="-0.7 0.4 0.6" dir="0.7 -0.2 -0.5" diffuse="0.35 0.42 0.55"/>
    <geom name="display_floor" type="plane" pos="0 0 -0.04" size="1.5 1.5 0.05"
          material="ground" contype="0" conaffinity="0"/>
    <body name="chassis" pos="0 0 {MODEL_ROOT_Z_M:g}">
      <inertial pos="0 0.005 0" mass="1.9" diaginertia="0.012 0.018 0.02"/>
{_fixed_visual_geoms()}
{_leg_xml("left", reference)}
{_leg_xml("right", reference)}
    </body>
    <camera name="overview" pos="0.31 -0.31 0.235"
            xyaxes="0.707 0.707 0 -0.28 0.28 0.918"/>
    <camera name="front" pos="0.38 0 0.14" xyaxes="0 1 0 -0.2 0 0.98"/>
    <camera name="side" pos="0 -0.40 0.15" xyaxes="1 0 0 0 0.2 0.98"/>
  </worldbody>
  <equality>
    <connect name="left_fivebar_closure" site1="left_closure" site2="left_wheel_center"
             solref="0.004 1" solimp="0.95 0.99 0.001"/>
    <connect name="right_fivebar_closure" site1="right_closure"
             site2="right_wheel_center"
             solref="0.004 1" solimp="0.95 0.99 0.001"/>
  </equality>
</mujoco>
"""
