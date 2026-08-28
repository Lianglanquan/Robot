from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

import mujoco

from .mechanical_envelope import cad_pose
from .mujoco_robot import (
    HIP_CAD_Y_MM,
    REFERENCE_LEG_MM,
    WHEEL_CENTER_X_M,
    WHEEL_RADIUS_M,
    Frame,
    _fixed_visual_geoms,
    _mesh_assets,
    _numbers,
    _quaternion_x,
    joint_configuration,
    posture_frames,
    relative_frame,
)

FloatArray = NDArray[np.float64]
EL05_UNIT_MASS_KG = 0.242
QD4310_UNIT_MASS_KG = 0.127
ACTIVE_LINK_ASSEMBLY_MASS_KG = 0.050
DISTAL_LINK_ASSEMBLY_MASS_KG = 0.040
WHEEL_WITHOUT_MOTOR_MASS_KG = 0.050
DYNAMIC_REFERENCE_ROOT_Z_M = (
    REFERENCE_LEG_MM / 1000.0 - HIP_CAD_Y_MM / 1000.0 + WHEEL_RADIUS_M
)


@dataclass(frozen=True)
class MassBudget:
    total_mass_kg: float
    chassis_lumped_mass_kg: float
    unresolved_fixed_mass_kg: float
    active_links_total_kg: float
    distal_links_total_kg: float
    wheel_assemblies_total_kg: float
    chassis_com_m: tuple[float, float, float]
    chassis_inertia_kg_m2: tuple[float, float, float]

    @property
    def accounted_mass_kg(self) -> float:
        return (
            self.chassis_lumped_mass_kg
            + self.active_links_total_kg
            + self.distal_links_total_kg
            + self.wheel_assemblies_total_kg
        )


@dataclass(frozen=True)
class WholeRobotMassProperties:
    mass_kg: float
    com_m: tuple[float, float, float]
    inertia_at_com_kg_m2: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]


def box_inertia(
    mass_kg: float, dimensions_m: tuple[float, float, float]
) -> tuple[float, float, float]:
    x, y, z = dimensions_m
    return (
        mass_kg * (y**2 + z**2) / 12.0,
        mass_kg * (x**2 + z**2) / 12.0,
        mass_kg * (x**2 + y**2) / 12.0,
    )


def cylinder_inertia_x(
    mass_kg: float, radius_m: float, length_m: float
) -> tuple[float, float, float]:
    axial = 0.5 * mass_kg * radius_m**2
    transverse = mass_kg * (3.0 * radius_m**2 + length_m**2) / 12.0
    return axial, transverse, transverse


def mass_budget(total_mass_kg: float) -> MassBudget:
    if total_mass_kg not in (2.0, 2.3, 2.5):
        raise ValueError("supported dynamic masses are 2.0, 2.3 and 2.5 kg")
    active_total = 4.0 * ACTIVE_LINK_ASSEMBLY_MASS_KG
    distal_total = 4.0 * DISTAL_LINK_ASSEMBLY_MASS_KG
    wheel_unit = QD4310_UNIT_MASS_KG + WHEEL_WITHOUT_MOTOR_MASS_KG
    wheel_total = 2.0 * wheel_unit
    chassis_mass = total_mass_kg - active_total - distal_total - wheel_total
    unresolved_fixed = chassis_mass - 4.0 * EL05_UNIT_MASS_KG
    if unresolved_fixed <= 0.0:
        raise ValueError("mass target cannot contain the known actuator masses")
    chassis_dimensions = (0.160, 0.150, 0.075)
    return MassBudget(
        total_mass_kg=total_mass_kg,
        chassis_lumped_mass_kg=chassis_mass,
        unresolved_fixed_mass_kg=unresolved_fixed,
        active_links_total_kg=active_total,
        distal_links_total_kg=distal_total,
        wheel_assemblies_total_kg=wheel_total,
        chassis_com_m=(0.0, 0.0, 0.025),
        chassis_inertia_kg_m2=box_inertia(chassis_mass, chassis_dimensions),
    )


def whole_robot_mass_properties(
    model: mujoco.MjModel, data: mujoco.MjData
) -> WholeRobotMassProperties:
    """Combine all modeled rigid-body masses in the current world pose."""
    masses = model.body_mass[1:]
    positions = data.xipos[1:]
    total_mass = float(np.sum(masses))
    com = np.sum(masses[:, np.newaxis] * positions, axis=0) / total_mass
    inertia = np.zeros((3, 3), dtype=float)
    identity = np.eye(3)
    for body_id in range(1, model.nbody):
        rotation = data.ximat[body_id].reshape(3, 3)
        body_inertia = rotation @ np.diag(model.body_inertia[body_id]) @ rotation.T
        offset = data.xipos[body_id] - com
        inertia += body_inertia + model.body_mass[body_id] * (
            np.dot(offset, offset) * identity - np.outer(offset, offset)
        )
    return WholeRobotMassProperties(
        mass_kg=total_mass,
        com_m=(float(com[0]), float(com[1]), float(com[2])),
        inertia_at_com_kg_m2=(
            (float(inertia[0, 0]), float(inertia[0, 1]), float(inertia[0, 2])),
            (float(inertia[1, 0]), float(inertia[1, 1]), float(inertia[1, 2])),
            (float(inertia[2, 0]), float(inertia[2, 1]), float(inertia[2, 2])),
        ),
    )


def _dynamic_leg_xml(side: str, reference: dict[str, Frame]) -> str:
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
    wheel_mass = QD4310_UNIT_MASS_KG + WHEEL_WITHOUT_MOTOR_MASS_KG
    wheel_inertia = cylinder_inertia_x(wheel_mass, WHEEL_RADIUS_M, 0.046)
    active_inertia = box_inertia(ACTIVE_LINK_ASSEMBLY_MASS_KG, (0.012, 0.050, 0.018))
    distal_inertia = box_inertia(DISTAL_LINK_ASSEMBLY_MASS_KG, (0.012, 0.105, 0.014))
    return f"""
      <body name="{side}_proximal_negative" pos="{_numbers(proximal_negative.origin_m)}"
            quat="{_quaternion_x(proximal_negative.angle_x_rad)}">
        <inertial pos="{link_x:g} 0.025 0" mass="{ACTIVE_LINK_ASSEMBLY_MASS_KG:g}"
                  diaginertia="{_numbers(active_inertia)}"/>
        <joint name="{side}_hip_negative" type="hinge" axis="1 0 0"
               range="-0.8 0.8" damping="0.08" armature="0.00008"/>
        <geom class="visual" mesh="{side}_proximal_negative" material="active_link"/>
        <geom class="visual" mesh="edulite_{side}_negative_rotor" material="rotor"/>
        <geom name="{side}_proximal_negative_collision" type="capsule"
              fromto="{link_x:g} 0 0 {link_x:g} 0.05 0" size="0.009"
              density="0" contype="2" conaffinity="1" group="3"
              rgba="0.1 0.65 1 0.15"/>
        <body name="{side}_distal_negative" pos="{_numbers(distal_negative.origin_m)}"
              quat="{_quaternion_x(distal_negative.angle_x_rad)}">
          <inertial pos="{link_x:g} 0.0525 0" mass="{DISTAL_LINK_ASSEMBLY_MASS_KG:g}"
                    diaginertia="{_numbers(distal_inertia)}"/>
          <joint name="{side}_knee_negative" type="hinge" axis="1 0 0"
                 range="-0.8 0.8" damping="0.03"/>
          <geom class="visual" mesh="{side}_distal_negative" material="passive_link"/>
          <geom name="{side}_distal_negative_collision" type="capsule"
                fromto="{link_x:g} 0 0 {link_x:g} 0.105 0" size="0.007"
                density="0" contype="2" conaffinity="1" group="3"
                rgba="0.1 0.65 1 0.15"/>
          <body name="{side}_wheel" pos="{_numbers(wheel.origin_m)}"
                quat="{_quaternion_x(wheel_relative_angle)}">
            <inertial pos="{side_x:g} 0 0" mass="{wheel_mass:g}"
                      diaginertia="{_numbers(wheel_inertia)}"/>
            <joint name="{side}_wheel_spin" type="hinge" axis="1 0 0"
                   damping="0.002" armature="0.00004"/>
            <geom class="visual" mesh="{side}_wheel" material="tire"/>
            <geom name="{side}_wheel_collision" type="cylinder"
                  pos="{side_x:g} 0 0" size="{WHEEL_RADIUS_M:g} 0.023"
                  quat="0.70710678 0 0.70710678 0" contype="2" conaffinity="1"
                  condim="4" friction="1.0 0.02 0.001" group="3"
                  rgba="0.1 0.65 1 0.18"/>
            <site name="{side}_wheel_center" size="0.003" rgba="0.1 0.8 1 0.8"/>
          </body>
        </body>
      </body>
      <body name="{side}_proximal_positive" pos="{_numbers(proximal_positive.origin_m)}"
            quat="{_quaternion_x(proximal_positive.angle_x_rad)}">
        <inertial pos="{link_x:g} 0.025 0" mass="{ACTIVE_LINK_ASSEMBLY_MASS_KG:g}"
                  diaginertia="{_numbers(active_inertia)}"/>
        <joint name="{side}_hip_positive" type="hinge" axis="1 0 0"
               range="-0.8 0.8" damping="0.08" armature="0.00008"/>
        <geom class="visual" mesh="{side}_proximal_positive" material="active_link"/>
        <geom class="visual" mesh="edulite_{side}_positive_rotor" material="rotor"/>
        <geom name="{side}_proximal_positive_collision" type="capsule"
              fromto="{link_x:g} 0 0 {link_x:g} 0.05 0" size="0.009"
              density="0" contype="2" conaffinity="1" group="3"
              rgba="0.1 0.65 1 0.15"/>
        <body name="{side}_distal_positive" pos="{_numbers(distal_positive.origin_m)}"
              quat="{_quaternion_x(distal_positive.angle_x_rad)}">
          <inertial pos="{link_x:g} 0.0525 0" mass="{DISTAL_LINK_ASSEMBLY_MASS_KG:g}"
                    diaginertia="{_numbers(distal_inertia)}"/>
          <joint name="{side}_knee_positive" type="hinge" axis="1 0 0"
                 range="-0.8 0.8" damping="0.03"/>
          <geom class="visual" mesh="{side}_distal_positive" material="passive_link"/>
          <geom name="{side}_distal_positive_collision" type="capsule"
                fromto="{link_x:g} 0 0 {link_x:g} 0.105 0" size="0.007"
                density="0" contype="2" conaffinity="1" group="3"
                rgba="0.1 0.65 1 0.15"/>
          <site name="{side}_closure" pos="0 0.105 0" size="0.003"
                rgba="1 0.45 0.1 0.8"/>
        </body>
      </body>"""


def build_dynamic_model_xml(asset_dir: Path, total_mass_kg: float = 2.5) -> str:
    import json

    manifest = json.loads((asset_dir / "manifest.json").read_text(encoding="utf-8"))
    asset_names = sorted(manifest["meshes"])
    reference = posture_frames(cad_pose(REFERENCE_LEG_MM))
    budget = mass_budget(total_mass_kg)
    return f"""<mujoco model="EduLite wheel-legged robot dynamic {total_mass_kg:g} kg">
  <compiler angle="radian" autolimits="true" balanceinertia="true"/>
  <option timestep="0.001" integrator="implicitfast" cone="elliptic"
          gravity="0 0 -9.80665" impratio="10"/>
  <size njmax="2000" nconmax="500"/>
  <visual>
    <global azimuth="135" elevation="-18" offwidth="960" offheight="720"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.75 0.75 0.75"
               specular="0.25 0.25 0.25"/>
    <quality shadowsize="4096" offsamples="4"/>
  </visual>
  <default>
    <default class="visual">
      <geom type="mesh" contype="0" conaffinity="0" group="1" density="0"/>
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
              reflectance="0.08" roughness="0.8"/>
    <material name="step_mat" rgba="0.82 0.30 0.07 1" metallic="0.1" roughness="0.75"/>
  </asset>
  <worldbody>
    <light pos="0 -0.5 1.2" dir="0 0.4 -1" diffuse="0.9 0.88 0.84" castshadow="true"/>
    <light pos="-0.7 0.4 0.6" dir="0.7 -0.2 -0.5" diffuse="0.35 0.42 0.55"/>
    <geom name="ground" type="plane" size="2 2 0.05" material="ground"
          contype="1" conaffinity="1" condim="4" friction="1.0 0.02 0.001"/>
    <geom name="step" type="box" pos="0 -0.38 -1" size="0.40 0.25 0.075"
          material="step_mat" contype="1" conaffinity="1" condim="4"
          rgba="0.82 0.30 0.07 0"
          friction="1.0 0.02 0.001"/>
    <body name="chassis" pos="0 0 {DYNAMIC_REFERENCE_ROOT_Z_M:g}">
      <freejoint name="chassis_free"/>
      <inertial pos="{_numbers(budget.chassis_com_m)}"
                mass="{budget.chassis_lumped_mass_kg:.9g}"
                diaginertia="{_numbers(budget.chassis_inertia_kg_m2)}"/>
{_fixed_visual_geoms()}
      <geom name="chassis_collision" type="box" pos="0 0 0.025"
            size="0.08 0.075 0.0375" density="0"
            contype="2" conaffinity="1" group="3" rgba="0.1 0.65 1 0.12"/>
      <site name="imu" pos="0 0 0.04" size="0.004" rgba="0.2 1 0.3 0.8"/>
{_dynamic_leg_xml("left", reference)}
{_dynamic_leg_xml("right", reference)}
    </body>
    <camera name="overview" mode="trackcom" target="chassis" pos="0.33 -0.42 0.26"
            xyaxes="0.79 0.61 0 -0.22 0.29 0.93"/>
    <camera name="side" mode="trackcom" target="chassis" pos="0 -0.48 0.17"
            xyaxes="1 0 0 0 0.22 0.98"/>
  </worldbody>
  <equality>
    <connect name="left_fivebar_closure" site1="left_closure"
             site2="left_wheel_center" solref="0.003 1"
             solimp="0.97 0.995 0.0005"/>
    <connect name="right_fivebar_closure" site1="right_closure"
             site2="right_wheel_center" solref="0.003 1"
             solimp="0.97 0.995 0.0005"/>
  </equality>
  <actuator>
    <motor name="left_hip_negative_motor" joint="left_hip_negative"
           ctrlrange="-6 6" forcerange="-6 6"/>
    <motor name="left_hip_positive_motor" joint="left_hip_positive"
           ctrlrange="-6 6" forcerange="-6 6"/>
    <motor name="right_hip_negative_motor" joint="right_hip_negative"
           ctrlrange="-6 6" forcerange="-6 6"/>
    <motor name="right_hip_positive_motor" joint="right_hip_positive"
           ctrlrange="-6 6" forcerange="-6 6"/>
    <motor name="left_wheel_motor" joint="left_wheel_spin"
           ctrlrange="-0.3 0.3" forcerange="-0.3 0.3"/>
    <motor name="right_wheel_motor" joint="right_wheel_spin"
           ctrlrange="-0.3 0.3" forcerange="-0.3 0.3"/>
  </actuator>
</mujoco>
"""


@dataclass(frozen=True)
class ControllerGains:
    leg_kp: float = 35.0
    leg_kd: float = 0.65
    pitch_kp: float = 3.4
    pitch_kd: float = 0.32
    position_kp: float = 0.95
    velocity_kd: float = 0.58


def initialize_dynamic_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    l0_mm: float = 90.0,
    drop_height_m: float = 0.0,
    pitch_rad: float = 0.0,
) -> None:
    mujoco.mj_resetData(model, data)
    free_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "chassis_free")
    qpos_address = model.jnt_qposadr[free_id]
    root_z = l0_mm / 1000.0 - HIP_CAD_Y_MM / 1000.0 + WHEEL_RADIUS_M
    data.qpos[qpos_address : qpos_address + 3] = (0.0, 0.0, root_z + drop_height_m)
    data.qpos[qpos_address + 3 : qpos_address + 7] = (
        np.cos(pitch_rad / 2.0),
        np.sin(pitch_rad / 2.0),
        0.0,
        0.0,
    )
    configuration = joint_configuration(l0_mm)
    for name, value in configuration.joint_positions.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint_id]] = value
    mujoco.mj_forward(model, data)


def chassis_pitch(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    rotation = data.xmat[body_id].reshape(3, 3)
    return float(np.arctan2(rotation[2, 1], rotation[2, 2]))


def leg_length_mm(model: mujoco.MjModel, data: mujoco.MjData, side: str) -> float:
    """Measure hip-midpoint to wheel-axis distance in the simulated assembly."""
    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")
    hip_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_hip_{kind}")
        for kind in ("negative", "positive")
    ]
    hip_midpoint = (data.xanchor[hip_ids[0]] + data.xanchor[hip_ids[1]]) / 2.0
    wheel_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_wheel_center"
    )
    return float(np.linalg.norm(data.site_xpos[wheel_id] - hip_midpoint) * 1000.0)


def apply_standing_controller(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    target_l0_mm: float,
    target_y_m: float = 0.0,
    gains: ControllerGains = ControllerGains(),
) -> None:
    targets = joint_configuration(target_l0_mm).joint_positions
    for side in ("left", "right"):
        for kind in ("negative", "positive"):
            joint_name = f"{side}_hip_{kind}"
            actuator_name = f"{joint_name}_motor"
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            actuator_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
            )
            position = data.qpos[model.jnt_qposadr[joint_id]]
            velocity = data.qvel[model.jnt_dofadr[joint_id]]
            torque = (
                gains.leg_kp * (targets[joint_name] - position)
                - gains.leg_kd * velocity
            )
            data.ctrl[actuator_id] = np.clip(torque, -6.0, 6.0)

    free_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "chassis_free")
    qpos_address = model.jnt_qposadr[free_id]
    dof_address = model.jnt_dofadr[free_id]
    position_y = data.qpos[qpos_address + 1]
    velocity_y = data.qvel[dof_address + 1]
    pitch = chassis_pitch(model, data)
    pitch_rate = data.qvel[dof_address + 3]
    wheel_torque = (
        gains.pitch_kp * pitch
        + gains.pitch_kd * pitch_rate
        - gains.position_kp * (position_y - target_y_m)
        - gains.velocity_kd * velocity_y
    )
    for side in ("left", "right"):
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{side}_wheel_motor"
        )
        data.ctrl[actuator_id] = np.clip(wheel_torque, -0.3, 0.3)


def set_step_height(model: mujoco.MjModel, height_m: float) -> None:
    if height_m not in (0.0, 0.05, 0.10, 0.15):
        raise ValueError("step height must be 0, 0.05, 0.10 or 0.15 m")
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "step")
    if height_m == 0.0:
        model.geom_pos[geom_id, 2] = -1.0
        model.geom_rgba[geom_id, 3] = 0.0
    else:
        model.geom_size[geom_id, 2] = height_m / 2.0
        model.geom_pos[geom_id, 2] = height_m / 2.0
        model.geom_rgba[geom_id, 3] = 1.0
