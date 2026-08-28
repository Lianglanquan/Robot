"""Event-driven first-pass controller for a 5 cm stair approach/climb test.

This is deliberately a small, interpretable state machine.  It does not claim
that the model can climb a stair; it records the first failure phase when it
cannot.
"""

from dataclasses import dataclass
from enum import Enum

import numpy as np

import mujoco

from .kinematics import analytic_jacobian
from .mujoco_dynamics import (
    ControllerGains,
    apply_standing_controller,
    chassis_pitch,
    leg_length_mm,
)
from .parameters import DEFAULT_PARAMETERS
from .stroke import normal_vertical_posture


class StairPhase(str, Enum):
    APPROACH = "APPROACH"
    CROUCH = "CROUCH"
    PUSH = "PUSH"
    FLIGHT = "FLIGHT"
    LANDING = "LANDING"
    RECOVER = "RECOVER"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"


@dataclass(frozen=True)
class StairControllerConfig:
    step_height_m: float = 0.05
    approach_target_y_m: float = -0.16
    crouch_target_y_m: float = -0.16
    push_target_y_m: float = -0.32
    landing_target_y_m: float = -0.32
    recover_target_y_m: float = -0.40
    stand_l0_mm: float = 90.0
    crouch_l0_mm: float = 70.0
    push_l0_mm: float = 120.0
    tuck_l0_mm: float = 120.0
    landing_l0_mm: float = 120.0
    landing_compression_l0_mm: float = 108.0
    push_force_n: float = 80.0
    push_wheel_torque_nm: float = 0.30
    push_duration_s: float = 0.45
    crouch_timeout_s: float = 0.90
    flight_timeout_s: float = 0.80
    min_flight_s: float = 0.15
    landing_timeout_s: float = 0.90
    recover_timeout_s: float = 1.50
    success_hold_s: float = 0.30
    overall_timeout_s: float = 6.0


@dataclass(frozen=True)
class StairTelemetry:
    time_s: float
    phase: StairPhase
    leg_length_mm: float
    chassis_y_m: float
    chassis_z_m: float
    vertical_velocity_m_s: float
    pitch_deg: float
    ground_wheel_contacts: int
    step_wheel_contacts: int
    both_wheels_on_step_top: bool
    max_hip_torque_nm: float
    max_wheel_torque_nm: float


def _geom_id(model: mujoco.MjModel, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


def _contact_flags(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[int, int]:
    ground_id = _geom_id(model, "ground")
    step_id = _geom_id(model, "step")
    wheel_ids = {
        _geom_id(model, f"{side}_wheel_collision")
        for side in ("left", "right")
    }
    ground = 0
    step = 0
    for contact in data.contact[: data.ncon]:
        pair = {contact.geom1, contact.geom2}
        if pair & wheel_ids and ground_id in pair:
            ground += 1
        if pair & wheel_ids and step_id in pair:
            step += 1
    return ground, step


def _wheel_top_flags(
    model: mujoco.MjModel, data: mujoco.MjData, height_m: float
) -> bool:
    step_id = _geom_id(model, "step")
    wheel_ids = {
        side: _geom_id(model, f"{side}_wheel_collision")
        for side in ("left", "right")
    }
    top_contact = {side: False for side in wheel_ids}
    for contact in data.contact[: data.ncon]:
        pair = {contact.geom1, contact.geom2}
        for side, wheel_id in wheel_ids.items():
            if wheel_id in pair and step_id in pair:
                top_contact[side] = True
    for side in wheel_ids:
        site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_wheel_center"
        )
        wheel_top_z = height_m + 0.0257
        if (
            not top_contact[side]
            or data.site_xpos[site_id, 2] < wheel_top_z - 0.008
        ):
            return False
    return True


class StairController:
    """Small closed-loop stair maneuver controller for the MuJoCo model."""

    def __init__(
        self,
        model: mujoco.MjModel,
        config: StairControllerConfig = StairControllerConfig(),
        gains: ControllerGains = ControllerGains(),
    ) -> None:
        if config.step_height_m not in (0.05, 0.10, 0.15):
            raise ValueError("supported stair heights are 0.05, 0.10 and 0.15 m")
        self.model = model
        self.config = config
        self.gains = gains
        self.phase = StairPhase.APPROACH
        self.phase_elapsed_s = 0.0
        self.started_s = 0.0
        self.airborne_seen = False
        self.success_elapsed_s = 0.0
        self.failure_reason = ""

    @property
    def terminal(self) -> bool:
        return self.phase in (StairPhase.SUCCESS, StairPhase.FAILED)

    def reset(self, time_s: float = 0.0) -> None:
        self.phase = StairPhase.APPROACH
        self.phase_elapsed_s = 0.0
        self.started_s = time_s
        self.airborne_seen = False
        self.success_elapsed_s = 0.0
        self.failure_reason = ""

    def _transition(self, phase: StairPhase) -> None:
        self.phase = phase
        self.phase_elapsed_s = 0.0

    def _fail(self, reason: str) -> None:
        self.failure_reason = reason
        self._transition(StairPhase.FAILED)

    def fail(self, reason: str) -> None:
        """Stop the maneuver with an explicit externally detected failure."""
        self._fail(reason)

    def _add_virtual_push(
        self, data: mujoco.MjData, force_n: float, l0_mm: float
    ) -> None:
        posture = normal_vertical_posture(l0_mm / 1000.0, DEFAULT_PARAMETERS)
        torques = (
            analytic_jacobian(posture.phi1, posture.phi4, DEFAULT_PARAMETERS).T
            @ np.array([force_n, 0.0])
        )
        for side in ("left", "right"):
            for index, kind in enumerate(("negative", "positive")):
                actuator_id = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_ACTUATOR,
                    f"{side}_hip_{kind}_motor",
                )
                data.ctrl[actuator_id] = np.clip(
                    data.ctrl[actuator_id] + torques[index], -6.0, 6.0
                )

    def step(self, data: mujoco.MjData) -> StairTelemetry:
        dt = float(self.model.opt.timestep)
        self.phase_elapsed_s += dt
        if self.started_s == 0.0:
            self.started_s = float(data.time)
        ground_contacts, step_contacts = _contact_flags(self.model, data)
        free_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "chassis_free"
        )
        qpos_address = self.model.jnt_qposadr[free_id]
        dof_address = self.model.jnt_dofadr[free_id]
        y = float(data.qpos[qpos_address + 1])
        z = float(data.qpos[qpos_address + 2])
        vz = float(data.qvel[dof_address + 2])
        pitch_deg = float(np.rad2deg(chassis_pitch(self.model, data)))
        l0 = leg_length_mm(self.model, data, "left")
        on_top = _wheel_top_flags(self.model, data, self.config.step_height_m)

        if (
            not self.terminal
            and float(data.time) - self.started_s > self.config.overall_timeout_s
        ):
            self._fail(f"overall timeout in {self.phase.value}")

        if self.phase == StairPhase.APPROACH:
            apply_standing_controller(
                self.model, data, target_l0_mm=self.config.stand_l0_mm,
                target_y_m=self.config.approach_target_y_m, gains=self.gains,
            )
            if step_contacts or self.phase_elapsed_s > 0.80:
                self._transition(StairPhase.CROUCH)

        elif self.phase == StairPhase.CROUCH:
            apply_standing_controller(
                self.model, data, target_l0_mm=self.config.crouch_l0_mm,
                target_y_m=self.config.crouch_target_y_m, gains=self.gains,
            )
            if (
                l0 < self.config.crouch_l0_mm + 3.0
                or self.phase_elapsed_s > self.config.crouch_timeout_s
            ):
                self._transition(StairPhase.PUSH)

        elif self.phase == StairPhase.PUSH:
            apply_standing_controller(
                self.model, data, target_l0_mm=self.config.push_l0_mm,
                target_y_m=self.config.push_target_y_m, gains=self.gains,
            )
            self._add_virtual_push(
                data, self.config.push_force_n, self.config.push_l0_mm
            )
            for side in ("left", "right"):
                actuator_id = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_ACTUATOR,
                    f"{side}_wheel_motor",
                )
                data.ctrl[actuator_id] = self.config.push_wheel_torque_nm
            if ground_contacts == 0 and step_contacts == 0:
                self.airborne_seen = True
                self._transition(StairPhase.FLIGHT)
            elif self.phase_elapsed_s > self.config.push_duration_s:
                self._transition(StairPhase.FLIGHT)

        elif self.phase == StairPhase.FLIGHT:
            apply_standing_controller(
                self.model, data, target_l0_mm=self.config.tuck_l0_mm,
                target_y_m=y, gains=self.gains,
            )
            data.ctrl[4:] = 0.0
            if ground_contacts == 0 and step_contacts == 0:
                self.airborne_seen = True
            if (
                self.airborne_seen
                and self.phase_elapsed_s >= self.config.min_flight_s
                and (ground_contacts > 0 or step_contacts > 0)
            ):
                self._transition(StairPhase.LANDING)
            elif self.phase_elapsed_s > self.config.flight_timeout_s:
                self._fail("no landing contact after push")

        elif self.phase == StairPhase.LANDING:
            apply_standing_controller(
                self.model, data, target_l0_mm=self.config.landing_l0_mm,
                target_y_m=y, gains=self.gains,
            )
            data.ctrl[4:] = 0.0
            if (
                l0 < self.config.landing_compression_l0_mm
                or self.phase_elapsed_s > self.config.landing_timeout_s
            ):
                self._transition(StairPhase.RECOVER)

        elif self.phase == StairPhase.RECOVER:
            apply_standing_controller(
                self.model, data, target_l0_mm=self.config.stand_l0_mm,
                target_y_m=self.config.recover_target_y_m, gains=self.gains,
            )
            stable = on_top and abs(pitch_deg) < 6.0 and abs(vz) < 0.15
            if stable:
                self.success_elapsed_s += dt
                if self.success_elapsed_s >= self.config.success_hold_s:
                    self._transition(StairPhase.SUCCESS)
            else:
                self.success_elapsed_s = 0.0
            if self.phase_elapsed_s > self.config.recover_timeout_s:
                self._fail("recovery did not establish two-wheel top contact")

        elif self.phase == StairPhase.SUCCESS:
            apply_standing_controller(
                self.model, data, target_l0_mm=self.config.stand_l0_mm,
                target_y_m=self.config.recover_target_y_m, gains=self.gains,
            )

        elif self.phase == StairPhase.FAILED:
            data.ctrl[:] = 0.0

        return StairTelemetry(
            time_s=float(data.time), phase=self.phase, leg_length_mm=l0,
            chassis_y_m=y, chassis_z_m=z, vertical_velocity_m_s=vz,
            pitch_deg=pitch_deg, ground_wheel_contacts=ground_contacts,
            step_wheel_contacts=step_contacts, both_wheels_on_step_top=on_top,
            max_hip_torque_nm=float(np.max(np.abs(data.ctrl[:4]))),
            max_wheel_torque_nm=float(np.max(np.abs(data.ctrl[4:]))),
        )
