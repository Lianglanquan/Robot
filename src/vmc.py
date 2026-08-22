from dataclasses import dataclass

import numpy as np

from .kinematics import analytic_jacobian
from .parameters import DEFAULT_PARAMETERS, FiveBarParameters


@dataclass(frozen=True)
class LegVelocity:
    dl0: float
    dphi0: float


@dataclass(frozen=True)
class JointTorques:
    t1: float
    t2: float


def leg_velocity(
    phi1: float,
    phi4: float,
    dphi1: float,
    dphi4: float,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> LegVelocity:
    """Map active-joint rates to virtual-leg extension and angular rates."""
    velocity = analytic_jacobian(phi1, phi4, parameters) @ np.array([dphi1, dphi4])
    return LegVelocity(dl0=float(velocity[0]), dphi0=float(velocity[1]))


def joint_torques(
    force: float,
    virtual_torque: float,
    phi1: float,
    phi4: float,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> JointTorques:
    """Map virtual-leg axial force and torque to active-joint torques."""
    torque = analytic_jacobian(phi1, phi4, parameters).T @ np.array(
        [force, virtual_torque]
    )
    return JointTorques(t1=float(torque[0]), t2=float(torque[1]))
