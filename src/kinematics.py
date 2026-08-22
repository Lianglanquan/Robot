from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .parameters import DEFAULT_PARAMETERS, FiveBarParameters


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class JointPositions:
    a: FloatArray
    b: FloatArray
    c: FloatArray
    d: FloatArray
    e: FloatArray


@dataclass(frozen=True)
class LegPose:
    xc: float
    yc: float
    l0: float
    phi0: float


def joint_positions(
    phi1: float,
    phi4: float,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> JointPositions:
    """Return A-B-C-D-E using the assembly branch from leg_func_calc.m."""
    p = parameters
    a = np.array([0.0, 0.0])
    b = np.array([p.l1 * np.cos(phi1), p.l1 * np.sin(phi1)])
    e = np.array([p.l5, 0.0])
    d = e + np.array([p.l4 * np.cos(phi4), p.l4 * np.sin(phi4)])

    a0 = 2.0 * p.l2 * (d[0] - b[0])
    b0 = 2.0 * p.l2 * (d[1] - b[1])
    c0 = p.l2**2 + (d[0] - b[0]) ** 2 + (d[1] - b[1]) ** 2 - p.l3**2
    discriminant = a0**2 + b0**2 - c0**2
    phi2 = 2.0 * np.arctan((b0 + np.sqrt(discriminant)) / (a0 + c0))
    c = b + np.array([p.l2 * np.cos(phi2), p.l2 * np.sin(phi2)])

    return JointPositions(a=a, b=b, c=c, d=d, e=e)


def forward_kinematics(
    phi1: float,
    phi4: float,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> LegPose:
    """Compute foot coordinates and virtual-leg pose."""
    points = joint_positions(phi1, phi4, parameters)
    virtual_x = points.c[0] - parameters.l5 / 2.0
    virtual_y = points.c[1]
    return LegPose(
        xc=float(points.c[0]),
        yc=float(points.c[1]),
        l0=float(np.hypot(virtual_x, virtual_y)),
        phi0=float(np.arctan2(virtual_y, virtual_x)),
    )
