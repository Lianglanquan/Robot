from collections.abc import Callable
from dataclasses import dataclass
from functools import lru_cache
from typing import Any

import numpy as np
import sympy as sp
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


NumericFunction = Callable[..., Any]


@lru_cache(maxsize=1)
def _model_functions() -> tuple[NumericFunction, NumericFunction]:
    phi1, phi4 = sp.symbols("phi1 phi4", real=True)
    l1, l2, l3, l4, l5 = sp.symbols("l1 l2 l3 l4 l5", positive=True)

    xb = l1 * sp.cos(phi1)
    yb = l1 * sp.sin(phi1)
    xd = l5 + l4 * sp.cos(phi4)
    yd = l4 * sp.sin(phi4)
    a0 = 2 * l2 * (xd - xb)
    b0 = 2 * l2 * (yd - yb)
    c0 = l2**2 + (xd - xb) ** 2 + (yd - yb) ** 2 - l3**2
    phi2 = 2 * sp.atan((b0 + sp.sqrt(a0**2 + b0**2 - c0**2)) / (a0 + c0))

    xc = xb + l2 * sp.cos(phi2)
    yc = yb + l2 * sp.sin(phi2)
    virtual_x = xc - l5 / 2
    l0 = sp.sqrt(virtual_x**2 + yc**2)
    phi0 = sp.atan2(yc, virtual_x)

    jacobian_expression = sp.Matrix(
        [
            [sp.diff(l0, phi1), sp.diff(l0, phi4)],
            [sp.diff(phi0, phi1), sp.diff(phi0, phi4)],
        ]
    )
    arguments = (phi1, phi4, l1, l2, l3, l4, l5)
    pose_function = sp.lambdify(arguments, (xc, yc, l0, phi0), "numpy", cse=True)
    jacobian_function = sp.lambdify(arguments, jacobian_expression, "numpy", cse=True)
    return pose_function, jacobian_function


def _arguments(
    phi1: float, phi4: float, parameters: FiveBarParameters
) -> tuple[float, ...]:
    return (
        phi1,
        phi4,
        parameters.l1,
        parameters.l2,
        parameters.l3,
        parameters.l4,
        parameters.l5,
    )


def _pose_values(
    phi1: float, phi4: float, parameters: FiveBarParameters
) -> FloatArray:
    pose_function, _ = _model_functions()
    return np.asarray(pose_function(*_arguments(phi1, phi4, parameters)), dtype=float)


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
    xc, yc, _, _ = _pose_values(phi1, phi4, parameters)
    c = np.array([xc, yc])

    return JointPositions(a=a, b=b, c=c, d=d, e=e)


def forward_kinematics(
    phi1: float,
    phi4: float,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> LegPose:
    """Compute foot coordinates and virtual-leg pose."""
    xc, yc, l0, phi0 = _pose_values(phi1, phi4, parameters)
    return LegPose(
        xc=float(xc),
        yc=float(yc),
        l0=float(l0),
        phi0=float(phi0),
    )


def analytic_jacobian(
    phi1: float,
    phi4: float,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> FloatArray:
    """Return d(l0, phi0) / d(phi1, phi4) from SymPy differentiation."""
    _, jacobian_function = _model_functions()
    return np.asarray(
        jacobian_function(*_arguments(phi1, phi4, parameters)), dtype=float
    ).reshape(2, 2)


def finite_difference_jacobian(
    phi1: float,
    phi4: float,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
    *,
    step: float = 1e-7,
) -> FloatArray:
    """Central-difference cross-check for the analytic Jacobian."""
    result = np.empty((2, 2), dtype=float)
    for column, delta in enumerate(((step, 0.0), (0.0, step))):
        plus = forward_kinematics(phi1 + delta[0], phi4 + delta[1], parameters)
        minus = forward_kinematics(phi1 - delta[0], phi4 - delta[1], parameters)
        result[0, column] = (plus.l0 - minus.l0) / (2.0 * step)
        angle_delta = np.arctan2(
            np.sin(plus.phi0 - minus.phi0), np.cos(plus.phi0 - minus.phi0)
        )
        result[1, column] = angle_delta / (2.0 * step)
    return result
