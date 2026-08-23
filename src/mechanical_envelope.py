from dataclasses import dataclass

import numpy as np

from .parameters import DEFAULT_PARAMETERS, FiveBarParameters
from .stroke import normal_vertical_posture, vertical_stroke_limits

CAD_POSE_COLUMNS = (
    "zone",
    "l0_mm",
    "phi1_deg",
    "phi4_deg",
    "theta_bc_deg",
    "theta_dc_deg",
    "ax_mm",
    "ay_mm",
    "bx_mm",
    "by_mm",
    "cx_mm",
    "cy_mm",
    "dx_mm",
    "dy_mm",
    "ex_mm",
    "ey_mm",
)


@dataclass(frozen=True)
class CadPose:
    """One exact Phase 3 posture expressed for rigid CAD placement."""

    zone: str
    l0_mm: float
    phi1_deg: float
    phi4_deg: float
    theta_bc_deg: float
    theta_dc_deg: float
    a_mm: tuple[float, float]
    b_mm: tuple[float, float]
    c_mm: tuple[float, float]
    d_mm: tuple[float, float]
    e_mm: tuple[float, float]

    def row(self) -> tuple[str | float, ...]:
        return (
            self.zone,
            self.l0_mm,
            self.phi1_deg,
            self.phi4_deg,
            self.theta_bc_deg,
            self.theta_dc_deg,
            *self.a_mm,
            *self.b_mm,
            *self.c_mm,
            *self.d_mm,
            *self.e_mm,
        )


def _zone(l0_mm: float) -> str:
    if l0_mm < 70:
        return "short_end_exploration"
    if l0_mm <= 120:
        return "baseline_70_120"
    return "long_end_exploration"


def cad_pose(
    l0_mm: float,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> CadPose:
    """Map a centered vertical Phase 3 posture into CAD pivot coordinates."""
    l0 = l0_mm / 1000.0
    lower, upper = vertical_stroke_limits(parameters)
    if not lower < l0 < upper:
        raise ValueError("l0_mm must lie inside the mathematical stroke")

    posture = normal_vertical_posture(l0, parameters)
    b = posture.metrics.b * 1000.0
    c = posture.metrics.c * 1000.0
    d = posture.metrics.d * 1000.0
    theta_bc = np.arctan2(c[1] - b[1], c[0] - b[0])
    theta_dc = np.arctan2(c[1] - d[1], c[0] - d[0])
    return CadPose(
        zone=_zone(l0_mm),
        l0_mm=float(l0_mm),
        phi1_deg=float(np.rad2deg(posture.phi1)),
        phi4_deg=float(np.rad2deg(posture.phi4)),
        theta_bc_deg=float(np.rad2deg(theta_bc)),
        theta_dc_deg=float(np.rad2deg(theta_dc)),
        a_mm=(0.0, 0.0),
        b_mm=(float(b[0]), float(b[1])),
        c_mm=(float(c[0]), float(c[1])),
        d_mm=(float(d[0]), float(d[1])),
        e_mm=(parameters.l5 * 1000.0, 0.0),
    )


def cad_pose_schedule(
    start_mm: int = 47,
    stop_mm: int = 152,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> tuple[CadPose, ...]:
    """Return the 1 mm collision-scan schedule, including both explorations."""
    if start_mm > stop_mm:
        raise ValueError("start_mm must not exceed stop_mm")
    return tuple(cad_pose(l0_mm, parameters) for l0_mm in range(start_mm, stop_mm + 1))
