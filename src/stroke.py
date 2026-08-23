from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .analysis import CLASS_CODES, PostureMetrics, analyze_posture
from .parameters import DEFAULT_PARAMETERS, FiveBarParameters

FloatArray = NDArray[np.float64]
NORMAL_PHI0 = np.pi / 2.0
NORMAL_LEFT_SIGN = 1
NORMAL_RIGHT_SIGN = -1
STROKE_COLUMNS = (
    "l0",
    "phi0",
    "phi1",
    "phi4",
    "xc",
    "yc",
    "sigma_min",
    "sigma_max",
    "condition_number",
    "serial_sine_left",
    "serial_sine_right",
    "serial_sine_min",
    "parallel_sine",
    "max_axial_force",
    "path_extension_speed",
    "force_speed_product",
    "dq1_dl0",
    "dq4_dl0",
    "j11",
    "j12",
    "class_code",
)


@dataclass(frozen=True)
class InverseKinematicsCandidate:
    phi1: float
    phi4: float
    left_sign: int
    right_sign: int
    b: FloatArray
    c: FloatArray
    d: FloatArray
    matches_positive_root: bool


@dataclass(frozen=True)
class StrokePosture:
    l0: float
    phi0: float
    phi1: float
    phi4: float
    metrics: PostureMetrics
    dq_dl0: FloatArray
    path_extension_speed: float
    serial_sine_left: float
    serial_sine_right: float
    parallel_sine: float
    force_speed_product: float


@dataclass(frozen=True)
class StrokeScan:
    resolution: int
    lower_singularity_m: float
    upper_singularity_m: float
    endpoint_margin_m: float
    values: FloatArray

    def column(self, name: str) -> FloatArray:
        try:
            index = STROKE_COLUMNS.index(name)
        except ValueError as error:
            raise KeyError(name) from error
        return self.values[:, index]


def _cross2(left: FloatArray, right: FloatArray) -> float:
    return float(left[0] * right[1] - left[1] * right[0])


def _chain_angles(
    target: FloatArray,
    origin: FloatArray,
    proximal_length: float,
    distal_length: float,
) -> tuple[float, float]:
    vector = target - origin
    distance = float(np.linalg.norm(vector))
    lower = abs(distal_length - proximal_length)
    upper = distal_length + proximal_length
    if distance < lower - 1e-14 or distance > upper + 1e-14:
        raise ValueError("target is outside a serial chain workspace")
    if distance <= np.finfo(float).eps:
        raise ValueError("target coincides with an active pivot")
    base_angle = float(np.arctan2(vector[1], vector[0]))
    cosine = (
        proximal_length**2 + distance**2 - distal_length**2
    ) / (2.0 * proximal_length * distance)
    offset = float(np.arccos(np.clip(cosine, -1.0, 1.0)))
    return base_angle + offset, base_angle - offset


def _positive_root_point(
    b: FloatArray, d: FloatArray, parameters: FiveBarParameters
) -> FloatArray | None:
    center_vector = d - b
    center_distance = float(np.linalg.norm(center_vector))
    if center_distance <= np.finfo(float).eps:
        return None
    along = (
        parameters.l2**2 - parameters.l3**2 + center_distance**2
    ) / (2.0 * center_distance)
    height_squared = parameters.l2**2 - along**2
    if height_squared < -1e-14:
        return None
    direction = center_vector / center_distance
    left_normal = np.array([-direction[1], direction[0]])
    return (
        b
        + along * direction
        + np.sqrt(max(0.0, height_squared)) * left_normal
    )


def inverse_kinematics_candidates(
    l0: float,
    phi0: float,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> tuple[InverseKinematicsCandidate, ...]:
    """Return the four serial-chain working modes for one virtual-leg pose."""
    p = parameters
    c = np.array(
        [p.l5 / 2.0 + l0 * np.cos(phi0), l0 * np.sin(phi0)], dtype=float
    )
    a = np.array([0.0, 0.0])
    e = np.array([p.l5, 0.0])
    left_angles = _chain_angles(c, a, p.l1, p.l2)
    right_angles = _chain_angles(c, e, p.l4, p.l3)
    candidates = []
    for left_index, left_sign in enumerate((1, -1)):
        for right_index, right_sign in enumerate((1, -1)):
            phi1 = left_angles[left_index]
            phi4 = right_angles[right_index]
            b = p.l1 * np.array([np.cos(phi1), np.sin(phi1)])
            d = e + p.l4 * np.array([np.cos(phi4), np.sin(phi4)])
            positive_root = _positive_root_point(b, d, p)
            matches = positive_root is not None and bool(
                np.linalg.norm(positive_root - c) <= 2e-13
            )
            candidates.append(
                InverseKinematicsCandidate(
                    phi1=phi1,
                    phi4=phi4,
                    left_sign=left_sign,
                    right_sign=right_sign,
                    b=b,
                    c=c,
                    d=d,
                    matches_positive_root=matches,
                )
            )
    return tuple(candidates)


def vertical_stroke_limits(
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> tuple[float, float]:
    """Return exact centered-vertical reach limits before joint or collision limits."""
    p = parameters
    horizontal_offset = p.l5 / 2.0
    lower_distances = (abs(p.l2 - p.l1), abs(p.l3 - p.l4))
    upper_distances = (p.l1 + p.l2, p.l3 + p.l4)
    lower = max(
        np.sqrt(max(0.0, distance**2 - horizontal_offset**2))
        for distance in lower_distances
    )
    upper_squared = [
        distance**2 - horizontal_offset**2 for distance in upper_distances
    ]
    if min(upper_squared) < 0.0:
        raise ValueError("the centered vertical line is unreachable")
    upper = min(np.sqrt(value) for value in upper_squared)
    if lower >= upper:
        raise ValueError("the centered vertical stroke has no interior")
    return float(lower), float(upper)


def normal_vertical_posture(
    l0: float,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> StrokePosture:
    """Evaluate the symmetric, outward-elbow assembly used by the original robot."""
    lower, upper = vertical_stroke_limits(parameters)
    if not lower < l0 < upper:
        raise ValueError("l0 must lie strictly between the serial singularities")
    candidate = next(
        item
        for item in inverse_kinematics_candidates(l0, NORMAL_PHI0, parameters)
        if item.left_sign == NORMAL_LEFT_SIGN
        and item.right_sign == NORMAL_RIGHT_SIGN
    )
    if not candidate.matches_positive_root:
        raise ValueError("normal mode does not match the positive-root assembly")
    metrics = analyze_posture(candidate.phi1, candidate.phi4, parameters)
    dq_dl0 = np.linalg.solve(metrics.jacobian, np.array([1.0, 0.0]))
    path_extension_speed = 1.0 / float(np.max(np.abs(dq_dl0)))

    a = np.array([0.0, 0.0])
    e = np.array([parameters.l5, 0.0])
    ab = metrics.b - a
    bc = metrics.c - metrics.b
    ed = metrics.d - e
    dc = metrics.c - metrics.d
    serial_sine_left = abs(_cross2(ab, bc)) / (parameters.l1 * parameters.l2)
    serial_sine_right = abs(_cross2(ed, dc)) / (parameters.l4 * parameters.l3)
    parallel_sine = abs(_cross2(bc, dc)) / (parameters.l2 * parameters.l3)

    return StrokePosture(
        l0=l0,
        phi0=NORMAL_PHI0,
        phi1=candidate.phi1,
        phi4=candidate.phi4,
        metrics=metrics,
        dq_dl0=dq_dl0,
        path_extension_speed=path_extension_speed,
        serial_sine_left=serial_sine_left,
        serial_sine_right=serial_sine_right,
        parallel_sine=parallel_sine,
        force_speed_product=metrics.max_axial_force * path_extension_speed,
    )


def _scan_row(posture: StrokePosture) -> tuple[float, ...]:
    metrics = posture.metrics
    return (
        posture.l0,
        posture.phi0,
        posture.phi1,
        posture.phi4,
        metrics.xc,
        metrics.yc,
        metrics.sigma_min,
        metrics.sigma_max,
        metrics.condition_number,
        posture.serial_sine_left,
        posture.serial_sine_right,
        min(posture.serial_sine_left, posture.serial_sine_right),
        posture.parallel_sine,
        metrics.max_axial_force,
        posture.path_extension_speed,
        posture.force_speed_product,
        float(posture.dq_dl0[0]),
        float(posture.dq_dl0[1]),
        float(metrics.jacobian[0, 0]),
        float(metrics.jacobian[0, 1]),
        CLASS_CODES[metrics.classification],
    )


def scan_normal_vertical_stroke(
    resolution: int = 1201,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
    *,
    endpoint_margin_m: float = 1e-6,
) -> StrokeScan:
    """Sample the continuous normal branch without evaluating singular endpoints."""
    if resolution < 2:
        raise ValueError("resolution must be at least 2")
    lower, upper = vertical_stroke_limits(parameters)
    if endpoint_margin_m <= 0.0 or 2.0 * endpoint_margin_m >= upper - lower:
        raise ValueError(
            "endpoint margin must be positive and smaller than half stroke"
        )
    lengths = np.linspace(
        lower + endpoint_margin_m,
        upper - endpoint_margin_m,
        resolution,
    )
    rows = [normal_vertical_posture(float(l0), parameters) for l0 in lengths]
    return StrokeScan(
        resolution=resolution,
        lower_singularity_m=lower,
        upper_singularity_m=upper,
        endpoint_margin_m=endpoint_margin_m,
        values=np.asarray([_scan_row(posture) for posture in rows], dtype=float),
    )
