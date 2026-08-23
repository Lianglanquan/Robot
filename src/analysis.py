from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .parameters import DEFAULT_PARAMETERS, FiveBarParameters


FloatArray = NDArray[np.float64]
RECOMMENDED_CONDITION_MAX = 5.0
RECOMMENDED_SIGMA_MIN = 0.010
USABLE_CONDITION_MAX = 20.0
USABLE_SIGMA_MIN = 0.002


@dataclass(frozen=True)
class PostureMetrics:
    phi1: float
    phi4: float
    b: FloatArray
    c: FloatArray
    d: FloatArray
    xc: float
    yc: float
    l0: float
    phi0: float
    jacobian: FloatArray
    physical_jacobian: FloatArray
    raw_sigma_min: float
    raw_condition_number: float
    sigma_min: float
    sigma_max: float
    condition_number: float
    determinant: float
    max_axial_force: float
    max_extension_speed: float
    classification: str


def _condition_number(singular_values: FloatArray) -> float:
    if singular_values[-1] == 0.0:
        return float("inf")
    return float(singular_values[0] / singular_values[-1])


def classify_posture(sigma_min: float, condition_number: float) -> str:
    """Classify numerical transmission quality using documented thresholds."""
    if (
        condition_number <= RECOMMENDED_CONDITION_MAX
        and sigma_min >= RECOMMENDED_SIGMA_MIN
    ):
        return "recommended"
    if condition_number < USABLE_CONDITION_MAX and sigma_min >= USABLE_SIGMA_MIN:
        return "usable"
    return "near_singular"


def analyze_posture(
    phi1: float,
    phi4: float,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> PostureMetrics:
    """Evaluate one posture with stable circle geometry and loop constraints."""
    p = parameters
    b = p.l1 * np.array([np.cos(phi1), np.sin(phi1)])
    d = np.array([p.l5, 0.0]) + p.l4 * np.array(
        [np.cos(phi4), np.sin(phi4)]
    )
    center_vector = d - b
    center_distance = float(np.linalg.norm(center_vector))
    if center_distance <= np.finfo(float).eps:
        raise ValueError("coupler-circle centers coincide")

    along = (
        p.l2**2 - p.l3**2 + center_distance**2
    ) / (2.0 * center_distance)
    height_squared = p.l2**2 - along**2
    if height_squared < -1e-15:
        raise ValueError("coupler circles do not intersect")

    direction = center_vector / center_distance
    left_normal = np.array([-direction[1], direction[0]])
    c = b + along * direction + np.sqrt(max(0.0, height_squared)) * left_normal

    bc = c - b
    dc = c - d
    constraint_matrix = np.vstack([bc, dc])
    b_phi = p.l1 * np.array([-np.sin(phi1), np.cos(phi1)])
    d_phi = p.l4 * np.array([-np.sin(phi4), np.cos(phi4)])
    constraint_rates = np.diag([float(bc @ b_phi), float(dc @ d_phi)])
    try:
        cartesian_jacobian = np.linalg.solve(constraint_matrix, constraint_rates)
    except np.linalg.LinAlgError as error:
        raise ValueError("coupler links are parallel") from error

    virtual_vector = c - np.array([p.l5 / 2.0, 0.0])
    l0 = float(np.linalg.norm(virtual_vector))
    if l0 <= np.finfo(float).eps:
        raise ValueError("virtual leg has zero length")
    radial = virtual_vector / l0
    tangential = np.array([-radial[1], radial[0]])
    jacobian = np.vstack([radial, tangential / l0]) @ cartesian_jacobian
    physical_jacobian = np.vstack([radial, tangential]) @ cartesian_jacobian

    raw_singular_values = np.linalg.svd(jacobian, compute_uv=False)
    singular_values = np.linalg.svd(physical_jacobian, compute_uv=False)
    radial_coefficients = np.abs(jacobian[0])
    force_denominator = float(np.max(radial_coefficients))
    max_axial_force = (
        float("inf") if force_denominator == 0.0 else 1.0 / force_denominator
    )
    sigma_min = float(singular_values[-1])
    condition_number = _condition_number(singular_values)

    return PostureMetrics(
        phi1=phi1,
        phi4=phi4,
        b=b,
        c=c,
        d=d,
        xc=float(c[0]),
        yc=float(c[1]),
        l0=l0,
        phi0=float(np.arctan2(virtual_vector[1], virtual_vector[0])),
        jacobian=jacobian,
        physical_jacobian=physical_jacobian,
        raw_sigma_min=float(raw_singular_values[-1]),
        raw_condition_number=_condition_number(raw_singular_values),
        sigma_min=sigma_min,
        sigma_max=float(singular_values[0]),
        condition_number=condition_number,
        determinant=float(np.linalg.det(physical_jacobian)),
        max_axial_force=max_axial_force,
        max_extension_speed=float(np.sum(radial_coefficients)),
        classification=classify_posture(sigma_min, condition_number),
    )
