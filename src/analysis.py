from dataclasses import dataclass
from typing import TypedDict

import numpy as np
from numpy.typing import NDArray

from .parameters import DEFAULT_PARAMETERS, FiveBarParameters

FloatArray = NDArray[np.float64]
RECOMMENDED_CONDITION_MAX = 5.0
RECOMMENDED_SIGMA_MIN = 0.010
USABLE_CONDITION_MAX = 20.0
USABLE_SIGMA_MIN = 0.002
CLASS_CODES = {"recommended": 0.0, "usable": 1.0, "near_singular": 2.0}
SCAN_COLUMNS = (
    "phi1",
    "phi4",
    "xc",
    "yc",
    "l0",
    "phi0",
    "raw_sigma_min",
    "raw_condition_number",
    "sigma_min",
    "sigma_max",
    "condition_number",
    "determinant",
    "max_axial_force",
    "max_extension_speed",
    "class_code",
    "j11",
    "j12",
    "j21",
    "j22",
)


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


@dataclass(frozen=True)
class WorkspaceScan:
    resolution: int
    values: FloatArray
    invalid_count: int

    def column(self, name: str) -> FloatArray:
        try:
            index = SCAN_COLUMNS.index(name)
        except ValueError as error:
            raise KeyError(name) from error
        return self.values[:, index]

    def upright_mask(self, half_width_degrees: float = 5.0) -> NDArray[np.bool_]:
        angle_error = np.arctan2(
            np.sin(self.column("phi0") - np.pi / 2.0),
            np.cos(self.column("phi0") - np.pi / 2.0),
        )
        return np.abs(angle_error) <= np.deg2rad(half_width_degrees)


class ScanSummary(TypedDict):
    resolution: int
    total_samples: int
    valid_samples: int
    invalid_samples: int
    upright_samples: int
    x_range_m: list[float]
    y_range_m: list[float]
    l0_range_m: list[float]
    classification_counts: dict[str, int]
    classification_percent: dict[str, float]
    raw_sigma_min: dict[str, float]
    raw_condition_number: dict[str, float]
    sigma_min_m_per_rad: dict[str, float]
    condition_number: dict[str, float]
    max_axial_force_n: dict[str, float]
    max_extension_speed_m_per_s: dict[str, float]


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


def _scan_row(metrics: PostureMetrics) -> tuple[float, ...]:
    return (
        metrics.phi1,
        metrics.phi4,
        metrics.xc,
        metrics.yc,
        metrics.l0,
        metrics.phi0,
        metrics.raw_sigma_min,
        metrics.raw_condition_number,
        metrics.sigma_min,
        metrics.sigma_max,
        metrics.condition_number,
        metrics.determinant,
        metrics.max_axial_force,
        metrics.max_extension_speed,
        CLASS_CODES[metrics.classification],
        float(metrics.jacobian[0, 0]),
        float(metrics.jacobian[0, 1]),
        float(metrics.jacobian[1, 0]),
        float(metrics.jacobian[1, 1]),
    )


def scan_workspace(
    resolution: int = 360,
    parameters: FiveBarParameters = DEFAULT_PARAMETERS,
) -> WorkspaceScan:
    """Scan a nonduplicated periodic grid over [-pi, pi) for both joints."""
    if resolution < 2:
        raise ValueError("resolution must be at least 2")
    angles = np.linspace(-np.pi, np.pi, resolution, endpoint=False)
    rows: list[tuple[float, ...]] = []
    invalid_count = 0
    for phi1 in angles:
        for phi4 in angles:
            try:
                metrics = analyze_posture(float(phi1), float(phi4), parameters)
            except ValueError:
                invalid_count += 1
                continue
            rows.append(_scan_row(metrics))
    return WorkspaceScan(
        resolution=resolution,
        values=np.asarray(rows, dtype=float).reshape(-1, len(SCAN_COLUMNS)),
        invalid_count=invalid_count,
    )


def best_class_xy_grid(
    scan: WorkspaceScan, bins: int | None = None
) -> tuple[FloatArray, FloatArray, FloatArray]:
    """Aggregate the best available posture class in each foot-space bin."""
    bin_count = bins if bins is not None else max(24, scan.resolution // 3)
    if bin_count < 2:
        raise ValueError("bins must be at least 2")
    x = scan.column("xc")
    y = scan.column("yc")
    x_edges = np.linspace(float(np.min(x)), float(np.max(x)), bin_count + 1)
    y_edges = np.linspace(float(np.min(y)), float(np.max(y)), bin_count + 1)
    x_index = np.clip(
        np.searchsorted(x_edges, x, side="right") - 1,
        0,
        bin_count - 1,
    )
    y_index = np.clip(
        np.searchsorted(y_edges, y, side="right") - 1,
        0,
        bin_count - 1,
    )
    grid = np.full((bin_count, bin_count), 3, dtype=np.int8)
    np.minimum.at(
        grid,
        (y_index, x_index),
        scan.column("class_code").astype(np.int8),
    )
    result = grid.astype(float)
    result[grid == 3] = np.nan
    return x_edges, y_edges, result


def _distribution(values: FloatArray) -> dict[str, float]:
    return {
        "min": float(np.min(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def summarize_scan(scan: WorkspaceScan) -> ScanSummary:
    """Return JSON-ready aggregate facts for reports and consistency checks."""
    class_codes = scan.column("class_code")
    counts = {
        name: int(np.count_nonzero(class_codes == code))
        for name, code in CLASS_CODES.items()
    }
    valid_count = len(scan.values)
    return {
        "resolution": scan.resolution,
        "total_samples": scan.resolution**2,
        "valid_samples": valid_count,
        "invalid_samples": scan.invalid_count,
        "upright_samples": int(np.count_nonzero(scan.upright_mask())),
        "x_range_m": [
            float(np.min(scan.column("xc"))),
            float(np.max(scan.column("xc"))),
        ],
        "y_range_m": [
            float(np.min(scan.column("yc"))),
            float(np.max(scan.column("yc"))),
        ],
        "l0_range_m": [
            float(np.min(scan.column("l0"))),
            float(np.max(scan.column("l0"))),
        ],
        "classification_counts": counts,
        "classification_percent": {
            name: count / valid_count * 100.0 for name, count in counts.items()
        },
        "raw_sigma_min": _distribution(scan.column("raw_sigma_min")),
        "raw_condition_number": _distribution(
            scan.column("raw_condition_number")
        ),
        "sigma_min_m_per_rad": _distribution(scan.column("sigma_min")),
        "condition_number": _distribution(scan.column("condition_number")),
        "max_axial_force_n": _distribution(scan.column("max_axial_force")),
        "max_extension_speed_m_per_s": _distribution(
            scan.column("max_extension_speed")
        ),
    }
