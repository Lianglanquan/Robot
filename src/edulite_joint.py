import math

FIVE_BAR_LENGTHS_MM = (50.0, 105.0, 105.0, 50.0, 60.0)

LEFT_OUTPUT_FACE_X_MM = -66.5
HIP_Y_MM = 34.0
ACTIVE_AXIS_Z_MM = (-30.0, 30.0)

EDULITE_BODY_LENGTH_MM = 44.0
EDULITE_OUTPUT_PATTERN_COUNT = 6
EDULITE_OUTPUT_PATTERN_PCD_MM = 24.0
EDULITE_OUTPUT_THREAD_MM = 4.0
EDULITE_OUTPUT_THREAD_DEPTH_MM = 3.0
EDULITE_OUTPUT_DOWEL_COUNT = 3
EDULITE_OUTPUT_DOWEL_PCD_MM = 17.7
EDULITE_OUTPUT_DOWEL_DIAMETER_MM = 4.0
EDULITE_OUTPUT_DOWEL_PROTRUSION_MM = 3.0
EDULITE_OUTPUT_DOWEL_CLOCK_DEG = 30.0
EDULITE_REAR_PATTERN_COUNT = 4
EDULITE_REAR_PATTERN_PCD_MM = 38.5
EDULITE_REAR_PATTERN_CLOCK_DEG = 53.0
EDULITE_REAR_THREAD_MM = 3.0
EDULITE_REAR_THREAD_DEPTH_MM = 11.0

PROXIMAL_LINK_THICKNESS_MM = 5.0
OFFSET_SPACER_THICKNESS_MM = 6.0
PROXIMAL_HUB_RADIUS_MM = 19.0
OUTPUT_CENTER_RELIEF_DIAMETER_MM = 10.0
OUTPUT_BOLT_CLEARANCE_DIAMETER_MM = 4.3
OUTPUT_DOWEL_CLEARANCE_DIAMETER_MM = 4.2

DIRECT_LINK_X_MM = (-71.5, -66.5)
OFFSET_SPACER_X_MM = (-72.5, -66.5)
OFFSET_LINK_X_MM = (-77.5, -72.5)

BRACKET_REAR_FACE_X_MM = LEFT_OUTPUT_FACE_X_MM + EDULITE_BODY_LENGTH_MM
BRACKET_PLATE_THICKNESS_MM = 4.0
BRACKET_FOOT_Y_MM = (4.0, 8.0)
BRACKET_PLATE_Y_MM = (8.0, 60.0)
BRACKET_Z_MM = (-58.0, 58.0)
REUSED_BASE_HOLES_XZ_MM = (
    (-57.5, -51.5),
    (-57.5, -8.5),
    (-57.5, 8.5),
    (-57.5, 51.5),
    (-37.75, -35.23),
    (-37.75, -24.77),
    (-37.75, 24.77),
    (-37.75, 35.23),
)


def circular_pattern_centers(
    count: int,
    pcd_mm: float,
    clock_deg: float = 0.0,
) -> tuple[tuple[float, float], ...]:
    """Return planar hole centers around the origin in millimetres."""
    radius = pcd_mm / 2.0
    return tuple(
        (
            radius * math.cos(math.radians(clock_deg + 360.0 * index / count)),
            radius * math.sin(math.radians(clock_deg + 360.0 * index / count)),
        )
        for index in range(count)
    )
