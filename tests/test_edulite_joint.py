import math

import pytest

from src.edulite_joint import (
    ACTIVE_AXIS_Z_MM,
    BASE_M3_SCREW_LENGTH_MM,
    BASE_M3_TAPPED_DIAMETER_MM,
    BASE_M3_THREAD_ENGAGEMENT_MM,
    BASEPLATE_THICKNESS_MM,
    BRACKET_FOOT_Y_MM,
    BRACKET_REAR_FACE_X_MM,
    DIRECT_LINK_X_MM,
    EDULITE_OUTPUT_DOWEL_CLOCK_DEG,
    EDULITE_OUTPUT_DOWEL_COUNT,
    EDULITE_OUTPUT_DOWEL_PCD_MM,
    EDULITE_OUTPUT_PATTERN_COUNT,
    EDULITE_OUTPUT_PATTERN_PCD_MM,
    EDULITE_REAR_PATTERN_CLOCK_DEG,
    EDULITE_REAR_PATTERN_COUNT,
    EDULITE_REAR_PATTERN_PCD_MM,
    FIVE_BAR_LENGTHS_MM,
    LEFT_OUTPUT_FACE_X_MM,
    OFFSET_LINK_X_MM,
    OFFSET_SPACER_X_MM,
    OUTPUT_BOLT_CLEARANCE_DIAMETER_MM,
    OUTPUT_CENTER_RELIEF_DIAMETER_MM,
    OUTPUT_DOWEL_CLEARANCE_DIAMETER_MM,
    PROXIMAL_HUB_RADIUS_MM,
    REUSED_BASE_HOLES_XZ_MM,
    circular_pattern_centers,
)


@pytest.mark.parametrize(
    ("count", "pcd_mm", "clock_deg"),
    [
        (
            EDULITE_OUTPUT_PATTERN_COUNT,
            EDULITE_OUTPUT_PATTERN_PCD_MM,
            0.0,
        ),
        (
            EDULITE_REAR_PATTERN_COUNT,
            EDULITE_REAR_PATTERN_PCD_MM,
            EDULITE_REAR_PATTERN_CLOCK_DEG,
        ),
        (
            EDULITE_OUTPUT_DOWEL_COUNT,
            EDULITE_OUTPUT_DOWEL_PCD_MM,
            EDULITE_OUTPUT_DOWEL_CLOCK_DEG,
        ),
    ],
)
def test_circular_patterns_preserve_declared_pcd(
    count: int, pcd_mm: float, clock_deg: float
) -> None:
    centers = circular_pattern_centers(count, pcd_mm, clock_deg)

    assert len(centers) == count
    assert all(math.hypot(*center) == pytest.approx(pcd_mm / 2.0) for center in centers)


def test_patch_preserves_five_bar_and_original_axial_layers() -> None:
    assert FIVE_BAR_LENGTHS_MM == (50.0, 105.0, 105.0, 50.0, 60.0)
    assert ACTIVE_AXIS_Z_MM == (-30.0, 30.0)
    assert DIRECT_LINK_X_MM[1] == LEFT_OUTPUT_FACE_X_MM
    assert OFFSET_SPACER_X_MM[1] == LEFT_OUTPUT_FACE_X_MM
    assert OFFSET_LINK_X_MM[1] == OFFSET_SPACER_X_MM[0]
    assert DIRECT_LINK_X_MM[1] - DIRECT_LINK_X_MM[0] == pytest.approx(5.0)
    assert OFFSET_SPACER_X_MM[1] - OFFSET_SPACER_X_MM[0] == pytest.approx(6.0)
    assert OFFSET_LINK_X_MM[1] - OFFSET_LINK_X_MM[0] == pytest.approx(5.0)


def test_new_output_hub_contains_dowels_and_bolt_pattern() -> None:
    relief_radius = OUTPUT_CENTER_RELIEF_DIAMETER_MM / 2.0
    dowel_radius = OUTPUT_DOWEL_CLEARANCE_DIAMETER_MM / 2.0
    dowel_pattern_radius = EDULITE_OUTPUT_DOWEL_PCD_MM / 2.0
    bolt_radius = OUTPUT_BOLT_CLEARANCE_DIAMETER_MM / 2.0
    pattern_radius = EDULITE_OUTPUT_PATTERN_PCD_MM / 2.0
    dowels = circular_pattern_centers(
        EDULITE_OUTPUT_DOWEL_COUNT,
        EDULITE_OUTPUT_DOWEL_PCD_MM,
        EDULITE_OUTPUT_DOWEL_CLOCK_DEG,
    )
    bolts = circular_pattern_centers(
        EDULITE_OUTPUT_PATTERN_COUNT, EDULITE_OUTPUT_PATTERN_PCD_MM
    )

    assert relief_radius + dowel_radius < dowel_pattern_radius
    assert min(
        math.dist(dowel, bolt) for dowel in dowels for bolt in bolts
    ) > dowel_radius + bolt_radius
    assert pattern_radius + bolt_radius < PROXIMAL_HUB_RADIUS_MM


def test_bracket_reuses_existing_base_holes_without_moving_axes() -> None:
    assert BRACKET_REAR_FACE_X_MM == pytest.approx(-22.5)
    assert len(REUSED_BASE_HOLES_XZ_MM) == 8
    assert {abs(x) for x, _z in REUSED_BASE_HOLES_XZ_MM} == {37.75, 57.5}
    assert BASE_M3_TAPPED_DIAMETER_MM == pytest.approx(2.5)
    assert BRACKET_FOOT_Y_MM[1] - BRACKET_FOOT_Y_MM[0] == pytest.approx(7.0)
    assert BASE_M3_SCREW_LENGTH_MM - BASEPLATE_THICKNESS_MM == pytest.approx(
        BASE_M3_THREAD_ENGAGEMENT_MM
    )
