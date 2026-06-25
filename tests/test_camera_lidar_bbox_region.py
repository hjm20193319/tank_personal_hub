import pytest

from camera_lidar_bbox_region import (
    BBOX_REGION_RATIO,
    calculate_bbox_angular_region,
)


def test_uses_centered_seventy_percent_of_bbox():
    result = calculate_bbox_angular_region(
        bbox=[100, 100, 300, 300],
        image_width=640,
        image_height=480,
        hfov_degrees=47.81061,
    )

    assert result["region_ratio"] == BBOX_REGION_RATIO
    assert result["inner_bbox"] == pytest.approx([130, 130, 270, 270])
    assert result["horizontal_start_angle"] == pytest.approx(
        (130 - 320) * 47.81061 / 640
    )
    assert result["horizontal_end_angle"] == pytest.approx(
        (270 - 320) * 47.81061 / 640
    )
    assert result["vertical_start_angle"] == pytest.approx(
        (130 - 240) * 28.0 / 480
    )
    assert result["vertical_end_angle"] == pytest.approx(
        (270 - 240) * 28.0 / 480
    )


def test_rejects_invalid_region_ratio():
    with pytest.raises(ValueError):
        calculate_bbox_angular_region(
            bbox=[0, 0, 100, 100],
            image_width=640,
            image_height=480,
            hfov_degrees=47.81061,
            region_ratio=1.1,
        )
