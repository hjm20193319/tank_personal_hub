from lidar_endpoint_store import (
    get_latest_lidar_points,
    normalize_lidar_points,
    update_lidar_points,
)


def make_point(is_detected=True):
    return {
        "angle": 361.0,
        "verticalAngle": 22.5,
        "distance": 12.0,
        "position": {"x": 1.0, "y": 2.0, "z": 3.0},
        "isDetected": is_detected,
        "channelIndex": 4,
    }


def test_normalizes_detected_info_endpoint_point():
    assert normalize_lidar_points([make_point()]) == [
        {
            "angle": 1.0,
            "vertical_angle": 22.5,
            "distance": 12.0,
            "x": 1.0,
            "y": 2.0,
            "z": 3.0,
            "channel_index": 4,
        }
    ]


def test_ignores_undetected_and_invalid_points():
    assert normalize_lidar_points([make_point(False), {"angle": 10}]) == []


def test_replaces_latest_snapshot():
    update_lidar_points([make_point()])
    first_snapshot = get_latest_lidar_points()
    update_lidar_points([])

    assert len(first_snapshot) == 1
    assert get_latest_lidar_points() == []
