import pytest

from camera_lidar_direction import (
    HFOV_DEG,
    build_lidar_detection_zone,
    calculate_bbox_relative_angle,
    get_bbox_lidar_information,
    get_bboxes_lidar_information,
)
from lidar_endpoint_store import update_lidar_points


def make_endpoint_point(
    angle,
    vertical_angle,
    x,
    y,
    z,
    distance,
    channel_index,
):
    return {
        "angle": angle,
        "verticalAngle": vertical_angle,
        "distance": distance,
        "position": {"x": x, "y": y, "z": z},
        "isDetected": True,
        "channelIndex": channel_index,
    }


def test_image_center_is_zero_degrees():
    result = calculate_bbox_relative_angle(
        bbox=[300, 0, 340, 100],
        image_width=640,
    )

    assert result["bbox_center_x"] == 320.0
    assert result["relative_angle_degrees"] == 0.0


def test_right_and_left_edges_use_half_horizontal_fov():
    right = calculate_bbox_relative_angle(
        bbox=[640, 0, 640, 100],
        image_width=640,
    )
    left = calculate_bbox_relative_angle(
        bbox=[0, 0, 0, 100],
        image_width=640,
    )

    assert right["relative_angle_degrees"] == pytest.approx(HFOV_DEG / 2)
    assert left["relative_angle_degrees"] == pytest.approx(-HFOV_DEG / 2)


def test_camera_offset_is_added_to_body_relative_turret_angle():
    zone = build_lidar_detection_zone(
        body_x=45,
        turret_x=60,
        camera_relative_angle=5,
    )

    assert zone["turret_relative_angle"] == 15.0
    assert zone["relative_angle"] == 20.0
    assert zone["intervals"] == [[18.0, 22.0]]


def test_lidar_zone_wraps_across_zero_degrees():
    zone = build_lidar_detection_zone(
        body_x=10,
        turret_x=10,
        camera_relative_angle=-1,
    )

    assert zone["relative_angle"] == 359.0
    assert zone["intervals"] == [[357.0, 360.0], [0.0, 1.0]]


def test_rejects_bbox_from_a_different_image_width():
    with pytest.raises(ValueError):
        calculate_bbox_relative_angle(
            bbox=[600, 0, 700, 100],
            image_width=640,
        )


def test_returns_lidar_average_for_bbox_direction(tmp_path):
    tank_log = tmp_path / "tank_info_log.txt"
    tank_log.write_text(
        "Player_Body_X,Player_Turret_X\n"
        "10,20\n",
        encoding="utf-8",
    )
    update_lidar_points(
        [
            make_endpoint_point(10, 2, 100, 4, 200, 8, 1),
            make_endpoint_point(10, 1, 100.2, 2, 200.2, 12, 2),
            make_endpoint_point(30, 2, 300, 3, 400, 50, 1),
        ]
    )

    result = get_bbox_lidar_information(
        bbox=[300, 0, 340, 100],
        image_width=640,
        tank_info_log=tank_log,
    )

    assert result["detection_zone"]["relative_angle"] == 10.0
    assert result["detection_zone"]["intervals"] == [[8.0, 12.0]]
    assert result["averages"] == {
        "x": 100.1,
        "y": 3.0,
        "z": 200.1,
        "distance": 10.0,
    }


def test_returns_information_for_every_bbox(tmp_path):
    tank_log = tmp_path / "tank_info_log.txt"
    tank_log.write_text(
        "Player_Body_X,Player_Turret_X\n"
        "0,0\n",
        encoding="utf-8",
    )
    update_lidar_points(
        [
            make_endpoint_point(338, 2, 10, 4, 20, 8, 1),
            make_endpoint_point(338, 1, 10.2, 2, 20.2, 12, 2),
            make_endpoint_point(22, 2, 30, 6, 40, 18, 1),
            make_endpoint_point(22, 1, 30.2, 4, 40.2, 22, 2),
        ]
    )

    results = get_bboxes_lidar_information(
        bboxes=[
            [0, 0, 40, 100],
            [600, 0, 640, 100],
        ],
        image_width=640,
        tank_info_log=tank_log,
    )

    assert len(results) == 2
    assert results[0]["averages"]["x"] == pytest.approx(10.1)
    assert results[1]["averages"]["x"] == pytest.approx(30.1)


def test_bbox_region_selects_farthest_vertical_surface_cluster(tmp_path):
    tank_log = tmp_path / "tank_info_log.txt"
    tank_log.write_text(
        "Player_Body_X,Player_Turret_X\n0,0\n",
        encoding="utf-8",
    )
    update_lidar_points(
        [
            make_endpoint_point(0, 1, 10, 2, 20, 10, 30),
            make_endpoint_point(0, 0, 10.2, 1, 20.2, 10.2, 31),
            make_endpoint_point(1, 1, 30, 4, 40, 20, 30),
            make_endpoint_point(1, 0, 30.2, 3, 40.2, 20.2, 31),
        ]
    )

    result = get_bbox_lidar_information(
        bbox=[280, 200, 360, 280],
        image_width=640,
        image_height=480,
        tank_info_log=tank_log,
    )

    assert result["angular_region"]["inner_bbox"] == pytest.approx(
        [292, 212, 348, 268]
    )
    assert result["distance_cluster_count"] == 2
    assert result["distance_candidates"] == [10.1, 20.1]
    assert result["averages"]["distance"] == 20.1
