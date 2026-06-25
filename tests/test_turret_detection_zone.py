from turret_detection_zone import (
    angle_is_in_detection_zone,
    calculate_detection_zone,
    read_latest_turret_angles,
)


def test_calculates_clockwise_body_relative_zone():
    zone = calculate_detection_zone(body_x=340, turret_x=350)

    assert zone["relative_angle"] == 10.0
    assert zone["start_angle"] == 8.0
    assert zone["end_angle"] == 12.0
    assert zone["intervals"] == [[8.0, 12.0]]


def test_splits_zone_when_it_crosses_zero_degrees():
    zone = calculate_detection_zone(body_x=350, turret_x=350)

    assert zone["relative_angle"] == 0.0
    assert zone["wraps_zero"] is True
    assert zone["intervals"] == [[358.0, 360.0], [0.0, 2.0]]
    assert angle_is_in_detection_zone(359, zone)
    assert angle_is_in_detection_zone(1, zone)
    assert not angle_is_in_detection_zone(10, zone)


def test_reads_last_valid_log_row(tmp_path):
    log_path = tmp_path / "tank_info_log.txt"
    log_path.write_text(
        "Player_Body_X,Player_Turret_X\n"
        "340,350\n"
        "\n"
        "invalid,row\n"
        "350,5\n",
        encoding="utf-8",
    )

    assert read_latest_turret_angles(log_path) == {
        "body_x": 350.0,
        "turret_x": 5.0,
    }
