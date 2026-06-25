from lidar_detection_zone_filter import (
    calculate_point_averages,
    cluster_points_by_distance,
    filter_lidar_points_for_detection_zone,
    filter_points_by_detection_zone,
    filter_vertical_surface_points,
    select_farthest_distance_cluster,
)


def make_point(angle, vertical_angle, x, y, z, distance=10.0):
    return {
        "angle": float(angle),
        "vertical_angle": float(vertical_angle),
        "distance": float(distance),
        "x": float(x),
        "y": float(y),
        "z": float(z),
    }


def test_filters_angles_in_wrapped_detection_zone():
    points = [
        make_point(359, 1, 0, 0, 0),
        make_point(1, 1, 0, 0, 0),
        make_point(10, 1, 0, 0, 0),
    ]
    zone = {"intervals": [[358.0, 360.0], [0.0, 2.0]]}

    filtered = filter_points_by_detection_zone(points, zone)

    assert [point["angle"] for point in filtered] == [359.0, 1.0]


def test_keeps_vertical_neighbors_with_small_horizontal_change():
    points = [
        make_point(18, 2, 10.0, 2.0, 20.0),
        make_point(18, 1, 10.4, 1.0, 20.3),
        make_point(18, 0, 15.0, 0.0, 25.0),
        make_point(19, 2, 30.0, 5.0, 40.0),
        make_point(19, 1, 30.2, 5.0, 40.2),
    ]

    filtered = filter_vertical_surface_points(points)

    assert filtered == points[:2]


def test_calculates_average_coordinates_and_distance():
    points = [
        make_point(18, 2, 10, 2, 20, distance=8),
        make_point(18, 1, 14, 4, 24, distance=12),
    ]

    assert calculate_point_averages(points) == {
        "x": 12.0,
        "y": 3.0,
        "z": 22.0,
        "distance": 10.0,
    }


def test_returns_none_when_no_points_are_available():
    assert calculate_point_averages([]) is None


def test_filters_horizontal_and_vertical_region_intersection():
    points = [
        make_point(1, -1, 0, 0, 0),
        make_point(1, 2, 0, 0, 0),
        make_point(10, -1, 0, 0, 0),
    ]

    filtered = filter_points_by_detection_zone(
        points,
        {"intervals": [[0, 2]]},
        {"start_angle": -2, "end_angle": 0},
    )

    assert filtered == points[:1]


def test_distance_clustering_auto_selects_farthest_cluster():
    points = [
        make_point(0, 1, 0, 0, 0, 10.0),
        make_point(0, 0, 0, 0, 0, 10.4),
        make_point(1, 1, 0, 0, 0, 20.0),
        make_point(1, 0, 0, 0, 0, 20.3),
    ]

    clusters = cluster_points_by_distance(points)
    selected = select_farthest_distance_cluster(clusters)

    assert len(clusters) == 2
    assert [point["distance"] for point in selected] == [20.0, 20.3]


def test_region_pipeline_keeps_vertical_surfaces_and_uses_farthest_distance():
    points = [
        make_point(0, 1, 10.0, 2.0, 20.0, 10.0),
        make_point(0, 0, 10.2, 1.0, 20.2, 10.2),
        make_point(1, 1, 30.0, 4.0, 40.0, 20.0),
        make_point(1, 0, 30.2, 3.0, 40.2, 20.2),
    ]

    result = filter_lidar_points_for_detection_zone(
        points,
        {"intervals": [[0, 2]]},
        {"start_angle": -1, "end_angle": 2},
    )

    assert result["surface_point_count"] == 4
    assert result["distance_cluster_count"] == 2
    assert result["distance_candidates"] == [10.1, 20.1]
    assert result["final_point_count"] == 2
    assert result["averages"]["distance"] == 20.1
