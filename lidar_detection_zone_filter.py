# /info EndPoint에서 받은 LiDAR 점 중 YOLO bbox 방향에 해당하는 각도 범위의 점을 추출한 뒤,
# 수직 물체 표면으로 보이는 점만 선별하여 목표의 대표 x, y, z 좌표와 거리를 계산하는 모듈

"""
lidar_detection_zone_filter.py

/info EndPoint에서 받은 LiDAR 데이터에서 특정 탐지 각도 구역에 해당하는 점들을 추출하고,
수직 물체 표면으로 보이는 점들의 평균 좌표와 거리를 계산하는 모듈이다.

주요 기능:
1. 메모리에 저장된 최신 LiDAR 스냅샷을 입력으로 받는다.
2. YOLO bbox 중앙 70%로 계산된 수평/수직 각도 교집합의 점을 선택한다.
3. 같은 수평 angle에서 인접한 vertical_angle 점들을 비교해
    y값은 달라지고 x-z 위치는 비슷한 점들을 수직 표면 후보로 판단한다.
4. 수직 표면점들을 거리순으로 정렬하고 인접 거리 차이로 군집을 자동 분리한다.
5. 평균 거리가 가장 큰 군집을 bbox의 최종 거리 후보로 선택한다.
6. 선택된 군집의 평균 x, y, z, distance를 계산한다.

입력:
- detection_zone:
    camera_lidar_direction.py에서 계산된 LiDAR 탐지 각도 구역
    예: {"intervals": [[13.0, 17.0]], ...}
- vertical_detection_zone:
    bbox 중앙 70%의 상하 경계를 변환한 수직각 범위

출력:
- zone_points:
    각도 구역에 포함된 LiDAR 후보 점들
- points:
    가장 먼 거리 군집으로 최종 선택된 수직 표면점들
- distance_candidates:
    자동 생성된 각 거리 군집의 평균 거리 목록
- averages:
    최종 점들의 평균 x, y, z, distance
    점이 없으면 None

주의:
- averages는 객체 중심이 아니라 LiDAR가 감지한 표면점들의 대표값이다.
- 군집 개수는 지정하지 않으며 인접 거리 차이가 기준값을 넘을 때 자동으로 분리한다.
- 현재 거리 군집 분리 기준은 DISTANCE_CLUSTER_GAP_METERS 상수로 관리한다.
- /info LiDAR 스냅샷과 카메라 프레임의 시간 동기화는 별도로 보장되지 않는다.
"""

import math
from collections import defaultdict

MAX_HORIZONTAL_CHANGE = 1.0     # 같은 물체 표면 판단 기준
MIN_Y_CHANGE = 1e-6             # 인접 채널로 탐지된 점의 높이 차이 기준
DISTANCE_CLUSTER_GAP_METERS = 2.0  # 인접 거리 차이가 이 값을 넘으면 새 군집으로 분리


#######################################################################################
# 특정 LiDAR 각도가 탐지 구역 intervals 안에 포함되는지 확인하는 함수
def angle_is_in_intervals(angle, intervals):
    normalized_angle = float(angle) % 360.0
    return any(
        float(start) <= normalized_angle <= float(end)
        for start, end in intervals
    )


#######################################################################################
# 전체 LiDAR 점 중에서 수평/수직 각도 교집합에 포함되는 점만 골라내는 함수
def filter_points_by_detection_zone(
    points,
    detection_zone,
    vertical_detection_zone=None,
):
    # 탐지 구역 내 실제 각도 범위
    intervals = detection_zone["intervals"]
    selected_points = []
    for point in points:
        if not angle_is_in_intervals(point["angle"], intervals):
            continue
        # 수직각 범위가 전달된 서버 경로에서는 bbox 70% 상하 범위도 함께 검사
        if vertical_detection_zone is not None:
            vertical_angle = float(point["vertical_angle"])
            if not (
                float(vertical_detection_zone["start_angle"])
                <= vertical_angle
                <= float(vertical_detection_zone["end_angle"])
            ):
                continue
        selected_points.append(point)
    return selected_points

#######################################################################################
# 각도 구역 안에 들어온 LiDAR 점 중에서, 수직 물체 표면처럼 보이는 점들만 선별하는 함수
def filter_vertical_surface_points(
    points,
    max_horizontal_change=MAX_HORIZONTAL_CHANGE,
    min_y_change=MIN_Y_CHANGE,
):
    """
    Keep adjacent vertical-channel points whose Y values differ while their
    horizontal X/Z separation remains within max_horizontal_change.
    """
    # 같은 수평 각도별로 그룹화 - 위 아래 채널과의 높이 차이를 확인하기 위해서
    grouped_points = defaultdict(list)
    for point in points:
        grouped_points[point["angle"]].append(point)

    # 중복 방지 용
    selected_ids = set()
    selected_points = []

    for angle_points in grouped_points.values():
        # vertical angle 순서 정렬
        ordered_points = sorted(
            angle_points,
            key=lambda point: point["vertical_angle"],
            reverse=True,
        )

        # 인접 채널끼리 비교
        for first, second in zip(ordered_points, ordered_points[1:]):
            # 높이 변화량 계산
            y_change = abs(second["y"] - first["y"])
            # 평면 거리 차이 계산 - 수직인 표면임을 이용해서
            horizontal_change = math.hypot(
                second["x"] - first["x"],
                second["z"] - first["z"],
            )
            # 수직인 표면 후보 조건
            # 높이는 다르다
            # 그런데 x-z 위치는 거의 같다
            # → 같은 물체의 수직 표면일 가능성이 있다
            if (
                y_change <= min_y_change
                or horizontal_change > max_horizontal_change
            ):
                continue

            # 조건 만족하는 점 선택 
            for point in (first, second):
                point_id = id(point)
                if point_id not in selected_ids:
                    selected_ids.add(point_id)
                    selected_points.append(point)

    return sorted(
        selected_points,
        key=lambda point: (
            point["angle"],
            -point["vertical_angle"],
            point["distance"],
        ),
    )


#######################################################################################
# 각도 영역의 점 선택 → 수직 표면 판정 → 거리 군집화 → 대표 평균 계산 함수
def filter_lidar_points_for_detection_zone(
    detected_points,
    detection_zone,
    vertical_detection_zone=None,
):
    # 수평 각도 범위에 들어온 점만 선택
    zone_points = filter_points_by_detection_zone(
        detected_points,
        detection_zone,
        vertical_detection_zone,
    )
    # 기존 방식과 동일하게 인접 수직 채널을 비교하여 수직 표면점만 선택
    surface_points = filter_vertical_surface_points(zone_points)
    distance_clusters = []
    # image_height가 없는 기존 호출은 군집화 없이 과거 평균 계산 방식을 유지
    if vertical_detection_zone is None:
        final_points = surface_points
    else:
        # 서버 경로에서는 군집 개수를 지정하지 않고 거리 간격으로 자동 분리
        distance_clusters = cluster_points_by_distance(surface_points)
        final_points = select_farthest_distance_cluster(distance_clusters)
    # 평균 계산
    averages = calculate_point_averages(final_points)

    return {
        "zone_point_count": len(zone_points),
        "surface_point_count": len(surface_points),
        "final_point_count": len(final_points),
        "zone_points": zone_points,
        "surface_points": surface_points,
        "points": final_points,
        "distance_cluster_count": len(distance_clusters),
        "distance_candidates": [
            calculate_point_averages(cluster)["distance"]
            for cluster in distance_clusters
        ],
        "averages": averages,
    }


#######################################################################################
# 거리순 인접 간격을 기준으로 군집 개수를 자동 결정하는 1차원 단일연결 군집화 함수
def cluster_points_by_distance(
    points,
    max_gap_meters=DISTANCE_CLUSTER_GAP_METERS,
):
    gap = float(max_gap_meters)
    if gap < 0:
        raise ValueError("max_gap_meters must not be negative")
    ordered_points = sorted(points, key=lambda point: point["distance"])
    if not ordered_points:
        return []

    # 가까운 점부터 시작해 바로 이전 점과의 거리 차이가 기준을 넘으면 새 군집 생성
    clusters = [[ordered_points[0]]]
    for point in ordered_points[1:]:
        previous = clusters[-1][-1]
        if float(point["distance"]) - float(previous["distance"]) > gap:
            clusters.append([])
        clusters[-1].append(point)
    return clusters


#######################################################################################
# 평균 거리가 가장 큰 군집을 bbox의 대표 거리 후보로 선택하는 함수
def select_farthest_distance_cluster(clusters):
    non_empty_clusters = [cluster for cluster in clusters if cluster]
    if not non_empty_clusters:
        return []
    return max(
        non_empty_clusters,
        key=lambda cluster: sum(
            float(point["distance"]) for point in cluster
        ) / len(cluster),
    )


#######################################################################################
# 최종 선택된 LiDAR 점들의 평균 x, y, z, distance를 계산하는 함수
def calculate_point_averages(points):
    if not points:
        return None

    point_count = len(points)
    # 평균 거리 계산 후 반환
    return {
        "x": sum(point["x"] for point in points) / point_count,
        "y": sum(point["y"] for point in points) / point_count,
        "z": sum(point["z"] for point in points) / point_count,
        "distance": (
            sum(point["distance"] for point in points) / point_count
        ),
    }
