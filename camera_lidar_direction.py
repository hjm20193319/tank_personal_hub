"""
camera_lidar_direction.py

YOLO bbox의 화면 위치를 이용해 객체 방향의 LiDAR 정보를 계산하는 모듈이다.

주요 기능:
1. bbox 중심 x좌표와 이미지 가로 길이, 카메라 HFOV를 이용해
    카메라 중심 기준 상대각을 계산한다.
2. 현재 차체 각도(body_x)와 포탑 각도(turret_x)를 읽어
    포탑의 차체 기준 상대각을 계산한다.
3. 카메라 상대각을 포탑 상대각에 더해 차체 기준 LiDAR 목표 각도를 계산한다.
4. bbox의 중앙 70% 픽셀 영역을 수평/수직 각도 범위로 변환한다.
5. 수평각과 수직각의 교집합에 포함되는 LiDAR 점을 조회한다.
6. 수직 표면 필터와 거리 군집화를 거쳐 선택된 대표 좌표/거리를 반환한다.

입력:
- bbox: YOLO bbox 좌표 [x1, y1, x2, y2]
- image_width: bbox 좌표계와 동일한 이미지 가로 픽셀 수
- image_height: bbox 좌표계와 동일한 이미지 세로 픽셀 수
- tank_info_log: 차체/포탑 각도 로그 파일
- LiDAR 데이터는 /info EndPoint에서 받은 최신 메모리 스냅샷을 사용한다.

출력:
- image_direction: bbox의 카메라 상대각 정보
- angular_region: bbox 중앙 70%의 픽셀/수평각/수직각 범위
- detection_zone: LiDAR 기준 탐지 각도 구역
- distance_candidates: 수직 표면점에서 자동 생성된 거리 군집 후보
- averages: 가장 먼 거리 군집의 평균 x, y, z, distance

주의:
- bbox 좌표와 image_width/image_height는 반드시 같은 좌표계를 사용해야 한다.
- 카메라 정면이 포탑 정면과 일치한다고 가정한다.
- 각도는 차체 정면 기준 시계방향 증가로 해석한다.
"""

from lidar_detection_zone_filter import (
    filter_lidar_points_for_detection_zone,
)
from lidar_endpoint_store import get_latest_lidar_points
from camera_lidar_bbox_region import calculate_bbox_angular_region
from turret_detection_zone import (
    DEFAULT_TANK_INFO_LOG,
    normalize_angle_degrees,
    read_latest_turret_angles,
)


HFOV_DEG = 47.81061     # 카메라 수평 시야각
LIDAR_MARGIN_DEGREES = 2.0  # 라이다 탐색 각도 허용 범위

#######################################################################################
# YOLO bbox 중심 x좌표를 카메라 화면 중심 기준 상대각으로 변환하는 함수
def calculate_bbox_relative_angle(
    bbox,
    image_width,
    hfov_degrees=HFOV_DEG,
):
    """
    Linearly approximate the bbox direction from the analyzed image center.

    The bbox coordinates and image_width must come from the same YOLO image.
    Right is positive and left is negative.
    """
    # 이미지 가로 길이 검증
    width = float(image_width)
    if width <= 0:
        raise ValueError("image_width must be greater than 0")

    # bbox 좌표 검증 
    x1, _, x2, _ = (float(value) for value in bbox)
    if x1 > x2:
        raise ValueError("bbox x1 must not be greater than x2")
    if x1 < 0 or x2 > width:
        raise ValueError(
            "bbox x coordinates must use the same image width coordinate system"
        )

    # 화면 중심, bbox 중심 계산
    center_x = width / 2.0
    bbox_center_x = (x1 + x2) / 2.0
    # 픽셀당 각도 계산
    pixel_angle_degrees = float(hfov_degrees) / width
    # 카메라 상대각 계산 - 객체가 카메라 중심 기준 몇도 방향에 있는지
    relative_angle_degrees = (
        bbox_center_x - center_x
    ) * pixel_angle_degrees

    return {
        "image_width": width,
        "center_x": center_x,
        "bbox_center_x": bbox_center_x,
        "pixel_angle_degrees": pixel_angle_degrees,
        "relative_angle_degrees": relative_angle_degrees,
    }

#######################################################################################
# 카메라 상대각을 현재 차체 기준 LiDAR 각도 구역으로 변환하는 함수
def build_lidar_detection_zone(
    body_x,
    turret_x,
    camera_relative_angle,
    margin_degrees=LIDAR_MARGIN_DEGREES,
):
    """
    Convert a camera-relative bbox direction to a body-relative LiDAR angle.

    LiDAR angles increase clockwise from the tank body heading. The camera is
    centered on the turret, so its right-positive offset is added to the
    turret's body-relative clockwise angle.
    """
    # 라이다 탐지 범위 margin 검증
    margin = float(margin_degrees)
    if not 0.0 <= margin <= 180.0:
        raise ValueError("margin_degrees must be between 0 and 180")

    # 각도 정규화 0 ~ 360도
    body_angle = normalize_angle_degrees(body_x)
    turret_angle = normalize_angle_degrees(turret_x)
    # 차체 기준 포탑 상대각 계산
    turret_relative_angle = normalize_angle_degrees(
        turret_angle - body_angle
    )
    # 라이다 기준 각도로 변환
    lidar_angle = normalize_angle_degrees(
        turret_relative_angle + float(camera_relative_angle)
    )
    # 탐지 구역 각도 범위 계산
    start_angle = normalize_angle_degrees(lidar_angle - margin)
    end_angle = normalize_angle_degrees(lidar_angle + margin)
    wraps_zero = start_angle > end_angle

    # 0도 경계 처리 부분
    if wraps_zero:
        intervals = [
            [start_angle, 360.0],
            [0.0, end_angle],
        ]
    else:
        intervals = [[start_angle, end_angle]]

    return {
        "body_x": body_angle,
        "turret_x": turret_angle,
        "turret_relative_angle": turret_relative_angle,
        "camera_relative_angle": float(camera_relative_angle),
        "relative_angle": lidar_angle,
        "margin_degrees": margin,
        "start_angle": start_angle,
        "end_angle": end_angle,
        "wraps_zero": wraps_zero,
        "intervals": intervals,
    }


#######################################################################################
# bbox 하나에 대해 70% 각도 영역 생성 → LiDAR 필터링 → 가장 먼 거리 군집 반환까지 수행하는 함수
def get_bbox_lidar_information(
    bbox,
    image_width,
    image_height=None,
    tank_info_log=DEFAULT_TANK_INFO_LOG,
):
    angles = read_latest_turret_angles(tank_info_log)
    detected_points = get_latest_lidar_points()
    return _get_bbox_lidar_information_from_snapshot(
        bbox,
        image_width,
        image_height,
        angles,
        detected_points,
    )


#######################################################################################
# 여러 bbox에 대해 하나의 최신 차체/포탑 각도와 하나의 최신 LiDAR 스냅샷을 공유해서 정보를 계산하는 함수
def get_bboxes_lidar_information(
    bboxes,
    image_width,
    image_height=None,
    tank_info_log=DEFAULT_TANK_INFO_LOG,
):
    """Return LiDAR information for every bbox using one shared data snapshot."""
    bbox_list = list(bboxes)
    if not bbox_list:
        return []

    angles = read_latest_turret_angles(tank_info_log)
    # /info EndPoint에서 받은 최신 라이다 데이터 읽기
    detected_points = get_latest_lidar_points()
    results = []

    for bbox in bbox_list:
        results.append(
            _get_bbox_lidar_information_from_snapshot(
                bbox,
                image_width,
                image_height,
                angles,
                detected_points,
            )
        )

    return results


#######################################################################################
# 하나의 공유 LiDAR/포탑 스냅샷에서 bbox 70% 각도 영역과 대표 거리를 계산하는 내부 함수
def _get_bbox_lidar_information_from_snapshot(
    bbox,
    image_width,
    image_height,
    angles,
    detected_points,
):
    image_direction = calculate_bbox_relative_angle(
        bbox=bbox,
        image_width=image_width,
    )
    angular_region = None
    vertical_detection_zone = None

    # image_height가 없는 기존 직접 호출은 과거 수평 중심 ±2도 방식을 유지
    if image_height is None:
        detection_zone = build_lidar_detection_zone(
            body_x=angles["body_x"],
            turret_x=angles["turret_x"],
            camera_relative_angle=image_direction["relative_angle_degrees"],
        )
    else:
        # 실제 서버 경로는 bbox 중앙 70%의 네 경계를 수평/수직 각도로 변환
        angular_region = calculate_bbox_angular_region(
            bbox=bbox,
            image_width=image_width,
            image_height=image_height,
            hfov_degrees=HFOV_DEG,
        )
        # 70% 수평각 범위의 중심과 반폭을 기존 wraparound 처리 함수에 전달
        horizontal_margin = (
            angular_region["horizontal_end_angle"]
            - angular_region["horizontal_start_angle"]
        ) / 2.0
        detection_zone = build_lidar_detection_zone(
            body_x=angles["body_x"],
            turret_x=angles["turret_x"],
            camera_relative_angle=angular_region[
                "horizontal_center_angle"
            ],
            margin_degrees=horizontal_margin,
        )
        # 수직각은 화면 아래 방향이 양수인 연속 범위로 사용
        vertical_detection_zone = {
            "start_angle": angular_region["vertical_start_angle"],
            "end_angle": angular_region["vertical_end_angle"],
        }

    # 각도 교집합 → 수직 표면 → 거리 군집 → 가장 먼 군집 순서로 처리
    lidar_result = filter_lidar_points_for_detection_zone(
        detected_points,
        detection_zone,
        vertical_detection_zone,
    )
    return {
        "image_direction": image_direction,
        "angular_region": angular_region,
        "detection_zone": detection_zone,
        "vertical_detection_zone": vertical_detection_zone,
        "zone_point_count": lidar_result["zone_point_count"],
        "surface_point_count": lidar_result["surface_point_count"],
        "final_point_count": lidar_result["final_point_count"],
        "distance_cluster_count": lidar_result[
            "distance_cluster_count"
        ],
        "distance_candidates": lidar_result["distance_candidates"],
        "zone_points": lidar_result["zone_points"],
        "averages": lidar_result["averages"],
    }
