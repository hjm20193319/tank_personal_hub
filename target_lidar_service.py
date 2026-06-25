# target_lidar_service.py는 YOLO 탐지 결과의 bbox 방향을 기준으로 LiDAR 정보를 조회하고, 
# 각 detection에 카메라 상대각·LiDAR 각도·평균 좌표·거리 정보를 추가한 뒤, 
# 현재 조준 대상 target의 LiDAR 평균값을 수직 조준 모듈로 넘겨주는 연결 모듈

"""
target_lidar_service.py

YOLO 탐지 결과와 LiDAR 정보를 연결하는 서비스 모듈이다.

주요 기능:
1. YOLO detection 리스트에서 bbox 좌표를 추출한다.
2. 각 bbox 중앙 70%의 수평/수직 각도 영역에 해당하는 LiDAR 정보를 계산한다.
3. 수직 표면 거리 군집 후보와 가장 먼 군집의 대표 거리를 detection에 연결한다.
4. detection 객체에 카메라 상대각, LiDAR 각도 범위, 거리 후보 정보를 추가한다.
5. 유효한 LiDAR 평균값이 있으면 detection에 lidarAverage를 추가한다.
6. 화면 표시용 className에 선택된 목표의 x-z 좌표, 높이, 거리를 붙인다.
7. 현재 조준 대상 target의 LiDAR 평균값을 반환하여 수직 조준 계산에 사용한다.

입력:
- detections: YOLO 탐지 결과 리스트
- image_width: YOLO 이미지 가로 픽셀 수
- image_height: YOLO 이미지 세로 픽셀 수
- target: 현재 조준 대상으로 선택된 detection 객체

출력:
- target_lidar_average:
    target 객체 방향의 LiDAR 평균값
    {"x": float, "y": float, "z": float, "distance": float}
    LiDAR 평균값이 없으면 None

주의:
- detections 리스트는 함수 내부에서 직접 수정된다.
- target 비교는 객체 동일성(is)을 기준으로 한다.
- LiDAR 스냅샷 또는 각도 계산 오류가 발생하면 빈 리스트를 반환하여 서버 중단을 방지한다.
"""

from camera_lidar_direction import get_bboxes_lidar_information

#######################################################################################
# YOLO detection 리스트에서 bbox만 꺼내서, 각 bbox에 대응되는 LiDAR 정보를 한 번에 계산하는 함수
def get_all_bbox_lidar_information(detections, image_width, image_height):
    try:
        return get_bboxes_lidar_information(
            # bbox 만 추출
            bboxes=[detection["bbox"] for detection in detections],
            image_width=image_width,
            image_height=image_height,
        )
    except (OSError, UnicodeError, ValueError):
        return []

#######################################################################################
# YOLO detection에 70% 각도 영역, 거리 군집 후보, 선택된 대표 거리를 추가하는 핵심 함수
def annotate_detections_with_lidar(
    detections,
    image_width,
    image_height,
    target=None,
    logger=print,
):
    # 모든 bbox네 대해 라이다 정보 가져오기
    lidar_information_list = get_all_bbox_lidar_information(
        detections,
        image_width,
        image_height,
    )
    # 라이다 평균값 초기화
    target_lidar_average = None

    # 객체와 라이다 정보를 하나씩 연결
    for detection, lidar_information in zip(detections, lidar_information_list):
        _annotate_single_detection(detection, lidar_information, logger)
        lidar_summary = lidar_information["averages"]
        if lidar_summary is None:
            continue
        
        # 라이다 평균값 추가
        detection["lidarAverage"] = dict(lidar_summary)

        # detection → target이면 라이다 정보 저장 → 포신 각도 계산에 사용 됨
        if detection is target:
            target_lidar_average = dict(lidar_summary)
        # 라이다 정보 표시용 문자열 추가
        detection["className"] = format_lidar_class_name(
            detection["baseClassName"],
            lidar_summary,
        )

    return target_lidar_average

#######################################################################################
# 탐지 객체 이름에 LiDAR 평균 좌표, 높이, 거리 정보를 붙여서 화면 표시용 문자열을 만드는 함수
def format_lidar_class_name(base_class_name, lidar_summary):
    return (
        f"{base_class_name} | "
        f"(x,z)=({lidar_summary['x']:.2f},"
        f"{lidar_summary['z']:.2f}) | "
        f"height={lidar_summary['y']:.2f}m | "
        f"distance={lidar_summary['distance']:.2f}m"
    )

#######################################################################################
# detection 하나에 카메라 상대각, LiDAR 기준 각도, LiDAR 탐지 범위를 추가하고 디버그 로그를 출력하는 내부 함수
def _annotate_single_detection(detection, lidar_information, logger):
    image_direction = lidar_information["image_direction"]  # 화면 중심으로부터 얼마나 떨어져 있는지
    angular_region = lidar_information["angular_region"]  # bbox 중앙 70% 픽셀/각도 영역
    detection_zone = lidar_information["detection_zone"]    # 라이다 기준 각도 구역 반환 결과
    vertical_detection_zone = lidar_information[
        "vertical_detection_zone"
    ]
    lidar_summary = lidar_information["averages"]           # 라이다 평균값
    detection["cameraRelativeAngle"] = image_direction[
        "relative_angle_degrees"
    ]
    detection["lidarAngle"] = detection_zone["relative_angle"]
    detection["lidarAngleRange"] = detection_zone["intervals"]
    # 디버깅 및 화면 연동에서 확인할 수 있도록 70% 영역과 각도 범위를 detection에 저장
    detection["lidarRegionRatio"] = angular_region["region_ratio"]
    detection["lidarRegionBBox"] = angular_region["inner_bbox"]
    detection["cameraHorizontalAngleRange"] = [
        angular_region["horizontal_start_angle"],
        angular_region["horizontal_end_angle"],
    ]
    detection["lidarVerticalAngleRange"] = [
        vertical_detection_zone["start_angle"],
        vertical_detection_zone["end_angle"],
    ]
    # 자동 생성된 모든 거리 군집의 평균 거리 후보 목록
    detection["lidarDistanceCandidates"] = lidar_information[
        "distance_candidates"
    ]

    # 디버그 로그
    logger(
        "[LiDAR debug] "
        f"class={detection['baseClassName']}, "
        f"confidence={detection['confidence']:.3f}, "
        f"lidar_angle={detection_zone['relative_angle']:.2f}, "
        f"range={detection_zone['intervals']}, "
        f"vertical_range={detection['lidarVerticalAngleRange']}, "
        f"zone_points={lidar_information['zone_point_count']}, "
        f"surface_points={lidar_information['surface_point_count']}, "
        f"clusters={lidar_information['distance_cluster_count']}, "
        f"distance_candidates={detection['lidarDistanceCandidates']}, "
        f"final_points={lidar_information['final_point_count']}, "
        f"averages={lidar_summary}",
        flush=True,
    )

    # bbox별 거리 군집 결과를 빠르게 확인하기 위한 눈에 띄는 디버그 로그
    cluster_distances = ", ".join(
        f"{distance:.2f}m"
        for distance in detection["lidarDistanceCandidates"]
    ) or "none"
    selected_distance = (
        f"{lidar_summary['distance']:.2f}m"
        if lidar_summary is not None
        else "none"
    )
    logger(
        "🟡 [LiDAR CLUSTERS] "
        f"class={detection['baseClassName']}, "
        f"bbox={[round(value, 2) for value in detection['bbox']]}, "
        f"pre_cluster_surface_points="
        f"{lidar_information['surface_point_count']}, "
        f"cluster_count={lidar_information['distance_cluster_count']}, "
        f"cluster_distances=[{cluster_distances}], "
        f"selected_distance={selected_distance}",
        flush=True,
    )

    # 상대적으로 작은 Human은 더 자세히 출력
    if detection["baseClassName"] == "Human":
        _log_human_lidar_points(lidar_information, logger)

#######################################################################################
# Human 객체 방향에 들어온 LiDAR 점들을 최대 30개까지 자세히 출력하는 디버깅 함수
# - 사람 방향에 실제 LiDAR 점이 들어오는지
# - vertical_angle별 점 분포가 어떤지
# - y값이 사람 높이처럼 나오는지
# - 최종 필터링 전 zone_points가 충분한지
def _log_human_lidar_points(lidar_information, logger):
    for point_index, point in enumerate(
        lidar_information["zone_points"][:30],
        start=1,
    ):
        logger(
            "[Human LiDAR point] "
            f"#{point_index} "
            f"angle={point['angle']:.2f}, "
            f"vertical_angle={point['vertical_angle']:.6f}, "
            f"distance={point['distance']:.4f}, "
            f"x={point['x']:.4f}, "
            f"y={point['y']:.4f}, "
            f"z={point['z']:.4f}",
            flush=True,
        )
