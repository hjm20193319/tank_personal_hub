"""
camera_lidar_bbox_region.py

YOLO bbox의 중앙 70% 영역을 카메라 수평/수직 각도 범위로 변환하는 모듈이다.

주요 기능:
1. 원본 bbox의 중심을 유지하면서 가로와 세로 길이를 각각 70%로 축소한다.
2. 축소된 내부 bbox의 좌우 경계를 카메라 수평 상대각 범위로 변환한다.
3. 축소된 내부 bbox의 상하 경계를 카메라 수직 상대각 범위로 변환한다.
4. LiDAR 각도 구역 계산에서 재사용할 픽셀 영역과 각도 범위를 함께 반환한다.

입력:
- bbox: YOLO bbox 좌표 [x1, y1, x2, y2]
- image_width, image_height: bbox 좌표계와 동일한 이미지 크기
- hfov_degrees, vfov_degrees: 카메라 수평/수직 시야각
- region_ratio: 원본 bbox에서 사용할 중앙 영역 비율

출력:
- inner_bbox: 원본 bbox의 중앙 70% 픽셀 영역
- horizontal_start_angle, horizontal_end_angle: 카메라 기준 수평각 범위
- vertical_start_angle, vertical_end_angle: 카메라 기준 수직각 범위

주의:
- 70% 영역은 원본 bbox의 각 변에서 15%씩 안쪽으로 줄인 영역이다.
- 각도 변환은 기존 프로젝트와 동일하게 FOV를 이미지 픽셀 수로 나눈 선형 근사를 사용한다.
- 화면 오른쪽과 아래쪽으로 갈수록 상대각이 증가한다.
"""


BBOX_REGION_RATIO = 0.70  # 원본 bbox에서 실제 LiDAR 탐지에 사용할 중앙 영역 비율
VFOV_DEG = 28.0           # 카메라 수직 시야각


def calculate_bbox_angular_region(
    bbox,
    image_width,
    image_height,
    hfov_degrees,
    vfov_degrees=VFOV_DEG,
    region_ratio=BBOX_REGION_RATIO,
):
    # 이미지 크기와 영역 비율 검증
    width = float(image_width)
    height = float(image_height)
    ratio = float(region_ratio)
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be greater than 0")
    if not 0.0 < ratio <= 1.0:
        raise ValueError("region_ratio must be greater than 0 and at most 1")

    # bbox 좌표가 동일한 이미지 좌표계 안에 있는지 검증
    x1, y1, x2, y2 = (float(value) for value in bbox)
    if x1 > x2 or y1 > y2:
        raise ValueError("bbox minimum coordinates must not exceed maximums")
    if x1 < 0 or x2 > width or y1 < 0 or y2 > height:
        raise ValueError("bbox coordinates must be inside the image")

    # 원본 bbox 중심을 유지한 채 가로/세로 길이를 70%로 축소
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    region_width = (x2 - x1) * ratio
    region_height = (y2 - y1) * ratio
    region_x1 = center_x - region_width / 2.0
    region_x2 = center_x + region_width / 2.0
    region_y1 = center_y - region_height / 2.0
    region_y2 = center_y + region_height / 2.0

    # 기존 선형 FOV 변환 방식으로 내부 bbox 네 경계를 각도로 변환
    horizontal_degrees_per_pixel = float(hfov_degrees) / width
    vertical_degrees_per_pixel = float(vfov_degrees) / height
    horizontal_center = (
        center_x - width / 2.0
    ) * horizontal_degrees_per_pixel
    horizontal_start = (
        region_x1 - width / 2.0
    ) * horizontal_degrees_per_pixel
    horizontal_end = (
        region_x2 - width / 2.0
    ) * horizontal_degrees_per_pixel
    vertical_start = (
        region_y1 - height / 2.0
    ) * vertical_degrees_per_pixel
    vertical_end = (
        region_y2 - height / 2.0
    ) * vertical_degrees_per_pixel

    # 후속 수평/수직 LiDAR 교집합 계산에 필요한 픽셀 및 각도 정보 반환
    return {
        "region_ratio": ratio,
        "source_bbox": [x1, y1, x2, y2],
        "inner_bbox": [region_x1, region_y1, region_x2, region_y2],
        "center_x": center_x,
        "center_y": center_y,
        "horizontal_degrees_per_pixel": horizontal_degrees_per_pixel,
        "vertical_degrees_per_pixel": vertical_degrees_per_pixel,
        "horizontal_center_angle": horizontal_center,
        "horizontal_start_angle": horizontal_start,
        "horizontal_end_angle": horizontal_end,
        "vertical_start_angle": vertical_start,
        "vertical_end_angle": vertical_end,
    }
