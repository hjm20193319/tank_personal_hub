# 전차 로그에서 최신 차체 각도와 포탑 각도를 읽어 포탑이 차체 기준 어느 방향을 보고 있는지 계산하고, 
# 그 방향 주변의 LiDAR 탐지 각도 구역을 생성하는 모듈

"""
turret_detection_zone.py

전차의 차체 각도와 포탑 각도를 이용해
차체 기준 포탑 탐지 구역을 계산하는 모듈이다.

주요 기능:
1. tank_info_log.txt에서 최신 Player_Body_X, Player_Turret_X 값을 읽는다.
2. 각도를 0도 이상 360도 미만 범위로 정규화한다.
3. 포탑 각도와 차체 각도의 차이를 이용해 차체 기준 포탑 상대각을 계산한다.
4. 포탑 상대각 기준 좌우 margin 범위를 탐지 구역으로 생성한다.
5. 탐지 구역이 0도 경계를 넘는 경우 두 개의 intervals로 분리한다.

입력:
- body_x: 차체 수평 각도
- turret_x: 포탑 수평 각도
- margin_degrees: 탐지 구역 좌우 허용 각도

출력:
- relative_angle: 차체 기준 포탑 상대각
- intervals: LiDAR 각도 필터링에 사용할 탐지 구역

주의:
- Player_Body_X와 Player_Turret_X가 수평 yaw 각도라는 전제를 사용한다.
- 최신 로그 행과 이미지/LiDAR 프레임의 시간 동기화는 별도로 보장되지 않는다.
"""

import csv
import json
from pathlib import Path

#######################################################################################
# 전차 상태 로그 파일 경로 - 각자 본인 경로 지정해줘야 함
DEFAULT_TANK_INFO_LOG = Path(
    r"C:\Users\acorn\Documents\Tank Challenge\log_data\tank_info_log.txt"
)
DEFAULT_MARGIN_DEGREES = 2.0    # 탐지 구역 좌우 허용 각도
REQUIRED_FIELDS = {"Player_Body_X", "Player_Turret_X"}


#######################################################################################
# 각도를 0도 이상 360도 미만 범위로 정규화하는 함수
def normalize_angle_degrees(angle):
    """Normalize an angle to the clockwise [0, 360) range."""
    return float(angle) % 360.0


#######################################################################################
# 차체 각도와 포탑 각도를 이용해, 차체 기준 포탑 상대각과 그 주변 탐지 구역을 계산하는 핵심 함수
def calculate_detection_zone(
    body_x,
    turret_x,
    margin_degrees=DEFAULT_MARGIN_DEGREES,
):
    """
    Calculate the turret direction relative to the tank body.

    Angles increase clockwise from the body heading. A turret aligned with the
    body is 0 degrees, and the detection zone extends by margin_degrees on
    both sides.
    """
    margin = float(margin_degrees)
    if not 0.0 <= margin <= 180.0:
        raise ValueError("margin_degrees must be between 0 and 180")

    body_angle = normalize_angle_degrees(body_x)
    turret_angle = normalize_angle_degrees(turret_x)
    # 포탑 상대각 계산
    relative_angle = normalize_angle_degrees(turret_angle - body_angle)
    # 탐지 구역 계산
    start_angle = normalize_angle_degrees(relative_angle - margin)
    end_angle = normalize_angle_degrees(relative_angle + margin)
    wraps_zero = start_angle > end_angle

    # 0도 경계 처리
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
        "relative_angle": relative_angle,
        "margin_degrees": margin,
        "start_angle": start_angle,
        "end_angle": end_angle,
        "wraps_zero": wraps_zero,
        "intervals": intervals,
    }


#######################################################################################
# 전차 로그 CSV에서 가장 마지막으로 유효한 차체/포탑 X 각도를 읽어오는 함수
def read_latest_turret_angles(log_path=DEFAULT_TANK_INFO_LOG):
    """Read body and turret X angles from the last valid CSV log row."""
    path = Path(log_path)
    if not path.is_file():
        raise FileNotFoundError(f"Tank info log not found: {path}")

    latest_angles = None
    with path.open("r", newline="", encoding="utf-8-sig") as log_file:
        reader = csv.DictReader(log_file)
        field_names = set(reader.fieldnames or ())
        missing_fields = sorted(REQUIRED_FIELDS - field_names)
        if missing_fields:
            raise ValueError(
                "Tank info log is missing columns: "
                + ", ".join(missing_fields)
            )

        for row in reader:
            try:
                body_x = float(row["Player_Body_X"])
                turret_x = float(row["Player_Turret_X"])
            except (TypeError, ValueError):
                continue

            latest_angles = {
                "body_x": body_x,
                "turret_x": turret_x,
            }

    # 마지막 행을 사용 - 최신 값 유지
    if latest_angles is None:
        raise ValueError(f"Tank info log has no valid data rows: {path}")

    return latest_angles


#######################################################################################
# 현재 로그 파일에서 최신 차체/포탑 각도를 읽고, 바로 탐지 구역까지 계산해서 반환하는 편의 함수
def get_current_detection_zone(
    log_path=DEFAULT_TANK_INFO_LOG,
    margin_degrees=DEFAULT_MARGIN_DEGREES,
):
    """Return the current body-relative turret detection zone."""
    angles = read_latest_turret_angles(log_path)
    # 탐지 구역 계산
    return calculate_detection_zone(
        body_x=angles["body_x"],
        turret_x=angles["turret_x"],
        margin_degrees=margin_degrees,
    )


#######################################################################################
# 어떤 각도가 현재 detection zone 안에 포함되는지 확인하는 함수
def angle_is_in_detection_zone(angle, detection_zone):
    """Return whether a body-relative clockwise angle is inside the zone."""
    normalized_angle = normalize_angle_degrees(angle)
    return any(
        start <= normalized_angle <= end
        for start, end in detection_zone["intervals"]
    )


# 디버깅 용
if __name__ == "__main__":
    print(
        json.dumps(
            get_current_detection_zone(),
            indent=2,
            ensure_ascii=False,
        )
    )