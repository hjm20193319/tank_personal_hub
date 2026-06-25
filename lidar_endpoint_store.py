"""
lidar_endpoint_store.py

/info EndPoint로 전달되는 lidarPoints를 기존 LiDAR 필터 모듈에서 사용하는
형식으로 변환하고, 가장 최근에 수신한 데이터를 메모리에 보관하는 모듈이다.

주요 기능:
1. JSON의 verticalAngle, position 구조를 기존 내부 필드 형식으로 변환한다.
2. isDetected=True인 실제 감지 점만 저장한다.
3. /info와 /detect 요청이 동시에 처리되어도 안전하도록 Lock으로 보호한다.
4. /detect에서 사용할 수 있도록 최신 LiDAR 스냅샷의 복사본을 반환한다.
"""

import threading

_snapshot_lock = threading.Lock()
_latest_lidar_points = []


#######################################################################################
# /info의 isDetected 값을 True/False로 해석하는 함수
def is_detected(value):
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


#######################################################################################
# /info의 lidarPoints를 기존 LiDAR 필터에서 사용하는 딕셔너리 형식으로 변환하는 함수
def normalize_lidar_points(lidar_points):
    normalized_points = []

    for point in lidar_points:
        if not isinstance(point, dict):
            continue
        if not is_detected(point.get("isDetected")):
            continue

        position = point.get("position", {})
        if not isinstance(position, dict):
            continue

        try:
            normalized_points.append(
                {
                    "angle": float(point["angle"]) % 360.0,
                    "vertical_angle": float(point["verticalAngle"]),
                    "distance": float(point["distance"]),
                    "x": float(position["x"]),
                    "y": float(position["y"]),
                    "z": float(position["z"]),
                    "channel_index": point.get("channelIndex"),
                }
            )
        except (KeyError, TypeError, ValueError):
            continue

    return normalized_points


#######################################################################################
# /info에서 받은 최신 lidarPoints를 메모리에 저장하는 함수
def update_lidar_points(lidar_points):
    normalized_points = normalize_lidar_points(lidar_points)

    with _snapshot_lock:
        global _latest_lidar_points
        _latest_lidar_points = normalized_points

    return len(normalized_points)


#######################################################################################
# /detect에서 사용할 최신 LiDAR 스냅샷을 복사해서 반환하는 함수
def get_latest_lidar_points():
    with _snapshot_lock:
        return [dict(point) for point in _latest_lidar_points]
