"""
shooting_module.py

LiDAR로 추정한 목표 평균 좌표를 기반으로 전차 포신의 수직 조준 명령을 생성하는 모듈이다.

주요 기능:
1. 전차 현재 위치와 목표 평균 좌표를 이용해 x-z 평면 수평거리를 계산한다.
2. 전차 y좌표와 포구 높이 보정값을 이용해 목표와 포구 사이의 높이 차이를 계산한다.
3. 이상적인 포물선 탄도 공식을 이용해 목표를 맞히기 위한 포신 수직각을 계산한다.
4. 계산된 목표 포신각이 시뮬레이터의 가능한 포신각 범위 안에 있는지 확인한다.
5. 현재 포신각과 목표 포신각을 비교해 R/F 명령과 weight를 생성한다.

입력:
- tank_position: 현재 전차 위치 {"x": float, "y": float, "z": float}
- obstacle_average: LiDAR 목표 평균 좌표 {"x": float, "y": float, "z": float, "distance": float}
- current_turret_y: 현재 포신 수직각(degree)

출력:
- turretRF: 포신 상하 조준 명령
    - "R": 포신 올림
    - "F": 포신 내림
    - "": 명령 없음
- target_angle_degrees: 탄도 공식으로 계산한 목표 포신각
- error_degrees: 목표 포신각과 현재 포신각의 차이
- is_aligned: 수직 조준 완료 여부
- is_possible: 포신각 범위 내 사격 가능 여부

주의:
- 현재 탄도 계산은 공기저항을 고려하지 않는 단순 포물선 모델이다.
- 포신각은 저각 해를 사용한다.
"""

import math
import os


GRAVITY = 9.81  # 중력 가속도 - 탄도 계산용
BARREL_HEIGHT_OFFSET = 2.56     # 포구 높이 보정값 - 강훈님 계산 결과 사용
DEFAULT_PROJECTILE_SPEED = 58.0     # 포탄 초기 속도 추정값 - 강훈님 계산 결과 사용
DEFAULT_VERTICAL_TOLERANCE_DEGREES = 0.1    # 수직 조준 완료 허용 오차
MIN_POSSIBLE_BARREL_ANGLE_DEGREES = -5.0    # 포신 최소 각도
MAX_POSSIBLE_BARREL_ANGLE_DEGREES = 10.0    # 포신 최대 각도
# 포신 최대 최소 명령 강도
MIN_TURRET_WEIGHT = 0.005
MAX_TURRET_WEIGHT = 0.7

#######################################################################################
# 환경변수에서 포탄 속도를 읽어오고, 없거나 잘못된 값이면 기본값을 반환하는 함수
def get_projectile_speed(default=DEFAULT_PROJECTILE_SPEED):
    """Return projectile speed from the environment, falling back safely."""
    try:
        return float(os.environ.get("PROJECTILE_SPEED", default))
    except (TypeError, ValueError):
        return float(default)



def normalize_signed_degrees(angle_degrees):
    """Normalize an angle to the signed [-180, 180) range."""
    return ((float(angle_degrees) + 180.0) % 360.0) - 180.0


def calculate_relative_vertical_angles(
    target_angle_degrees,
    current_turret_y,
    player_body_y,
):
    """Return target/current turret angles in the tank-body-relative frame."""
    body_y_raw = float(player_body_y or 0.0)
    body_y_signed = normalize_signed_degrees(body_y_raw)
    turret_y_raw = float(current_turret_y)
    turret_y_signed = normalize_signed_degrees(turret_y_raw)
    target_y_signed = normalize_signed_degrees(target_angle_degrees)
    return {
        "player_body_y_raw": body_y_raw,
        "player_body_y_signed": body_y_signed,
        "turret_y_raw": turret_y_raw,
        "turret_y_signed": turret_y_signed,
        "turret_y_relative": normalize_signed_degrees(turret_y_raw - body_y_raw),
        "target_angle_raw": float(target_angle_degrees),
        "target_angle_signed": target_y_signed,
        "target_angle_relative": normalize_signed_degrees(
            target_y_signed - body_y_signed
        ),
    }

def calculate_horizontal_distance(tank_position, obstacle_position):
    """Calculate horizontal x/z distance from the tank to an obstacle."""
    tank_x = float(tank_position["x"])
    tank_z = float(tank_position["z"])
    obstacle_x = float(obstacle_position["x"])
    obstacle_z = float(obstacle_position["z"])
    # 피타고라스 거리 계산 공식
    return math.hypot(obstacle_x - tank_x, obstacle_z - tank_z)

#######################################################################################
# 전차 위치와 목표 좌표를 이용해 목표물을 맞히기 위한 포신 수직각을 라디안 단위로 계산하는 핵심 함수
def calculate_barrel_vertical_angle(
    tank_position,
    obstacle_average,
    projectile_speed=None,
    gravity=GRAVITY,
    barrel_height_offset=BARREL_HEIGHT_OFFSET,
):
    """
    Calculate the low-arc barrel vertical angle in radians.

    Formula:
    theta = atan((v^2 - sqrt(v^4 - g(gR^2 + 2dy v^2))) / (gR))
    """
    # 포탄 속도 결정
    speed = (
        get_projectile_speed()
        if projectile_speed is None
        else float(projectile_speed)
    )
    # 포탄 속도 0 방지
    if speed <= 0:
        raise ValueError("projectile_speed must be greater than 0")

    # 수평 거리 계산 - 미리 정의된 함수 사용
    horizontal_distance = calculate_horizontal_distance(
        tank_position,
        obstacle_average,
    )
    # 수평 거리 0 방지(검증)
    if horizontal_distance <= 0:
        raise ValueError("horizontal distance must be greater than 0")

    # 높이 차이 계산
    tank_y = float(tank_position.get("y", 0.0))
    obstacle_y = float(obstacle_average["y"])
    # 목표 높이 - 포구 높이
    delta_y = obstacle_y - (tank_y + float(barrel_height_offset))

    # 탄도 공식 분석
    speed_squared = speed * speed
    # 판별식 → 음수면 포격 불가
    discriminant = speed_squared * speed_squared - gravity * (
        gravity * horizontal_distance * horizontal_distance
        + 2.0 * delta_y * speed_squared
    )
    if discriminant < 0:
        raise ValueError("target is out of ballistic range")

    numerator = speed_squared - math.sqrt(discriminant)
    denominator = gravity * horizontal_distance
    return math.atan(numerator / denominator)


#######################################################################################
# 라디안으로 계산된 포신각을 도 단위로 변환해서 반환하는 함수
def calculate_barrel_vertical_angle_degrees(*args, **kwargs):
    return math.degrees(calculate_barrel_vertical_angle(*args, **kwargs))

#######################################################################################
# 현재 포신각과 목표 포신각을 비교해서 R/F 명령을 만드는 함수
def calculate_turret_rf_command(
    current_turret_y,
    target_angle_degrees,
    tolerance_degrees=DEFAULT_VERTICAL_TOLERANCE_DEGREES,
):
    """Return the turretRF command needed to match the vertical angle."""
    # 오차 계산
    error = float(target_angle_degrees) - float(current_turret_y)
    tolerance = abs(float(tolerance_degrees))

    # 조준 완료 판단 - 포신 조절 중단
    if abs(error) <= tolerance:
        return {
            "turretRF": {"command": "", "weight": 0.0},
            "error_degrees": error,
            "is_aligned": True,
        }

    # 회전 weight 계산 - PID 사용X
    weight = min(
        max(abs(error) / 10.0, MIN_TURRET_WEIGHT),
        MAX_TURRET_WEIGHT,
    )
    return {
        "turretRF": {
            "command": "R" if error > 0 else "F",
            "weight": weight,
        },
        "error_degrees": error,
        "is_aligned": False,
    }

#######################################################################################
# 계산된 목표 포신각이 실제 가능한 포신각 범위 안에 있는지 검사하는 함수
def barrel_angle_is_possible(
    angle_degrees,
    min_angle_degrees=MIN_POSSIBLE_BARREL_ANGLE_DEGREES,
    max_angle_degrees=MAX_POSSIBLE_BARREL_ANGLE_DEGREES,
):
    angle = float(angle_degrees)
    return float(min_angle_degrees) <= angle <= float(max_angle_degrees)


#######################################################################################
# 탄도 계산과 포신 상하 명령 생성을 하나로 묶은 수직 조준 관리 클래스 - AimCoordinator에서 사용 됨
class ShootingAimer:
    def __init__(
        self,
        projectile_speed=None,
        tolerance_degrees=DEFAULT_VERTICAL_TOLERANCE_DEGREES,
        logger=print,
    ):
        self.projectile_speed = projectile_speed
        self.tolerance_degrees = tolerance_degrees
        self.logger = logger
        self._alignment_reported = False
        self._impossible_reported = False

    # 로그 출력 상태 초기화 - 상태 갱신
    def reset(self):
        self._alignment_reported = False
        self._impossible_reported = False

    # 전차 위치, 목표 평균 좌표, 현재 포신각을 이용해 최종 turretRF 명령을 만드는 핵심 메서드
    def build_command(
        self,
        tank_position,
        obstacle_average,
        current_turret_y,
        player_body_y=0.0,
    ):
        target_angle = calculate_barrel_vertical_angle_degrees(
            tank_position,
            obstacle_average,
            projectile_speed=self.projectile_speed,
        )
        horizontal_distance = calculate_horizontal_distance(
            tank_position,
            obstacle_average,
        )
        relative_angles = calculate_relative_vertical_angles(
            target_angle_degrees=target_angle,
            current_turret_y=current_turret_y,
            player_body_y=player_body_y,
        )
        target_relative_angle = relative_angles["target_angle_relative"]
        turret_relative_y = relative_angles["turret_y_relative"]

        if not barrel_angle_is_possible(target_relative_angle):
            if not self._impossible_reported:
                self.logger('❌ 사격 불가능')
                self._impossible_reported = True
            self._alignment_reported = False
            return {
                "turretRF": {"command": "", "weight": 0.0},
                "error_degrees": None,
                "is_aligned": False,
                "is_possible": False,
                "target_angle_degrees": target_relative_angle,
                "raw_target_angle_degrees": target_angle,
                "horizontal_distance": horizontal_distance,
                **relative_angles,
            }

        self._impossible_reported = False
        command = calculate_turret_rf_command(
            current_turret_y=turret_relative_y,
            target_angle_degrees=target_relative_angle,
            tolerance_degrees=self.tolerance_degrees,
        )

        if command["is_aligned"]:
            if not self._alignment_reported:
                self.logger('🟢 조준 완료')
                self._alignment_reported = True
        else:
            self._alignment_reported = False

        return {
            **command,
            "is_possible": True,
            "target_angle_degrees": target_relative_angle,
            "raw_target_angle_degrees": target_angle,
            "horizontal_distance": horizontal_distance,
            **relative_angles,
        }
