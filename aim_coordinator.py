"""
aim_coordinator.py

YOLO 탐지 결과와 LiDAR 기반 목표 위치 정보를 이용해
전차의 포탑 좌우 조준 명령(turretQE)과 포신 상하 조준 명령(turretRF)을
통합 관리하는 모듈이다.

주요 역할:
1. /detect에서 전달된 YOLO 탐지 결과를 기반으로 수평 조준 명령을 생성한다.
2. target 객체의 LiDAR 평균 위치/거리 정보를 저장한다.
3. /get_action에서 현재 전차 위치와 포신각을 입력받아 수직 조준 명령을 생성한다.
4. 이동 명령, 포탑 조준 명령, 포신 조준 명령, 발사 여부를 포함한 최종 action을 반환한다.

연결 모듈:
- HorizontalAimer: bbox 중심과 화면 중심을 비교해 Q/E 명령 생성
- ShootingAimer: LiDAR 좌표와 탄도 공식을 이용해 R/F 명령 생성

주의:
- /detect와 /get_action은 서로 다른 타이밍에 호출되므로,
    최신 조준 명령을 내부 상태(_latest_aim_command)에 저장한다.
- 오래된 탐지 결과 사용을 막기 위해 command_timeout_seconds 이후 명령을 초기화한다.
"""

import time     # 조준 명령 최신화 확인용
from threading import Lock  # 요청 동시 접근 시 데이터 충돌 방지

from horizontal_aim import HorizontalAimer  # bbox 기반 좌우 포탑 조준
from shooting_module import ShootingAimer   # 라이다 좌표 기반 상하 포신 조준

# 최신 조준 명령을 몇 초 동안 유지할건지 - 시뮬레이터의 실시간성이 좋지 않은 것을 해결하기 위함
AIM_COMMAND_TIMEOUT_SECONDS = 0.8

# 수평 조준 모듈과 수직 조준 모듈을 연결하고, 최종 전차 제어 명령을 만드는 클래스
class AimCoordinator:
    def __init__(
        self,
        horizontal_aimer=None,  
        shooting_aimer=None,    
        command_timeout_seconds=AIM_COMMAND_TIMEOUT_SECONDS,
        logger=print,
    ):
        self.horizontal_aimer = horizontal_aimer or HorizontalAimer()   # 수평 조준기
        self.shooting_aimer = shooting_aimer or ShootingAimer(logger=logger)    # 수직 조준기
        self.command_timeout_seconds = command_timeout_seconds  # 최신 조준 명령 유지 시간
        self.logger = logger
        self._lock = Lock()
        # 최신 조준 명령 저장소
        self._latest_aim_command = {
            "turretQE": {"command": "", "weight": 0.0},
            "target_lidar_average": None,
            "updated_at": 0.0,
        }
        self._latest_player_body_y = 0.0

    def update_player_body_y(self, player_body_y):
        try:
            value = float(player_body_y)
        except (TypeError, ValueError):
            return
        with self._lock:
            self._latest_player_body_y = value

    # YOLO 탐지 결과 → target 선택, 수평 조준 계산 → 최신 명령 갱신
    def update_from_detections(self, detections, image_width, image_height):
        with self._lock:
            if detections:  # 탐지 결과가 있으면, 
                # 신뢰도가 가장 높은 객체를 target으로 선택
                target = max(detections, key=lambda item: item["confidence"])
                # bbox 좌표 이용해서 수평 조준 명령 생성
                aim_command = self.horizontal_aimer.build_command(
                    target["bbox"],
                    image_width,
                )
            else:   # 탐지 결과 없으면 빈 명령으로 둠
                target = None
                aim_command = self.horizontal_aimer.build_no_target_command(
                    image_width,
                )

            # 최신 명령 갱신
            self._latest_aim_command = {
                "turretQE": aim_command["turretQE"],
                "target_lidar_average": None,
                "updated_at": time.monotonic(),
            }
        
        # 수평 조준 상태 터미널 출력
        self._log_auto_aim(image_width, image_height, aim_command)
        return target, aim_command  # target, 조준 계산 결과 반환

    # 라이다 평균 거리 저장
    def update_target_lidar_average(self, target_lidar_average):
        with self._lock:
            self._latest_aim_command["target_lidar_average"] = (    # 계산 결과를 최신 조준 명령에 저장
                target_lidar_average
            )

    # 전차 상태를 입력받아 최종 제어 명령을 생성하는 함수
    def build_action(self, data):
        # /get_action에서 들어온 JSON 데이터
        position = data.get("position", {})
        turret = data.get("turret", {})

        pos_x = position.get("x", 0)
        pos_y = position.get("y", 0)
        pos_z = position.get("z", 0)

        turret_x = turret.get("x", 0)
        turret_y = turret.get("y", 0)

        # 현재 상태 확인용 로그 출력
        self.logger(
            f"[player] Current position: x={pos_x}, y={pos_y}, z={pos_z}",
            flush=True,
        )
        self.logger(f"🚀 Turret received: x={turret_x}, y={turret_y}")
        # 최신 수평 조준 명령 읽기
        turret_qe, target_lidar_average = self._read_latest_command()
        player_body_y = self._read_latest_player_body_y()
        # 수직 포신 명령 생성
        turret_rf = self._build_turret_rf(
            tank_position={"x": pos_x, "y": pos_y, "z": pos_z},
            target_lidar_average=target_lidar_average,
            turret_y=turret_y,
            player_body_y=player_body_y,
        )
        # 최종 제어 명령 생성
        command = {
            "moveWS": {"command": "STOP", "weight": 1.0},
            "moveAD": {"command": "", "weight": 0.0},
            "turretQE": turret_qe,
            "turretRF": turret_rf,
            "fire": False,  # 현재는 test용으로 발사는 수동으로 제어
        }

        self.logger("💫 Sent Combined Action:", command)
        return command      # 최종 제어 명령 반환

    # 최신 조준 명령 읽기 (오래된 명령은 버리기)
    def _read_latest_command(self):
        with self._lock:
            aim_command_age = ( # 명령 갱신 주기 계산
                time.monotonic() - self._latest_aim_command["updated_at"]
            )
            # 명령이 유효 판정
            if aim_command_age <= self.command_timeout_seconds:
                return (    # 수평 조준 명령과 라이다 평균 값 반환
                    dict(self._latest_aim_command["turretQE"]),
                    self._latest_aim_command["target_lidar_average"],
                )

            # 명령 만료시 초기화
            self.horizontal_aimer.reset()
            self.shooting_aimer.reset()
            return {"command": "", "weight": 0.0}, None

    def _read_latest_player_body_y(self):
        with self._lock:
            return self._latest_player_body_y

    # 포신 상하 조준 명령 생성 함수
    def _build_turret_rf(
        self,
        tank_position,
        target_lidar_average,
        turret_y,
        player_body_y,
    ):
        # 라이다 평균값이 없으면 빈 명령
        if target_lidar_average is None:
            self.shooting_aimer.reset()
            return {"command": "", "weight": 0.0}

        try:    # 수직 조준 명령 계산
            shooting_result = self.shooting_aimer.build_command(
                tank_position=tank_position,
                obstacle_average=target_lidar_average,
                current_turret_y=turret_y,
                player_body_y=player_body_y,
            )
        except (KeyError, TypeError, ValueError) as error:
            self.shooting_aimer.reset()
            self.logger(f"[Shooting] Unable to calculate aim: {error}", flush=True)
            return {"command": "", "weight": 0.0}

        # 계산 결과 로그 출력, 수직 조준 명령 반환
        self._log_shooting_result(shooting_result, turret_y)
        return shooting_result["turretRF"]

    # 수평 조준 명령 로그 출력 - 디버깅 용
    def _log_auto_aim(self, image_width, image_height, aim_command):
        self.logger(
            "Auto Aim | "
            f"image={image_width}x{image_height}, "
            f"bbox_center_x={aim_command['bbox_center_x']}, "
            f"screen_center_x={aim_command['screen_center_x']}, "
            f"center_range={aim_command['center_range']}, "
            f"error_x={aim_command['error_x']}, "
            f"pid_output={aim_command['pid_output']:.4f}, "
            f"aligned={aim_command['is_aligned']}, "
            f"turretQE={aim_command['turretQE']}",
            flush=True,
        )

    # 수직 조준 명령 터미널에 출력용 함수
    def _log_shooting_result(self, shooting_result, turret_y):
        if shooting_result["is_possible"]:  # 사격이 가능한 경우면 출력
            self.logger(
                "[Shooting] "
                f"target_angle={shooting_result['target_angle_degrees']:.3f}, "
                f"raw_target_angle={shooting_result.get('raw_target_angle_degrees', shooting_result['target_angle_degrees']):.3f}, "
                f"player_body_y_raw={shooting_result.get('player_body_y_raw', 0.0):.3f}, "
                f"player_body_y_signed={shooting_result.get('player_body_y_signed', 0.0):.3f}, "
                f"turret_y_raw={shooting_result.get('turret_y_raw', float(turret_y)):.3f}, "
                f"turret_y_relative={shooting_result.get('turret_y_relative', float(turret_y)):.3f}, "
                f"error={shooting_result['error_degrees']:.3f}, "
                f"horizontal_distance={shooting_result['horizontal_distance']:.3f}, "
                f"turretRF={shooting_result['turretRF']}",
                flush=True,
            )
            return

        # 사격 불가능 경우 (허용 각도 범위 밖)
        self.logger(
            '❌ [Shooting] impossible '
            f"target_angle={shooting_result['target_angle_degrees']:.3f}, "
            f"raw_target_angle={shooting_result.get('raw_target_angle_degrees', shooting_result['target_angle_degrees']):.3f}, "
            f"player_body_y_raw={shooting_result.get('player_body_y_raw', 0.0):.3f}, "
            f"player_body_y_signed={shooting_result.get('player_body_y_signed', 0.0):.3f}, "
            f"turret_y_raw={shooting_result.get('turret_y_raw', float(turret_y)):.3f}, "
            f"turret_y_relative={shooting_result.get('turret_y_relative', float(turret_y)):.3f}, "
            f"allowed_range=-5.000..10.000, "
            f"horizontal_distance={shooting_result['horizontal_distance']:.3f}",
            flush=True,
        )
