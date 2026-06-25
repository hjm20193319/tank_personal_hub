"""
horizontal_aim.py

YOLO 탐지 결과의 bbox 중심 x좌표를 이용해 전차 포탑의 수평 조준 명령을 생성하는 모듈이다.

주요 기능:
1. bbox 중심 x좌표와 화면 중심 x좌표의 오차(error_x)를 계산한다.
2. 오차가 허용 범위 이내이면 조준 완료로 판단하고 포탑 명령을 제거한다.
3. 오차가 존재하면 PID 제어를 이용해 회전 방향(Q/E)과 회전 강도(weight)를 계산한다.
4. target이 없는 경우 이전 PID 상태를 초기화하고 빈 조준 명령을 반환한다.

입력:
- YOLO bbox: [x1, y1, x2, y2]
- image_width: 이미지 가로 픽셀 수

출력:
- turretQE: 포탑 좌우 회전 명령
    - "Q": 왼쪽 회전
    - "E": 오른쪽 회전
    - "": 회전 없음
- weight: 회전 강도
- error_x, pid_output, is_aligned 등 디버깅 정보
"""

import time


CENTER_MARGIN_PX = 1.5  # 화면 중심 범위를 좌우 1.5 픽셀로 지정
ALIGN_EXIT_MARGIN_PX = 2    # 조준 완료에서 벗어남을 판단하는 기준 - 히스테리시스 역할

# PID 제어 게인 값
KP = 0.6
KI = 0.0
KD = 0.05

MIN_TURRET_WEIGHT = 0.005   # 포탑 회전 명령 최소값
MAX_TURRET_WEIGHT = 0.7     # 포탑 회전 명령 최대값
SMALL_WEIGHT_CUTOFF = 0.0   # 너무 작은 명령은 무시해서 떨림을 제한 - 현재는 사용하지 X


#######################################################################################
# 수평 조준 상태를 기억하면서, 매 프레임 들어오는 bbox 정보를 이용해 포탑 회전 명령을 생성
"""
bbox 기반 수평 조준 제어기.

YOLO bbox 중심 x좌표와 화면 중심 x좌표를 비교해
포탑을 왼쪽(Q) 또는 오른쪽(E)으로 회전시키는 명령을 생성한다.

특징:
- PID 제어를 사용해 회전 강도(weight)를 계산한다.
- 화면 중앙 근처에서는 조준 완료로 판단하고 명령을 제거한다.
- target이 사라지거나 조준이 완료되면 PID 내부 상태를 초기화한다.
"""
class HorizontalAimer:
    def __init__(
        self,
        center_margin_px=CENTER_MARGIN_PX,
        align_exit_margin_px=ALIGN_EXIT_MARGIN_PX,
        kp=KP,
        ki=KI,
        kd=KD,
        min_turret_weight=MIN_TURRET_WEIGHT,
        max_turret_weight=MAX_TURRET_WEIGHT,
        small_weight_cutoff=SMALL_WEIGHT_CUTOFF,
    ):
        self.center_margin_px = center_margin_px
        self.align_exit_margin_px = align_exit_margin_px
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_turret_weight = min_turret_weight
        self.max_turret_weight = max_turret_weight
        self.small_weight_cutoff = small_weight_cutoff
        self.is_aligned = False
        self._prev_error = 0.0
        self._integral = 0.0
        self._prev_time = None

    # 수평 조준기 초기화 - 탐지 대상X, timeout, 이전 PID 값 초기화
    def reset(self):
        self.is_aligned = False
        self._prev_error = 0.0
        self._integral = 0.0
        self._prev_time = None

    # 탐지 대상이 없으면 포탑 정지 명령 생성 함수
    def build_no_target_command(self, image_width):
        self.reset()
        screen_center_x = float(image_width / 2)
        return {
            "turretQE": {"command": "", "weight": 0.0},
            "bbox_center_x": None,
            "screen_center_x": screen_center_x,
            "center_range": [
                float(screen_center_x - self.center_margin_px),
                float(screen_center_x + self.center_margin_px),
            ],
            "error_x": None,
            "pid_output": 0.0,
            "is_aligned": False,
        }

    # 탐지된 bbox를 기준으로 포탑을 왼쪽(Q) 또는 오른쪽(E)으로 얼마나 돌릴지 계산하는 핵심 함수
    def build_command(self, bbox, image_width, now=None):
        # bbox 중심 계산
        x1, _, x2, _ = bbox
        bbox_center_x = (x1 + x2) / 2
        # 화면 중심 계산
        screen_center_x = image_width / 2
        # 중앙 허용 범위 계산
        center_left = screen_center_x - self.center_margin_px
        center_right = screen_center_x + self.center_margin_px
        # X축 오차 계산(중앙 범위와 bbox 중심 사이 오차)
        error_x = bbox_center_x - screen_center_x
        # 포탑 회전 명령 생성
        alignment_margin = (
            self.align_exit_margin_px
            if self.is_aligned
            else self.center_margin_px
        )
        # bbox 중심이 중앙에 들어온 경우
        if abs(error_x) <= alignment_margin:
            self.is_aligned = True
            turret_command = ""
            turret_weight = 0.0
            pid_output = 0.0
            self._reset_pid_terms()
        # 중앙에서 벗어난 경우
        else:
            self.is_aligned = False
            pid_output = self._calculate_pid_output(error_x, screen_center_x, now)
            turret_command = "Q" if pid_output < 0 else "E" # PID 출력 부호에 따라 회전 방향 결정
            # turret weight 제한 (너무 큰 출력 방지)
            turret_weight = min(
                max(abs(pid_output), self.min_turret_weight),
                self.max_turret_weight,
            )
            # 너무 작은 weight 계산 값은 제거 → 0으로 처리 - 미세 떨림 방지
            if turret_weight < self.small_weight_cutoff:
                turret_command = ""
                turret_weight = 0.0

        return {
            "turretQE": {   # 실제 포신 제어에 쓰는 값 반환
                "command": turret_command,
                "weight": turret_weight,
            },
            # 디버깅용, 로그 출력용 반환 값
            "bbox_center_x": float(bbox_center_x),
            "screen_center_x": float(screen_center_x),
            "center_range": [float(center_left), float(center_right)],
            "error_x": float(error_x),
            "pid_output": float(pid_output),
            "is_aligned": self.is_aligned,
        }

    # PID 계산에 사용하는 내부 변수 초기화 함수
    # 조준 완료 상태에서 초기화할 때 - 완료 상태에서 더 이상 계산을 하지 않게 하기 위해서
    def _reset_pid_terms(self):
        self._prev_error = 0.0
        self._integral = 0.0
        self._prev_time = None

    # 픽셀 단위의 x축 오차를 정규화하고, PID 제어식을 적용해서 포탑 회전 강도를 계산하는 함수
    def _calculate_pid_output(self, error_x, screen_center_x, now):
        # PID dt 계산에 사용
        if now is None:
            now = time.monotonic()

        # dt 계산
        if self._prev_time is None:
            dt = 0.033  # 초기값 - 30FPS 기준 프레임 시간
        else:
            dt = now - self._prev_time
            if dt <= 0:
                dt = 0.033

        # 오차 정규화 - 픽셀 오차를 화면 중심값으로 나눠서
        normalized_error = (
            error_x / screen_center_x if screen_center_x != 0 else 0.0
        )
        # I 항 계산 → 현재 코드는 I 게인을 사용하지 않긴 함
        self._integral += normalized_error * dt
        # D 항 계산
        derivative = (normalized_error - self._prev_error) / dt
        # PID 출력 계산
        # pid_output = KP × 현재오차 + KI × 누적오차 + KD × 오차변화율
        pid_output = (
            self.kp * normalized_error
            + self.ki * self._integral
            + self.kd * derivative
        )

        # 이전 값 갱신 → 다음 D 항 계산을 위해 저장
        self._prev_error = normalized_error
        self._prev_time = now
        # PID 출력 반환
        return pid_output