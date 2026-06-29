import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

from flask import Flask, request, jsonify

from aim_coordinator import AimCoordinator
from lidar_endpoint_store import update_lidar_points
from lidar_top_view import lidar_top_view_blueprint
from target_lidar_service import annotate_detections_with_lidar
from yolo_detector import TargetDetector
import json


app = Flask(__name__)

# 테스트 전용 LiDAR top view - 삭제 시 import와 아래 등록 코드를 함께 제거
app.register_blueprint(lidar_top_view_blueprint)

# 디버깅전용 요청 로그 출력 EndPoint
# @app.before_request
# def log_all_requests():
#     print(f"REQUEST: {request.method} {request.path}", flush=True)


detector = TargetDetector("best.pt")    # YOLO 모델을 이용해서 객체 탐지 - 26n 사용 
aim_coordinator = AimCoordinator()      # 탐지 결과로 포탑 조준 명령 생성 객체

#######################################################################################
# YOLO 탐지 수행 EndPoint
@app.route('/detect', methods=['POST'])
def detect():
    image = request.files.get('image')
    if not image:
        return jsonify({"error": "No image received"}), 400

    image_path = 'temp_image.jpg'
    image.save(image_path)

    # YOLO 탐지 수횅 - detector 객체 사용
    detection_result = detector.detect_image(image_path)

    # 신뢰도 높은 객체를 target으로 선택 → 수평 조준 갱신(turretQE 명령 생성)
    target, _ = aim_coordinator.update_from_detections( # 포탑 조준 명령 생성 객체 이용
        detection_result.detections,
        detection_result.image_width,
        detection_result.image_height,
    )

    # 각 bbox 중앙 70% 영역에서 수직 표면을 찾고, 가장 먼 거리 군집의 평균값 저장
    target_lidar_average = annotate_detections_with_lidar(
        detection_result.detections,
        detection_result.image_width,
        detection_result.image_height,
        target=target,
    )

    # target의 라이다 평균값 저장
    aim_coordinator.update_target_lidar_average(target_lidar_average)

    return jsonify(detection_result.detections) # YOLO 탐지 결과 반환

#######################################################################################
# 좌우 스테레오 이미지 수신 EndPoint
@app.route('/stereo_image', methods=['POST'])
def stereo_image():
    left_image = request.files.get('left_image')
    right_image = request.files.get('right_image')

    if not left_image or not right_image:
        return jsonify({"result": "error", "message": "Left or Right image missing"}), 400

    left_path = "temp_left.jpg"
    right_path = "temp_right.jpg"

    try:
        left_image.save(left_path)
        right_image.save(right_path)
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)}), 500

    return jsonify({"result": "success"})
    

#######################################################################################
# 현재 상태 확인용, 디버깅
@app.route('/info', methods=['POST'])
def info():
    data = request.get_json(force=True)
    if not data:
        return jsonify({"error": "No JSON received"}), 400

    # /info EndPoint로 받은 최신 라이다 정보를 메모리에 저장
    if "lidarPoints" in data:
        lidar_points = data.get("lidarPoints")
        if isinstance(lidar_points, list):
            detected_point_count = update_lidar_points(lidar_points)
            print(
                "[LiDAR endpoint] "
                f"received={len(lidar_points)}, "
                f"detected={detected_point_count}",
                flush=True,
            )

    if "playerBodyY" in data:
        aim_coordinator.update_player_body_y(data.get("playerBodyY"))

    player_pos = data.get("playerPos", {})
    if player_pos:
        print(
            "[player] Current position: "
            f"x={player_pos.get('x')}, y={player_pos.get('y')}, z={player_pos.get('z')}",
            flush=True,
        )

    # print(
    #     "[/info 전체 데이터]\n"
    #     + json.dumps(data, indent=2, ensure_ascii=False),
    #     flush=True,
    # )

    return jsonify({"status": "success", "control": ""})    # 자동정지, 리셋 기능 구현 가능


#######################################################################################
# 포탑 조준 명령 생성 EndPoint, 추가로 이동 명령도 이곳에 구현
@app.route('/get_action', methods=['POST'])
def get_action():
    data = request.get_json(force=True) # 현재 전차 위치와 turret 각도 데이터 받음
    return jsonify(aim_coordinator.build_action(data))  # 


#######################################################################################
# 포탄 충돌 지점 수신 EndPoint
@app.route('/update_bullet', methods=['POST'])
def update_bullet():
    data = request.get_json()
    if not data:
        return jsonify({"status": "ERROR", "message": "Invalid request data"}), 400

    # 맞은 객체 이름까지 출력
    print(f"💥 Bullet Impact at X={data.get('x')}, Y={data.get('y')}, Z={data.get('z')}, Target={data.get('hit')}")
    return jsonify({"status": "OK", "message": "Bullet impact data received"})


#######################################################################################
# 목적지 좌표 수진 EndPoint
# 경로 생성 목적지 등등으로 사용
@app.route('/set_destination', methods=['POST'])
def set_destination():
    data = request.get_json()
    if not data or "destination" not in data:
        return jsonify({"status": "ERROR", "message": "Missing destination data"}), 400

    try:
        x, y, z = map(float, data["destination"].split(","))
        print(f"🚩 Destination set to: x={x}, y={y}, z={z}")
        return jsonify({"status": "OK", "destination": {"x": x, "y": y, "z": z}})
    except Exception as e:
        return jsonify({"status": "ERROR", "message": f"Invalid format: {str(e)}"}), 400


#######################################################################################
# 장애물 정보 수신 EndPoint
@app.route('/update_obstacle', methods=['POST'])
def update_obstacle():
    data = request.get_json()
    if not data:
        return jsonify({'status': 'error', 'message': 'No data received'}), 400

    print("🪨 Obstacle Data:", data)
    return jsonify({'status': 'success', 'message': 'Obstacle data received'})

#######################################################################################
# 충돌 이벤트 수신 EndPoint
@app.route('/collision', methods=['POST']) 
def collision():
    data = request.get_json()
    if not data:
        return jsonify({'status': 'error', 'message': 'No collision data received'}), 400

    object_name = data.get('objectName')
    position = data.get('position', {})
    x = position.get('x')
    y = position.get('y')
    z = position.get('z')

    print(f"🫨 Collision Detected - Object: {object_name}, Position: ({x}, {y}, {z})")

    return jsonify({'status': 'success', 'message': 'Collision data received'})


#######################################################################################
# 시뮬레이터 초기 설정
@app.route('/init', methods=['GET'])
def init():
    config = {
        "startMode": "start",  # Options: "start" or "pause"
        "blStartX": 60,  #Blue Start Position
        "blStartY": 10,
        "blStartZ": 27,
        "rdStartX": 5, #Red Start Position  일단은 다른 환경 테스트 용으로 코너에 설치함
        "rdStartY": 10,
        "rdStartZ": 290,
        "trackingMode": False,
        "detectMode": True,
        "logMode": True,
        "stereoCameraMode": False,
        "enemyTracking": False,
        "saveSnapshot": False,
        "saveLog": True,
        "saveLidarData": False,
        "lux": 30000,
        "destoryObstaclesOnHit" : True
    }
    print("🔰Initialization config sent via /init:", config)
    return jsonify(config)

#######################################################################################
# 시작 명령 수신 확인용 EndPoint
@app.route('/start', methods=['GET'])
def start():
    print("🛫 /start command received")
    return jsonify({"control": ""})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
