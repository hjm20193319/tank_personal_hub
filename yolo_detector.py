"""
yolo_detector.py

Ultralytics YOLO 모델을 이용해 입력 이미지에서 객체를 탐지하고,
프로젝트에서 사용할 detection 형식으로 변환하는 모듈이다.

주요 기능:
1. best.pt YOLO 모델을 로드한다.
2. 입력 이미지에 대해 YOLO 추론을 수행한다.
3. YOLO raw box 결과 [x1, y1, x2, y2, confidence, class_id]를 읽는다.
4. 프로젝트에서 사용할 클래스만 필터링한다.
5. 각 객체를 className, baseClassName, bbox, confidence, color 등을 포함한 dict로 변환한다.
6. detection 리스트와 원본 이미지 크기를 DetectionResult로 반환한다.

입력:
- image_path:
    탐지할 이미지 파일 경로

출력:
- DetectionResult:
    detections:
        필터링된 detection 리스트
    image_height:
        입력 이미지 세로 픽셀 수
    image_width:
        입력 이미지 가로 픽셀 수

주의:
- TARGET_CLASSES의 class_id 매핑은 YOLO 모델의 실제 클래스 순서와 일치해야 한다.
- bbox 형식은 [x1, y1, x2, y2]이며, 뒤쪽 수평 조준과 LiDAR 각도 계산에 사용된다.
"""

from dataclasses import dataclass

from ultralytics import YOLO


#######################################################################################
# 클래스 맵핑
TARGET_CLASSES = {
    0: "House",
    1: "Human",
    2: "Tank",
    3: "car",
}

CLASS_COLORS = {
    0: "#FF3333",
    1: "#00CC66",
    2: "#CC0000",
    3: "#33FF33",
}

#######################################################################################

@dataclass
class DetectionResult:  # YOLO 탐지 결과와 이미지 크기를 함께 담는 결과 객체
    detections: list
    image_height: int
    image_width: int

#######################################################################################
# YOLO 모델을 로드하고, 이미지 추론 결과를 프로젝트용 detection 형식으로 변환하는 클래스
class TargetDetector:
    def __init__(
        self,
        model_path="best.pt",
        target_classes=None,
        class_colors=None,
        logger=print,
    ):
        self.model = YOLO(model_path)
        self.target_classes = target_classes or TARGET_CLASSES
        self.class_colors = class_colors or CLASS_COLORS
        self.logger = logger
        self.logger(self.model.names)

    # 이미지 파일 하나를 YOLO로 추론하고, 필터링된 detection 결과와 이미지 크기를 반환하는 핵심 함수
    def detect_image(self, image_path):
        results = self.model(image_path)
        boxes = results[0].boxes.data.cpu().numpy()
        # 이미지 크기 추출 - 조준에 사용
        image_height, image_width = results[0].orig_shape

        return DetectionResult(
            detections=self._filter_boxes(boxes),
            image_height=image_height,
            image_width=image_width,
        )

    # YOLO raw bbox 결과 중 필요한 클래스만 골라서 프로젝트에서 쓰는 detection 딕셔너리 형태로 변환하는 함수
    def _filter_boxes(self, boxes):
        filtered_results = []
        for box in boxes:
            class_id = int(box[5])
            if class_id not in self.target_classes:
                continue

            class_name = self.target_classes[class_id]
            filtered_results.append(
                {
                    "className": class_name,
                    "baseClassName": class_name,
                    "bbox": [float(coord) for coord in box[:4]],
                    "confidence": float(box[4]),
                    "color": self.class_colors[class_id],
                    "filled": False,
                    "updateBoxWhileMoving": True,
                }
            )

        return filtered_results
