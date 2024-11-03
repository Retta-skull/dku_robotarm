import cv2
import numpy as np
import logging
from ultralytics import YOLO

logging.getLogger('ultralytics').setLevel(logging.WARNING)

class YOLODetector:
    def __init__(self, model_path, offset_x=0, offset_y=0, scale_factor=1.0):
        self.model = YOLO(model_path, verbose=False)
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.scale_factor = scale_factor
        self.frame = None  # 현재 프레임을 저장할 변수

    def process_frame(self, frame):
        results = self.model(frame)

        for result in results:
            boxes = result.boxes  # 탐지된 객체의 바운딩 박스 정보
            
            for box in boxes:
                confidence = box.conf[0]
                if confidence >= 0.85:  # 신뢰도 0.9 이상인 경우에만 처리
                    x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                    class_id = int(box.cls[0])
                    label = f"{self.model.names[class_id]}: {confidence:.2f}"
                    
                    # 중앙 좌표 계산 및 스케일 변환
                    center_x = (x_min + x_max) // 2 - self.offset_x
                    center_y = (y_min + y_max) // 2 - self.offset_y
                    center_x_cm = center_x * self.scale_factor
                    center_y_cm = -(center_y * self.scale_factor)
                    center_label_cm = f"Center: ({center_x_cm:.2f} cm, {center_y_cm:.2f} cm)"
                    
                    # 결과 표시
                    self.draw_detection(frame, x_min, y_min, x_max, y_max, label, center_x, center_y, center_label_cm)
                    print(f"Detected: {label}")
                    print(f"  Bounding Box - x_min: {x_min}, y_min: {y_min}, x_max: {x_max}, y_max: {y_max}")
                    print(f"  Center (cm) - x: {center_x_cm:.2f} cm, y: {center_y_cm:.2f} cm")                    
        return frame

    def draw_detection(self, frame, x_min, y_min, x_max, y_max, label, center_x, center_y, center_label_cm):
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 중앙 좌표 표시
        cv2.circle(frame, (center_x + self.offset_x, center_y + self.offset_y), 5, (255, 0, 0), -1)
        cv2.putText(frame, center_label_cm, (center_x + self.offset_x, center_y + self.offset_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    def show_coordinates(self, event, x, y, flags, param):
        """
        마우스 클릭 이벤트를 처리하여 클릭한 위치의 픽셀 좌표 및 색상 값 출력
        """
        if event == cv2.EVENT_LBUTTONDOWN and self.frame is not None:  # 마우스 왼쪽 버튼 클릭 시
            # 클릭한 위치의 픽셀 값 (BGR)
            pixel_value = self.frame[y, x]
            print(f"좌표: (x={x}, y={y}), 픽셀 값 (BGR): {pixel_value}")

    def run(self):
        capture = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not capture.isOpened():
            print("카메라를 열 수 없습니다.")
            return

        cv2.namedWindow("YOLO Detection")
        # 'YOLO Detection' 창에 마우스 콜백 설정
        cv2.setMouseCallback("YOLO Detection", self.show_coordinates)

        while True:
            ret, frame = capture.read()
            if not ret or frame is None:
                print("프레임을 읽을 수 없습니다.")
                break

            # 현재 프레임을 self.frame에 저장
            self.frame = frame

            # 프레임을 처리하여 객체 탐지 결과 표시
            frame = self.process_frame(frame)
            cv2.imshow("YOLO Detection", frame)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    detector = YOLODetector(
        model_path="D:\\종합설계\\code\\camera\\best.pt", 
        offset_x=639, 
        offset_y=688, 
        scale_factor=0.0537634408602151
    )
    detector.run()
