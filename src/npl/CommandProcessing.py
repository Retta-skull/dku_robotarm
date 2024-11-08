import sys
import os
import yaml
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)
from gpt_api.DataPreprocessing import ActionData

class ActionLoader:
    def __init__(self, file_path):
        self.file_path = file_path  # YAML 파일 경로를 인스턴스 변수로 저장
        self.yaml_data = self._load_yaml()  # YAML 데이터를 미리 로드하여 저장

    # YAML 파일 로드 메서드
    def _load_yaml(self):
        with open(self.file_path, 'r', encoding='utf-8') as file:
            return yaml.safe_load(file)

def main():
    file_path = "/home/retta/ros2_ws/src/dku_robotarm/src/npl/robot_commands.yaml"  # YAML 파일 이름 설정
    action_loader = ActionLoader(file_path)

    # 사용자 명령어에서 추출한 intent와 YOLO 결과
    intent = "집어줘"
    yolo_detections = {"red_block": [100, 150, 200]}  # YOLO로부터 받은 좌표 예시

    # ActionData 리스트 생성
    actions = action_loader.get_actions(intent, yolo_detections)
    print(actions)

if __name__ == '__main__':
    main()