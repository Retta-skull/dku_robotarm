#!/usr/bin/env python3

import rclpy
import json
import threading
from rclpy.node import Node
from std_msgs.msg import String
from config import ASSISTANT_ID, API_KEY
from gpt_api.ChatHandler import ChatThreadHandler
from gpt_api.DataPreprocessing import ResponseParser
from gpt_api.RobotAction import ActionExecutor, Robot
from gpt_api.Transfersound import TextToSpeech, SpeechProcessor

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # 퍼블리셔 및 GPT 관련 초기화
        self.robot = Robot(self)
        self.chat_handler = ChatThreadHandler(assistant_id=ASSISTANT_ID, api_key=API_KEY, logger=self.get_logger())
        self.tts = TextToSpeech(API_KEY)
        self.sp = SpeechProcessor(API_KEY)
        self.CurrentPosition = [0, 18, 20]

        # YOLODetector에서 퍼블리시한 detection_data 구독
        self.create_subscription(String, 'detection_data', self.handle_detection_data, 10)
        self.detection_data = ""  # 최신 데이터만 유지

        # 메인 루프를 별도의 스레드로 실행
        self.loop_thread = threading.Thread(target=self.main_loop)
        self.loop_thread.start()

    def handle_detection_data(self, msg):
        # 수신한 탐지 데이터를 최신 데이터로 갱신
        self.detection_data = msg.data
        # self.get_logger().info(f"Detection Data: {msg.data}")
        if isinstance(self.detection_data, str):
            self.detection_data = json.loads(self.detection_data)

    def main_loop(self):
        while rclpy.ok():
            temp_file = self.sp.record_audio()
            transcription = self.sp.transcribe_audio(temp_file)
            self.get_logger().info(f"Transcription: {transcription}")

            data_string = ""
            for block in self.detection_data:
                if isinstance(block, dict):  # 딕셔너리인지 확인
                    name = block.get("label", "unknown")
                    coordinates = [block.get("x", 0), block.get("y", 0), block.get("z", 0)]
                    data_string += f"\n -name: \"{name}\"\n coordinates: {coordinates}\n"
                else:
                    self.get_logger().error("오류: 예상한 데이터 형식이 아닙니다.", block)

            message = f"Order: {transcription}\n CurrentPosition: {self.CurrentPosition} \nData: {data_string}"
            response = self.chat_handler.run_chat(message)
            self.get_logger().info(f"GPT API 응답: {response}")

            # 응답을 파싱하여 액션 실행
            parser = ResponseParser(response)
            parsed = parser.get_parsed_response()
            self.tts.speak(parsed.reply)

            if parsed and hasattr(parsed, 'actions'):
                executor = ActionExecutor(parsed.actions, self.robot)
                executor.execute_all()
                self.CurrentPosition = executor.UpdatePosition()

            # 루프마다 5초 대기
            threading.Event().wait(1)

def main():
    rclpy.init()
    robot_controller = RobotController()
    
    try:
        rclpy.spin(robot_controller)  # ROS 콜백 비동기 실행
    except KeyboardInterrupt:
        robot_controller.get_logger().info("RobotController node interrupted by user.")
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
