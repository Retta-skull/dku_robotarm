#!/usr/bin/env python3
import rclpy
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
        self.chat_handler = ChatThreadHandler(assistant_id=ASSISTANT_ID, api_key=API_KEY)
        self.tts = TextToSpeech(API_KEY)
        self.sp = SpeechProcessor(API_KEY)
        self.CurrentPosition = [0, 20, 20]

        # YOLODetector에서 퍼블리시한 detection_data 구독
        self.create_subscription(String, 'detection_data', self.handle_detection_data, 10)
        self.detection_data = ""  # 최신 데이터만 유지

        # Timer 설정: 2초마다 main_loop 작업 실행
        self.timer = self.create_timer(5.0, self.main_loop)

    def handle_detection_data(self, msg):
        # 수신한 탐지 데이터를 최신 데이터로 갱신
        self.detection_data = msg.data
        self.get_logger().info(f"Detection Data: {msg.data}")

    def main_loop(self):
        temp_file = self.sp.record_audio()
        transcription = self.sp.transcribe_audio(temp_file)
        self.get_logger().info(f"Transcription: {transcription}")

        # 명령 메시지 생성 및 GPT API 전송
        message = f"""Order: {transcription}
                    CurrentPosition: {self.CurrentPosition}
                    Data: {self.detection_data}"""
        response = self.chat_handler.run_chat(message)
        self.get_logger().info(f"GPT API 응답: {response}")

        # 응답을 파싱하여 액션 실행
        parser = ResponseParser(response)
        parsed = parser.get_parsed_response()
        self.tts.speak(parsed.reply)

        if parsed and hasattr(parsed, 'actions'):
            executor = ActionExecutor(parsed.actions, self.robot)
            self.CurrentPosition = executor.UpdatePosition()
            executor.execute_all()

def main():
    rclpy.init()
    robot_controller = RobotController()
    rclpy.spin(robot_controller)  # 콜백을 비동기적으로 계속 수신
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
