#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from config import ASSISTANT_ID, API_KEY
from gpt_api.ChatHandler import ChatThreadHandler
from gpt_api.DataPreprocessing import ResponseParser
from gpt_api.RobotAction import ActionExecutor, Robot
from gpt_api.Transfersound import TextToSpeech, SpeechProcessor

def main():
    # ROS 2 초기화
    rclpy.init()

    # 노드 생성
    node = Node('robot_controller')

    # 로봇 제어 클래스 초기화
    robot = Robot(node)

    # GPT API 핸들러 초기화
    chat_handler = ChatThreadHandler(assistant_id=ASSISTANT_ID, api_key=API_KEY)
    tts = TextToSpeech(API_KEY)
    sp = SpeechProcessor(API_KEY)

    while rclpy.ok():
        # 음성 인식 시작
        temp_file = sp.record_audio()
        transcription = sp.transcribe_audio(temp_file)
        node.get_logger().info(f"Transcription: {transcription}")

        # 명령 메시지 정의
        CurrentPosition = [20, 20, 20]
        message = f"""
        Order: {transcription}
        CurrentPosition: {CurrentPosition}
        Data: [
        {{name: \"빨간블럭\", coordinates: (30, 10, 5)}},
        {{name: \"파란블럭\", coordinates: (20, 10, 5)}}, 
        {{name: \"초록블럭\", coordinates: (25, 10, 5)}}]
        """
        # GPT API에 명령 전송 및 응답 받기
        node.get_logger().info("GPT API에 명령을 전송 중...")
        response = chat_handler.run_chat(message)
        node.get_logger().info(f"GPT API 응답: {response}")

        # 응답 파싱
        parser = ResponseParser(response)
        parsed = parser.get_parsed_response()
        tts.speak(parsed.reply)

        if not parsed or not hasattr(parsed, 'actions'):
            node.get_logger().error("파싱된 응답에 액션 데이터가 없습니다.")
            continue

        # 액션 실행기 초기화 및 실행
        executor = ActionExecutor(parsed.actions, robot)
        executor.execute_all()
        CurrentPosition = executor.UpdatePosition()

        # ROS 2 퍼블리셔가 메시지를 전송할 시간을 줌
        rclpy.spin_once(node, timeout_sec=1.0)

if __name__ == "__main__":
    main()