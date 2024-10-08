#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpt_api.ChatHandler import ChatThreadHandler
from gpt_api.DataPreprocessing import ResponseParser
from gpt_api.RobotAction import ActionExecutor, Robot
from config import ASSISTANT_ID, API_KEY

def main():
    # ROS 2 초기화
    rclpy.init()

    # 노드 생성
    node = Node('robot_controller')

    # 로봇 제어 클래스 초기화
    robot = Robot(node)

    # GPT API 핸들러 초기화
    chat_handler = ChatThreadHandler(assistant_id=ASSISTANT_ID, api_key=API_KEY)

    # 명령 메시지 정의
    message = """
    Order: 왼쪽의 빨간블럭을 파란블럭 위치로 옮겨줘
    Data: [
      {name: "빨간블럭", coordinates: (20, 30, 10)},
      {name: "빨간블럭", coordinates: (10, 30, 10)},
      {name: "파란블럭", coordinates: (60, 25, 55)},
      {name: "초록블럭", coordinates: (35, 18, 45)}
    ]
    """

    # GPT API에 명령 전송 및 응답 받기
    node.get_logger().info("GPT API에 명령을 전송 중...")
    response = chat_handler.run_chat(message)
    node.get_logger().info(f"GPT API 응답: {response}")

    # 응답 파싱
    parser = ResponseParser(response)
    parsed = parser.get_parsed_response()

    if not parsed or not hasattr(parsed, 'actions'):
        node.get_logger().error("파싱된 응답에 액션 데이터가 없습니다.")
        rclpy.shutdown()
        return

    # 액션 실행기 초기화 및 실행
    executor = ActionExecutor(parsed.actions, robot)
    executor.execute_all()

    # ROS 2 퍼블리셔가 메시지를 전송할 시간을 줌
    rclpy.spin_once(node, timeout_sec=1.0)

    # ROS 2 종료
    rclpy.shutdown()

if __name__ == "__main__":
    main()
