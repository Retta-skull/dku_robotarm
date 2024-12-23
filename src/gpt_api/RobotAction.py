#!/usr/bin/env python3

import rclpy
import time
import math
from typing import List
from std_msgs.msg import Float32MultiArray
from gpt_api.DataPreprocessing import ActionData

class Robot:
    def __init__(self, node: rclpy.node.Node):
        self.node = node
        self.move_publisher = self.node.create_publisher(Float32MultiArray, '/target_position', 10)
        self.gripper_publisher = self.node.create_publisher(Float32MultiArray, '/gripper_angle', 10)
        self.carpus_publisher = self.node.create_publisher(Float32MultiArray, '/carpus_angle', 10)
        self.node.get_logger().info("로봇 제어 퍼블리셔 초기화 완료.")
        self.CurrentPosition = []
        self.move_robot([0, 18, 20])
        self.set_carpus([0, 18, 20])

    def move_robot(self, coordinates):
        if len(coordinates) != 3:
            self.node.get_logger().error("MOVE 명령을 위해서는 x, y, z의 3개의 입력 값이 필요합니다.")
            return
        msg = Float32MultiArray()
        msg.data = coordinates
        self.move_publisher.publish(msg)
        self.node.get_logger().info(f"MOVE 명령 실행: {coordinates}")
        self.CurrentPosition = coordinates

    def set_carpus(self, xyz):
        x, y, z = xyz
        try:
            msg = Float32MultiArray()
            theta_xy = math.atan2(y, x)  # 라디안 단위로 반환됨
            # if x >= 0:
            #     msg.data = [math.degrees(theta_xy) + 90]  # 라디안을 각도로 변환
            # elif x < 0:
            #     msg.data = [math.degrees(theta_xy) - 90]
            msg.data = [math.degrees(theta_xy)]
            self.carpus_publisher.publish(msg)
            self.node.get_logger().info(f"손목 각도 설정: XY 평면 각도={math.degrees(theta_xy):.2f}°")
        except Exception as e:
            self.node.get_logger().error(f"손목 각도 설정 중 오류 발생: {e}")

    def open_gripper(self):
        msg = Float32MultiArray()
        msg.data = [40]
        self.gripper_publisher.publish(msg)
        self.node.get_logger().info("그리퍼 열기 명령 실행.")

    def close_gripper(self):
        msg = Float32MultiArray()
        msg.data = [160]
        self.gripper_publisher.publish(msg)
        self.node.get_logger().info("그리퍼 닫기 명령 실행.")

class Action:
    def __init__(self, action_type, parameters=None):
        self.action_type = action_type  # 'MOVE', 'OPEN_GRIPPER', 'CLOSE_GRIPPER'
        self.parameters = parameters  # MOVE의 경우 좌표 등

    def execute(self, robot: Robot):
        if self.action_type == 'MOVE':
            robot.set_carpus(self.parameters)
            time.sleep(1)
            robot.move_robot(self.parameters)
        elif self.action_type == 'OPEN_GRIPPER':
            robot.open_gripper()
        elif self.action_type == 'CLOSE_GRIPPER':
            robot.close_gripper()
        else:
            robot.node.get_logger().error(f"알 수 없는 명령어: {self.action_type}")

class ActionExecutor:
    def __init__(self, actions: List[ActionData], robot: Robot):
        self.actions = actions  # ActionData 객체들의 리스트
        self.robot = robot

    def UpdatePosition(self):
        self.robot.node.get_logger().info(f"현재 위치: {self.robot.CurrentPosition}")
        return self.robot.CurrentPosition
    
    def execute_all(self):
        for idx, action_data in enumerate(self.actions, start=1):
            action = Action(action_data.action_type, action_data.parameters)
            self.robot.node.get_logger().info(f"--- {idx}번째 명령어: {action.action_type} ---")
            action.execute(self.robot)
            time.sleep(2)
            if action.action_type == 'MOVE':
                self.UpdatePosition()
        self.robot.node.get_logger().info("모든 명령어가 성공적으로 실행되었습니다.")