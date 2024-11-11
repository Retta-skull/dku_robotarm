#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from roboticstoolbox import ERobot, IK_NR
from spatialmath import SE3
from ament_index_python.packages import get_package_share_directory
import os

ERROR_THRESHOLD = 0.01

class AngleSetter(Node):
    def __init__(self):
        super().__init__('angle_setter')
        self.publisher = self.create_publisher(Float32MultiArray, '/joint_angles', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/target_position',
            self.target_position_callback,
            10
        )

        try:
            package_share_directory = get_package_share_directory('dku_robotarm')
            urdf_path = os.path.join(package_share_directory, 'urdf', 'dku_robotarm.urdf')
            links, name, urdf_string, urdf_filepath = ERobot.URDF_read(urdf_path)
            self.robot = ERobot(links, name=name, urdf_string=urdf_string, urdf_filepath=urdf_filepath, gripper_links=links[10])
            self.solver = IK_NR()  # IK_NR 솔버 초기화
            self.get_logger().info(f"URDF 파일 '{urdf_path}'을 성공적으로 로드했습니다.")
        except Exception as e:
            self.get_logger().error(f"URDF 파일 로드 실패: {e}")
            raise e

    def target_position_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().error(f"Invalid target position data length: expected 3, got {len(msg.data)}")
            return

        x, y, z = msg.data
        target_position = [x / 10.0, y / 10.0, z / 10.0]  # 단위 변환 (예: mm -> cm)
        self.get_logger().info(f"Received Target Position: {target_position}")

        # 목표 위치를 SE3 객체로 변환
        target_pose = SE3(target_position[0], target_position[1], target_position[2])

        try:
            # IK_NR 솔버를 사용해 역운동학 계산
            ik_solution = self.solver.solve(self.robot.ets(), target_pose)

            if ik_solution is None:
                self.get_logger().error("IK 솔버가 수렴하지 않았습니다. 다른 목표 위치를 시도하세요.")
                return

            joint_angles = [np.degrees(angle) for angle in ik_solution.q[:4]]
            angle_msg = Float32MultiArray()
            angle_msg.data = joint_angles
            self.publisher.publish(angle_msg)
            self.get_logger().info(f"Published Joint Angles: {angle_msg.data}")

        except (ValueError, ArithmeticError) as e:
            self.get_logger().error(f"IK calculation error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error during IK calculation: {e}")

def main(args=None):
    rclpy.init(args=args)
    angle_setter = AngleSetter()
    try:
        rclpy.spin(angle_setter)
    except KeyboardInterrupt:
        angle_setter.get_logger().info("AngleSetter node interrupted by user.")
    finally:
        angle_setter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
