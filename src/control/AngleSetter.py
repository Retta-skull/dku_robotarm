#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from ikpy.chain import Chain
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
        self.subscription
        try:
            package_share_directory = get_package_share_directory('dku_robotarm')
            urdf_path = os.path.join(package_share_directory, 'urdf', 'dku_robotarm.urdf')
            self.chain = Chain.from_urdf_file(urdf_path, active_links_mask= [1, 1, 1, 1, 1, 0, 0])
            self.get_logger().info(f"URDF 파일 '{urdf_path}'을 성공적으로 로드했습니다.")
            for i, link in enumerate(self.chain.links):
                print(f"Index {i}: Link name = {link.name}")

        except Exception as e:
            self.get_logger().error(f"URDF 파일 로드 실패: {e}")
            raise e

    def target_position_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().error(f"Invalid target position data length: expected 3, got {len(msg.data)}")
            return

        x, y, z = msg.data

        if x > 0 and x < 10:
            pass
        elif x >= 10 and x < 20:
            x = x - 1
            y = y + 1
        elif x >= 20:
            x = x - 1
            y = y + 2
        elif x < 0 and x > -10:
            pass
        elif x <= -10 and x > -20:
            x = x - 1
        elif x <= -20:
            x = x - 2
        target_position = [x / 10.0, y / 10.0, z / 10.0]  # 단위 변환 (예: mm -> cm)
        target_orientation = [0, -90, 0]
        self.get_logger().info(f"Received Target Position: {target_position}")

        try:
            ik_solution = self.chain.inverse_kinematics(
                target_position,
                target_orientation,
                max_iter=1000,  
            )
            end_effector_position = self.chain.forward_kinematics(ik_solution)[:3, 3]
            self.get_logger().info(f"Calculated End Effector Position: {end_effector_position}")

            # 오차 계산
            error_vector = np.array(target_position) - np.array(end_effector_position)
            error_magnitude = np.linalg.norm(error_vector)

            self.get_logger().info(f"Position error within tolerance: {error_magnitude:.4f}m. Proceeding with movement.")

            joint_angles = [np.degrees(angle) for angle in ik_solution[1:5]]  # 필요에 따라 인덱스 조정
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
