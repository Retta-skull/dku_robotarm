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
        self.subscription  # prevent unused variable warning

        # URDF 파일을 통해 체인 생성 (상대 경로 사용)
        try:
            package_share_directory = get_package_share_directory('robotarm')
            urdf_path = os.path.join(package_share_directory, 'urdf', 'robotarm.urdf')
            self.chain = Chain.from_urdf_file(urdf_path)
            self.get_logger().info(f"URDF 파일 '{urdf_path}'을 성공적으로 로드했습니다.")
        except Exception as e:
            self.get_logger().error(f"URDF 파일 로드 실패: {e}")
            raise e

    def target_position_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().error(f"Invalid target position data length: expected 3, got {len(msg.data)}")
            return

        x, y, z = msg.data
        target_position = [x / 10.0, y / 10.0, z / 10.0]  # 단위 변환 (예: mm -> m)
        self.get_logger().info(f"Received Target Position: {target_position}")

        try:
            ik_solution = self.chain.inverse_kinematics(
                target_position,
                max_iter=1000,  # 최대 반복 횟수
                # 기타 매개변수
            )
            end_effector_position = self.chain.forward_kinematics(ik_solution)[:3, 3]
            self.get_logger().info(f"Calculated End Effector Position: {end_effector_position}")

            # 오차 계산
            error_vector = np.array(target_position) - np.array(end_effector_position)
            error_magnitude = np.linalg.norm(error_vector)

            # if error_magnitude > ERROR_THRESHOLD:
            #     self.get_logger().error(f"Position error too large: {error_magnitude:.4f}m. Movement aborted.")
            #     return
            # else:
            self.get_logger().info(f"Position error within tolerance: {error_magnitude:.4f}m. Proceeding with movement.")

            # 조인트 각도 (degree 단위) 추출 및 퍼블리시
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
