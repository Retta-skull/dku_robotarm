#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.get_logger().info("JointStatePublisher 노드가 시작되었습니다.")

        # 초기 조인트 각도 (도 단위로 가정)
        self.base_joint = 0.0
        self.link1_link2 = 0.0
        self.link2_link3 = 0.0
        self.link3_link4 = 0.0
        self.link4_link5 = 0.0
        self.main_gripper1 = 0.0
        self.main_gripper2 = 0.0
        self.sub_gripper1 = 0.0
        self.sub_gripper2 = 0.0
        self.end_effector = 0.0

        # 조인트 상태 퍼블리셔 생성
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info("'/joint_states' 토픽에 대한 퍼블리셔가 생성되었습니다.")

        # 조인트 각도 구독자 생성
        self.joint_subscription = self.create_subscription(
            Float32MultiArray,
            '/joint_angles',
            self.angle_callback,
            10
        )
        self.gripper_subscription = self.create_subscription(
            Float32MultiArray,
            '/gripper_angle',
            self.Gripper_callback,
            10
        )
        
        self.joint_subscription  # prevent unused variable warning
        self.gripper_subscription
        self.get_logger().info("'/joint_angles' 토픽에 대한 구독자가 생성되었습니다.")
        self.get_logger().info("'/gripper_angle' 토픽에 대한 구독자가 생성되었습니다.")

        # 퍼블리시 타이머 설정 (0.1초 간격)
        self.timer = self.create_timer(0.1, self.publish_joint_state)
        self.get_logger().info("퍼블리시 타이머가 설정되었습니다. 0.1초마다 조인트 상태를 퍼블리시합니다.")

    def angle_callback(self, msg):
        self.get_logger().debug(f"Received message on '/joint_angles': {msg.data}")
        if len(msg.data) == 4:
            self.base_joint, self.link1_link2, self.link2_link3, self.link3_link4 = msg.data
            self.get_logger().info(
                f"Received radians: base_joint={math.radians(self.base_joint):.3f}, "
                f"link1_link2={math.radians(self.link1_link2):.3f}, "
                f"link2_link3={math.radians(self.link2_link3):.3f}, "
                f"link3_link4={math.radians(self.link3_link4):.3f}"
            )
        else:
            self.get_logger().error(f"Invalid data length: expected 1 or 4, got {len(msg.data)}")

    def Gripper_callback(self, msg):
        if len(msg.data) == 1:
            grip_angle = msg.data[0]/2
            self.main_gripper1 = -grip_angle
            self.main_gripper2 = grip_angle
            self.sub_gripper1 = grip_angle
            self.sub_gripper2 = -grip_angle
            self.get_logger().info(
                f"Received radians: grip_angle={math.radians(self.main_gripper1):.3f} "
            )
        else:
            self.get_logger().error(f"Invalid data length: expected 1, got {len(msg.data)}")

    def publish_joint_state(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['base_joint', 'link1_link2', 'link2_link3', 'link3_link4', 'link4_link5', 'main_gripper1', 'main_gripper2', 'sub_gripper1', 'sub_gripper2', 'end_effector']
        joint_state.position = [
            math.radians(self.base_joint), 
            math.radians(self.link1_link2), 
            math.radians(self.link2_link3), 
            math.radians(self.link3_link4),
            math.radians(self.link4_link5),
            math.radians(self.main_gripper1),
            math.radians(self.main_gripper2),
            math.radians(self.sub_gripper1),
            math.radians(self.sub_gripper2),
            math.radians(self.end_effector),
        ]
        joint_state.velocity = []
        joint_state.effort = []

        self.publisher.publish(joint_state)
        self.get_logger().debug(f"Published JointState: {joint_state}")

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        print("JointStatePublisher 노드가 키보드 인터럽트로 종료됩니다.")
    finally:
        if rclpy.ok():
            joint_state_publisher.destroy_node()
            try:
                rclpy.shutdown()
            except rclpy.RCLError as e:
                print(f"Shutdown 오류 발생: {e}")
        print("ROS2가 정상적으로 종료되었습니다.")

if __name__ == '__main__':
    main()
