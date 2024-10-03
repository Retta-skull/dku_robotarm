#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Header
import numpy as np

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('mark_publisher')
        self.publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/target_position',
            self.target_position_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info("MarkerPublisher 노드가 시작되었습니다. '/target_position' 토픽을 구독 중입니다.")

    def target_position_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().error(f"Invalid target position data length: expected 3, got {len(msg.data)}")
            return

        x, y, z = msg.data
        self.get_logger().info(f"Received target position: x={x}, y={y}, z={z}")

        try:
            # 마커 생성
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'base_link'  # 마커가 표시될 프레임
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # 마커의 위치와 크기 설정
            marker.pose.position.x = x / 10
            marker.pose.position.y = y / 10
            marker.pose.position.z = z / 10
            marker.pose.orientation.w = 1.0

            # 크기를 크게 설정
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # 마커 퍼블리시
            self.publisher.publish(marker)
            self.get_logger().info(f"Published Marker at: ({marker.pose.position.x}, {marker.pose.position.y}, {marker.pose.position.z})")

        except Exception as e:
            self.get_logger().error(f"Failed to publish marker: {e}")

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()
    try:
        rclpy.spin(marker_publisher)
    except KeyboardInterrupt:
        marker_publisher.get_logger().info("MarkerPublisher 노드가 키보드 인터럽트로 종료됩니다.")
    finally:
        marker_publisher.destroy_node()
        rclpy.shutdown()
        print("ROS2가 정상적으로 종료되었습니다.")

if __name__ == '__main__':
    main()
