#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Header
import json
import time

class MarkArrayPublisher(Node):
    def __init__(self):
        super().__init__('markarray_publisher')
        self.publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        self.subscription = self.create_subscription(
            String,
            '/detection_data',
            self.target_position_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info("MarkArrayPublisher 노드가 시작되었습니다. '/detection_data' 토픽을 구독 중입니다.")

        # 큐브별 마지막 인식 시간을 저장할 딕셔너리
        self.last_seen = {}
        self.delete_timeout = 1.0  # 3초 동안 인식되지 않으면 삭제

    def target_position_callback(self, msg):
        try:
            # JSON 문자열을 파싱하여 객체 리스트 추출
            data = json.loads(msg.data)
            if not isinstance(data, list):
                self.get_logger().error("Expected a list of objects in JSON data.")
                return

            marker_array = MarkerArray()  # 여러 마커를 위한 MarkerArray 생성
            current_time = time.time()
            active_labels = set()  # 현재 프레임에서 인식된 레이블 집합

            for idx, obj in enumerate(data):
                label = obj.get("label")
                x = obj.get("x")
                y = obj.get("y")
                z = obj.get("z")

                if x is None or y is None or z is None or label is None:
                    self.get_logger().error("Invalid data format: 'x', 'y', 'z', and 'label' are required for each object.")
                    continue

                self.get_logger().info(f"Received target position: label={label}, x={x}, y={y}, z={z}")

                # 마커 생성
                marker = Marker()
                marker.header = Header()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = 'base_link'
                marker.id = hash(label)  # 레이블을 기반으로 고유 ID 생성
                marker.type = Marker.CUBE
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

                # 레이블에 따라 색상 설정
                if label == "red-cube":
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                elif label == "yellow-cube":
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                elif label == "blue-cube":
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.color.a = 1.0
                else:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                    marker.color.a = 1.0

                # 현재 프레임에서 인식된 레이블 기록
                active_labels.add(label)

                # 마지막 인식 시간 갱신
                self.last_seen[label] = current_time

                # MarkerArray에 마커 추가
                marker_array.markers.append(marker)
                self.get_logger().info(f"Prepared Marker for {label} at: ({marker.pose.position.x}, {marker.pose.position.y}, {marker.pose.position.z})")

            # 일정 시간 동안 인식되지 않은 마커를 삭제
            for label, last_time in list(self.last_seen.items()):
                if label not in active_labels and (current_time - last_time > self.delete_timeout):
                    # 해당 레이블의 마커를 삭제
                    delete_marker = Marker()
                    delete_marker.header.frame_id = 'base_link'
                    delete_marker.id = hash(label)
                    delete_marker.action = Marker.DELETE
                    marker_array.markers.append(delete_marker)
                    self.get_logger().info(f"Deleted marker for label: {label}")
                    del self.last_seen[label]

            # 모든 마커를 포함한 MarkerArray 퍼블리시
            self.publisher.publish(marker_array)
            self.get_logger().info("Published all markers in MarkerArray")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON data: {e}")

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkArrayPublisher()
    try:
        rclpy.spin(marker_publisher)
    except KeyboardInterrupt:
        marker_publisher.get_logger().info("MarkArrayPublisher 노드가 키보드 인터럽트로 종료됩니다.")
    finally:
        marker_publisher.destroy_node()
        rclpy.shutdown()
        print("ROS2가 정상적으로 종료되었습니다.")

if __name__ == '__main__':
    main()
