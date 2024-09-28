#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Interface(Node):
    def __init__(self):
        super().__init__('xyz_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, '/target_position', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초 간격으로 입력을 받음

    def timer_callback(self):
        try:
            xyz = input("Input your xyz (space-separated): ")
            xyz_values = list(map(float, xyz.strip().split()))
            if len(xyz_values) != 3:
                self.get_logger().error("3 input values required (x y z)")
                return
            msg = Float32MultiArray()
            msg.data = xyz_values
            self.publisher.publish(msg)
            self.get_logger().info(f"Published Target Position: {msg.data}")
        except ValueError as e:
            self.get_logger().error(f"Invalid input: {e}")
        except EOFError:
            self.get_logger().info("EOF detected. Shutting down XYZPublisher node.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    xyz_publisher = Interface()
    try:
        rclpy.spin(xyz_publisher)
    except KeyboardInterrupt:
        xyz_publisher.get_logger().info("XYZPublisher node interrupted by user.")
    finally:
        xyz_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
