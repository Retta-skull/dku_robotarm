#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
import math

class Move(Node):
    def __init__(self):
        super().__init__('xyz_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, '/target_position', 10)
        self.carpus_publisher = self.create_publisher(Float32MultiArray, '/carpus_angle', 10)
        self.get_logger().info("Move 노드가 초기화되었으며 목표 위치를 퍼블리시할 준비가 되었습니다.")
        self.move_xyz([0, 20, 20])

    def move_xyz(self, xyz):
        try:
            if len(xyz) != 3:
                self.get_logger().error("Move를 위해서는 x, y, z의 3개의 입력 값이 필요합니다.")
                return
            msg = Float32MultiArray()
            msg.data = xyz
            self.publisher.publish(msg)
            self.get_logger().info(f"목표 위치 퍼블리시됨: {msg.data}")
        except ValueError as e:
            self.get_logger().error(f"Move에 잘못된 입력: {e}")
        except Exception as e:
            self.get_logger().error(f"Move에서 예상치 못한 오류 발생: {e}")

    def set_carpus(self, xyz):
        x, y, z = xyz
        try:
            msg = Float32MultiArray()
            theta_xy = math.atan2(y, x)  # 라디안 단위로 반환됨
            msg.data = [math.degrees(theta_xy)+90]  # 라디안을 각도로 변환
            self.carpus_publisher.publish(msg)
            self.get_logger().info(f"손목 각도 설정: XY 평면 각도={math.degrees(theta_xy):.2f}°")
        except Exception as e:
            self.get_logger().error(f"손목 각도 설정 중 오류 발생: {e}")

class Gripper(Node):
    def __init__(self):
        super().__init__('grip_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, '/gripper_angle', 10)
        self.get_logger().info("Gripper 노드가 초기화되었으며 그리퍼 각도를 퍼블리시할 준비가 되었습니다.")

    def set_gripper(self, angle):
        try:
            if not isinstance(angle, (int, float)):
                self.get_logger().error("그리퍼 각도는 숫자여야 합니다.")
                return
            msg = Float32MultiArray()
            msg.data = [float(angle)]
            self.publisher.publish(msg)
            self.get_logger().info(f"그리퍼 각도 퍼블리시됨: {msg.data[0]}")
        except ValueError as e:
            self.get_logger().error(f"그리퍼에 잘못된 입력: {e}")
        except Exception as e:
            self.get_logger().error(f"그리퍼에서 예상치 못한 오류 발생: {e}")

    def gripper_open(self):
        try:
            msg = Float32MultiArray()
            msg.data = [float(40)]
            self.publisher.publish(msg)
            self.get_logger().info("그리퍼 열기")
        except Exception as e:
            self.get_logger().error(f"그리퍼 오류 : {e}")

    def gripper_close(self):
        try:
            msg = Float32MultiArray()
            msg.data = [float(160)]
            self.publisher.publish(msg)
            self.get_logger().info("그리퍼 닫기")
        except Exception as e:
            self.get_logger().error(f"그리퍼 오류 : {e}")


class Interface(Node):
    def __init__(self):
        super().__init__('interface_node')
        self.move_node = Move()
        self.gripper_node = Gripper()
        self.get_logger().info("인터페이스 노드가 시작되었습니다. 명령을 대기 중...")

        # 사용자 입력을 처리하는 별도의 스레드 시작
        self.input_thread = threading.Thread(target=self.get_user_input)
        self.input_thread.daemon = True  # 메인 프로그램이 종료될 때 스레드도 종료되도록 설정
        self.input_thread.start()

    def get_user_input(self):
        while rclpy.ok():
            try:
                user_input = input("명령 입력 (Move x y z | open/close): ")
                if user_input.strip() == "":
                    continue
                parts = user_input.strip().split()

                if parts[0].lower() == "move":
                    if len(parts) != 4:
                        self.get_logger().error("알 수 없는 명령입니다.")
                        continue
                    try:
                        x, y, z = map(float, parts[1:4])
                        self.move_node.move_xyz([x, y, z])
                        self.move_node.set_carpus([x, y, z])
                    except ValueError:
                        self.get_logger().error("Move 명령은 세 개의 숫자 값을 필요로 합니다.")
                elif user_input == "close":
                    self.gripper_node.gripper_close()
                elif user_input == "open":
                    self.gripper_node.gripper_open()
                else:
                    self.get_logger().error("알 수 없는 명령입니다.")
            
            except EOFError:
                self.get_logger().info("EOF 감지. 인터페이스 노드를 종료합니다.")
                rclpy.shutdown()
            except Exception as e:
                self.get_logger().error(f"입력 처리 중 오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    interface = Interface()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(interface)
    executor.add_node(interface.move_node)
    executor.add_node(interface.gripper_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        interface.get_logger().info("KeyboardInterrupt 감지. 종료합니다.")
    finally:
        # 모든 노드를 안전하게 종료
        executor.shutdown()
        interface.move_node.destroy_node()
        interface.gripper_node.destroy_node()
        interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
