#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')

        # I2C 통신 초기화
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
        
        # 서보 모터 설정   
    
        self.base_joint = servo.Servo(self.pca.channels[0])    # 0번 채널: Base Joint
        self.link1_2 = servo.Servo(self.pca.channels[1])       # 1번 채널: Link1_2
        self.link2_3 = servo.Servo(self.pca.channels[2])       # 2번 채널: Link2_3
        self.link3_4 = servo.Servo(self.pca.channels[3])       # 3번 채널: Link3_4
        self.gripper = servo.Servo(self.pca.channels[5])

        
        self.base_joint.set_pulse_width_range(500, 2500)
        self.link1_2.set_pulse_width_range(500, 2500)
        self.link2_3.set_pulse_width_range(500, 2500)
        self.link3_4.set_pulse_width_range(500, 2500)
        self.gripper.set_pulse_width_range(500, 2500)

        # /joint_angles 토픽 구독
        self.joint_angles_subscription = self.create_subscription(
            Float32MultiArray,
            '/joint_angles',
            self.Move_callback,
            10
        )

        # /gripper_angle 토픽 구독
        self.gripper_angle_subscription = self.create_subscription(
            Float32MultiArray,
            '/gripper_angle',
            self.Gripper_callback,
            10
        )


    def Move_callback(self, msg):
        """ /joint_angles 토픽에서 받은 각도 값으로 서보 모터 제어 """
        angles = msg.data

        if len(angles) >= 4:
            # 각각의 모터에 각도를 설정 (변환 적용)
            self.set_joint_angle("base_joint", self.convert_angle(angles[0]))
            time.sleep(0.1)
            self.set_joint_angle("link1_2", 180 - self.convert_angle(angles[1]))
            time.sleep(0.1)
            self.set_joint_angle("link2_3", self.convert_angle(angles[2]))
            time.sleep(0.1)
            self.set_joint_angle("link3_4", self.convert_angle(angles[3]))
            self.get_logger().info(f"Received joint angles: {self.convert_angle(angles[0]),180 - self.convert_angle(angles[1]), self.convert_angle(angles[2]), self.convert_angle(angles[3])}")
        else:
            self.get_logger().warn("Received joint angles message has insufficient data.")

    def Gripper_callback(self, msg):
        print("grip")
        angle = msg.data
        self.set_joint_angle("gripper", angle[0])
        self.get_logger().info(f"Received gripper angles: {angle[0]}")



    def convert_angle(self, angle):
        """-90 ~ 90 범위의 각도를 0 ~ 180도로 변환"""
        return (angle + 90)

    def set_joint_angle(self, joint_name, angle):
        """주어진 관절을 특정 각도로 이동"""
        if joint_name == "base_joint":
            self.base_joint.angle = angle
        elif joint_name == "link1_2":
            self.link1_2.angle = angle
        elif joint_name == "link2_3":
            self.link2_3.angle = angle
        elif joint_name == "link3_4":
            self.link3_4.angle = angle
        elif joint_name == "gripper":
            self.gripper.angle = angle
        else:
            self.get_logger().warn(f"'{joint_name}'는 유효한 관절 이름이 아닙니다.")

    def disable_all_motors(self):
        """모든 서보 모터의 신호를 비활성화하여 힘 제거"""
        for i in range(5):
            self.pca.channels[i].duty_cycle = 0

    def cleanup(self):
        """PWM 드라이버 해제 및 모터 신호 비활성화"""
        self.disable_all_motors()
        self.pca.deinit()


def main(args=None):
    rclpy.init(args=args)

    # 노드 생성
    robot_arm = RobotArmController()

    try:
        rclpy.spin(robot_arm)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드를 종료하고 모터 해제
        robot_arm.cleanup()
        robot_arm.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
