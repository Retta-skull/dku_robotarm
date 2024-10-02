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
        self.angle = 0
    
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
        while(1):
            self.input_test()


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
    
    def input_test(self):
        data = input("name, angle : ")
        data = data.split(" ")
        self.set_joint_angle(data[0], int(data[1]))


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
        robot_arm.cleanup()
        robot_arm.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
