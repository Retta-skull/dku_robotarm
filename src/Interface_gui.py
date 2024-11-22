#!/usr/bin/env python3
import sys
import threading
import rclpy
from PyQt5.QtWidgets import QApplication
from Interface_gui.robot_controller_node import RobotControllerNode
from Interface_gui.robot_controller_app import RobotControllerApp

def main():
    rclpy.init()
    ros_node = RobotControllerNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,))
    ros_thread.start()

    app = QApplication(sys.argv)
    window = RobotControllerApp(ros_node)
    window.show()
    app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()


if __name__ == "__main__":
    main()
