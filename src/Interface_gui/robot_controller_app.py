#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTextEdit, QVBoxLayout, QWidget, QLabel, QHBoxLayout
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap, QImage, QTextCursor

class RobotControllerApp(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("Robot Controller GUI")
        self.resize(1920, 1020)
        self.ros_node = ros_node

        # GUI Widgets
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)

        self.image_label = QLabel()
        self.image_label.setStyleSheet("border: 1px solid black; background-color: #dddddd;")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setFixedSize(1280, 720)

        self.left_image_label = QLabel()
        self.left_image_label.setStyleSheet("border: 1px solid black; background-color: #dddddd;")
        self.left_image_label.setAlignment(Qt.AlignCenter)
        self.left_image_label.setFixedSize(480, 720)

        fixed_image_path = "/home/retta/ros2_ws/src/dku_robotarm/src/Interface_gui/image/resized_image.png"
        fixed_pixmap = QPixmap(fixed_image_path)
        self.left_image_label.setPixmap(fixed_pixmap.scaled(self.left_image_label.size(), Qt.KeepAspectRatio))

        self.detection_text = QTextEdit()
        self.detection_text.setReadOnly(True)

        # Layout Setup
        self.init_ui()

        # Timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_display)
        self.timer.timeout.connect(self.update_image)
        self.timer.start(100)

    def init_ui(self):
        main_layout = QVBoxLayout()

        top_layout = QHBoxLayout()
        top_layout.addWidget(self.left_image_label)
        top_layout.addWidget(self.image_label)
        main_layout.addLayout(top_layout)

        bottom_layout = QHBoxLayout()
        log_layout = QVBoxLayout()
        log_layout.addWidget(QLabel("Log:"))
        log_layout.addWidget(self.log_text)
        bottom_layout.addLayout(log_layout)

        detection_layout = QVBoxLayout()
        detection_layout.addWidget(QLabel("Detection Data:"))
        detection_layout.addWidget(self.detection_text)
        bottom_layout.addLayout(detection_layout)

        main_layout.addLayout(bottom_layout)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def update_display(self):
        try:
            log_output = self.ros_node.get_logger_output()
            if log_output:
                self.log_text.moveCursor(QTextCursor.End)
                self.log_text.insertPlainText(log_output)
        except Exception as e:
            print(f"Error updating log text: {e}")

        try:
            detection_data_text = self.ros_node.process_data()
            self.detection_text.setPlainText(f"Detection Data:\n{detection_data_text}")
        except Exception as e:
            print(f"Error updating detection data: {e}")

    def update_image(self):
        if self.ros_node.detection_image is not None:
            try:
                cv_image = self.ros_node.detection_image
                height, width, channel = cv_image.shape
                bytes_per_line = 3 * width
                qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)

                pixmap = QPixmap.fromImage(qt_image)
                self.image_label.setPixmap(pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio))
            except Exception as e:
                print(f"Error updating image: {e}")
