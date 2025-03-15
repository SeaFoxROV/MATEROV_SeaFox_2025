#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QComboBox
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.image_data = [None] * 4  # Store latest frame for each camera

        self.topic_names = [
            'camera1/image_raw',
            'camera2/image_raw',
            'camera3/image_raw',
            'camera4/image_raw'
        ]

        # Create subscribers for each camera topic
        self.subscribers = []
        for i, topic in enumerate(self.topic_names):
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, idx=i: self.callback(msg, idx),
                10
            )
            self.subscribers.append(sub)

    def callback(self, msg, index):
        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_data[index] = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")


class CameraGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node  # ROS2 subscriber node

        # Window setup
        self.setWindowTitle('ROV Camera Viewer')
        self.setFixedSize(800, 600)

        # Dropdown selector
        self.dropdown = QComboBox()
        self.dropdown.addItems([
            'Camera 1',
            'Camera 2',
            'Camera 3',
            'Camera 4'
        ])
        self.dropdown.currentIndexChanged.connect(self.change_camera)

        # Label to display image
        self.label = QLabel()
        self.label.setFixedSize(780, 540)  # Adjust as needed for your camera resolution

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.dropdown)
        layout.addWidget(self.label)
        self.setLayout(layout)

        # Camera index to display
        self.current_camera_index = 0

        # Timer to update GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(33)  # ~30 FPS

    def change_camera(self, index):
        self.current_camera_index = index

    def update_image(self):
        frame = self.node.image_data[self.current_camera_index]
        if frame is not None:
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            q_img = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img).scaled(self.label.width(), self.label.height())
            self.label.setPixmap(pixmap)
        else:
            # Optional: Show "No signal" image or text when no frame available
            self.label.setText("No signal from camera.")


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    # ROS2 subscriber node
    camera_node = CameraSubscriber()

    # PyQt5 GUI
    gui = CameraGUI(camera_node)
    gui.show()

    # QTimer to handle ROS callbacks
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(camera_node, timeout_sec=0.01))
    spin_timer.start(10)

    # App execution
    exit_code = app.exec_()

    # Cleanup
    camera_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
