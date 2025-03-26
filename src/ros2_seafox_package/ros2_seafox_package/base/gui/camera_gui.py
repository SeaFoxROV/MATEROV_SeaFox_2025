#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QComboBox,QSplitter
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt

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
        self.setFixedSize(1320, 960)

        # Dropdown selector
        self.dropdown = QComboBox()
        self.dropdown.addItems([
            'Camera 1 and 2',
            'Camera 2 and 3',
            'Camera 3 and 4',
            'Camera 4 and 1'
        ])
        self.dropdown2 = QComboBox()
        self.dropdown2.addItems([
            'Camera 1 and 2',
            'Camera 2 and 3',
            'Camera 3 and 4',
            'Camera 4 and 1'
        ])

        self.dropdown.currentIndexChanged.connect(self.change_camera)
        self.dropdown2.currentIndexChanged.connect(self.change_camera2)

        # Label to display image
        self.label = QLabel()
        self.label.setFixedSize(640, 480)  # Adjust as needed for your camera resolution
        self.label2 = QLabel()
        self.label2.setFixedSize(640, 480)  # Adjust as needed for your camera resolution
        self.label3 = QLabel()
        self.label3.setFixedSize(640, 480)  # Adjust as needed for your camera resolution
        self.label4 = QLabel()
        self.label4.setFixedSize(640, 480)  # Adjust as needed for your camera resolution

        # Layout
        layout = QVBoxLayout()
        dropsplitter = QSplitter(Qt.Horizontal)
        dropsplitter.addWidget(self.dropdown)
        dropsplitter.addWidget(self.dropdown2)
        layout.addWidget(dropsplitter)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.label)
        splitter.addWidget(self.label2)
        layout.addWidget(splitter)

        #remove for only 2 cameras
        splitter2 = QSplitter(Qt.Horizontal)
        splitter2.addWidget(self.label3)
        splitter2.addWidget(self.label4)
        layout.addWidget(splitter2)

        self.setLayout(layout)

        # Camera index to display
        self.current_camera_index = 0
        self.current_camera_index2 = 1
        self.current_camera_index3 = 2
        self.current_camera_index4 = 3

        # Timer to update GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(33)  # ~30 FPS

    def change_camera(self, index):
        self.current_camera_index = index
        self.current_camera_index3 = (index+1) if self.current_camera_index < 3 else 0

    def change_camera2(self, index2):
        self.current_camera_index2 = index2
        self.current_camera_index4 = (index2+1) if self.current_camera_index2 < 3 else 0


    def update_image(self):
        #top left
        frame = self.node.image_data[self.current_camera_index]
        if frame is not None:
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            q_img = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img).scaled(self.label.width(), self.label.height())
            self.label.setPixmap(pixmap)
        else:
            # Optional: Show "No signal" image or text when no frame available
            self.label.setText(f"\t\tNo signal from camera{self.current_camera_index+1}")

        frame2 = self.node.image_data[self.current_camera_index2]
        #top right
        if frame2 is not None:
            height, width, channel = frame2.shape
            bytes_per_line = 3 * width
            q_img2 = QImage(frame2.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap2 = QPixmap.fromImage(q_img2).scaled(self.label2.width(), self.label2.height())
            self.label2.setPixmap(pixmap2)
        else:
            # Optional: Show "No signal" image or text when no frame available
            self.label2.setText(f"\t\tNo signal from camera{self.current_camera_index2+1}")
        
        frame3 = self.node.image_data[self.current_camera_index3]
        #bottom left
        if frame3 is not None:
            height, width, channel = frame3.shape
            bytes_per_line = 3 * width
            q_img3 = QImage(frame3.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap3 = QPixmap.fromImage(q_img3).scaled(self.label2.width(), self.label2.height())
            self.label3.setPixmap(pixmap3)
        else:
            # Optional: Show "No signal" image or text when no frame available
            self.label3.setText(f"\t\tNo signal from camera{self.current_camera_index3+1}")

        frame4 = self.node.image_data[self.current_camera_index4]
        #bottom right
        if frame4 is not None:
            height, width, channel = frame4.shape
            bytes_per_line = 3 * width
            q_img4 = QImage(frame4.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap4 = QPixmap.fromImage(q_img4).scaled(self.label2.width(), self.label2.height())
            self.label4.setPixmap(pixmap4)
        else:
            # Optional: Show "No signal" image or text when no frame available
            self.label4.setText(f"\t\tNo signal from camera{self.current_camera_index4+1}")



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
