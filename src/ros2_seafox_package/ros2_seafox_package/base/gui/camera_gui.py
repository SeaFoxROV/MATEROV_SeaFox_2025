#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QComboBox, QSplitter
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.image_data = [None, None, None]  # store latest frames for left, right, and realsense

        self.topic_names = [
            'camera_left/image_raw',
            'camera_right/image_raw',
            'camera_realsense/image_raw'
        ]
        # Create subscribers for the three camera topics
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
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_data[index] = cv_image
        except Exception as e:
            print()
            #self.get_logger().error(f"Failed to convert image: {e}")

class CameraGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node  # ROS2 subscriber node

        # Create a publisher to send the selected camera indexes to the publisher node
        self.selection_pub = self.node.create_publisher(Int32MultiArray, 'camera_selection', 10)
        self.pixel_pos = self.node.create_publisher(Int32MultiArray, 'pixel_position', 10)

        self.setWindowTitle('ROV Camera Viewer')
        self.setFixedSize(1320, 960)

        # Dropdown selector for different pairs of cameras
        self.dropdown = QComboBox()
        self.dropdown.addItems([
            'Camera 1 and 2',
            'Camera 3 and 4'
        ])
        self.dropdown.setFixedSize(1320, 20)
        self.dropdown.currentIndexChanged.connect(self.change_camera)

        # Labels for displaying cameras
        self.label_realsense = QLabel()
        self.label_realsense.setFixedSize(640, 480)
        self.label_left = QLabel()
        self.label_left.setFixedSize(640, 480)
        self.label_right = QLabel()
        self.label_right.setFixedSize(640, 480)

        # Layout
        layout = QVBoxLayout()
        dropsplitter = QSplitter(Qt.Horizontal)
        dropsplitter.addWidget(self.dropdown)
        layout.addWidget(dropsplitter)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.label_left)
        splitter.addWidget(self.label_right)
        layout.addWidget(splitter)
        layout.addWidget(self.label_realsense)
        self.setLayout(layout)

        # Default camera pair (using 0-indexed camera numbers corresponding to physical devices)
        # For example: selecting "Camera 1 and 2" corresponds to indexes [0, 1]
        self.current_camera_pair = [0, 1]

        # Publish the default camera pair
        self.send_camera_selection(self.current_camera_pair)

        # Timer to update the GUI with new images
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(33)  # ~30 FPS

    def change_camera(self, index):
        # Mapping selection index to a pair of camera indexes (0 to 3)
        # Modify these mappings as needed.
        if index == 0:
            self.current_camera_pair = [0, 1]
        elif index == 1:
            self.current_camera_pair = [2, 3]

        self.send_camera_selection(self.current_camera_pair)

    def send_camera_selection(self, pair):
        # Create and publish an Int32MultiArray message containing the two camera indexes.
        msg = Int32MultiArray()
        msg.data = pair
        self.selection_pub.publish(msg)
        print(f"Published new camera selection: {pair}")

    def getPos(self , event):
        x = event.pos().x()
        y = event.pos().y() 
        msg = Int32MultiArray()
        msg.data = [x,y]
        self.pixel_pos.publish(msg)


    def update_image(self):
        # Update RealSense image
        frame_realsense = self.node.image_data[2]
        if frame_realsense is not None:
            height, width, channel = frame_realsense.shape
            bytes_per_line = 3 * width
            q_img = QImage(frame_realsense.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img).scaled(self.label_realsense.width(), self.label_realsense.height())
            self.label_realsense.setPixmap(pixmap)
            self.label_realsense.mousePressEvent = self.getPos
        else:
            self.label_realsense.setText("No signal from RealSense")

        # Update left image
        frame_left = self.node.image_data[0]
        if frame_left is not None:
            height, width, channel = frame_left.shape
            bytes_per_line = 3 * width
            q_img = QImage(frame_left.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img).scaled(self.label_left.width(), self.label_left.height())
            self.label_left.setPixmap(pixmap)
        else:
            self.label_left.setText("No signal from left camera")

        # Update right image
        frame_right = self.node.image_data[1]
        if frame_right is not None:
            height, width, channel = frame_right.shape
            bytes_per_line = 3 * width
            q_img = QImage(frame_right.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img).scaled(self.label_right.width(), self.label_right.height())
            self.label_right.setPixmap(pixmap)
        else:
            self.label_right.setText("No signal from right camera")

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    # Create the ROS2 subscriber node
    camera_node = CameraSubscriber()

    # Create the GUI and pass the ROS node to it
    gui = CameraGUI(camera_node)
    gui.showMaximized()
    gui.show()

    # Timer to process ROS callbacks
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(camera_node, timeout_sec=0.01))
    spin_timer.start(10)

    exit_code = app.exec_()

    camera_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
