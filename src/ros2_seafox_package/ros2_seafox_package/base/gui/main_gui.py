import sys
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QComboBox, QSplitter
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt

import rclpy
from rclpy.node import Node
from camera_gui import CameraGUI

class UltimateSubscriber(Node):
    def __init__(self):
        super().__init__('ultimate_subscriber')


class MainFui(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node  # ROS2 subscriber node

        # Create a publisher to send the selected camera indexes to the publisher node
        self.selection_pub = self.node.create_publisher(Int32MultiArray, 'camera_selection', 10)

        self.setWindowTitle('ROV Camera Viewer')
        self.setFixedSize(1320, 960)

        # Dropdown selector dropdownabel()
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

if __name__ == '__main__':
    main()