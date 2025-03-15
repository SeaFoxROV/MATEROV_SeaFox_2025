#!/usr/bin/env python3
# main_gui.py
import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

import rclpy
from .camera_publisher import CameraPublisher
from .gui_cameras import CameraGUI

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    # Instantiate the ROS subscriber node and the GUI
    camera_subscriber = CameraPublisher()
    gui = CameraGUI(camera_subscriber)
    gui.show()

    # QTimer to periodically process ROS callbacks
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(camera_subscriber, timeout_sec=0.01))
    spin_timer.start(10)

    exit_code = app.exec_()

    # Clean up ROS before exiting
    camera_subscriber.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
