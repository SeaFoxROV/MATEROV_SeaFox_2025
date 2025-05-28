#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QPushButton, QLabel, QWidget, QVBoxLayout, QComboBox, QSplitter
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt
import signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Empty
from cv_bridge import CvBridge

#component for camera gui

class CamerasSubscriber(Node):
    def __init__(self):
        super().__init__("support_cameras")

        self.bridge = CvBridge()


        self.frames = {
            'frontal': None,
            'apoyo_1': None,
            'apoyo_2': None
        }
        self.image_data = [None] * len(self.frames)

        self.subscribers = []
        for i, topic in enumerate(self.frames):
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, idx=i: self.callback(msg, idx),
                10
            )
            self.subscribers.append(sub)


        self.get_logger().info("Cameras node has started!")


    def callback(self, msg, index):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_data[index] = cv_image
            self.get_logger().info("Cameras!")

        except Exception as e:
            print(f"Error converting image: {e}")

class Camaras(QWidget):
    def __init__(self, node, width = 680):
        super().__init__()
        self.node = node  # Nodo ROS2 suscriptor

        self.label_left = QLabel()
        self.label_left.setFixedSize(width, (width*3)//4)
        self.label_middle = QLabel()
        self.label_middle.setFixedSize(width, (width*3)//4)
        self.label_right = QLabel()
        self.label_right.setFixedSize(width, (width*3)//4)

        # label of the cameras
        layout = QVBoxLayout(self)
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.label_left)
        splitter.addWidget(self.label_middle)
        splitter.addWidget(self.label_right)
        layout.addWidget(splitter)

        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(33)


    def update_image(self):

        real_frame = self.node.image_data[2]
        if real_frame is not None:
            height, width, channel = real_frame.shape
            bytes_per_line = 3 * width
            q_img = QImage(real_frame.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img).scaled(self.label_middle.width(), self.label_middle.height())
            self.label_middle.setPixmap(pixmap)
     
            # self.label_middle.mousePressEvent = self.getPos
        else:
            self.label_middle.setText("No signal from label_middle")


        # Actualiza la imagen de la cámara izquierda
        frame_left = self.node.image_data[0]
        if frame_left is not None:
            height, width, channel = frame_left.shape
            bytes_per_line = 3 * width
            q_img = QImage(frame_left.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img).scaled(self.label_left.width(), self.label_left.height())
            self.label_left.setPixmap(pixmap)
        else:
            self.label_left.setText("No signal from left camera")

        # Actualiza la imagen de la cámara derecha
        frame_right = self.node.image_data[1]
        if frame_right is not None:
            height, width, channel = frame_right.shape
            bytes_per_line = 3 * width
            q_img = QImage(frame_right.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img).scaled(self.label_right.width(), self.label_right.height())
            self.label_right.setPixmap(pixmap)
        else:
            self.label_right.setText("No signal from right camera")
def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QApplication.quit()

if __name__ == "__main__":
    rclpy.init(args=None)
    signal.signal(signal.SIGINT, sigint_handler)


    app = QApplication(sys.argv)

    # Crear el nodo ROS2 suscriptor
    camera_node = CamerasSubscriber()

    # Crear la GUI y pasarle el nodo ROS2
    gui = Camaras(camera_node)
    gui.showMaximized()
    gui.show()

    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(camera_node, timeout_sec=0.01))
    spin_timer.start(10)

    exit_code = app.exec_()

    camera_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)