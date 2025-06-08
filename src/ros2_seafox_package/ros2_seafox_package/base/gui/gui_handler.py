import sys
from PyQt5.QtWidgets import QApplication, QPushButton, QLabel, QWidget, QVBoxLayout, QComboBox, QSplitter
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt
import signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Empty, Int8MultiArray, Bool
from cv_bridge import CvBridge
import threading

from ros2_seafox_package.base.gui.components import MainWindow

#component for camera gui

class GUI_Node(Node):
    """
    Nodo para suscribirse a todo lo que necesite hacer display en la GUI, imagenes y sensores
    """
    def __init__(self):
        super().__init__("GUI_node")

        self.bridge = CvBridge()


        self.frames = {
            'frontal': None,
            'apoyo_1': None,
            'apoyo_2': None,
            'realsense': None,
        }
        self.image_data = [None] * len(self.frames)

        self.subscribers = []
        self.realsense = None
        #subscribers de camaras
        for i, topic in enumerate(self.frames):
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, idx=i: self.camara_callback(msg, idx),
                10
            )
            self.subscribers.append(sub)

        #Subscriber de video de realsesene
        self.create_subscription(
            Image,
            'camera_realsense/image_raw',
            self.realsense_callback,
            10
        )

        #Publisher de permisos de camaras
        self.permission_video = self.create_publisher(
        Int8MultiArray, 'video_permission', 10
        )

        #Publisher of coordinates of the pixel position
        self.pos = self.create_publisher(
            Int32MultiArray,
            'pixel_position',
            10
        )

        #Publisher to kill nodes
        self.measure_node = self.create_publisher(
            Bool,
            'measure_node',
            10
        )

        self.get_logger().info("GUI node has started!")


    def camara_callback(self, msg, index):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # self.get_logger().info(f"Received image from camera {index}")
            self.image_data[index] = cv_image

        except Exception as e:
            print(f"Error converting image: {e}")
    
    def realsense_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # self.get_logger().info("Received image from realsense")
            self.realsense = cv_img
        except Exception as e:
            self.get_logger().error(f"Error CvBridge: {e}")

def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QApplication.quit()



def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, sigint_handler)
    
    app = QApplication(sys.argv)


    # Crear el nodo ROS2 suscriptor
    camera_node = GUI_Node()
    
    # Timer para procesar los callbacks de ROS
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(camera_node, timeout_sec=0.001))
    spin_timer.start(1)

    # Timer para ejecutar los callbacks de ROS en un thread diferente
    # spin_timer = threading.Thread(target=rclpy.spin, args=(camera_node,), daemon=True)
    # spin_timer.start()
    
    # Crear la GUI y pasarle el nodo ROS2
    gui = MainWindow(camera_node, None)
    gui.showMaximized()
    gui.show()

    exit_code = app.exec_()

    camera_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()