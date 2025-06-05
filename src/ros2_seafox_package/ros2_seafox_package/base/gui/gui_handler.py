import sys
from PyQt5.QtWidgets import QApplication, QPushButton, QLabel, QWidget, QVBoxLayout, QComboBox, QSplitter
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt
import signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Empty, Int8MultiArray
from cv_bridge import CvBridge

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
            'depth': None,
        }
        self.image_data = [None] * len(self.frames)

        self.subscribers = []

        #subscribers de camaras
        for i, topic in enumerate(self.frames):
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, idx=i: self.camara_callback(msg, idx),
                10
            )
            self.subscribers.append(sub)


        self.get_logger().info("GUI node has started!")


    def camara_callback(self, msg, index):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_data[index] = cv_image

        except Exception as e:
            print(f"Error converting image: {e}")

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

    #Publisher de permisos de camaras
    permission_video = camera_node.create_publisher(
        Int8MultiArray, 'video_permission', 10
        )
    
    # Crear la GUI y pasarle el nodo ROS2
    gui = MainWindow(camera_node, permission_video, None)
    gui.showMaximized()
    gui.show()

    # Timer para procesar los callbacks de ROS
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(camera_node, timeout_sec=0.01))
    spin_timer.start(10)

    exit_code = app.exec_()

    camera_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()