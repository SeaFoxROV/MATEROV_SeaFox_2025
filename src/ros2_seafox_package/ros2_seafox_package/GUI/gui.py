import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QPushButton, QHBoxLayout
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer


class WebcamSubscriber(Node):
    def _init_(self, label, topic='webcam_image_gray', node_name="webcam_subscriber"):
        super()._init_(node_name)
        for i in range(4):
            self.subscription[i] = self.create_subscription(
                Image,
                'camera'+str(i)+'/image_raw',
                self.listener_callback,
                10)
        self.bridge = CvBridge()
        self.label = label

    def listener_callback(self, msg):
        try:
            # Convierte el mensaje ROS2 Image a un arreglo de OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, channels = cv_image.shape
            bytes_per_line = channels * width
            # Convierte el arreglo OpenCV a una imagen QImage
            qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format.Format_BGR888)
            pixmap = QPixmap.fromImage(qt_image)
            self.label.setPixmap(pixmap)
        except Exception as e:
            self.get_logger().error(f"Error al procesar el mensaje: {e}")
nodo= WebcamSubscriber(QLabel)


class VideoWindow(QWidget):
    def _init_(self):
        super()._init_()
        self.setWindowTitle("Live Stream Subscriber")
        self.layout = QVBoxLayout()

        # Etiqueta para mostrar la imagen
        self.label = QLabel("Esperando video...")
        self.label.setScaledContents(True)
        self.layout.addWidget(self.label)

        # Botones para cambiar el tópico
        self.button_layout = QHBoxLayout()
        self.color_button = QPushButton("Ver a Color")
        self.gray_button = QPushButton("Ver en Escala de Grises")
        self.button_layout.addWidget(self.color_button)
        self.button_layout.addWidget(self.gray_button)
        self.layout.addLayout(self.button_layout)

        self.setLayout(self.layout)

        # Nodo inicial con el tópico en escala de grises
        self.current_topic = 'webcam_image_gray'
        self.subscriber_node = WebcamSubscriber(self.label, topic=self.current_topic)

        # Conectar los botones a sus funciones
        self.color_button.clicked.connect(self.switch_to_color)
        self.gray_button.clicked.connect(self.switch_to_gray)