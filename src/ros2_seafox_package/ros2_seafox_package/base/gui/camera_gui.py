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

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.image_data = [None, None, None]

        self.topic_names = [
            'camera_left/image_raw',
            'camera_right/image_raw',
            'camera_realsense/image_raw'
        ]
        self.yolo_topic_names = [
            'yolocamera_left/image_raw',
            'yolocamera_right/image_raw',
            'camera_yolo/image_raw'
        ]
        
        # Create publisher for resetting cameras
        self.reset_pub = self.create_publisher(Empty, 'reset_cameras', 10)

        # Subscribe to regular camera topics
        self.subscribers = []
        for i, topic in enumerate(self.topic_names):
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, idx=i: self.callback(msg, idx),
                10
            )
            self.subscribers.append(sub)

        # Subscribers for YOLO topics
        self.yolosubscribers = []
        for i, topic in enumerate(self.yolo_topic_names):
            yolo_sub = self.create_subscription(
                Image,
                topic,
                lambda msg, idx=i: self.callback(msg, idx),
                10
            )
            self.yolosubscribers.append(yolo_sub)

    def callback(self, msg, index):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_data[index] = cv_image
        except Exception as e:
            print(f"Error converting image: {e}")

class CameraGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node  # Nodo ROS2 suscriptor

        self.setWindowTitle('ROV Camera Viewer')
        self.setFixedSize(1320, 960)

        # Publicador para la selección de cámaras (se envían pares de índices)
        self.camera_selection_pub = self.node.create_publisher(Int32MultiArray, 'camera_selection', 10)
        # Publicador para el modo (0: normal, 1: YOLO)
        self.mode_pub = self.node.create_publisher(Int32MultiArray, 'mode_selection', 10)
        # Publicador para la posición de píxeles (ya implementado)
        self.pixel_pos = self.node.create_publisher(Int32MultiArray, 'pixel_position', 10)

        # Dropdown selector para pares de cámaras
        self.dropdown = QComboBox()
        self.dropdown.addItems([
            'Camera 1 and 2',
            'Camera 3 and 4'
        ])
        self.dropdown.setFixedSize(1320, 20)
        self.dropdown.currentIndexChanged.connect(self.change_camera)

        # Labels para mostrar las imágenes
        self.label_realsense = QLabel()
        self.label_realsense.setFixedSize(640, 480)
        self.label_left = QLabel()
        self.label_left.setFixedSize(640, 480)
        self.label_right = QLabel()
        self.label_right.setFixedSize(640, 480)

        # Botón para activar/desactivar YOLO
        self.yolobtn = QPushButton("Activar YOLO", self)
        self.yolobtn.setCheckable(True)
        self.yolobtn.clicked.connect(self.toggle_view)
        self.yolobtn.setFixedSize(150, 100)

        # Botón para resetear las cámaras
        self.resetbtn = QPushButton("Reset Cameras", self)
        self.resetbtn.clicked.connect(self.reset_cameras)
        self.resetbtn.setCheckable(False)  # Normal push button behavior
        self.resetbtn.setFixedSize(150, 100)


        # Layout
        layout = QVBoxLayout()
        dropsplitter = QSplitter(Qt.Horizontal)
        dropsplitter.addWidget(self.dropdown)
        layout.addWidget(dropsplitter)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.label_left)
        splitter.addWidget(self.label_right)
        layout.addWidget(splitter)


        splitter2 = QSplitter(Qt.Horizontal)
        splitter2.addWidget(self.label_realsense)
        splitter2.addWidget(self.yolobtn)
        splitter2.addWidget(self.resetbtn)   # New reset button added here
        layout.addWidget(splitter2)
        self.setLayout(layout)

        # Par de cámaras por defecto (índices 0 y 1)
        self.current_camera_pair = [0, 1]
        self.send_camera_selection(self.current_camera_pair)

        # Publica el modo por defecto (0: normal)
        self.send_mode_selection(0)

        # Timer para actualizar la GUI (~30 FPS)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(33)

    def reset_cameras(self):
        # Publish an empty message using the publisher defined in the node.
        msg = Empty()
        self.node.reset_pub.publish(msg)
        print("Published reset cameras command.")


    def change_camera(self, index):
        # Mapea el índice del dropdown a un par de cámaras (ajusta según tus dispositivos)
        if index == 0:
            self.current_camera_pair = [0, 1]
        elif index == 1:
            self.current_camera_pair = [2, 3]
        self.send_camera_selection(self.current_camera_pair)

    def toggle_view(self):
        # Cuando se pulsa el botón, se actualiza el texto y se publica el modo en el tópico mode_selection
        if self.yolobtn.isChecked():
            self.yolobtn.setText("Desactivar YOLO")
            self.send_mode_selection(1)  # 1 para YOLO
            self.update_yolo_images(True)
        else:
            self.yolobtn.setText("Activar YOLO")
            self.send_mode_selection(0)  # 0 para imágenes normales
            self.update_yolo_images(False)

    def send_camera_selection(self, pair):
        msg = Int32MultiArray()
        msg.data = pair
        self.camera_selection_pub.publish(msg)
        print(f"Published new camera selection: {pair}")

    def send_mode_selection(self, mode):
        msg = Int32MultiArray()
        msg.data = [mode]
        self.mode_pub.publish(msg)
        print(f"Published new mode selection: {mode}")

    def getPos(self, event):
        x = event.pos().x()
        y = event.pos().y() 
        msg = Int32MultiArray()
        msg.data = [x, y]
        self.pixel_pos.publish(msg)

    def update_yolo_images(self, enable_yolo):
        # Actualiza las imágenes de las cámaras izquierda y derecha según si se activa YOLO o no
        if enable_yolo:
            self.label_left.setPixmap(self.get_pixmap(self.node.image_data[0], True))
            self.label_right.setPixmap(self.get_pixmap(self.node.image_data[1], True))
        else:
            self.label_left.setPixmap(self.get_pixmap(self.node.image_data[0], False))
            self.label_right.setPixmap(self.get_pixmap(self.node.image_data[1], False))

    def get_pixmap(self, frame, is_yolo):
        if frame is not None:
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            q_img = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            return QPixmap.fromImage(q_img).scaled(640, 480)
        else:
            return QPixmap()

    def update_image(self):
        # Actualiza la imagen de RealSense
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

###
def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QApplication.quit()

def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, sigint_handler)

    app = QApplication(sys.argv)

    # Crear el nodo ROS2 suscriptor
    camera_node = CameraSubscriber()

    # Crear la GUI y pasarle el nodo ROS2
    gui = CameraGUI(camera_node)
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
