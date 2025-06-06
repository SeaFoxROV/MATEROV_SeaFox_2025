

#!/usr/bin/env python3
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QSplitter, QTableWidget, QTableWidgetItem,
    QDialog, QSizePolicy
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt
import sys
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge
from ros2_seafox_package.base.gui.imageroll import ImagePopup
import pyrealsense2 as rs
from std_msgs.msg import Int8MultiArray, Int32MultiArray
# ----------------------------------------
# Widget principal que muestra 3 cámaras (más grandes)
# ----------------------------------------
class Camaras(QWidget):
    def __init__(self, node, width=400, parent=None):  # duplicado de tamaño de 200 a 400
        super().__init__(parent)
        self.node = node
        self.permission_video = [1, 1, 1, 1]
        self.real = RealsenseViewerWidget(self.node)
        
        # Tres labels para frontal, apoyo1, apoyo2
        self.label_left   = QLabel(); self.label_left.setFixedSize(width, (width*3)//4)
        self.label_middle = QLabel(); self.label_middle.setFixedSize(width, (width*3)//4)
        self.label_right  = QLabel(); self.label_right.setFixedSize(width, (width*3)//4)

        # Crosses to cancel the image
        self.cancel_left = QPushButton("X")
        self.cancel_left.clicked.connect(lambda: self.close_image(0))
        self.cancel_center = QPushButton("X")
        self.cancel_center.clicked.connect(lambda: self.close_image(1))
        self.cancel_right = QPushButton("X")
        self.cancel_right.clicked.connect(lambda: self.close_image(2))
        self.cancel_realsense = QPushButton("X")
        self.cancel_realsense.clicked.connect(lambda: self.close_image(3))
        for button in (self.cancel_left, self.cancel_right, self.cancel_center, self.cancel_realsense):
            button.setFixedSize(30, 30)
            button.setStyleSheet("background-color: red; color: white; border-radius: 15px;")
            button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        layout = QHBoxLayout(self)
        layout.addStretch()
        layout.addWidget(self.label_left)
        layout.addWidget(self.label_middle)
        layout.addWidget(self.label_right)
        layout.addWidget(self.cancel_left, alignment=Qt.AlignTop)
        layout.addWidget(self.cancel_center, alignment=Qt.AlignTop)
        layout.addWidget(self.cancel_right, alignment=Qt.AlignTop)
        layout.addWidget(self.cancel_realsense, alignment=Qt.AlignTop)
        layout.addStretch()
        self.setLayout(layout)

        # Temporizador para refrescar imágenes
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_image)
        self.timer.start(33)

    def update_image(self):
        for i, (label, frame) in enumerate(zip(
            (self.label_left, self.label_middle, self.label_right),
            (self.node.image_data[0], self.node.image_data[1], self.node.image_data[2])
        )):
            if frame is not None and self.permission_video[i]== True:
                h, w, _ = frame.shape
                img = QImage(frame.data, w, h, 3*w, QImage.Format_RGB888).rgbSwapped()
                label.setPixmap(QPixmap.fromImage(img).scaled(
                    label.width(), label.height(), Qt.KeepAspectRatio))
            else:
                label.setText("No signal")

    def close_image(self, camera_index):
        if camera_index == 0:
            self.label_left.clear()
            if self.permission_video[0] == 1:
                self.permission_video[0] = 0
            else:
                self.permission_video[0] = 1
            self.publish_video_permission()
        elif camera_index == 1:
            self.label_middle.clear()
            if self.permission_video[1] == 1:
                self.permission_video[1] = 0
            else:
                self.permission_video[1] = 1
            self.publish_video_permission()
        elif camera_index == 2:
            self.label_right.clear()
            if self.permission_video[2] == 1:
                self.permission_video[2] = 0
            else:
                self.permission_video[2] = 1
            self.publish_video_permission()
        elif camera_index == 3:
            self.real.close_video()
            if self.permission_video[3] == 1:
                self.permission_video[3] = 0
            else:
                self.permission_video[3] = 1
            self.publish_video_permission()
        else:
            raise ValueError("Índice de cámara no válido")
        
    def publish_video_permission(self):
        msg = Int8MultiArray()
        msg.data = [int(val) for val in self.permission_video] # Turn to int
        self.node.permission_video.publish(msg)

# ----------------------------------------
# Viewer grande para RealSense
# ----------------------------------------
class RealsenseViewerWidget(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        layout = QVBoxLayout(self)
        self.video_label = QLabel()
        # Tamaño más grande que las cámaras pequeñas
        self.video_label.setFixedSize(640, 480)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color:#222; border:1px solid #555;")
        layout.addWidget(self.video_label, alignment=Qt.AlignCenter)

        # Timer para refrescar RealSense
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_image)
        self.timer.start(33)

        self.permission = True

    def update_image(self):
        frame = self.node.realsense  # RealSense fram
        if frame is not None and self.permission:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, _ = frame.shape
            img = QImage(frame.data, w, h, 3*w, QImage.Format_RGB888)
            pix = QPixmap.fromImage(img)
            self.video_label.setPixmap(pix)
            self.video_label.mousePressEvent = self.getPos
        else:
            self.video_label.setText("No signal")

    def getPos(self, event):
        x = event.pos().x()
        y = event.pos().y()
        pos = [x, y]
        self.node.get_logger().info(f"Posición del pixel: x={x}, y={y}")
        # Publicar la posición del pixel
        msg = Int32MultiArray()
        msg.data = pos
        self.node.pos.publish(msg) 
    def close_video(self):
        if self.permission == True:
            self.permission = False
        else:
            self.permission = True
        self.video_label.clear()

import sys 

import rclpy

from rclpy.node import Node 

from std_msgs.msg import Float32MultiArray,Int16MultiArray

from scipy.optimize import curve_fit

from os import path

PATH = path.dirname(__file__)

class newton_to_pwm(Node):
    """
    Class that implements the kinematics.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__("thrust")
        
        self.pwm_fit_params = newton_to_pwm.generate_pwm_fit_params()

        self.subscription = self.create_subscription(Float32MultiArray, "motor_values", self.pwm_callback,10)
            
        self.create_subscription(Float32MultiArray, '/joystick_data', self.joystick_callback, 100)

        self.pwm_pub = self.create_publisher(Int16MultiArray, "pwm_values", 10)

        self.joystick_data = None

    @staticmethod
    def newtons_to_pwm(x: float, a: float, b: float, c: float, d: float, e: float, f: float) -> float:
        """
        Converts desired newtons into its corresponding PWM value

        Args:
            x: The force in newtons desired
            a-f: Arbitrary parameters to map newtons to pwm, see __generate_curve_fit_params()

        Returns:
            PWM value corresponding to the desired thrust
        """
        return (a * x**5) + (b * x**4) + (c * x**3) + (d * x**2) + (e * x) + f

    @staticmethod
    def generate_pwm_fit_params():
        x = []
        y = []

        with open(PATH + "/data/newtons_to_pwm.tsv", "r") as file:
            for data_point in file:
                data = data_point.split("\t")
                x.append(data[0])
                y.append(data[1])

        optimal_params, param_covariance = curve_fit(newton_to_pwm.newtons_to_pwm, x, y)
        return optimal_params
    
    def joystick_callback(self, msg):
        """Callback para guardar los datos del joystick"""
        self.joystick_data = msg.data
    
    def pwm_callback(self, motor_values):
        # Creamos un nuevo mensaje para publicar PWM
        pwm_msg = Int16MultiArray()
        pwm_msg.data = [1500] * 6

        # Iteramos sobre los valores recibidos en motor_values.data
        for index, newton in enumerate(motor_values.data):
            pwm = int(newton_to_pwm.newtons_to_pwm(
                newton,
                self.pwm_fit_params[0],
                self.pwm_fit_params[1],
                self.pwm_fit_params[2],
                self.pwm_fit_params[3],
                self.pwm_fit_params[4],
                self.pwm_fit_params[5]
            ))
            # Limitar el rango de pwm
            up = 1750
            down = 1250
            
            pwm = up if pwm > up else down if pwm < down else pwm
        
            # Si el valor en newton es 0, lo asignamos a 1500
            if newton == 0:
                pwm = 1500
            pwm_msg.data[index] = pwm
        # if pwm_msg.data[3] > 1700 and pwm_msg.data[2] < 1200:

        if pwm_msg.data[2]>1600:
            pwm_msg.data[2] = 1850
        if pwm_msg.data[2]<1400:
            pwm_msg.data[2] = 1150
        if pwm_msg.data[3]>1600:
            pwm_msg.data[3] = 1850
        if pwm_msg.data[3]<1400:
            pwm_msg.data[3] = 1150

        if self.joystick_data is not None:
            if bool(self.joystick_data[8]):#derecha
                pwm_msg.data[0] = 1250 
                pwm_msg.data[1] = 1750 
                pwm_msg.data[4] = 1750 
                pwm_msg.data[5] = 1250 
            if bool(self.joystick_data[7]):#izquierda
                pwm_msg.data[0] = 1750 
                pwm_msg.data[1] = 1250 
                pwm_msg.data[4] = 1250 
                pwm_msg.data[5] = 1750 

        pwm_msg.data[3] += 15
        self.pwm_pub.publish(pwm_msg)

    
    def __del__(self):
        pwm_values = Int16MultiArray()
        pwm_values.data = [1500] * 6
        self.pwm_pub.publish(pwm_values)

def main(args=None):
    rclpy.init(args=args)
    node = newton_to_pwm()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        del node
        rclpy.shutdown()    


if __name__ == "__main__":
    main(sys.argv)
    #             distance = distance - 3.4
    #         elif 1.80 <= distance <= 1.99:
    #             distance = distance - 9
    #         elif 2 <= distance <= 2.10:
    #             distance = distance - 12.8

    #         # Convertir a mensaje Float32 y publicarlo
    #         msg = Float32()
    #         msg.data = float(distance)
    #         self.distance_publisher.publish(msg)
    #         self.get_logger().info(f"Distance between points: {distance:.2f} meters")

# ----------------------------------------
# Popups omitidos (mantener mismos de antes)
# ----------------------------------------
# ... ObjectDetectionPopup, MeasurePopup, PhotoSpherePopup definitions ...
# ------------------------------
# Popup de Object Detection
# ------------------------------
class ObjectDetectionPopup(QDialog):
    def __init__(self, node):
        super().__init__()
        self.yolo = False
        self.model = YOLO(r'/home/seafoxinventive/MATEROV_SeaFox_2025/src/ros2_seafox_package/ros2_seafox_package/rov/cameras/yolo_model/best.pt')

        self.node = node
        self.setWindowTitle("Object Detection (YOLO)")
        self.setFixedSize(500, 420)

        layout = QVBoxLayout(self)
        self.video_label = QLabel(); self.video_label.setFixedSize(480, 270)
        self.video_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.video_label)

        self.video_label_yolo = QLabel(); self.video_label_yolo.setFixedSize(480, 270)
        self.video_label_yolo.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.video_label_yolo)

        self.selector = QComboBox()
        self.selector.addItems(["Frontal", "Apoyo 1", "Apoyo 2", "RealSense"])
        layout.addWidget(self.selector)

        btn_layout = QHBoxLayout()
        self.yolo_btn  = QPushButton("YOLO")
        self.yolo_btn.clicked.connect(self.set_Yolo)
        self.close_btn = QPushButton("Cerrar")
        self.yolo_btn.setStyleSheet("font-size:18px; padding:10px;")
        self.close_btn.clicked.connect(self.close)
        btn_layout.addWidget(self.yolo_btn)
        btn_layout.addWidget(self.close_btn)
        layout.addLayout(btn_layout)

        self.timer = QTimer(self); self.timer.timeout.connect(self.update_image); self.timer.start(33)
    
    def set_Yolo(self):
        if self.yolo:
            self.yolo = False
        else:
            self.yolo = True
        self.bridge = CvBridge()                                                  

    def closeEvent(self, a0):
        self.yolo = False
        super().closeEvent(a0)

    def update_image(self):
        idx = self.selector.currentIndex()
        if idx < 3:
            frame = self.node.image_data[idx]
        else:
            frame = getattr(self.node, "realsense_frame", None)
        if self.yolo:
            self.video_label.setVisible(False)
            self.video_label_yolo.setVisible(True)
            try:
                results = self.model(frame)
                annotated_frame = results[0].plot()
                h, w, _ = annotated_frame.shape
                img = QImage(annotated_frame.data, w, h, 3 * w, QImage.Format_RGB888).rgbSwapped()
                self.video_label_yolo.setPixmap(QPixmap.fromImage(img).scaled(
                    self.video_label_yolo.width(), self.video_label_yolo.height(), Qt.KeepAspectRatio))
                # self.logger("procesamiento yolo")
            except Exception as e:
                print(f"Error in YOLO processing: {e}")
        elif frame is not None:
            self.video_label_yolo.setVisible(False)
            self.video_label.setVisible(True)
            h, w, _ = frame.shape
            img = QImage(frame.data, w, h, 3*w, QImage.Format_RGB888).rgbSwapped()
            self.video_label.setPixmap(QPixmap.fromImage(img).scaled(
                self.video_label.width(), self.video_label.height(), Qt.KeepAspectRatio))
            print("Normal")
        else:
            self.video_label.setText("No signal")


# ------------------------------
# Popup de Measure (solo RealSense)
# ------------------------------
class MeasurePopup(QDialog):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Measurement Tool")
        self.setFixedSize(500, 360)

        layout = QVBoxLayout(self)
        self.video_label = QLabel(); self.video_label.setFixedSize(480, 270)
        self.video_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.video_label)

        btn_layout = QHBoxLayout()
        self.measure_btn = QPushButton("Medir")
        self.close_btn   = QPushButton("Cerrar")
        self.measure_btn.setStyleSheet("font-size:18px; padding:10px;")
        self.close_btn.clicked.connect(self.close)
        btn_layout.addWidget(self.measure_btn)
        btn_layout.addWidget(self.close_btn)
        layout.addLayout(btn_layout)

        self.timer = QTimer(self); self.timer.timeout.connect(self.update_image); self.timer.start(33)

    def update_image(self):
        frame = getattr(self.node, "realsense_frame", None)
        if frame is not None:
            h, w, _ = frame.shape
            img = QImage(frame.data, w, h, 3*w, QImage.Format_RGB888).rgbSwapped()
            self.video_label.setPixmap(QPixmap.fromImage(img).scaled(
                self.video_label.width(), self.video_label.height(), Qt.KeepAspectRatio))
        else:
            self.video_label.setText("No signal")


# ------------------------------
# Popup de PhotoSphere
# ------------------------------
# ----------------------------------------
# Popup de PhotoSphere con captura y Stitch
# ----------------------------------------
from ros2_seafox_package.base.gui.img_stitcher import ImageStitcher

import cv2
class PhotoSpherePopup(QDialog):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("PhotoSphere")
        self.setFixedSize(520, 500)

        layout = QVBoxLayout(self)
        self.video_label = QLabel()
        self.video_label.setFixedSize(500, 280)
        self.video_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.video_label)

        # Selector de cámara
        self.selector = QComboBox()
        self.selector.addItems(["Frontal", "Apoyo 1", "Apoyo 2", "RealSense"])
        layout.addWidget(self.selector)

        # Botones: Capturar, Stitch, Cerrar
        btn_layout = QHBoxLayout()
        self.capture_btn = QPushButton("Capturar")
        self.stitch_btn = QPushButton("Stitch")
        self.close_btn   = QPushButton("Cerrar")
        self.capture_btn.setStyleSheet("font-size:14px; padding:8px;")
        self.stitch_btn.setStyleSheet("font-size:14px; padding:8px;")
        self.close_btn.clicked.connect(self.close)
        btn_layout.addWidget(self.capture_btn)
        btn_layout.addWidget(self.stitch_btn)
        btn_layout.addWidget(self.close_btn)
        layout.addLayout(btn_layout)

        # Conectar señales
        self.capture_btn.clicked.connect(self.capture_frame)
        self.stitch_btn.clicked.connect(self.image_roll)

        # Lista de cuadros capturados
        self.captured_frames = []

        # Timer para refrescar vista
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_image)
        self.timer.start(33)
        self.i = 0
    def image_roll(self):
        # Abre un pop-up para mostrar las imágenes capturadas
        if not self.captured_frames:
            print("No hay imágenes capturadas para mostrar.")
            return
        image_paths = self.captured_frames
        popup = ImagePopup(image_paths, self)
        popup.exec_()

    def update_image(self):
        idx = self.selector.currentIndex()
        if idx < 3:
            frame = self.node.image_data[idx]
        else:
            frame = getattr(self.node, "realsense_frame", None)
        if frame is not None:
            h, w, _ = frame.shape
            img = QImage(frame.data, w, h, 3*w, QImage.Format_RGB888).rgbSwapped()
            self.video_label.setPixmap(QPixmap.fromImage(img).scaled(
                self.video_label.width(), self.video_label.height(), Qt.KeepAspectRatio))
        else:
            self.video_label.setText("No signal")

    def capture_frame(self):
        # Captura el frame actual y lo añade a la lista
        idx = self.selector.currentIndex()
        if idx < 3:
            frame = self.node.image_data[idx]
        else:
            frame = getattr(self.node, "realsense_frame", None)
        if frame is not None:
            self.captured_frames.append(frame)
            cv2.imwrite(f"hgeswhj{self.i}", frame)
            self.i += 1
            print(f"Frame capturado de cámara {self.selector.currentText()} (total {len(self.captured_frames)})")

    def perform_stitch(self):
        # Realiza el stitch de las imágenes capturadas
        stitcher = ImageStitcher()
        result = stitcher.stitch_images(self.captured_frames)
        if result is not None:
            cv2.imshow('Fotoesfera Stitch', result)
            cv2.waitKey(0)
    


    def closeEvent(self, event):
        # Limpiar lista al cerrar
        self.captured_frames.clear()
        event.accept()

# ----------------------------------------
# Botonera con popups
# ----------------------------------------
class FeatureButtonsWidget(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        layout = QVBoxLayout(self)
        for text, slot in [
            ("Object Detection", self.open_object_detection_popup),
            ("Measure",          self.open_measure_popup),
            ("PhotoSphere",      self.open_photosphere_popup)
        ]:
            btn = QPushButton(text)
            btn.setStyleSheet("font-size:12px; padding:60px 10px; min-width:80px;")
            btn.clicked.connect(slot)
            btn.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
            layout.addWidget(btn)
        layout.addStretch()

    def open_object_detection_popup(self):
        ObjectDetectionPopup(self.node).exec_()
    def open_measure_popup(self):
        MeasurePopup(self.node).exec_()
    def open_photosphere_popup(self):
        PhotoSpherePopup(self.node).exec_()

# ----------------------------------------
# Tabla de estado
# ----------------------------------------
class StatusTableWidget(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        table = QTableWidget(3, 3)
        table.setMaximumWidth(230)
        table.setHorizontalHeaderLabels(["Item", "...", "Status"])
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        items = [("Comms","...", "OK"),("Cams","...", "OK"),("Joy","...", "OK")]
        for r, (k, m, v) in enumerate(items):
            table.setItem(r, 0, QTableWidgetItem(k))
            table.setItem(r, 1, QTableWidgetItem(m))
            table.setItem(r, 2, QTableWidgetItem(v))
        table.resizeColumnsToContents()
        layout.addWidget(table)

# ----------------------------------------
# Ventana Principal con nuevo layout
# ----------------------------------------
class MainWindow(QMainWindow):
    def __init__(self, node, permission_video, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Interfaz del ROV")
        self.setMinimumSize(1200, 800)
        self.node = node
        self.permission_video = permission_video
        self.permission_video = [1, 1, 1, 1, 1]

        central = QWidget()
        main_v = QVBoxLayout(central)

        # Fila superior: botones - RealSense viewer - tabla
        top_h = QHBoxLayout()
        top_h.addWidget(FeatureButtonsWidget(node), 1)
        top_h.addWidget(RealsenseViewerWidget(node), 3)
        top_h.addWidget(StatusTableWidget(node), 1)
        main_v.addLayout(top_h)

        # Fila inferior: cámaras pequeñas
        main_v.addWidget(Camaras(node, 400))

        self.setCentralWidget(central)


# ----------------------------------------
# Ejecución de prueba (sin ROS2)
# ----------------------------------------
if __name__ == "__main__":
    import numpy as np
    app = QApplication(sys.argv)
    class DummyNode:
        def __init__(self):
            self.image_data = [np.random.randint(0,255,(240,320,3),dtype=np.uint8) for _ in range(3)]
            self.realsense_frame = np.random.randint(0,255,(480,640,3),dtype=np.uint8)
    win = MainWindow(DummyNode())
    win.show()
    sys.exit(app.exec_())
