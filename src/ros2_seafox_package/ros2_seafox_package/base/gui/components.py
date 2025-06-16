

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
# import pyrealsense2 as rs
from std_msgs.msg import Int8MultiArray, Int32MultiArray, Bool
# ----------------------------------------
# Widget principal que muestra 3 cámaras (más grandes)
# ----------------------------------------
class Camaras(QWidget):
    def __init__(self, node, width=400, parent=None):  # duplicado de tamaño de 200 a 400
        super().__init__(parent)
        self.node = node
        self.permission_video = [1, 1, 1]
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

    def close_image(self, camera_index):
        if camera_index == 0:
            if self.permission_video[0] == 1:
                # self.label_left.clear()
                self.permission_video[0] = 0
            else:
                self.permission_video[0] = 1
            self.publish_video_permission()
        elif camera_index == 1:
            if self.permission_video[1] == 1:
                # self.label_middle.clear()
                self.permission_video[1] = 0
            else:
                self.permission_video[1] = 1
            self.publish_video_permission()
        elif camera_index == 2:
            if self.permission_video[2] == 1:
                # self.label_right.clear()
                self.permission_video[2] = 0
            else:
                self.permission_video[2] = 1
            self.publish_video_permission()
        elif camera_index == 3:
            if self.permission_video[3] == 1:
                # self.real.close_video()
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
        # self.video_label.setFixedSize(640, 480)

        self.video_label.setFixedSize(1200, 960)
        self.video_label.setScaledContents(True)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding
        )
        layout.addWidget(self.video_label)
        self.video_label.setStyleSheet("background-color:#222; border:1px solid #555;")
        layout.addWidget(self.video_label, alignment=Qt.AlignCenter)

        self.permission = True

    def close_video(self):
        if self.permission == True:
            self.permission = False
        else:
            self.permission = True
        self.video_label.clear()

    def getPos(self, event):
        x = event.pos().x()
        y = event.pos().y()
        pos = [x, y]
        self.node.get_logger().info(f"Posición del pixel: x={x}, y={y}")
        msg = Int32MultiArray()
        msg.data = pos
        self.node.pos.publish(msg) 

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
        self.measure_node = False
        self.setWindowTitle("Measurement Tool")
        self.setFixedSize(1400, 1200)

        layout = QVBoxLayout(self)
        self.video_label = QLabel()
        self.video_label.setFixedSize(1200, 800)
        self.video_label.setScaledContents(True)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding
        )
        layout.addWidget(self.video_label)

        btn_layout = QHBoxLayout()
        self.close_btn   = QPushButton("Cerrar")
        self.close_btn.clicked.connect(self.close)
        btn_layout.addWidget(self.close_btn)
        layout.addLayout(btn_layout)

        self.measure_label = QLabel(f"Distance measured: {self.node.measure} m")
        layout.addWidget(self.measure_label)


    def showEvent(self, event):
        super().showEvent(event)
        self.on_open()

    def getPos(self, event):
        x = event.pos().x()
        y = event.pos().y()
        pos = [x, y]
        self.node.get_logger().info(f"Posición del pixel: x={x}, y={y}")
        msg = Int32MultiArray()
        msg.data = pos
        self.node.pos.publish(msg) 

    def on_open(self):
        self.measure_node = True
        msg = Bool()
        msg.data = True
        self.node.measure_node.publish(msg)
        self.node.get_logger().info("Mensaje publicado")
    
    def closeEvent(self, event):
        self.measure_node = False
        msg = Bool()
        msg.data = False
        self.node.measure_node.publish(msg)
        self.node.get_logger().info("Mensaje publicado")


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
    def __init__(self, node, realsense_measure, parent=None):
        super().__init__(parent)
        self.node = node
        self.realsense_measure = realsense_measure
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
        self.realsense_measure.exec_()
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

        # Widgets
        self.realsense_widget = RealsenseViewerWidget(node)
        self.camaras_widget = Camaras(node, 400)
        self.realsense_measure = MeasurePopup(node)

        central = QWidget()
        main_v = QVBoxLayout(central)

        # Fila superior: botones - RealSense viewer - tabla
        top_h = QHBoxLayout()
        top_h.addWidget(FeatureButtonsWidget(node, self.realsense_measure), 1)
        top_h.addWidget(self.realsense_widget, 3)
        top_h.addWidget(StatusTableWidget(node), 1)
        main_v.addLayout(top_h)

        # Fila inferior: cámaras pequeñas
        main_v.addWidget(self.camaras_widget)

        self.setCentralWidget(central)

        # A single qtimer for the entire GUI
        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self.refresh_gui)
        self.gui_timer.start(33)

    def refresh_gui(self):
        if not self.realsense_measure.measure_node:
            # Update image of the RealSense
            # frame_rs = self.node.image_data[3]  # RealSense fram by index
            frame_rs = self.node.realsense  # RealSense fram

            if frame_rs is not None:
                frame_rs = cv2.cvtColor(frame_rs, cv2.COLOR_BGR2RGB)
                h, w, _ = frame_rs.shape
                img = QImage(frame_rs.data, w, h, 3*w, QImage.Format_RGB888)
                pix = QPixmap.fromImage(img)
                pix = pix.scaled(1200, 960, Qt.KeepAspectRatio)
                self.realsense_widget.video_label.setPixmap(pix)
                self.realsense_widget.video_label.mousePressEvent = self.realsense_measure.getPos
            else:
                self.realsense_widget.video_label.setText(":,v")

            # Update the normal cameras
            label = (self.camaras_widget.label_left, self.camaras_widget.label_middle, self.camaras_widget.label_right)

            for i, label in enumerate(label):
                frame = self.node.image_data[i]
                if frame is not None and self.camaras_widget.permission_video[i] == 1:
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w, _ = rgb.shape
                    img = QImage(frame.data, w, h, 3*w, QImage.Format_RGB888).rgbSwapped()
                    label.setPixmap(QPixmap.fromImage(img).scaled(
                        label.width(), label.height(), Qt.KeepAspectRatio))
                else:
                    label.setText("No signal")
        else:
            frame_rs = self.node.realsense  # RealSense fram
            self.realsense_measure.measure_label.setText(f"Distance measured: {self.node.measure} m")

            if frame_rs is not None:
                frame_rs = cv2.cvtColor(frame_rs, cv2.COLOR_BGR2RGB)
                h, w, _ = frame_rs.shape
                img = QImage(frame_rs.data, w, h, 3*w, QImage.Format_RGB888)
                pix = QPixmap.fromImage(img)

                # Escalamos el pixmap a 800×600 manteniendo proporciones:
                pix = pix.scaled(1200, 960, Qt.KeepAspectRatio)

                self.realsense_measure.video_label.setPixmap(pix)
                self.realsense_measure.video_label.mousePressEvent = self.realsense_measure.getPos
            else:
                self.realsense_measure.video_label.setText(":v")


        


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
