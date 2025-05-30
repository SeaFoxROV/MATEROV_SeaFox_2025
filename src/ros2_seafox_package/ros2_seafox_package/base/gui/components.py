

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

# ----------------------------------------
# Widget principal que muestra 3 cámaras (más grandes)
# ----------------------------------------
class Camaras(QWidget):
    def __init__(self, node, width=400):  # duplicado de tamaño de 200 a 400
        super().__init__()
        self.node = node

        # Tres labels para frontal, apoyo1, apoyo2
        self.label_left   = QLabel(); self.label_left.setFixedSize(width, (width*3)//4)
        self.label_middle = QLabel(); self.label_middle.setFixedSize(width, (width*3)//4)
        self.label_right  = QLabel(); self.label_right.setFixedSize(width, (width*3)//4)

        layout = QHBoxLayout(self)
        layout.addStretch()
        layout.addWidget(self.label_left)
        layout.addWidget(self.label_middle)
        layout.addWidget(self.label_right)
        layout.addStretch()
        self.setLayout(layout)

        # Temporizador para refrescar imágenes
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_image)
        self.timer.start(33)

    def update_image(self):
        for label, frame in zip(
            (self.label_left, self.label_middle, self.label_right),
            (self.node.image_data[0], self.node.image_data[1], self.node.image_data[2])
        ):
            if frame is not None:
                h, w, _ = frame.shape
                img = QImage(frame.data, w, h, 3*w, QImage.Format_RGB888).rgbSwapped()
                label.setPixmap(QPixmap.fromImage(img).scaled(
                    label.width(), label.height(), Qt.KeepAspectRatio))
            else:
                label.setText("No signal")

# ----------------------------------------
# Viewer grande para RealSense
# ----------------------------------------
class CameraViewerWidget(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
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

    def update_image(self):
        frame = getattr(self.node, "realsense_frame", None)
        if frame is not None:
            h, w, _ = frame.shape
            img = QImage(frame.data, w, h, 3*w, QImage.Format_RGB888).rgbSwapped()
            pix = QPixmap.fromImage(img).scaled(
                self.video_label.width(), self.video_label.height(), Qt.KeepAspectRatio)
            self.video_label.setPixmap(pix)
        else:
            self.video_label.setText("No signal")

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
        self.model = YOLO(r'src/ros2_seafox_package/ros2_seafox_package/rov/cameras/yolo_model/best.pt')

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
        # # Recorre cada cámara y procesa según el modo actual
        # for i, frame_msg in enumerate(self.video_label):
        #     if frame_msg is None:
        #         continue
        #     try:
        #         # Conversión de ROS2 a OpenCV
        #         cv_frame = self.bridge.imgmsg_to_cv2(frame_msg, desired_encoding='bgr8')
        #     except Exception as e:
        #         self.get_logger().error(f"Error converting image: {e}")
        #         continue

        #     # Modo YOLO: se procesa la imagen y se publica la imagen anotada
        #     results = self.model(cv_frame)
        #     annotated_frame = results[0].plot()
        #     try:
        #         img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        #         self.video_label_yolo = img_msg
        #     except Exception as e:
        #         self.get_logger().error(f"Error converting YOLO image back to ROS2: {e}")
        #         continue
        #     # self.yolo_publishers[i].publish(img_msg)
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
            try:
                results = self.model(frame)
                annotated_frame = results[0].plot()
                h, w, _ = annotated_frame.shape
                img = QImage(annotated_frame.data, w, h, 3 * w, QImage.Format_RGB888).rgbSwapped()
                self.video_label_yolo.setPixmap(QPixmap.fromImage(img).scaled(
                    self.video_label_yolo.width(), self.video_label_yolo.height(), Qt.KeepAspectRatio))
                print("Yolo")
            except Exception as e:
                print(f"Error in YOLO processing: {e}")
        elif frame is not None:
            self.video_label_yolo.setVisible(False)
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
class PhotoSpherePopup(QDialog):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("PhotoSphere")
        self.setFixedSize(500, 420)

        layout = QVBoxLayout(self)
        self.video_label = QLabel(); self.video_label.setFixedSize(480, 270)
        self.video_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.video_label)

        self.selector = QComboBox()
        self.selector.addItems(["Frontal", "Apoyo 1", "Apoyo 2", "RealSense"])
        layout.addWidget(self.selector)

        btn_layout = QHBoxLayout()
        self.capture_btn = QPushButton("Capturar")
        self.close_btn   = QPushButton("Cerrar")
        self.capture_btn.setStyleSheet("font-size:18px; padding:10px;")
        self.close_btn.clicked.connect(self.close)
        btn_layout.addWidget(self.capture_btn)
        btn_layout.addWidget(self.close_btn)
        layout.addLayout(btn_layout)

        self.timer = QTimer(self); self.timer.timeout.connect(self.update_image); self.timer.start(33)

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
    def __init__(self, node):
        super().__init__()
        self.setWindowTitle("Interfaz del ROV")
        self.setMinimumSize(1200, 800)

        central = QWidget()
        main_v = QVBoxLayout(central)

        # Fila superior: botones - RealSense viewer - tabla
        top_h = QHBoxLayout()
        top_h.addWidget(FeatureButtonsWidget(node), 1)
        top_h.addWidget(CameraViewerWidget(node), 3)
        top_h.addWidget(StatusTableWidget(node), 1)
        main_v.addLayout(top_h)

        # Fila inferior: cámaras pequeñas
        main_v.addWidget(Camaras(node))

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
