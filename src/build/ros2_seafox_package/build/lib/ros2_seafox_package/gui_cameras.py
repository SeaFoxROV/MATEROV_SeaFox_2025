# camera_gui.py
from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
import cv2

class CameraGUI(QWidget):
    def __init__(self, camera_subscriber):
        super().__init__()
        self.camera_subscriber = camera_subscriber
        self._init_ui()

    def _init_ui(self):
        self.setWindowTitle('Camera Viewer')
        layout = QHBoxLayout()

        # Create a QLabel for each camera image
        self.labels = {}
        for topic in self.camera_subscriber.topics:
            label = QLabel(self)
            label.setFixedSize(320, 240)  # You can adjust the display size as needed
            layout.addWidget(label)
            self.labels[topic] = label

        self.setLayout(layout)

        # QTimer to refresh the images
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_images)
        self.timer.start(30)  # update every 30ms

    def update_images(self):
        for topic, label in self.labels.items():
            img = self.camera_subscriber.latest_images.get(topic)
            if img is not None:
                # Convert from BGR (OpenCV) to RGB (Qt)
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                h, w, ch = img_rgb.shape
                bytes_per_line = ch * w
                qt_image = QImage(img_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qt_image)
                label.setPixmap(pixmap.scaled(label.size()))
