from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSplitter,
    QDialog, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
import sys

import numpy as np
import pandas as pd
import cv2
import pytesseract
from PIL import Image
from difflib import SequenceMatcher

class Video(QWidget):
    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)
        self.captured_text = []
        self.i = 0

        # Label to show the video stream
        self.video_label = QLabel("Video Stream")
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_label.setStyleSheet("background-color: black; border: 2px solid red; border-radius: 10px;")
        splitter_video = QSplitter(Qt.Horizontal)
        splitter_video.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        splitter_video.setMinimumHeight(300)
        splitter_video.addWidget(self.video_label)

        # Buttons for photo capture
        self.btn_gallery = QPushButton("Save Photo")
        self.btn_gallery.clicked.connect(self.save_photo)
        self.btn_capture = QPushButton("Compare Photo")
        self.btn_capture.clicked.connect(self.compare_photo)
        self.btn_history = QPushButton("History")
        self.btn_history.clicked.connect(self.show_history)
        for btn in (self.btn_gallery, self.btn_capture, self.btn_history):
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            btn.setStyleSheet("background-color: lightgray; border: 1px solid black; border-radius: 5px;")
        splitter_btns = QSplitter(Qt.Horizontal)
        splitter_btns.addWidget(self.btn_gallery)
        splitter_btns.addWidget(self.btn_capture)
        splitter_btns.addWidget(self.btn_history)

        # Label for information
        self.label_info = QLabel("Information Label")
        self.label_info.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label_info.setStyleSheet("background-color: lightblue; border: 1px solid black; border-radius: 5px;")
        splitter_btns.addWidget(self.label_info)
        splitter_btns.setMinimumHeight(100)

        layout = QVBoxLayout(self)
        layout.addWidget(splitter_video)
        layout.addWidget(splitter_btns)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_video_stream)
        self.timer.start(33)
    
    def update_video_stream(self):
        ret, frame = self.cap.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, _ = frame_rgb.shape
            img = QImage(frame_rgb.data, w, h, 3*w, QImage.Format_RGB888)
            pix = QPixmap.fromImage(img)
            size = self.video_label.size()
            pix = pix.scaled(size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.video_label.setPixmap(pix)
        else:
            self.video_label.setText(":,v")

    def save_photo(self):
        ret, frame = self.cap.read()
        if ret:
            cv2.imwrite(f'photo {self.i}.jpg', frame)
            self.captured_text.append(pytesseract.image_to_string(f"photo {self.i}.jpg"))
            self.label_info.setText(f"Captured Text: {self.captured_text[self.i]}")
            self.i += 1
            print(self.captured_text)

    def compare_photo(self):
        ret, frame = self.cap.read()
        if ret:
            cv2.imwrite('photo_to_compare.jpg', frame)
            text = pytesseract.image_to_string('photo_to_compare.jpg')
            for compare in self.captured_text:
                similarity = SequenceMatcher(None, text, compare).ratio()
                if similarity > 0.5:
                    self.label_info.setText(f"The eADN sequence is similar to sequence number {(self.captured_text.index(compare)+1)} with similarity {similarity:.2f}")
                    break
                else:
                    self.label_info.setText(f"No similar eADN sequence found. The probability is of {similarity:.2f} with the last sequence.")    

    def show_history(self):
        print(self.captured_text)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("eDNA Camera")
        self.setGeometry(100, 100, 800, 600)
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)
        self.layout.addWidget(Video())
        print(pytesseract.image_to_string(Image.open('/home/seafoxinventive/MATEROV_SeaFox_2025/src/ros2_seafox_package/ros2_seafox_package/vision/image0.png')))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())