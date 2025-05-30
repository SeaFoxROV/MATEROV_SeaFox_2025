import sys
import cv2
from PyQt5.QtWidgets import (
    QApplication, QDialog, QLabel, QPushButton, QVBoxLayout, QDialogButtonBox
)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt


class ImagePopup(QDialog):
    def __init__(self, images, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Visor de Imágenes (Pop-up)")
        self.images = images
        self.index = 0

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)

        self.next_button = QPushButton("Siguiente")
        self.next_button.clicked.connect(self.show_next_image)

        self.close_button = QPushButton("Cerrar")
        self.close_button.clicked.connect(self.close)

        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(self.next_button)
        layout.addWidget(self.close_button)

        self.setLayout(layout)
        self.resize(800, 600)

        self.show_image(self.index)

    def show_image(self, index):
        if 0 <= index < len(self.images):
            img = cv2.imread(self.images[index])
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            h, w, ch = img.shape
            bytes_per_line = ch * w
            qimg = QImage(img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)

            self.image_label.setPixmap(pixmap.scaled(
                self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def show_next_image(self):
        self.index = (self.index + 1) % len(self.images)
        self.show_image(self.index)


# Ejemplo de uso
if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Rutas de ejemplo (reemplaza con tus propias imágenes)
    rutas = [
        "imagen1.jpg",
        "imagen2.jpg",
        "imagen3.jpg"
    ]

    popup = ImagePopup(rutas)
    popup.exec_()  # Esto lanza el pop-up como modal

    sys.exit()
