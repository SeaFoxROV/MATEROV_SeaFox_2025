from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSplitter,
    QDialog, QSizePolicy
)
from PyQt5.QtCore import Qt
import sys

class Graphs(QWidget):
    def __init__(self):
        super().__init__()
        #Labels to show the graphs
        self.label_right = QLabel("Derecha")
        self.label_left = QLabel("Izquierda")
        
        for lbl in (self.label_left, self.label_right):
            lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            lbl.setStyleSheet("background-color: white; border: 2px solid blue;border-radius: 10px;")

        #Splitter of the labels
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.label_left)
        splitter.addWidget(self.label_right)

        #Buttons for the table
        self.btn_left = QPushButton("Data table 1")
        self.btn_right = QPushButton("Data table 2")

        #Buttons splitter
        splitter_btns = QSplitter(Qt.Horizontal)
        splitter_btns.addWidget(self.btn_left)
        splitter_btns.addWidget(self.btn_right)

        #Main splitter
        splitter_main = QSplitter(Qt.Vertical)
        splitter_main.addWidget(splitter)
        splitter_main.addWidget(splitter_btns)
        splitter_main.setStretchFactor(0,10)
        splitter_main.setStretchFactor(1,1)

        layout = QHBoxLayout(self)
        layout.addWidget(splitter_main)

class Tables(QDialog):
    def __init__(self):
        super().__init__()

class Buttons(QWidget):
    def __init__(self):
        super().__init__()
        #Buttons for the functionality
        self.btn_start = QPushButton("btn inicio")
        self.btn_ping = QPushButton("btn ping")
        self.btn_finish = QPushButton("btn finish")
        self.label_info = QLabel("label info")

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.btn_start)
        splitter.addWidget(self.btn_ping)
        splitter.addWidget(self.btn_finish)
        splitter.addWidget(self.label_info)

        layout = QHBoxLayout(self)
        layout.addWidget(splitter)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Float GUI")
        self.setMinimumSize(1200,800)

        #Widgets
        self.graphs = Graphs()
        self.buttons = Buttons()

        central = QWidget()
        main = QVBoxLayout(central)
        main.setContentsMargins(10,10,10,10)
        main.setSpacing(10)
        #Fila superior
        top = QHBoxLayout()
        top.addWidget(self.graphs)
        #Fila inferior
        bottom = QVBoxLayout()
        bottom.addWidget(self.buttons)
        main.addLayout(top, 3)
        main.addLayout(bottom, 1)

        self.setCentralWidget(central)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())