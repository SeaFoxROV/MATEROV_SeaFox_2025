import sys
import time
from PyQt5.QtWidgets import QMainWindow, QApplication, QMenu, QAction, QStyle, qApp
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QTimer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MainWindow(QMainWindow):
    def __init__(self, parent = None):
        super(MainWindow, self).__init__(parent)
