import sys
import threading
from PyQt5.QtWidgets import QApplication, QSplitter, QLabel, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QFrame
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node

class UltimateSubscriber(Node):
    def __init__(self):
        super().__init__('ultimate_subscriber')

class MainGui(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node 

        self.setWindowTitle('MainGUI')
        self.setFixedSize(1320, 960)

        # Label
        self.floatgui = QLabel("Float")
        self.floatgui.setFixedSize(480, 20)
        self.floatgui.setAlignment(Qt.AlignCenter)  # Align text inside the label

        # Buttons
        self.graficar = QPushButton("Graficar")
        self.graficar.setFixedSize(240, 20)

        self.conexion = QPushButton("Conexion")
        self.conexion.setFixedSize(240, 20)

        # Layout for label (centered)
        label_layout = QHBoxLayout()
        label_layout.addStretch()
        label_layout.addWidget(self.floatgui)
        label_layout.addStretch()

        # Layout for buttons
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.conexion)
        button_layout.addWidget(self.graficar)

        # Wrapping buttons in a widget to use with QSplitter
        button_widget = QWidget()
        button_widget.setLayout(button_layout)

        # Splitter (contains label & buttons)
        screen_splitter = QSplitter(Qt.Vertical)
        screen_splitter.setFrameShape(QFrame.NoFrame)
        
        # Wrapping label layout in a widget for QSplitter
        label_widget = QWidget()
        label_widget.setLayout(label_layout)

        screen_splitter.addWidget(label_widget)
        screen_splitter.addWidget(button_widget)

        # Main layout
        self.layout = QVBoxLayout()
        self.layout.addWidget(screen_splitter)
        self.setLayout(self.layout)

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    sub = UltimateSubscriber()
    
    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin_thread, args=(sub,), daemon=True)
    ros_thread.start()
    
    app = QApplication(sys.argv)
    gui = MainGui(sub)
    gui.show()
    
    sys.exit(app.exec_())

    rclpy.shutdown()  # Ensure ROS2 is properly shutdown

if __name__ == '__main__':
    main()
