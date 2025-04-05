#!/usr/bin/env python3
import sys
import threading
from PyQt5.QtWidgets import QApplication, QSplitter, QLabel, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QFrame
from PyQt5.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

class UltimateSubscriber(Node):
    def __init__(self):
        super().__init__('ultimate_subscriber')
        # Crear suscripción al tópico "distance_point" con el callback adecuado
        self.distance_subscription = self.create_subscription(
            Float32,
            'distance_point',
            self.distance_callback,
            10
        )
        self.joystick_buttons = self.create_subscription(
            Float32MultiArray,
            'joystick_data',
            self.joystick_callback,
            10
        )
        self.latest_distance = None
        self.latest_buttons = []

    def distance_callback(self, msg: Float32):
        self.latest_distance = msg.data
        self.get_logger().info(f"Received distance: {msg.data:.2f} meters")

    def joystick_callback(self, msg: Float32MultiArray):
        self.latest_buttons = msg.data
        self.get_logger().info(f"Received buttons: {msg.data:.2f} meters")

class MainGui(QWidget):
    def __init__(self, node: UltimateSubscriber):
        super().__init__()
        self.node = node 

        self.setWindowTitle('MainGUI')
        self.setFixedSize(1320, 960)

        # Label para mostrar la distancia recibida
        self.floatgui = QLabel("Distance: -- m")
        self.floatgui.setFixedSize(480, 20)
        self.floatgui.setAlignment(Qt.AlignCenter)

        # Botones de ejemplo
        self.graficar = QPushButton("Graficar")
        self.graficar.setFixedSize(240, 20)
        self.conexion = QPushButton("Conexion")
        self.conexion.setFixedSize(240, 20)

        # Layout para el label (centrado)
        label_layout = QHBoxLayout()
        label_layout.addStretch()
        label_layout.addWidget(self.floatgui)
        label_layout.addStretch()

        # Layout para los botones
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.conexion)
        button_layout.addWidget(self.graficar)

        # Widget que envuelve los botones para usar con QSplitter
        button_widget = QWidget()
        button_widget.setLayout(button_layout)

        # QSplitter (contiene el label y los botones)
        screen_splitter = QSplitter(Qt.Vertical)
        screen_splitter.setFrameShape(QFrame.NoFrame)
        label_widget = QWidget()
        label_widget.setLayout(label_layout)
        screen_splitter.addWidget(label_widget)
        screen_splitter.addWidget(button_widget)

        # Layout principal
        main_layout = QVBoxLayout()
        main_layout.addWidget(screen_splitter)
        self.setLayout(main_layout)

        # Timer para actualizar el label con el último valor recibido
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_distance)
        self.timer.start(100)  # Actualiza cada 100 ms

    def update_distance(self):
        if self.node.latest_distance is not None:
            self.floatgui.setText(f"Distance: {self.node.latest_distance:.2f} m")
        else:
            self.floatgui.setText("Distance: -- m")

def ros_spin_thread(node: Node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    sub = UltimateSubscriber()
    
    # Ejecutar el spin de ROS2 en un hilo separado para no bloquear la GUI
    ros_thread = threading.Thread(target=ros_spin_thread, args=(sub,), daemon=True)
    ros_thread.start()
    
    app = QApplication(sys.argv)
    gui = MainGui(sub)
    gui.show()
    
    exit_code = app.exec_()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
