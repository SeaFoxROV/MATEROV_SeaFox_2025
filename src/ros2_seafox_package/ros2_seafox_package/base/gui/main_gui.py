#!/usr/bin/env python3
import sys
import threading
from PyQt5.QtWidgets import QApplication, QSplitter, QLabel, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QFrame
from PyQt5.QtCore import Qt, QTimer
import rclpy
import signal
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

class UltimateSubscriber(Node):
    def __init__(self):
        super().__init__('ultimate_subscriber')
        # Suscripción al tópico "distance_point"
        self.distance_subscription = self.create_subscription(
            Float32,
            'distance_point',
            self.distance_callback,
            10
        )
        # Suscripción al tópico "joystick_data"
        self.joystick_buttons = self.create_subscription(
            Float32MultiArray,
            'joystick_data',
            self.joystick_callback,
            10
        )
        self.latest_distance = None
        self.latest_buttons = None

    def distance_callback(self, msg: Float32):
        self.latest_distance = msg.data
        #self.get_logger().info(f"Received distance: {msg.data:.2f} meters")

    def joystick_callback(self, msg: Float32MultiArray):
        self.latest_buttons = msg.data
        #self.get_logger().info(f"Received buttons: {msg.data}")

class MainGui(QWidget):
    def __init__(self, node: UltimateSubscriber):
        super().__init__()
        self.node = node 

        self.setWindowTitle('MainGUI')
        self.setFixedSize(1320, 960)

        # Label para mostrar la distancia recibida
        self.distance_points = QLabel("Distance: -- m")
        self.distance_points.setFixedSize(480, 20)
        self.distance_points.setAlignment(Qt.AlignCenter)

        # Botones de ejemplo
        self.graficar = QPushButton("Graficar")
        self.graficar.setFixedSize(240, 20)
        self.conexion = QPushButton("Conexion")
        self.conexion.setFixedSize(240, 20)

        # Layout para el label de la distancia (centrado)
        label_layout = QHBoxLayout()
        label_layout.addStretch()
        label_layout.addWidget(self.distance_points)
        label_layout.addStretch()

        # Layout para los botones
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.conexion)
        button_layout.addWidget(self.graficar)

        # Widget que envuelve los botones para usar con QSplitter
        button_widget = QWidget()
        button_widget.setLayout(button_layout)

        # QSplitter (contiene el widget de botones y el label de distancia)
        screen_splitter = QSplitter(Qt.Horizontal)
        screen_splitter.setFrameShape(QFrame.NoFrame)
        label_widget = QWidget()
        label_widget.setLayout(label_layout)
        screen_splitter.addWidget(button_widget)
        screen_splitter.addWidget(label_widget)

        # Nuevo label para mostrar el estado de los botones
        self.button_status = QLabel("Buttons pressed: None")
        self.button_status.setAlignment(Qt.AlignCenter)
        self.button_status.setFixedHeight(20)

        # Layout principal
        main_layout = QVBoxLayout()
        main_layout.addWidget(screen_splitter)
        main_layout.addWidget(self.button_status)
        self.setLayout(main_layout)

        # Timer para actualizar el label de distancia y el de botones cada 100 ms
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)

    def update_ui(self):
        # Actualizar la distancia
        if self.node.latest_distance is not None:
            self.distance_points.setText(f"Distance: {self.node.latest_distance:.2f} m")
        else:
            self.distance_points.setText("Distance: -- m")
        # Actualizar el estado de los botones mostrando el array completo
        if self.node.latest_buttons is not None:
            self.button_status.setText("Buttons: " + str(self.node.latest_buttons))
        else:
            self.button_status.setText("Buttons: None")


def ros_spin_thread(node: Node):
    rclpy.spin(node)


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QApplication.quit()



def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, sigint_handler)

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
