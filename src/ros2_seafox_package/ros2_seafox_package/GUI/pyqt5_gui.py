import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QSpinBox
from PyQt5.QtCore import QTimer

class Window(QWidget):
    def _init_(self):
        super()._init_()

        # Configuración de la ventana
        self.setWindowTitle("Publicador ROS2 con PyQt5")
        self.setGeometry(100, 100, 400, 200)

        # Layout y elementos de la ventana
        self.layout = QVBoxLayout()

        self.label = QLabel("Selecciona un número para publicar:")
        self.layout.addWidget(self.label)

        self.spin_box = QSpinBox()
        self.spin_box.setRange(0, 100)
        self.layout.addWidget(self.spin_box)

        self.publish_button = QPushButton("Publicar Número")
        self.layout.addWidget(self.publish_button)

        self.status_label = QLabel("Estado: Esperando publicación")
        self.layout.addWidget(self.status_label)

        self.setLayout(self.layout)

        self.node = ROS2Node()
        
        self.publish_button.clicked.connect(self.publish_number)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.process_ros_events)
        self.timer.start(10)  

    def publish_number(self):
        number = self.spin_box.value()
        self.node.publish_number(number)
        self.status_label.setText(f"Estado: Número {number} publicado.")

    def process_ros_events(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)

class ROS2Node(Node):
    def _init_(self):
        super()._init_('qt_ros_publisher')
        self.publisher = self.create_publisher(Int32, 'number_topic', 10)
        self.get_logger().info("Nodo ROS2 inicializado y listo para publicar en 'number_topic'.")

    def publish_number(self, number):
        msg = Int32()
        msg.data = number
        self.publisher.publish(msg)
        self.get_logger().info(f"Publicado: {msg.data}")

def main():
    rclpy.init()

    # Crear la aplicación Qt
    app = QApplication(sys.argv)
    window = Window()
    window.show()

    # Ejecutar el bucle de la aplicación Qt
    sys.exit(app.exec_())

if __name__ == "_main_":
    main