#!/usr/bin/env python3
import sys
import threading
from PyQt5.QtWidgets import (
    QApplication, QSplitter, QLabel, QWidget,
    QPushButton, QVBoxLayout, QHBoxLayout, QFrame, QLineEdit
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QPainter, QColor
import rclpy
import signal
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

# --- Widget para mostrar el control de Xbox ---
class XboxControlWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.bg_image = QPixmap("src/ros2_seafox_package/ros2_seafox_package/imgs/xbox.png")  # Ruta a tu imagen
        self.buttons_state = [0] * 16 
        
        # Mapeo de botones digitales a sus posiciones en la imagen (ejemplo)
        self.button_positions = {
            6: (305, 158),  # A
            7: (332, 130),  # B
            8: (273, 130),  # X
            9: (305, 100),  # Y
            10: (100, 57),   # LT
            11: (320, 57),  # RT
            12: (170, 130), # BACK
            13: (235, 130), # SELECT
            14: (167, 195), # crossx (parte del D-pad)
            15: (147, 170)  # crossy (parte del D-pad)
        }
        # Ubicaciones base para los sticks analógicos (ejemplo)
        self.analog_positions = {
            0: (100, 130),  # Left Stick: posición neutra
            3: (255, 190)   # Right Stick: posición neutra
        }
        self.setMinimumSize(600, 400)

    def update_buttons(self, buttons_data):
        if len(buttons_data) != 16:
            print("Error: se esperaban 16 elementos en buttons_data")
            return
        self.buttons_state = buttons_data
        self.update()  # Solicita repintar el widget

    def paintEvent(self, event):
        painter = QPainter(self)
        # Dibuja la imagen de fondo; si no se carga, se pinta un fondo gris
        if not self.bg_image.isNull():
            painter.drawPixmap(self.rect(), self.bg_image)
        else:
            painter.fillRect(self.rect(), QColor(100, 100, 100))
        
        # --- Dibujar indicadores para botones digitales ---
        for idx, pos in self.button_positions.items():
            if idx >= len(self.buttons_state):
                continue

            # Para crossx y crossy, si el valor es -1 se asigna una nueva posición
            if idx == 14 and self.buttons_state[idx] == -1:
                # Si cross x es -1, se indicará "izquierda" en otra posición
                new_pos = (127, 195)  # Coordenadas para indicar izquierda (ajusta según tus necesidades)
            elif idx == 15 and self.buttons_state[idx] == -1:
                # Si cross y es -1, se indicará "abajo" en otra posición
                new_pos = (147, 220)  # Coordenadas para indicar abajo (ajusta según tus necesidades)
            else:
                new_pos = pos

            # Se pinta rojo si el botón está activado (valor distinto de 0 o -1 según se requiera)
            if self.buttons_state[idx]:
                painter.setBrush(QColor("red"))
            else:
                painter.setBrush(QColor(200, 200, 200, 100))
            radius = 15
            painter.drawEllipse(
                int(new_pos[0] - radius), int(new_pos[1] - radius),
                int(radius * 2), int(radius * 2)
            )
        
        # --- Dibujar indicadores para sticks analógicos ---
        # Stick izquierdo: índices 0 y 1 (leftx, lefty)
        leftx = self.buttons_state[0]
        lefty = self.buttons_state[1]
        lx, ly = self.analog_positions[0]
        offset = 20  # factor de desplazamiento visual
        adjusted_left_x = int(lx + leftx * offset)
        adjusted_left_y = int(ly + lefty * offset)
        painter.setBrush(QColor("blue"))
        painter.drawEllipse(
            adjusted_left_x - 10, adjusted_left_y - 10,
            20, 20
        )
        
        # Stick derecho: índices 3 y 4 (rightx, righty)
        rightx = self.buttons_state[3]
        righty = self.buttons_state[4]
        rx, ry = self.analog_positions[3]
        adjusted_right_x = int(rx + rightx * offset)
        adjusted_right_y = int(ry + righty * offset)
        painter.setBrush(QColor("green"))
        painter.drawEllipse(
            adjusted_right_x - 10, adjusted_right_y - 10,
            20, 20
        )
        
        # --- Dibujar indicadores para triggers ---
        # Left trigger: índice 2 (dibuja una barra)
        left_trigger = self.buttons_state[2]
        if left_trigger:
            painter.setBrush(QColor("orange"))
            painter.drawRect(
                50, 10, int(left_trigger * 50), 10
            )
        # Right trigger: índice 5
        right_trigger = self.buttons_state[5]
        if right_trigger:
            painter.setBrush(QColor("orange"))
            painter.drawRect(
                305, 10, int(right_trigger * 50), 10
            )
        # Al salir del método se libera el contexto del pintor.

# --- Nodo ROS2 que se suscribe a distance_point y joystick_data ---
class UltimateSubscriber(Node):
    def __init__(self):
        super().__init__('ultimate_subscriber')
        self.distance_subscription = self.create_subscription(
            Float32,
            'distance_point',
            self.distance_callback,
            10
        )
        self.joystick_subscription = self.create_subscription(
            Float32MultiArray,
            'joystick_data',
            self.joystick_callback,
            10
        )
        self.imu_subscriber = self.create_subscription(
            Float32,
            'imu',
            self.imu_callback,
            10
        )
        self.latest_distance = None
        self.latest_buttons = None
        
        '''
        self.textbox = QLineEdit(self)
        self.textbox.move(20, 20)
        self.textbox.resize(280,40)
        '''

    def distance_callback(self, msg: Float32):
        self.latest_distance = msg.data

    def joystick_callback(self, msg: Float32MultiArray):
        self.latest_buttons = msg.data

# --- Interfaz principal ---
class MainGui(QWidget):
    def __init__(self, node: UltimateSubscriber):
        super().__init__()
        self.node = node 
        self.setWindowTitle('MainGUI')
        self.setFixedSize(1320, 960)

        self.distance_points = QLabel("Distance: -- m")
        self.distance_points.setFixedSize(480, 20)
        self.distance_points.setAlignment(Qt.AlignCenter)

        self.graficar = QPushButton("Graficar")
        self.graficar.setFixedSize(240, 20)
        self.conexion = QPushButton("Conexion")
        self.conexion.setFixedSize(240, 20)

        label_layout = QHBoxLayout()
        label_layout.addStretch()
        label_layout.addWidget(self.distance_points)
        label_layout.addStretch()

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.conexion)
        button_layout.addWidget(self.graficar)

        button_widget = QWidget()
        button_widget.setLayout(button_layout)

        screen_splitter = QSplitter(Qt.Horizontal)
        screen_splitter.setFrameShape(QFrame.NoFrame)
        label_widget = QWidget()
        label_widget.setLayout(label_layout)
        screen_splitter.addWidget(button_widget)
        screen_splitter.addWidget(label_widget)

        screen_splitter2 = QSplitter(Qt.Horizontal)
        # Instanciamos el widget del control Xbox
        self.xbox_widget = XboxControlWidget()
        self.xbox_widget.setFixedSize(400, 400)
        screen_splitter2.addWidget(self.xbox_widget)

        self.button_status = QLabel("Buttons pressed: None")
        self.button_status.setAlignment(Qt.AlignCenter)
        self.button_status.setFixedHeight(20)

        main_layout = QVBoxLayout()
        main_layout.addWidget(screen_splitter)
        main_layout.addWidget(self.button_status)
        main_layout.addWidget(screen_splitter2)
        self.setLayout(main_layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)

    def update_ui(self):
        if self.node.latest_distance is not None:
            self.distance_points.setText(f"Distance: {self.node.latest_distance:.2f} m")
        else:
            self.distance_points.setText("Distance: -- m")
        if self.node.latest_buttons is not None:
            self.button_status.setText("Buttons: " + str(self.node.latest_buttons))
            self.xbox_widget.update_buttons(self.node.latest_buttons)
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
