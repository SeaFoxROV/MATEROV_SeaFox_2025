import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time

class RosserialNode(Node):
    def __init__(self):
        super().__init__('rosserial_node')

        # Inicializar valores de motores en 0
        self.motor_values = [0.0] * 8

        # Suscribirse al tópico de comandos de motores
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'thruster_cmd',
            self.cmd_callback,
            10)

        # Publicador para enviar datos de sensores
        self.publisher_ = self.create_publisher(Float32MultiArray, 'sensors_data', 10)

        # Configuración del puerto serial al Arduino
        try:
            self.arduino = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.01)  # Cambiar según el puerto real
            time.sleep(1)  # Esperar a que el puerto se estabilice
            self.get_logger().info("Conectado al Arduino")
        except Exception as e:
            self.get_logger().error(f"No se pudo conectar al Arduino: {e}")

        # Timer para ejecutar la función cada 10 ms
        self.create_timer(0.01, self.update_motors)
        self.create_timer(0.01, self.read_sensors)  # Nuevo timer para leer sensores

    def cmd_callback(self, msg):
        """ Callback que actualiza los valores de los motores. """
        self.motor_values = list(msg.data)

    def update_motors(self):
        """ Se ejecuta cada 10 ms, imprime y envía los datos actuales de los motores. """
        output = ";".join([f"{value:.2f}" for value in self.motor_values]) + ";\n"

        # Imprimir en la consola
        self.get_logger().info(f"Datos de motores: {output.strip()}")

        # Enviar al Arduino por serial
        try:
            self.arduino.write(output.encode())
        except Exception as e:
            self.get_logger().error(f"Error al escribir en el Arduino: {e}")

    def read_sensors(self):
        """ Lee datos del monitor serial y los publica en el tópico `sensor_data`. """
        try:
            line = self.arduino.readline().decode().strip()
            if line:
                data = list(map(float, line.split(";")))  # Convertir cadena a lista de floats
                if len(data) == 8:  # Verificar que sean 8 valores
                    msg = Float32MultiArray()
                    msg.data = data
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Datos de sensores recibidos: {data}")
                else:
                    self.get_logger().warn(f"Formato inválido: {line}")
        except Exception as e:
            self.get_logger().error(f"Error al leer del Arduino: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RosserialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo interrumpido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
