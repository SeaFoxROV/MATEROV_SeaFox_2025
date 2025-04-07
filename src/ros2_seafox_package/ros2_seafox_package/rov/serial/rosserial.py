import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Int16MultiArray,Bool
import serial
import time

def map_range(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class RosserialNode(Node):
    def __init__(self):
        super().__init__('rosserial_node')

        # Inicializar valores de motores en 0
        self.motor_values = [0.0] * 8

        # Suscribirse al tópico de comandos de motores
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'pwm_values',
            self.cmd_callback,
            10)

        # Publicador para enviar datos de sensores
        self.imu_publisher= self.create_publisher(Float32MultiArray, 'imu', 10)
        self.bar_publisher = self.create_publisher(Float32MultiArray, 'bar02', 10)
        self.leak_publisher = self.create_publisher(Bool, 'leak_sensor', 10)

        # Configuración del puerto serial al Arduino
        try:
            self.arduino = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.01)  # Cambiar según el puerto real
            time.sleep(1)  # Esperar a que el puerto se estabilice
            self.get_logger().info("Conectado al Arduino")
        except Exception as e:
            self.get_logger().error(f"No se pudo conectar al Arduino: {e}")

        # Timer para ejecutar la función cada 10 ms
        self.create_timer(0.05, self.update_motors)
        self.create_timer(0.05, self.read_sensors)  # Nuevo timer para leer sensores

    def cmd_callback(self, msg):
        """ Callback que actualiza los valores de los motores. """
        self.motor_values = list(msg.data)

    def update_motors(self):
        """ Se ejecuta cada 10 ms, imprime y envía los datos actuales de los motores. """

        output = ";".join([f"{value}" for value in self.motor_values]) + ";\n"

        # Imprimir en la consola
        #self.get_logger().info(f"Datos de motores: {output.strip()}")

        # Enviar al Arduino por serial
        try:
            self.arduino.write(output.encode())
        except Exception as e:
            self.get_logger().error(f"Error al escribir en el Arduino: {e}")

    def read_sensors(self):
        """ Lee datos del monitor serial y los publica en el tópico `sensor_data`. """
        while self.arduino.in_waiting > 0:
            try:
                line = self.arduino.readline().decode().strip()
                if line:
                    data = line.split(";")

                    
                    imu_msg = Float32MultiArray()
                    bar_msg = Float32MultiArray()
                    leak_msg = Bool()
                    
                    imu_msg.data = [0.0] * 6  # Inicializa con 7 elementos

                    bar_msg.data = [0.0] * 3  # Inicializa con 7 elementos


                    for i in range(6):
                        imu_msg.data[i] = float(data[i])
                    bar_msg.data[0] = float(data[6])
                    bar_msg.data[1] = float(data[7])
                    bar_msg.data[2] = float(data[8])

                    leak_msg.data = bool(float(data[9]))

                    self.imu_publisher.publish(imu_msg)
                    self.bar_publisher.publish(bar_msg)
                    self.leak_publisher.publish(leak_msg)

                    print(data[-3:])
                    self.get_logger().info(f"Datos de sensores recibidos: {data}")
            except Exception as e: 
                #print("error")
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
