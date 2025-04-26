import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Bool
import serial
import serial.tools.list_ports
import time

def map_range(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class RosserialNode(Node):
    def __init__(self):
        super().__init__('rosserial_node')

        self.motor_values = [1500.0] * 8  # Inicia en modo seguro
        self.last_cmd_time = time.time()  # Última vez que llegó un mensaje
        self.gripper_values = [1500.0] * 3  # Inicia en modo seguro

        self.subscription = self.create_subscription(
            Int16MultiArray,
            'pwm_values',
            self.cmd_callback,
            10)
        ###self.subgripper = self.create_subscription(
        #    Float32MultiArray,
        #    'gripper_pwm',
        #    self.cmd_callback,
        #    10)
        ###
        self.imu_publisher = self.create_publisher(Float32MultiArray, 'imu', 10)
        self.bar_publisher = self.create_publisher(Float32MultiArray, 'bar02', 10)
        self.leak_publisher = self.create_publisher(Bool, 'leak_sensor', 10)

        self.puerto = self.get_port()
        try:
            self.arduino = serial.Serial(self.puerto, 115200, timeout=0.01)
            self.get_logger().info(f"Conectado al Arduino en el puerto {self.puerto}")
        except Exception as e:
            self.get_logger().error(f"No se pudo conectar al Arduino: {e}")
            self.arduino = None

        self.create_timer(0.05, self.update_motors)

    def get_port(self):
        puertos = serial.tools.list_ports.comports()
        for puerto in puertos:
            if "USB" in puerto.device or "ACM" in puerto.device:
                return puerto.device
        return None

    def cmd_callback(self, msg):
        self.pwm_values = list(msg.data)
        self.last_cmd_time = time.time()

    def update_motors(self):
        if self.arduino is None:
            return

        # Watchdog: si no llegan mensajes en 0.5 segundos, enviar 1500
        #if time.time() - self.last_cmd_time > 0.5:
        #    self.motor_values = [1500.0] * 8

        # Pequeña zona muerta
        output_values = []
        for motor_value in self.pwm_values:
            if 1600 > motor_value and 1400<motor_value:
                motor_value = 1500
            output_values.append(motor_value)

        output = ";".join([f"{int(value)}" for value in output_values]) + ";\n"
        try:
            self.arduino.write(output.encode())
            self.get_logger().info(f"Datos de motores: {output.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error al escribir en el Arduino: {e}")

    def stop_motors(self):
        if self.arduino:
            stop_values = [1500] * 8
            output = ";".join([str(v) for v in stop_values]) + ";\n"
            try:
                self.arduino.write(output.encode())
                self.get_logger().info("Motores detenidos (1500)")
            except Exception as e:
                self.get_logger().error(f"No se pudo detener los motores: {e}")
        else:
            self.get_logger().warn("No hay conexión serial para detener los motores.")

def main(args=None):
    rclpy.init(args=args)
    node = RosserialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo interrumpido por el usuario.")
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
