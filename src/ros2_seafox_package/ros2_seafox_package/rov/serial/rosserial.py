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

        self.motor_values = [1500.0] * 6 
        self.gripper_values = [1500.0] * 3

        self.last_cmd_time = time.time()  # Última vez que llegó un mensaje
        self.gripper_values = [1500.0] * 3  # Inicia en modo seguro

        self.subscription = self.create_subscription(
            Int16MultiArray,
            'pwm_values',
            self.cmd_callback,
            10)
        self.gripper_sub = self.create_subscription(
            Float32MultiArray,
            'gripper_pwm',
            self.gripper_callback,
            10)

        self.imu_publisher = self.create_publisher(Float32MultiArray, 'imu', 10)
        self.bar_publisher = self.create_publisher(Float32MultiArray, 'bar02', 10)
        self.leak_publisher = self.create_publisher(Bool, 'leak_sensor', 10)

        
        self.arduino = serial.Serial("/dev/esp32", 115200, timeout=0.01)
        self.get_logger().info(f"Conectado al Arduino en el puerto serial")
    
        self.create_timer(0.05, self.update_motors)


    def cmd_callback(self, msg):
        self.motor_values = list(msg.data)
        self.last_cmd_time = time.time()

    def gripper_callback(self, msg):
        # Verificar que los índices no excedan el tamaño de motor_values
        if len(msg.data) >= 3:
            self.gripper_values[0] = msg.data[0]
            self.gripper_values[1] = msg.data[1]
            self.gripper_values[2] = msg.data[2]  # Asumiendo que el índice 6 es para el 3er valor
            self.last_cmd_time = time.time()
        else:
            self.get_logger().warn("Se recibieron datos incorrectos para el gripper.")

    def update_motors(self):
        if self.arduino is None:
            return

        # Watchdog: si no llegan mensajes en 0.5 segundos, enviar 1500
        if time.time() - self.last_cmd_time > 0.5:
            self.motor_values = [1500.0] * 9  # Volver a valores seguros

        # Pequeña zona muerta (de 1400 a 1600)
        output_values = []
        for motor_value in self.motor_values:
            if 1400 < motor_value < 1600:
                motor_value = 1500  # Aplicamos la zona muerta
            output_values.append(motor_value)
        
        for gripper_value in self.gripper_values:
            output_values.append(gripper_value)

        output = ";".join([f"{int(value)}" for value in output_values]) + ";\n"
        try:
            self.arduino.write(output.encode())
            self.get_logger().info(f"Datos de motores: {output.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error al escribir en el Arduino: {e}")

    def stop_motors(self):
        if self.arduino:
            stop_values = [1500] * 10  # Detener todos los motores (10 valores)
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
