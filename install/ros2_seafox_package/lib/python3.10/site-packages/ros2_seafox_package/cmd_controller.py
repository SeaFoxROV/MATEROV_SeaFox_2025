import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile
import signal

NUM_DOF = 6
NUM_THRUSTERS = 8

# Matriz de ecuaciones, modificarla según la configuración de los propulsores
equation_matrix = [
    [1, -1, 0, 0, 0, 0, 1, 1],    # Lz
    [0, 0, -1, -1, 1, 1, 0, 0],    # Lx
    [0, 0, 1, -1, -1, 1, 0, 0],    # Ly
    [0, 0, -1, 1, -1, 1, 0, 0],    # Rz
    [-1, -1, 0, 0, 0, 0, 1, -1],   # Rx
    [1, -1, 0, 0, 0, 0, -1, -1]     # Ry
]

# Definir grupos de propulsores para escalar
group1 = [0, 1, 6, 7]
group2 = [2, 3, 4, 5]

class CmdController(Node):
    def __init__(self):
        super().__init__('cmd_controller_node')
        qos = QoSProfile(depth=10)
        # Se crea un publicador que envía un Float32MultiArray con la lista de comandos para los propulsores
        self.pub_thrusters = self.create_publisher(Float32MultiArray, 'thruster_cmd', qos)
        
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.last_thrusters_command = [0.0] * NUM_THRUSTERS
        self.thrusters_speed_limit = [75] * NUM_THRUSTERS
        self.max_jerk = 1.2
        self.last_time = self.get_clock().now()
        
        signal.signal(signal.SIGINT, self.on_shutdown)

    def scale_group(self, thruster_temp, group):
        max_value = 1.0
        for i in group:
            max_value = max(max_value, abs(thruster_temp[i]))
        if max_value > 1.0:
            scale_factor = 1.0 / max_value
            for i in group:
                thruster_temp[i] *= scale_factor

    def cmd_vel_callback(self, msg: Twist):
        cmd_msg = [
            float(msg.linear.z),
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.angular.z),
            float(msg.angular.x),
            float(msg.angular.y)
        ]
        
        thruster_temp = [0.0] * NUM_THRUSTERS
        
        # Calcular los comandos de los propulsores utilizando la matriz de ecuaciones
        for i in range(NUM_THRUSTERS):
            for j in range(NUM_DOF):
                thruster_temp[i] += equation_matrix[j][i] * cmd_msg[j]
        
        # Escalar cada grupo para que los valores se mantengan en el rango [-1, 1]
        self.scale_group(thruster_temp, group1)
        self.scale_group(thruster_temp, group2)
        
        # Aplicar límites de velocidad a los propulsores
        for i in range(NUM_THRUSTERS):
            thruster_temp[i] *= self.thrusters_speed_limit[i] / 100.0
        
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        
        # Aplicar limitación de jerk y mapear de [-1, 1] a [0, 255]
        for i in range(NUM_THRUSTERS):
            diff = thruster_temp[i] - self.last_thrusters_command[i]
            delta_speed = self.max_jerk * dt
            if diff > delta_speed:
                diff = delta_speed
            if diff < -delta_speed:
                diff = -delta_speed
            self.last_thrusters_command[i] += diff
            # Mapeo de [-1,1] a [0,255]
            thruster_temp[i] = float((self.last_thrusters_command[i] + 1) * 127.5)
            thruster_temp[i] = max(0.0, min(255.0, thruster_temp[i]))
        
        # Crear y publicar el mensaje Float32MultiArray
        thruster_array = Float32MultiArray()
        thruster_array.data = thruster_temp
        self.pub_thrusters.publish(thruster_array)
        
        self.last_time = now

    def on_shutdown(self, sig, frame):
        self.get_logger().info("Cerrando cmd_controller, enviando comando de stop (127) a todos los propulsores...")
        stop_array = Float32MultiArray()
        stop_array.data = [127.0] * NUM_THRUSTERS
        self.pub_thrusters.publish(stop_array)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CmdController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
