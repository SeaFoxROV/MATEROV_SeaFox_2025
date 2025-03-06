import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Ecuaciones(Node):
    def __init__(self):
        super().__init__('ecuaciones_publisher')
        # Subscribe to the joystick data.
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joystick_data',
            self.joystick_callback,
            10)
        # Publisher for setpoint_motores.
        self.publisher_ = self.create_publisher(Float32MultiArray, 'setpoint_motores', 10)
        # Timer for periodically asking for user input.
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_joystick_data = []

    def joystick_callback(self, msg):
        self.last_joystick_data = msg.data
        self.get_logger().info(f"Received joystick data: {msg.data}")

    def timer_callback(self):
        # Blocking call to get user input; adjust as needed.
        number = input("Ingrese un número para publicar: ")
        try:
            number = int(number)
            msg = Float32MultiArray()
            msg.data = [float(number)]  # Ensure data is a list of floats.
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing: {number}")
        except ValueError:
            print("Por favor ingrese un número válido.")

def main():
    rclpy.init()
    node = Ecuaciones()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
