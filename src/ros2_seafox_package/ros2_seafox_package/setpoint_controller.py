import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class SetpointController(Node):
    def __init__(self):
        super().__init__('setpoint_controller')
        self.setpoint_x_pub = self.create_publisher(Float64, '/desired_angle/x', 100)
        self.setpoint_y_pub = self.create_publisher(Float64, '/desired_angle/y', 100)
        
        self.desired_angle_x = 0.0
        self.desired_angle_y = 3.141516        
        self.timer = self.create_timer(0.1, self.publish_setpoints)

    def publish_setpoints(self):
        msg_x = Float64()
        msg_x.data = self.desired_angle_x
        self.setpoint_x_pub.publish(msg_x)
        
        msg_y = Float64()
        msg_y.data = self.desired_angle_y
        self.setpoint_y_pub.publish(msg_y)
        
        self.get_logger().info(f'Published setpoints: X={self.desired_angle_x}, Y={self.desired_angle_y}')

def main(args=None):
    rclpy.init(args=args)
    node = SetpointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()