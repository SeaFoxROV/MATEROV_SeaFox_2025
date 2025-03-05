import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion

class QuaternionToEuler(Node):
    def __init__(self):
        super().__init__('quaternion_to_euler')
        
        self.euler_x_pub = self.create_publisher(Float64, '/orientation/euler/x', 10)
        self.euler_y_pub = self.create_publisher(Float64, '/orientation/euler/y', 10)
        self.euler_z_pub = self.create_publisher(Float64, '/orientation/euler/z', 10)
        self.orientations_euler = self.create_publisher(Quaternion, '/orientation/euler_all', 10)
        
        self.subscription = self.create_subscription(
            Quaternion,
            '/orientation',
            self.quaternion_to_euler_callback,
            10
        )
        self.subscription
    
    def quaternion_to_euler_callback(self, msg):
        quaternion = [msg.x, msg.y, msg.z, msg.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        roll_msg = Float64()
        pitch_msg = Float64()
        yaw_msg = Float64()
        
        roll_msg.data = roll
        pitch_msg.data = pitch
        yaw_msg.data = yaw
        
        self.euler_x_pub.publish(pitch_msg)
        self.euler_y_pub.publish(yaw_msg)
        self.euler_z_pub.publish(roll_msg)
        
        all_msg = Quaternion()
        all_msg.x = pitch
        all_msg.y = yaw
        all_msg.z = roll
        
        self.orientations_euler.publish(all_msg)
        
        self.get_logger().info(f'Published Euler Angles: Roll={roll}, Pitch={pitch}, Yaw={yaw}')


def main(args=None):
    rclpy.init(args=args)
    node = QuaternionToEuler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
