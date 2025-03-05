import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller_node')
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enable_pid_pub = self.create_publisher(Bool, '/pid_enable', 10)
        
        # Subscribers
        self.create_subscription(Joy, '/joy', self.joy_callback, 100)
        self.create_subscription(Float64, '/control_effort/angular/x', self.roll_effort_callback, 100)
        self.create_subscription(Float64, '/control_effort/angular/y', self.pitch_effort_callback, 100)
        
        self.last_user_velocity_command = Twist()
        self.last_command_time = self.get_clock().now()
        self.roll_pid_effort = 0.0
        self.pitch_pid_effort = 0.0
        enable_msg = Bool()
        enable_msg.data = True
        self.enable_pid_pub.publish(enable_msg)
        
        self.timer = self.create_timer(1.0/30.0, self.motion)

    def joy_callback(self, msg: Joy):
        if len(msg.axes) < 6:
            self.get_logger().error(f"Received less axes than expected: got {len(msg.axes)}, expected at least 6")
            return
        # Map joystick axes to velocity command
        left_joy_x = msg.axes[0]      # Sides
        left_joy_y = msg.axes[1]      # Forward/Back
        right_joy_x = msg.axes[2]     # Rotation Z
        # right_joy_y is not used
        left_trigger = msg.axes[4]    # Up
        right_trigger = msg.axes[5]   # Down

        self.last_user_velocity_command.linear.x = -left_joy_y
        self.last_user_velocity_command.linear.y = left_joy_x
        self.last_user_velocity_command.linear.z = right_trigger - left_trigger
        self.last_user_velocity_command.angular.z = right_joy_x
        
        self.last_command_time = self.get_clock().now()

    def roll_effort_callback(self, msg: Float64):
        self.roll_pid_effort = msg.data

    def pitch_effort_callback(self, msg: Float64):
        self.pitch_pid_effort = msg.data

    def motion(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_command_time).nanoseconds / 1e9
        if elapsed > 1.0:
            self.last_user_velocity_command = Twist()
        
        cmd_vel = Twist()
        cmd_vel.linear.x = self.last_user_velocity_command.linear.x
        cmd_vel.linear.y = self.last_user_velocity_command.linear.y
        cmd_vel.linear.z = self.last_user_velocity_command.linear.z
        cmd_vel.angular.x = self.last_user_velocity_command.angular.x + self.roll_pid_effort
        cmd_vel.angular.y = self.last_user_velocity_command.angular.y - self.pitch_pid_effort
        cmd_vel.angular.z = self.last_user_velocity_command.angular.z

        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
