import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller_node')
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/desired_twist', 10)

        self.gripper_pub = self.create_publisher(Float32MultiArray, '/gripper_pwm', 10)
        
        # Subscribers
        self.create_subscription(Float32MultiArray, '/joystick_data', self.joy_callback, 100)
        #self.create_subscription(Float64, '/control_effort/angular/x', self.roll_effort_callback, 100)
        #self.create_subscription(Float64, '/control_effort/angular/y', self.pitch_effort_callback, 100)
        
        self.last_user_velocity_command = Twist()

        self.pwm_tick = 10

        self.pwms = Float32MultiArray()

        self.pwms.data = [1500.0]*3

        self.last_command_time = self.get_clock().now()
        #self.roll_pid_effort = 0.0
        #self.pitch_pid_effort = 0.0
        #enable_msg = Bool()
        #enable_msg.data = True
        #self.enable_pid_pub.publish(enable_msg)
        
        self.timer = self.create_timer(1.0/30.0, self.motion)

    def joy_callback(self, msg: Joy):
        if len(msg.data) < 6:
            self.get_logger().error(f"Received less axes than expected: got {len(msg.data)}, expected at least 6")
            return
        #[leftx,lefty,lefttrigger,rightx,righty,rightrigger,A,B,X,Y,LT,RT,BACK,SELECT,crossx,crossy]
        # Map joystick axes to velocity command
        left_joy_x = msg.data[0]      # Sides
        left_joy_y = msg.data[1]      # Forward/Back
        left_trigger = msg.data[2]    #doiwn
        right_trigger = msg.data[5]      # jup   Z
        right_joy_x = msg.data[3]
        right_joy_y = msg.data[4]


        #ajustar drift de x ES UNA CONSTANTE LO PUEDES RESTAR ASI NOMAS
        #ajustar deadzon de y (no es una constante)
        self.last_user_velocity_command.linear.x = -left_joy_y
        self.last_user_velocity_command.linear.y = left_joy_x
        self.last_user_velocity_command.linear.z = (right_trigger - left_trigger)/2
        self.last_user_velocity_command.angular.x = right_joy_x
        self.last_user_velocity_command.angular.y = right_joy_y
        
        self.pwms.data[0] += msg.data[15]*self.pwm_tick

        if self.pwms.data[0]<500:
            self.pwms.data[0] = 500
        if self.pwms.data[0]>2500:
            self.pwms.data[0] = 2500

        self.pwms.data[1] += msg.data[14]*self.pwm_tick

        if self.pwms.data[1]<500:
            self.pwms.data[1] = 500
        if self.pwms.data[1]>2500:
            self.pwms.data[1] = 2500
                    
        self.pwms.data[2] += (msg.data[10]*self.pwm_tick)-(msg.data[11]*self.pwm_tick)
        
        if self.pwms.data[2]<500:
            self.pwms.data[2] = 500
        if self.pwms.data[2]>2500:
            self.pwms.data[2] = 2500
        
        self.last_command_time = self.get_clock().now()

 #   def roll_effort_callback(self, msg: Float64):
 #         self.roll_pid_effort = msg.data

 #   def pitch_effort_callback(self, msg: Float64):
 #       self.pitch_pid_effort = msg.data

    def motion(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_command_time).nanoseconds / 1e9
        if elapsed > 1.0:
            self.last_user_velocity_command = Twist()
        
        cmd_vel = Twist()
        cmd_vel.linear.x = self.last_user_velocity_command.linear.x
        cmd_vel.linear.y = self.last_user_velocity_command.linear.y
        cmd_vel.linear.z = self.last_user_velocity_command.linear.z
        cmd_vel.angular.x = self.last_user_velocity_command.angular.x# + self.roll_pid_effort
        cmd_vel.angular.y = self.last_user_velocity_command.angular.y #- self.pitch_pid_effort
        #cmd_vel.angular.z = self.last_user_velocity_command.angular.z
        
        self.gripper_pub.publish(self.pwms)

        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()