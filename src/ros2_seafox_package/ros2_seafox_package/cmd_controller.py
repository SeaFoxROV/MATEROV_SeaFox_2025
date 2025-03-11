import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from rclpy.qos import QoSProfile
import math
import signal

#Necessary to preevnt damaging thrusters

NUM_DOF = 6
NUM_THRUSTERS = 8

# Equation matrix, change this to match thruster config
equation_matrix = [
    [1, -1, 0, 0, 0, 0, 1, 1],    # Lz
    [0, 0, -1, -1, 1, 1, 0, 0],    # Lx
    [0, 0, 1, -1, -1, 1, 0, 0],    # Ly
    [0, 0, -1, 1, -1, 1, 0, 0],    # Rz
    [-1, -1, 0, 0, 0, 0, 1, -1],   # Rx
    [1, -1, 0, 0, 0, 0, -1, -1]     # Ry
]

# Define thruster groups for scaling
group1 = [0, 1, 6, 7]
group2 = [2, 3, 4, 5]

class CmdController(Node):
    def __init__(self):
        super().__init__('cmd_controller_node')
        qos = QoSProfile(depth=10)
        self.pub_thrusters = {
            'front_bottom_right': self.create_publisher(UInt8, 'thruster/front_bottom_right', qos),
            'front_bottom_left':  self.create_publisher(UInt8, 'thruster/front_bottom_left', qos),
            'back_bottom_left':   self.create_publisher(UInt8, 'thruster/back_bottom_left', qos),
            'back_bottom_right':  self.create_publisher(UInt8, 'thruster/back_bottom_right', qos),
            'front_top_right':    self.create_publisher(UInt8, 'thruster/front_top_right', qos),
            'front_top_left':     self.create_publisher(UInt8, 'thruster/front_top_left', qos),
            'back_top_left':      self.create_publisher(UInt8, 'thruster/back_top_left', qos),
            'back_top_right':     self.create_publisher(UInt8, 'thruster/back_top_right', qos)
        }
        
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
        
        # Calculate thruster commands using the equation matrix
        for i in range(NUM_THRUSTERS):
            for j in range(NUM_DOF):
                thruster_temp[i] += equation_matrix[j][i] * cmd_msg[j]
        
        # Scale each thruster group to ensure values remain within [-1, 1]
        self.scale_group(thruster_temp, group1)
        self.scale_group(thruster_temp, group2)
        
        # Apply thruster speed limits
        for i in range(NUM_THRUSTERS):
            thruster_temp[i] *= self.thrusters_speed_limit[i] / 100.0
        
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        
        thruster_msgs = {}
        # Apply jerk limiting and map from [-1,1] to [0,255] (centered at 127.5)
        for i in range(NUM_THRUSTERS):
            diff = thruster_temp[i] - self.last_thrusters_command[i]
            delta_speed = self.max_jerk * dt
            if diff > delta_speed:
                diff = delta_speed
            if diff < -delta_speed:
                diff = -delta_speed
            self.last_thrusters_command[i] += diff
            mapped = int(round((self.last_thrusters_command[i] + 1) * 127.5))
            mapped = max(0, min(255, mapped))
            thruster_msgs[i] = mapped
        
        msg0 = UInt8()
        msg0.data = thruster_msgs[0]
        self.pub_thrusters['front_bottom_right'].publish(msg0)
        
        msg1 = UInt8()
        msg1.data = thruster_msgs[1]
        self.pub_thrusters['front_bottom_left'].publish(msg1)
        
        msg2 = UInt8()
        msg2.data = thruster_msgs[2]
        self.pub_thrusters['back_bottom_left'].publish(msg2)
        
        msg3 = UInt8()
        msg3.data = thruster_msgs[3]
        self.pub_thrusters['back_bottom_right'].publish(msg3)
        
        msg4 = UInt8()
        msg4.data = thruster_msgs[4]
        self.pub_thrusters['front_top_right'].publish(msg4)
        
        msg5 = UInt8()
        msg5.data = thruster_msgs[5]
        self.pub_thrusters['front_top_left'].publish(msg5)
        
        msg6 = UInt8()
        msg6.data = thruster_msgs[6]
        self.pub_thrusters['back_top_left'].publish(msg6)
        
        msg7 = UInt8()
        msg7.data = thruster_msgs[7]
        self.pub_thrusters['back_top_right'].publish(msg7)
        
        self.last_time = now



    def on_shutdown(self, sig, frame):
        self.get_logger().info("Shutting down the cmd_controller, sending stop command (127) to all thrusters...")
        stop_msg = UInt8()
        stop_msg.data = 127
        for key in self.pub_thrusters:
            self.pub_thrusters[key].publish(stop_msg)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CmdController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()