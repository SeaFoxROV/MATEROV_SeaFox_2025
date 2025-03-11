import rclpy
from rclpy.node import Node
    

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller_node')
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Motion Controller Node has been started')