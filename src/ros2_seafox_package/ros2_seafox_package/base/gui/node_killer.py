import rclpy
from rclpy.node import Node
from ros2_seafox_package.rov.cameras.camera_publisher import CameraPublisher
from ros2_seafox_package.rov.cameras.realsense import RealSenseNode
from std_msgs.msg import Bool

class Node_Killer(Node):

    def __init__(self):
        super().__init__('node_killer')
        
        self.create_subscription(
            Bool,
            "measure_node",
            self.kill_node_callback,
            10
        )
        self.node_cameras = CameraPublisher()
        self.node_realsense = RealSenseNode()
        self.node_cameras.start()
    
    def kill_node_callback(self, msg):
        self.get_logger().info("Received message to kill node")
        if msg.data:
            self.node_cameras.destroy_node()
            self.get_logger().info("Node cameras has been killed")
            self.node_realsense.start()
        else:
            self.node_realsense.destroy_node()
            self.get_logger().info("Node realsense has been killed")
            self.node_cameras.start()

def main(args=None):
    rclpy.init(args=args)
    node = Node_Killer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()