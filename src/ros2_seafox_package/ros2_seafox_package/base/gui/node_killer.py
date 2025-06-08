from rclpy.node import Node
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
    
    def kill_node_callback(self, msg):
        self.get_logger().info("Received message to kill node")
        if msg.data:
            print("Hola")
        else:
            print("Adios")