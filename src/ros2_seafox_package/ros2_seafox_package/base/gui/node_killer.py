import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class Node_Killer(Node):
    def __init__(self):
        super().__init__('node_killer')
        
        self.create_subscription(
            Bool,
            "measure_node",
            self.kill_node_callback,
            10
        )

        self.change_state_camera_publisher = self.create_client(
            ChangeState,
            '/camera_publisher/change_state'
        )
        while not self.change_state_camera_publisher.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for camera_publisher change_state service...')
        self.configured = False
        

    # def configure_camera_publisher(self):
    #     if not self.configured:
    #         self.configured = True
    #         req = ChangeState.Request()

    #         self.get_logger().info('Configuring CameraPublisher node...')
    #         req.transition.id = Transition.TRANSITION_CONFIGURE
    #         future = self.change_state_camera_publisher.call_async(req)
    #         # rclpy.spin_until_future_complete(self, future)
    #         self.get_logger().info('CameraPublisher node configured!')
            
    #         self.get_logger().info('Activating CameraPublisher node...')
    #         req.transition.id = Transition.TRANSITION_ACTIVATE
    #         future = self.change_state_camera_publisher.call_async(req)
    #         # rclpy.spin_until_future_complete(self, future)
    #         self.get_logger().info('CameraPublisher node activated!')
    
    def kill_node_callback(self, msg):
        self.get_logger().info('Received request to kill CameraPublisher node...')
        if msg.data:
            req = ChangeState.Request()
            #Configuring the CameraPublisher node
            self.get_logger().info('Configuring CameraPublisher node...')
            req.transition.id = Transition.TRANSITION_CONFIGURE
            future = self.change_state_camera_publisher.call_async(req)
            # rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('CameraPublisher node configured!')
            
            # Activating the CameraPublisher node
            self.get_logger().info('Activating CameraPublisher node...')
            req.transition.id = Transition.TRANSITION_ACTIVATE
            future = self.change_state_camera_publisher.call_async(req)
            # rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('CameraPublisher node activated!')

            # Desactivating the CameraPublisher node
            self.get_logger().info('Deactivating CameraPublisher node...')
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_DEACTIVATE
            future = self.change_state_camera_publisher.call_async(req)
            # rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('CameraPublisher node deactivated!')
        else:
            req = ChangeState.Request()
            #Configuring the CameraPublisher node
            self.get_logger().info('Configuring CameraPublisher node...')
            req.transition.id = Transition.TRANSITION_CONFIGURE
            future = self.change_state_camera_publisher.call_async(req)
            # rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('CameraPublisher node configured!')

            # Activating the CameraPublisher node
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVATE
            future = self.change_state_camera_publisher.call_async(req)
            # rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('CameraPublisher node activated!')
            self.configured = True

def main(args=None):
    rclpy.init(args=args)
    node = Node_Killer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()