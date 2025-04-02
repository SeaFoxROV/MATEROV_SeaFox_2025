import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense')
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        
        try:
            self.pipeline.start(config)
            self.get_logger().info("RealSense iniciada correctamente")
        except Exception as e:
            self.get_logger().error(f"Error iniciando la RealSense: {str(e)}")
            return
        
        self.timer = self.create_timer(0.033, self.capture_frame)  # 30 FPS
    
    def capture_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            return
        
        color_image = np.asanyarray(color_frame.get_data())
        msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.image_publisher.publish(msg)
            
    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
