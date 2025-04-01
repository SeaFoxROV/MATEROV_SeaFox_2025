import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs  # Ensure this module is installed

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('prueba')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self.info_callback, 10)
        self.depth_intrinsics = None

    def info_callback(self, msg: CameraInfo):
        # Convert ROS CameraInfo to a pyrealsense2.intrinsics object.
        self.depth_intrinsics = rs.intrinsics()
        self.depth_intrinsics.width = msg.width
        self.depth_intrinsics.height = msg.height
        self.depth_intrinsics.ppx = msg.K[2]
        self.depth_intrinsics.ppy = msg.K[5]
        self.depth_intrinsics.fx = msg.K[0]
        self.depth_intrinsics.fy = msg.K[4]
        # If your camera model uses distortion, set the model and coefficients accordingly.
        # For example, if no distortion:
        self.depth_intrinsics.model = rs.distortion.none
        self.depth_intrinsics.coeffs = [0, 0, 0, 0, 0]

    def depth_callback(self, msg: Image):
        if self.depth_intrinsics is None:
            # Wait until intrinsics are received from CameraInfo
            return
        # Convert the ROS Image to a NumPy array (using the 'passthrough' encoding if needed)
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Choose a pixel (u, v) you want to deproject
        u, v = 320, 240  # example pixel
        # Extract the depth value (ensure it’s in meters; you might need to apply a scale factor)
        depth_value = float(depth_image[v, u])
        # Use pyrealsense2 to deproject the pixel into a 3D point
        point_3d = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [u, v], depth_value)
        self.get_logger().info(f'3D point at ({u},{v}): {point_3d}')

def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
