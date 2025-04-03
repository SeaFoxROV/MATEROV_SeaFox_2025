import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RealSenseNode(Node):
    # Global list to store selected points
    # Each element will be a tuple: (x_pixel, y_pixel, [X, Y, Z] in meters)
    points = []
    def __init__(self):
        super().__init__('realsense')
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) #color stream
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) #depth stream
        
        self.image_publisher = self.create_publisher(Image, 'camera_realsense/image_raw', 10)
        self.bridge = CvBridge()    
        
        try:
            self.pipeline.start(config)
            self.get_logger().info("RealSense iniciada correctamente")
        except Exception as e:
            self.get_logger().error(f"Error iniciando la RealSense: {str(e)}")
            return
        
        self.timer = self.create_timer(0.033, self.capture_frame)  # 30 FPS

        cv2.setMouseCallback('RealSense', self.mouse_callback)

        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # Optionally, draw the selected points on the color image
            for pt in points:
                cv2.circle(color_image, (pt[0], pt[1]), 5, (0, 0, 255), -1)
            
            # Display the color image
            cv2.imshow('RealSense', color_image)
            key = cv2.waitKey(1)
            # Exit on 'Esc' key press
            if key == 27:
                break
    
    def capture_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            return
        
        color_image = np.asanyarray(color_frame.get_data())
        msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.image_publisher.publish(msg)
    
    def mouse_callback(event, x, y, flags, param): #The third and forth variables are for additional parameters
        global points, depth_frame, color_intrinsics
        # On left mouse button click:
        if event == cv2.EVENT_LBUTTONDOWN:
            # If already two points are selected, reset for a new measurement
            if len(points) >= 2:
                points = []
            # Get the depth at the clicked pixel
            depth = depth_frame.get_distance(x, y)
            if depth == 0:
                print(f"No valid depth data at ({x}, {y}). Try another point.")
                return
            # Deproject from pixel coordinates to 3D space
            point_3d = rs.rs2_deproject_pixel_to_point(color_intrinsics, [x, y], depth)
            points.append((x, y, point_3d))
            # If two points are now selected, compute the distance
            if len(points) == 2:
                p1 = np.array(points[0][2])
                p2 = np.array(points[1][2])
                distance = np.linalg.norm(p1 - p2)
                #Minus the average error distance of the camera
                if distance >=1 and distance <= 1.10:
                    distance-3.4
                elif distance >=1.11 and distance <= 1.39:
                    distance-3.8
                elif distance >=1.40 and distance <= 1.59:
                    distance-4
                elif distance >=1.60 and distance <= 1.79:
                    distance-3.4
                elif distance >=1.80 and distance <= 1.99:
                    distance-9
                elif distance >=2 and distance <= 2.10:
                    distance-12.8
                print(f"Distance between points: {distance:.2f} meters")
                print(f"Depth distance: {depth:.2f} meters")
                
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