#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()
        self.topic_names = [
            'camera1/image_raw',
            'camera2/image_raw',
            'camera3/image_raw',
            'camera4/image_raw'
        ]
        self.image_publishers = [self.create_publisher(Image, topic, 10) for topic in self.topic_names]

        # Open camera devices (adjust indexes as needed)
        self.captures = [cv2.VideoCapture(i) for i in range(0,7, 2)] 
        
        for i, cap in enumerate(self.captures):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            if not cap.isOpened():
                self.get_logger().error(f"Failed to open camera {i}")
            else:
                self.get_logger().info(f"Camera {i} opened successfully.")

        # Timer to publish frames at roughly 30Hz
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("CameraPublisher node has started!")

    def timer_callback(self):
        for i, cap in enumerate(self.captures):
            ret, frame = cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_publishers[i].publish(msg)
            else:
                self.get_logger().warning(f"Camera {i} frame not available.")

    def destroy_node(self):
        for cap in self.captures:
            cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
