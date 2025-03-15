#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()

        # List of topics for each camera
        self.topic_names = [
            'camera1/image_raw',
            'camera2/image_raw',
            'camera3/image_raw',
            'camera4/image_raw'
        ]
        self.publishers = []
        for topic in self.topic_names:
            pub = self.create_publisher(Image, topic, 10)
            self.publishers.append(pub)

        # Open each camera. Adjust indexes if needed.
        self.captures = []
        for i in range(4):
            cap = cv2.VideoCapture(i)
            if not cap.isOpened():
                self.get_logger().error(f'Camera {i} failed to open.')
            self.captures.append(cap)

        # Publish frames at ~30Hz
        timer_period = 0.033  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        for i, cap in enumerate(self.captures):
            ret, frame = cap.read()
            if ret:
                # Convert the OpenCV image (BGR) to a ROS Image message
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publishers[i].publish(msg)
            else:
                self.get_logger().warning(f"Camera {i} frame not available.")

    def destroy_node(self):
        # Release the camera resources
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
