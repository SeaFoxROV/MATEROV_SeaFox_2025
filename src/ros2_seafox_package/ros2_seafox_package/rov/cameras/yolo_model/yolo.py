#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class Yolo(Node):
    def __init__(self):
        super().__init__('yolo')
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'camera_yolo/image_raw', 10)
        self.image_subscriber = self.create_subscription(
            Image,
            'camera_realsense/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("YOLO node has started!")
        self.model = YOLO(r'src/ros2_seafox_package/ros2_seafox_package/rov/cameras/YOLO_model/best.pt')

    def image_callback(self, msg):
        try:
            # ROS2 to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return
        
        results = self.model(frame)
        annotated_frame = results[0].plot()

        cv2.imshow('YOLO Detection', annotated_frame)
        cv2.waitKey(1)

        try:
            # OpenCV to ROS2
            annotated_frame = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image back to ROS2: {e}")
            return
        # Publish the processed image
        self.image_publisher.publish(annotated_frame)