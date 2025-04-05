#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2

class YOLO(Node):
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
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return
        
        results = self.model(frame)
        annotated_frame = results[0].plot()

        cv2.imshow('YOLO Detection', annotated_frame)
        # Publish the processed image
        self.image_publisher.publish(annotated_frame)