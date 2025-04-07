#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()

        # Define two topics: left and right camera frames
        self.topic_names = [
            'camera_left/image_raw',
            'camera_right/image_raw'
        ]
        self.image_publishers = [self.create_publisher(Image, topic, 10) for topic in self.topic_names]

        # Define the physical camera devices available
        # (Here we assume you have 4 physical cameras at these device indexes)
        cameras_index = [0, 8, 3, 6]  # adjust as needed
        self.captures = [cv2.VideoCapture(i) for i in cameras_index]
        for i, cap in enumerate(self.captures):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            if not cap.isOpened():
                print()
                #self.get_logger().error(f"Failed to open camera {i}")
            else:
                print()
                #self.get_logger().info(f"Camera {i} opened successfully.")

        # Default active cameras for left and right (using indexes in the cameras_index list)
        self.active_indexes = [0, 1]
        
        # Subscription to receive new camera selections from the GUI (or another node)
        self.selection_sub = self.create_subscription(
            Int32MultiArray,
            'camera_selection',
            self.selection_callback,
            10
        )

        # Timer to publish frames at ~30Hz from only the active cameras
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("CameraPublisher node has started!")

    def selection_callback(self, msg):
        # Expecting msg.data to be a list of two integers representing the camera indexes (0 to 3)
        if len(msg.data) == 2:


            self.active_indexes = list(msg.data)

            

            self.get_logger().info(f"Active camera indexes updated: {self.active_indexes}")
        else:
            print()
            #self.get_logger().warning("Received invalid camera selection (expected 2 indexes).")

    def timer_callback(self):
        # For left (publisher index 0) and right (publisher index 1) cameras:
        for pub_idx, cam_idx in enumerate(self.active_indexes):
            if cam_idx < len(self.captures):
                ret, frame = self.captures[cam_idx].read()
                if ret:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    self.image_publishers[pub_idx].publish(msg)
                else:
                    print()
                    #self.get_logger().warning(f"Camera {cam_idx} frame not available.")
            else:
                
                self.get_logger().warning(f"Invalid camera index: {cam_idx}")

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
