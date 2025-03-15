# camera_subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()
        self.topics = [
            'camera1/image_raw',
            'camera2/image_raw',
            'camera3/image_raw',
            'camera4/image_raw'
        ]
        # Dictionary to store the latest image from each topic
        self.latest_images = {topic: None for topic in self.topics}

        # Create a subscriber for each camera topic
        for topic in self.topics:
            self.create_subscription(Image, topic, self._make_callback(topic), 10)

    def _make_callback(self, topic):
        def callback(msg):
            try:
                # Convert the ROS image to an OpenCV image (BGR format)
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.latest_images[topic] = cv_image
            except Exception as e:
                self.get_logger().error(f"Error converting image on {topic}: {e}")
        return callback
