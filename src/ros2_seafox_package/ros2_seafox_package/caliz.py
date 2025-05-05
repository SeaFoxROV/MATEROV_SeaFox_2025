#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Caliz(Node):
    def __init__(self):
        super().__init__('caliz_node')
        self.bridge = CvBridge()

        # Suscripción al flujo YOLO de la cámara izquierda
        self.subscription = self.create_subscription(
            Image,
            'camera_left/image_raw',  # Ajusta si es otro flujo
            self.yolo_callback,
            10
        )

        self.get_logger().info("Caliz node is subscribed to YOLO output!")

    def yolo_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Aquí puedes aplicar análisis o visualización
            cv2.imshow("YOLO Output", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing YOLO image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Caliz()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
