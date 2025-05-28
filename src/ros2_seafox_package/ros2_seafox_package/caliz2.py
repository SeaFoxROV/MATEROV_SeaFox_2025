#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()

        # Diccionario para almacenar imágenes por tópico
        self.frames = {
            'frontal': None,
            'apoyo_1': None,
            'apoyo_2': None
        }

        # Suscribirse a los tópicos de las cámaras
        self.create_subscription(Image, 'frontal', self.callback_frontal, 10)
        self.create_subscription(Image, 'apoyo_1', self.callback_apoyo1, 10)
        self.create_subscription(Image, 'apoyo_2', self.callback_apoyo2, 10)

        # Temporizador para mostrar imágenes
        self.create_timer(0.03, self.display_frames)  # ~30 fps

        self.get_logger().info("CameraViewer node has started!")

    def callback_frontal(self, msg):
        self.frames['frontal'] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def callback_apoyo1(self, msg):
        self.frames['apoyo_1'] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def callback_apoyo2(self, msg):
        self.frames['apoyo_2'] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def display_frames(self):
        for name, frame in self.frames.items():
            if frame is not None:
                cv2.imshow(name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    viewer = CameraViewer()
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
