#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()

        # Define los nombres de los tópicos para cada cámara
        self.topic_names = [
            'frontal',
            'apoyo_1',
            'apoyo_2',
            'realsense',

        ]
        self.image_publishers = [self.create_publisher(Image, topic, 10) for topic in self.topic_names]

        # Abre las cámaras
        self.cam_frontal = cv2.VideoCapture('/dev/camaras/frontal')
        self.cam_apoyo1 = cv2.VideoCapture('/dev/camaras/apoyo_1')
        self.cam_apoyo2 = cv2.VideoCapture('/dev/camaras/apoyo_2')
        self.cam_realsense = cv2.VideoCapture('/dev/camaras/realsense')

        # Guarda las cámaras en una lista para fácil manejo
        self.captures = [self.cam_frontal, self.cam_apoyo1, self.cam_apoyo2, self.cam_realsense]

        # Configura parámetros de captura
        for cam in [self.cam_frontal, self.cam_apoyo1, self.cam_apoyo2, self.cam_realsense]:
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cam.set(cv2.CAP_PROP_FPS, 25)
        # self.cam_realsense.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.cam_realsense.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # self.cam_realsense.set(cv2.CAP_PROP_FPS, 30)

        # Verifica si las cámaras se abrieron correctamente
        for i, cam in enumerate(self.captures):
            if not cam.isOpened():
                self.get_logger().error(f"Failed to open camera {self.topic_names[i]}")

        # Subscripción para resetear cámaras
        self.create_subscription(Empty, 'reset_cameras', self.reset_cameras_callback, 10)

        # Temporizador para publicar a 30 Hz
        self.timer = self.create_timer(0.033, self.timer_callback)

        self.get_logger().info("CameraPublisher node has started!")

    def reset_cameras_callback(self, msg):
        self.get_logger().info("Resetting cameras...")

        for cam in self.captures:
            cam.release()

        # Reabrir cámaras
        self.cam_frontal = cv2.VideoCapture('/dev/camaras/frontal')
        self.cam_apoyo1 = cv2.VideoCapture('/dev/camaras/apoyo_1')
        self.cam_apoyo2 = cv2.VideoCapture('/dev/camaras/apoyo_2')
        self.cam_realsense = cv2.VideoCapture('/dev/camaras/realsense')

        self.captures = [self.cam_frontal, self.cam_apoyo1, self.cam_apoyo2, self.cam_realsense]

        for i, cam in enumerate(self.captures):
            if not cam.isOpened():
                self.get_logger().error(f"Failed to reopen camera {self.topic_names[i]}")

    def timer_callback(self):
        for i, cam in enumerate(self.captures):
            ret, frame = cam.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_publishers[i].publish(msg)

    def destroy_node(self):
        for cam in self.captures:
            cam.release()
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
