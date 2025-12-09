#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys


class CameraPublisher(Node):
    def __init__(self, camera_index):
        super().__init__('camera_publisher')

        self.bridge = CvBridge()

        self.get_logger().info(f"Opening camera at index {camera_index}")
        self.cap = cv2.VideoCapture(camera_index)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera index {camera_index}")
        else:
            self.get_logger().info(f"Camera {camera_index} opened successfully!")

        # Configuración de la cámara
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 25)

        # Publicador único (tópico realsense)
        self.publisher = self.create_publisher(Image, 'realsense', 10)

        # Timer a ~30 Hz
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        # Asegurar que el frame esté en BGR8
        # Si viene en YUYV (H, W, 2), lo convertimos:
        if len(frame.shape) == 3 and frame.shape[2] == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)


        def destroy_node(self):
            if self.cap.isOpened():
                self.cap.release()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Se espera UN argumento: el índice de la cámara
    if len(sys.argv) < 2:
        print("Uso: camera_publisher <camera_index>")
        return

    cam_index = int(sys.argv[1])

    node = CameraPublisher(cam_index)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
