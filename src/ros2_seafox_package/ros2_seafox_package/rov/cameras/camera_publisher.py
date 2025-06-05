#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge
from std_msgs.msg import Int8MultiArray
import cv2

import os
import re

def video_index_from_symlink(syspath):
    """
    Dado el path de un symlink en /dev (p. ej. '/dev/camaras/apoyo_1'),
    devuelve el índice entero X si apunta a '/dev/videoX'. Si no es un
    enlace válido o no apunta a /dev/video*, devuelve None.
    """
    # 1) Primero convierte el symlink en su ruta real:
    try:
        real = os.path.realpath(syspath)  # e.g. '/dev/video2'
    except Exception:
        return None

    # 2) Extrae el número final, si coincide con '/dev/video(\d+)'
    m = re.match(r".*/video(\d+)$", real)
    if m:
        return int(m.group(1))
    return None



class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()

        # Define los nombres de los tópicos para cada cámara
        self.topic_names = [
            'frontal',
            'apoyo_1',
            'apoyo_2',
            'realsense'
        ]
        self.image_publishers = [self.create_publisher(Image, topic, 10) for topic in self.topic_names]

        self.permission_cameras = [1] * len(self.topic_names)

        self.subscription_permission_cameras = self.create_subscription(
            Int8MultiArray,
            'video_permission',
            self.permission_callback,
            10
        )

        # Abre las cámaras
        self.cam_frontal = cv2.VideoCapture('/dev/camaras/frontal')
        self.cam_apoyo1 = cv2.VideoCapture('/dev/camaras/apoyo_1')
        self.cam_apoyo2 = cv2.VideoCapture('/dev/camaras/apoyo_2')

        self.indice_realsense = max([5,4,3,2,1,0].pop(video_index_from_symlink('/dev/camaras/frontal')).pop(video_index_from_symlink('/dev/camaras/apoyo_1')).pop(video_index_from_symlink('/dev/camaras/apoyo_2')))
        
        self.cam_realsense = cv2.VideoCapture(self.indice_realsense)
        self.get_logger().warn("Realsense camera not found")
        
        if not self.cam_realsense.isOpened():
            self.get_logger().warn("Realsense camera not found")
        else:
            self.get_logger().info("Realsense camera found and opened successfully")

        # Guarda las cámaras en una lista para fácil manejo
        self.captures = [self.cam_frontal, self.cam_apoyo1, self.cam_apoyo2, self.cam_realsense]

        # Configura parámetros de captura
        for cam in self.captures:
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cam.set(cv2.CAP_PROP_FPS, 25)

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

    def permission_callback(self, msg):
        self.permission_cameras = msg.data
        self.get_logger().info("Data received")
        for i in range(len(self.permission_cameras)):
            if self.permission_cameras[i] == 0 and self.captures[i].isOpened():
                self.get_logger().info("Camera disabled")
                self.captures[i].release()
            else:
                if not self.captures[i].isOpened():
                    self.get_logger().info("Camera enabled")
                    if i == 0:
                        self.cam_frontal = cv2.VideoCapture('/dev/camaras/frontal')                
                        self.captures[i] = self.cam_frontal
                    elif i == 1:
                        self.cam_apoyo1 = cv2.VideoCapture('/dev/camaras/apoyo_1')
                        self.captures[i] = self.cam_apoyo1
                    elif i == 2:
                        self.cam_apoyo2 = cv2.VideoCapture('/dev/camaras/apoyo_2')
                        self.captures[i] = self.cam_apoyo2
                    elif i == 3:
                        self.cam_realsense = cv2.VideoCapture(self.indice_realsense)  
                        self.captures[i] = self.cam_realsense
                    
                    if self.captures[i].isOpened():
                        self.captures[i].set(cv2.CAP_PROP_FRAME_WIDTH,  640)
                        self.captures[i].set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        self.captures[i].set(cv2.CAP_PROP_FPS,         25)
                    else:
                        self.get_logger().error(f"Failed to reopen camera {self.topic_names[i]}")

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
