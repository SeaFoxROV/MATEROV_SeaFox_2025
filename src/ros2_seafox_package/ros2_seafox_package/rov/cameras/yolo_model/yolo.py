#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class ImageViewer(Node):
    def __init__(self):
        super().__init__('imageViewer')
        self.bridge = CvBridge()

        # Tópicos para imágenes normales y procesadas
        self.topic_names = [
            'vcamera_left/image_raw',
            'vcamera_right/image_raw',
            'vcamera_realsense/image_raw'
        ]
        self.subtopic_names = [
            'camera_left/image_raw',
            'camera_right/image_raw',
            'camera_realsense/image_raw'
        ]
        self.yolo_topic_names = [
            'yolocamera_left/image_raw',
            'yolocamera_right/image_raw',
            'camera_yolo/image_raw'
        ]
        
        # Publicadores para imágenes normales
        self.img_publishers = []
        for topic in self.topic_names:
            pub = self.create_publisher(Image, topic, 10)
            self.img_publishers.append(pub)
        
        # Publicadores para imágenes procesadas con YOLO
        self.yolo_publishers = []
        for topic in self.yolo_topic_names:
            pub = self.create_publisher(Image, topic, 10)
            self.yolo_publishers.append(pub)
        
        # Arreglo para almacenar la última imagen recibida de cada cámara
        self.latest_frames = [None] * len(self.topic_names)

        # Suscripción a cada tópico de cámara
        self.subscribers = []
        for i, topic in enumerate(self.subtopic_names):
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, idx=i: self.image_callback(msg, idx),
                10
            )
            self.subscribers.append(sub)
        
        # Suscripción para el modo de selección (se espera un único entero: 0 para normal, 1 para YOLO)
        self.camera_mode = 0
        self.create_subscription(
            Int32MultiArray,
            'mode_selection',
            self.selection_callback,
            10
        )
        
        self.get_logger().info("ImageViewer node has started!")
        self.model = YOLO(r'src/ros2_seafox_package/ros2_seafox_package/rov/cameras/yolo_model/best.pt')
        # Timer que se activa cada 33ms (~30 FPS)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def selection_callback(self, msg):
        # Se espera que msg.data sea una lista con un único entero
        if len(msg.data) == 1:
            self.camera_mode = msg.data[0]
            self.get_logger().info(f"Active mode updated: {self.camera_mode}")
        else:
            self.get_logger().warning("Invalid mode selection received")

    def image_callback(self, msg, idx):
        # Almacena la última imagen recibida para cada cámara
        self.latest_frames[idx] = msg

    def timer_callback(self):
        # Recorre cada cámara y procesa según el modo actual
        for i, frame_msg in enumerate(self.latest_frames):
            if frame_msg is None:
                continue
            try:
                # Conversión de ROS2 a OpenCV
                cv_frame = self.bridge.imgmsg_to_cv2(frame_msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f"Error converting image: {e}")
                continue

            if self.camera_mode == 0:
                # Modo normal: se re-publica la imagen original
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
                except Exception as e:
                    self.get_logger().error(f"Error converting image back to ROS2: {e}")
                    continue
                self.img_publishers[i].publish(img_msg)
            else:
                # Modo YOLO: se procesa la imagen y se publica la imagen anotada
                results = self.model(cv_frame)
                annotated_frame = results[0].plot()
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                except Exception as e:
                    self.get_logger().error(f"Error converting YOLO image back to ROS2: {e}")
                    continue
                self.yolo_publishers[i].publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
