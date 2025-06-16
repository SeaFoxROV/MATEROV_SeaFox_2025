#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge
from std_msgs.msg import Int8MultiArray
import cv2

import os
import subprocess

def find_video_index_by_name(name_substring, max_index=10):
    """
    Busca entre /dev/video0 .. /dev/video{max_index-1} aquel dispositivo
    cuyo nombre contenga 'name_substring'. Devuelve la lista de índices que coincidan.
    """
    encontrados = []
    for i in range(max_index):
        dev = f"/dev/video{i}"
        if not os.path.exists(dev):
            continue
        # Ejecutar: v4l2-ctl --device=/dev/video{i} --info    <-- debe estar instalado
        try:
            info = subprocess.check_output(
                ["v4l2-ctl", "--device", dev, "--info"],
                stderr=subprocess.DEVNULL,
                encoding="utf-8"
            )
        except subprocess.CalledProcessError:
            continue

        # Por ejemplo, la línea “Driver name” o “Card type” suele contener algo reconocible
        if name_substring.lower() in info.lower():
            encontrados.append(i)
    return encontrados



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

        self.indice_realsense = max(find_video_index_by_name('realsense'))        
        self.cam_realsense = cv2.VideoCapture(self.indice_realsense)
        
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
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("CameraPublisher node has started!")

    def reset_cameras_callback(self, msg):
        self.get_logger().info("Resetting cameras...")

        for cam in self.captures:
            cam.release()

        # Reabrir cámaras
        self.cam_frontal = cv2.VideoCapture('/dev/camaras/frontal')
        self.cam_apoyo1 = cv2.VideoCapture('/dev/camaras/apoyo_1')
        self.cam_apoyo2 = cv2.VideoCapture('/dev/camaras/apoyo_2')
        # self.cam_realsense = cv2.VideoCapture('/dev/camaras/realsense')

        self.captures = [self.cam_frontal, self.cam_apoyo1, self.cam_apoyo2]

        for i, cam in enumerate(self.captures):
            if not cam.isOpened():
                self.get_logger().error(f"Failed to reopen camera {self.topic_names[i]}")

    def timer_callback(self):
        for i, cam in enumerate(self.captures):
            if self.permission_cameras[i] == 0:
                continue

            if cam is None or not cam.isOpened():
                continue

            ret, frame = cam.read()
            if not ret:
                continue
            
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_publishers[i].publish(msg)

    def permission_callback(self, msg):
        self.permission_cameras = msg.data
        self.get_logger().info(f"permission_callback: {self.permission_cameras}")

        for i in range(len(self.permission_cameras)):
            permiso = self.permission_cameras[i]

            # 1) Si permiso = 0 Y está abierta → liberar
            if permiso == 0 and self.captures[i] is not None and self.captures[i].isOpened():
                self.get_logger().info(f"Camera '{self.topic_names[i]}' disabled → calling release()")
                self.captures[i].release()

            # 2) Si permiso = 1 Y está cerrada (o nunca se abrió) → reabrir
            elif permiso == 1 and (self.captures[i] is None or not self.captures[i].isOpened()):
                self.get_logger().info(f"Camera '{self.topic_names[i]}' enabled → reopening")
                if i == 0:
                    self.cam_frontal = cv2.VideoCapture('/dev/camaras/frontal')
                    self.captures[i] = self.cam_frontal

                elif i == 1:
                    self.cam_apoyo1 = cv2.VideoCapture('/dev/camaras/apoyo_1')
                    self.captures[i] = self.cam_apoyo1

                elif i == 2:
                    self.cam_apoyo2 = cv2.VideoCapture('/dev/camaras/apoyo_2')
                    self.captures[i] = self.cam_apoyo2

                # elif i == 3:
                #     # Asegúrate de que self.indice_realsense fue calculado en __init__
                #     self.cam_realsense = cv2.VideoCapture(self.indice_realsense)
                #     self.captures[i] = self.cam_realsense

                # Si reabriste con éxito, fija ancho/alto/FPS
                cam_nueva = self.captures[i]
                if cam_nueva is not None and cam_nueva.isOpened():
                    cam_nueva.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
                    cam_nueva.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    cam_nueva.set(cv2.CAP_PROP_FPS,         25)
                else:
                    self.get_logger().error(f"Failed to reopen camera '{self.topic_names[i]}'")

            # 3) En cualquier otro caso (permiso=0 y ya estaba cerrada, o permiso=1 y ya estaba abierta),
            #    NO hago nada.


    def destroy_node(self):
        for cam in self.captures:
            cam.release()
        super().destroy_node()

    # def create_node(self,args=None):
    #     rclpy.init(args=args)
    #     try:
    #         rclpy.spin(self)
    #     except KeyboardInterrupt:
    #         pass
    #     finally:
    #         self.destroy_node()
    #         # rclpy.shutdown()

# if __name__ == '__main__':
#     main()
