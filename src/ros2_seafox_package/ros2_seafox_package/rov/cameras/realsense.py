#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Empty



class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense')
        
        self.points = []  # Lista global para almacenar puntos seleccionados
        self.pipeline = rs.pipeline()
        config = rs.config()
    

        self.image_publisher = self.create_publisher(Image, 'camera_realsense/image_raw', 10)
        self.distance_publisher = self.create_publisher(Float32, 'distance_point', 10)
        self.bridge = CvBridge()

        # Suscripción para recibir la posición del pixel desde la GUI
        self.pixel_pos = self.create_subscription(
            Int32MultiArray,
            'pixel_position',
            self.pixelpos,
            10
        )

        # # Add subscriber for resetting the RealSense camera
        # self.create_subscription(Empty, 'reset_cameras', self.reset_realsense_callback, 10)


        try:
            config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 1920, 1080, rs.format.z16, 30)
            self.pipeline.start(config)
            for _ in range(5):
                self.pipeline.wait_for_frames(timeout_ms=2000)            
            self.get_logger().info("RealSense iniciada correctamente")
        except Exception as e:
            self.get_logger().error(f"Error iniciando la RealSense: {str(e)}")
            return

        # Timer para capturar frames a ~30 FPS
        self.timer = self.create_timer(0.1, self.capture_frame)

    def reset_realsense_callback(self, msg):
        self.get_logger().info("Resetting RealSense camera...")
        try:
            # Stop and restart the pipeline using the same configuration.
            self.pipeline.stop()
            config = rs.config()
            config.enable_stream(rs.stream.color, 1200, 960, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 1200, 960, rs.format.z16, 30)
            self.pipeline.start(config)
            self.get_logger().info("RealSense reset successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to reset RealSense camera: {e}")


    def pixelpos(self, pos):
        # Se recibe el mensaje con la posición del pixel desde la GUI
        x = pos.data[0]
        y = pos.data[1]
        self.get_logger().info(f"Posición recibida: x={x}, y={y}")

        # Si ya hay dos puntos, reiniciamos para una nueva medición
        if len(self.points) >= 2:
            self.points = []

        # Verificamos que se disponga del frame de profundidad
        if not hasattr(self, 'depth_frame') or self.depth_frame is None:
            self.get_logger().warn("No hay frame de profundidad disponible")
            return

        # Obtener la distancia en el pixel
        depth = self.depth_frame.get_distance(x, y)
        if depth == 0:
            self.get_logger().warn(f"No hay datos de profundidad válidos en ({x}, {y}). Intenta otro punto.")
            return

        # Deproyectar el pixel a coordenadas 3D usando los intrínsecos del sensor de color
        point_3d = rs.rs2_deproject_pixel_to_point(self.color_intrinsics, [x, y], depth)
        self.points.append((x, y, point_3d))
        self.get_logger().info(f"Punto agregado. Total puntos: {len(self.points)}")

        if len(self.points) == 2:
            p1 = np.array(self.points[0][2])
            p2 = np.array(self.points[1][2])
            distance = np.linalg.norm(p1 - p2)

            # Ajuste de distancia
            if 1 <= distance <= 1.10:
                distance = distance - 3.4
            elif 1.11 <= distance <= 1.39:
                distance = distance - 3.8
            elif 1.40 <= distance <= 1.59:
                distance = distance - 4
            elif 1.60 <= distance <= 1.79:
                distance = distance - 3.4
            elif 1.80 <= distance <= 1.99:
                distance = distance - 9
            elif 2 <= distance <= 2.10:
                distance = distance - 12.8

            # Convertir a mensaje Float32 y publicarlo
            msg = Float32()
            msg.data = float(distance)
            self.distance_publisher.publish(msg)
            self.get_logger().info(f"Distance between points: {distance:.2f} meters")
    def capture_frame(self):
        # Esperar y obtener frames de profundidad y color
        frames = self.pipeline.wait_for_frames()
        self.depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not self.depth_frame or not color_frame:
            return

        # Actualizar los intrínsecos del sensor de color
        color_profile = color_frame.get_profile()
        self.color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()

        # Convertir el frame de color a un arreglo de NumPy
        color_image = np.asanyarray(color_frame.get_data())

        # Dibujar los puntos seleccionados sobre la imagen (opcional)
        for pt in self.points:
            cv2.circle(color_image, (pt[0], pt[1]), 5, (0, 0, 255), -1)

        # Publicar la imagen
        msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.image_publisher.publish(msg)
        #self.get_logger().info(f"Tiempo de captura: {b-a} segundos")


    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
