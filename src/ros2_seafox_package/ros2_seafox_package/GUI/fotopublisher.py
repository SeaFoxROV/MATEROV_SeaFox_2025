import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_color = self.create_publisher(Image, 'webcam_image_color', 10)
        self.publisher_gray = self.create_publisher(Image, 'webcam_image_gray', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publica cada 0.1 segundos (~10 FPS)
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo acceder a la cámara.")
            exit(1)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No se pudo leer un frame de la cámara.")
            return

        # Publicar imagen a color
        msg_color = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_color.publish(msg_color)

        # Convertir a escala de grises y publicar
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        msg_gray = self.bridge.cv2_to_imgmsg(gray_frame, encoding='mono8')
        self.publisher_gray.publish(msg_gray)

        self.get_logger().info("Imágenes publicadas en 'webcam_image_color' y 'webcam_image_gray'.")

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()  # Liberar la cámara al cerrar el nodo


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo finalizado.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()