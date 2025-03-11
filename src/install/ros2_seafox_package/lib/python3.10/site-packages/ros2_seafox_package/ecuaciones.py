import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

pygame.init()

class Ecuaciones(Node):
    def __init__(self):
        super().__init__('ecuaciones_publisher')
        self.subscriber = self.create_subscription(Float32MultiArray, 'joystick_data', 10)

        
        self.publisher_ = self.create_publisher(Float32MultiArray, 'setpoint_motores', 10)


        self.timer = self.create_timer(0.1, self.timer_callback)


def timer_callback(self):
        number = input("Ingrese un número para publicar: ")
        try:
            number = int(number)
            msg = Float32MultiArray()
            msg.data = number
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publicando: {number}')
        except ValueError:
            print("Por favor ingrese un número válido.")



def main():
    rclpy.init()
    node = Ecuaciones() 
    clock = pygame.time.Clock()
    
    try:
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    joy = pygame.joystick.Joystick(event.device_index)
                    joy.init()
                    node.joysticks[joy.get_instance_id()] = joy
                    print(f"Joystick {joy.get_instance_id()} connected")
                elif event.type == pygame.JOYDEVICEREMOVED:
                    del node.joysticks[event.instance_id]
                    print(f"Joystick {event.instance_id} disconnected")
            
            node.publish_joystick_data()
            clock.tick(30)
    
    except KeyboardInterrupt:
        pass
    finally:
        pygame.joystick.quit()
        rclpy.shutdown()


if __name__ == "__main__":
    main()