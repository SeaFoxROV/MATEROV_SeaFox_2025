import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


pygame.init()
#[leftx,lefty,lefttrigger,rightx,righty,rightrigger,A,B,X,Y,LT,RT,BACK,SELECT,crossx,crossy]
class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joystick_data', 10)
        self.joysticks = {}
        pygame.joystick.init()
        self.init_joysticks()

    def init_joysticks(self):
        for i in range(pygame.joystick.get_count()):
            joy = pygame.joystick.Joystick(i)
            joy.init()
            self.joysticks[joy.get_instance_id()] = joy
            print(f"Joystick {joy.get_instance_id()} connected")

    def publish_joystick_data(self):
        msg = Float32MultiArray()
        data = []
        
        for joystick in self.joysticks.values():
            # Leer valores de los ejes
            for i in range(joystick.get_numaxes()):
                data.append(float(joystick.get_axis(i)))  # Mantener como float
            
            # Leer valores de los botones (0.0 o 1.0)
            for i in range(8):  # Solo botones del 0 al 7
                data.append(float(joystick.get_button(i)))
            
            # Leer valores de los hats (tupla de dos valores)
            for i in range(joystick.get_numhats()):
                hat = joystick.get_hat(i)
                data.extend([float(hat[0]), float(hat[1])])

        msg.data = data
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = JoystickPublisher()
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