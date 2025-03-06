import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

pygame.init()

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joystick_data', 10)
        self.joysticks = {}
        pygame.joystick.init()
        self.init_joysticks()

    def init_joysticks(self):
        # Initialize any connected joysticks.
        for i in range(pygame.joystick.get_count()):
            joy = pygame.joystick.Joystick(i)
            joy.init()
            self.joysticks[joy.get_instance_id()] = joy
            print(f"Joystick {joy.get_instance_id()} connected")

    def publish_joystick_data(self):
        # Read and publish joystick inputs.
        msg = Float32MultiArray()
        data = []
        
        for joystick in self.joysticks.values():
            # Read axes values.
            for i in range(joystick.get_numaxes()):
                data.append(float(joystick.get_axis(i)))
            # Read first 8 button states.
            for i in range(8):
                data.append(float(joystick.get_button(i)))
            # Read hat (D-pad) values.
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
                    if event.instance_id in node.joysticks:
                        del node.joysticks[event.instance_id]
                        print(f"Joystick {event.instance_id} disconnected")
            node.publish_joystick_data()
            # Process any pending ROS callbacks.
            rclpy.spin_once(node, timeout_sec=0)
            clock.tick(30)  # Run at 30 Hz.
    except KeyboardInterrupt:
        pass
    finally:
        pygame.joystick.quit()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
