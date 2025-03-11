import pygame # library of the joystick
import rclpy #ROS 2 library
from rclpy.node import Node #Class to create a ROS 2 node
from std_msgs.msg import Float32MultiArray #Meessage type to publish an array of float numbers

pygame.init() #Initialize the pygame library

class JoystickPublisher(Node): #Node to publish the joystick data
    def __init__(self): #Constructor
        super().__init__('joystick_publisher') #Sets node name
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joystick_data', 10) #Create the publisher, it publishes a Float32MultiArray message type
        self.joysticks = {} #Dictionary to store the joysticks
        pygame.joystick.init() #Initialize the joysticks
        self.init_joysticks() #Initialize the joysticks

    def init_joysticks(self):
        for i in range(pygame.joystick.get_count()): #Number of joysticks connected
            joy = pygame.joystick.Joystick(i) #Get the joystick
            joy.init() #Initialize the joystick
            self.joysticks[joy.get_instance_id()] = joy #Add the joystick to the dictionary
            print(f"Joystick {joy.get_instance_id()} connected") #Confirms joystick is connected

    def publish_joystick_data(self):
        msg = Float32MultiArray() #Create the message
        data = [] #List to store the data
        
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

        msg.data = data #assigns de value to the message
        self.publisher_.publish(msg) #publish the message


def main():
    rclpy.init() #Initialize ROS 2
    node = JoystickPublisher() #Create the node
    clock = pygame.time.Clock() #Create a clock to control the loop
    
    try:
        while rclpy.ok(): #As longs ROS2 is running
            for event in pygame.event.get(): #Listens to the events
                if event.type == pygame.JOYDEVICEADDED: #If a joystick is connected
                    joy = pygame.joystick.Joystick(event.device_index) #Get the joystick
                    joy.init() #Initialize the joystick
                    node.joysticks[joy.get_instance_id()] = joy #Add the joystick to the dictionary
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