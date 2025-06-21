#!/usr/bin/env python3
"""
ROS2 node that publishes a message each time the spacebar is pressed.

This node configures the terminal in raw mode to capture keystrokes without Enter.
When the space key is detected, it publishes a std_msgs/String with a custom payload.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import sys
import termios
import tty
import threading

class SpacebarPublisher(Node):
    def __init__(self):
        super().__init__('spacebar')
        self.publisher_ = self.create_publisher(Bool, 'space_event', 10)
        self.get_logger().info('Press [spacebar] to publish, [Ctrl] to exit.')

        # Save terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Launch key listener thread
        thread = threading.Thread(target=self.listen_keys, daemon=True)
        thread.start()
        self.maivaxi = False

    def listen_keys(self):
        try:
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == ' ':
                    if self.maivaxi:
                        self.maivaxi = False
                        msg = Bool()
                        msg.data = False
                        self.publisher_.publish(msg)
                        self.get_logger().info('On')
                    else:
                        self.maivaxi = True
                        msg = Bool()
                        msg.data = True
                        self.publisher_.publish(msg)
                        self.get_logger().info('Off')
                    
        except Exception as e:
            self.get_logger().error(f'Error in key listener: {e}')
        finally:
            # Restore on exit
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def destroy_node(self):
        # Restore terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpacebarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()