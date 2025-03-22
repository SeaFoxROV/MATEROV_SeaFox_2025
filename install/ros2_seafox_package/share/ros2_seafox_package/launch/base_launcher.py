from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='1_joystick_reader',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='joystick_reader',
            output='screen',
        ),
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='2_joystick_to_twist',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='joystick_to_twist',
            output='screen',
        ),
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='twist_to_pwm',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='twist_to_pwm',
            output='screen',
        ),
    ])