from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='control',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='control',
            output='screen',
        ),
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='motion_controller',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='motion_controller',
            output='screen',
        ),
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='cmd_controller',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='cmd_controller',
            output='screen',
        ),
    ])