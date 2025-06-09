from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='rosserial',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='rosserial',
            output='screen',
        ),
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='node_killer',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='node_killer',
            output='screen',
        ),          

        
    
    ])