from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='camera_publisher',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='camera_publisher',
            output='screen',
        ),
    
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='realsense',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='realsense',
            output='screen',
        ),
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='camera_gui',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='camera_gui',
            output='screen',
        ),
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='main_gui',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='main_gui',
            output='screen',
        ),
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='imageViewer',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='imageViewer',
            output='screen',
        ),

    ])