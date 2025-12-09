from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de rosserial
        Node(
            package='ros2_seafox_package',
            executable='rosserial',
            name='rosserial',
            output='screen',
        ),

        # Nodo de cámara con argumento para elegir índice
        Node(
            package='ros2_seafox_package',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            arguments=['4'],   # <<< AQUÍ eliges la cámara (/dev/video4)
        ),
    ])
