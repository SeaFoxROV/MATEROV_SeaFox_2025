from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    # Declarar argumento camera_index (default = 0)
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Índice de la cámara (video index)'
    )

    # Crear objeto LaunchConfiguration
    camera_index = LaunchConfiguration('camera_index')

    return LaunchDescription([
        camera_index_arg,

        Node(
            package='ros2_seafox_package',
            executable='rosserial',
            name='rosserial',
            output='screen',
        ),

        Node(
            package='ros2_seafox_package',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            arguments=[camera_index],   # <-- AQUÍ ya funciona
        ),
    ])
