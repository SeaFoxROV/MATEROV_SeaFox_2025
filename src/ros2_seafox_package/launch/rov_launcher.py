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
            executable='joystick_reader',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='joystick_reader',
            output='screen',
        ),
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='joystick_to_twist',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='joystick_to_twist',
            output='screen',
        ),
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='twist_to_newtons',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='twist_to_newtons',
            output='screen',
        ),
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='newtons_to_pwm',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='newtons_to_pwm',
            output='screen',
        ),
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='rosserial',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='rosserial',
            output='screen',
        ),
        Node(
            package='ros2_seafox_package',  # Reemplaza con el nombre de tu paquete
            executable='yolo',  # Reemplaza con el nombre del ejecutable de tu nodo
            name='yolo',
            output='screen',
        ),
        
    ])