from setuptools import find_packages, setup

package_name = 'ros2_seafox_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/ros2_seafox_package/launch', ['launch/base_launcher.py']),  # Agregar esta línea
        #('share/ros2_seafox_package/launch', ['launch/rov_launcher.py']),  # Agregar esta línea
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seafoxinventive',
    maintainer_email='seafoxROV@gmail.com',
    description='SeafoxInventive 2025 MATEROV Competition',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'control = ros2_seafox_package.control:main',
        'rov_control = ros2_seafox_package.rov_control:main',
        'setpoint_controller = ros2_seafox_package.setpoint_controller:main',
        'quaternion_to_euler = ros2_seafox_package.quaternion:main',
        'cmd_controller = ros2_seafox_package.cmd_controller:main',
        
        #'otro_nodo = otro_nodo:main',
        ],
    },
)
