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
        ('share/ros2_seafox_package/launch', ['launch/base_launcher.py']), 
        ('share/ros2_seafox_package/launch', ['launch/rov_launcher.py']),  
        ('share/ros2_seafox_package/launch', ['launch/camera_launcher.py']),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seafoxinventive',
    maintainer_email='seafoxROV@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        #Base scripts
            #control scripts
                'joystick_reader = ros2_seafox_package.base.control.a_joystick_reader:main',
                'joystick_to_twist = ros2_seafox_package.base.control.b_joystick_to_twist:main',
                'twist_to_newtons = ros2_seafox_package.base.control.c_twist_to_newtons:main',
                'newtons_to_pwm = ros2_seafox_package.base.control.d_newtons_to_pwm:main',
            
            #gui scripts
                'gui_handler = ros2_seafox_package.base.gui.gui_handler:main',
                'node_killer = ros2_seafox_package.base.gui.node_killer:main',

        #ROV scripts
            #cameras scripts
                'camera_publisher = ros2_seafox_package.rov.cameras.camera_publisher:main',
                'realsense = ros2_seafox_package.rov.cameras.realsense:main',
                'imageViewer = ros2_seafox_package.rov.cameras.yolo_model.yolo:main',
                
            #serial
                'rosserial = ros2_seafox_package.rov.serial.rosserial:main',
                'spacebar = ros2_seafox_package.base.control.spacebar:main'

        #'otro_nodo = otro_nodo:main',
        ],
    },
)
