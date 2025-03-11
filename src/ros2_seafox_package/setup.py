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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'control = ros2_seafox_package.control:main',
        'motion_controller = ros2_seafox_package.motion_controller:main',
        #'otro_nodo = otro_nodo:main',
        ],
    },
)
