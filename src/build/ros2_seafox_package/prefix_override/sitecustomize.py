import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ruben_yh/Desktop/seafox/MATEROV_SeaFox_2025/src/install/ros2_seafox_package'
