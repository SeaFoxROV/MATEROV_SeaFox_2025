import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cedricmtz/Documents/SeaFox/SEAFOX/MATEROV_SeaFox_2025/src/install/ros2_seafox_package'
