import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robotron/robotron-ros2/src/xarm_ros2/install/uf_ros_lib'
