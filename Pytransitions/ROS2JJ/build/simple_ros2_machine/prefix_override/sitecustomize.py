import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mvfs/Python_SM/Pytransitions/Drone/ROS2JJ/install/simple_ros2_machine'
