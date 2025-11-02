import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/spaget/manipal/warehouse/src/ros2slam/install/minibotslam'
