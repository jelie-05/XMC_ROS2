import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/swadiryus/projects/XMC_ROS2/src/install/launch_node'
