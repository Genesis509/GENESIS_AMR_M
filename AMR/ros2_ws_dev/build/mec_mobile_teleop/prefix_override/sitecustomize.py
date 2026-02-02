import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/genesis/GENESIS_BOT2_/ros2_ws_dev/install/mec_mobile_teleop'
