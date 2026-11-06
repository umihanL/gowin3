import sys
if sys.prefix == '/root/rescue/ros2_ws/.venv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/rescue/system_20261010/ros2_ws/install/rescue_vision'
