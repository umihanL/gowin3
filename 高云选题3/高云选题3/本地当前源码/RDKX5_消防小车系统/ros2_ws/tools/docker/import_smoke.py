import importlib
import json
import platform

import cv2
import numpy
import rclpy
import serial

modules = ["camera_node", "perception_node", "fire_aim_node", "mission_node", "fpga_bridge_node", "scene_node"]
for module in modules:
    importlib.import_module("rescue_vision." + module)
importlib.import_module("rescue_interfaces.msg")
rclpy.init()
node = rclpy.create_node("rescue_environment_import_smoke")
node.destroy_node()
rclpy.shutdown()
print(json.dumps({"passed": True, "platform": platform.platform(), "python": platform.python_version(),
                  "ros_distro": "humble", "opencv": cv2.__version__, "numpy": numpy.__version__,
                  "pyserial": serial.__version__, "imported_node_modules": modules,
                  "rclpy_node_create_destroy": "passed", "hardware_accessed": False}, indent=2))
