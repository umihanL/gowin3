from setuptools import find_packages, setup
from glob import glob
from pathlib import Path
web_files=[]
web_root=Path('../../web')
for folder in web_root.rglob('*') if web_root.exists() else []:
    if folder.is_dir():continue
    web_files.append(('share/rescue_vision/web/'+str(folder.parent.relative_to(web_root)),[str(folder)]))
setup(name='rescue_vision',version='0.1.0',packages=find_packages(),
      package_data={'rescue_vision': ['_native_preprocess.so', 'native_preprocess.cpp',
                                     '_native_runtime.so', 'native_runtime.cpp',
                                     '_native_remap.so', 'native_remap.cpp']},
      data_files=[('share/ament_index/resource_index/packages',['resource/rescue_vision']),
                  ('share/rescue_vision',['package.xml']),
                  ('share/rescue_vision/launch',glob('launch/*.py')),
                  ('share/rescue_vision/config',glob('../../config/*.json'))]+web_files,
      install_requires=['setuptools'],zip_safe=True,
      maintainer='Rescue car project',maintainer_email='local@example.invalid',
      description='ROS 2 rescue-car perception and UART control',license='Apache-2.0',
      entry_points={'console_scripts':[
          'stereo_camera = rescue_vision.camera_node:main',
          'mipi_camera = rescue_vision.mipi_camera_node:main',
          'fast_mipi_perception = rescue_vision.fast_mipi_perception_node:main',
          'stereo_depth = rescue_vision.stereo_depth_node:main',
          'road_vision = rescue_vision.road_vision_node:main',
          'fire_targets = rescue_vision.fire_target_node:main',
          'task_interaction = rescue_vision.task_interaction_node:main',
          'extension_task = rescue_vision.extension_task_node:main',
          'task_scene = rescue_vision.task_scene_node:main',
          'mipi_building_ocr = rescue_vision.mipi_building_ocr_node:main',
          'auto_fire = rescue_vision.auto_fire_node:main',
          'fpga_navigation = rescue_vision.fpga_navigation_node:main',
          'perception = rescue_vision.perception_node:main',
          'fire_aim = rescue_vision.fire_aim_node:main',
          'local_actuator = rescue_vision.local_actuator_node:main',
          'pd42_test = rescue_vision.pd42_test_node:main',
          'mission = rescue_vision.mission_node:main',
          'fpga_bridge = rescue_vision.fpga_bridge_node:main',
          'scene = rescue_vision.scene_node:main',
          'interaction = rescue_vision.interaction_node:main',
          'navigation = rescue_vision.navigation_node:main',
          'voice = rescue_vision.voice_node:main',
          'telemetry = rescue_vision.telemetry_node:main']})
