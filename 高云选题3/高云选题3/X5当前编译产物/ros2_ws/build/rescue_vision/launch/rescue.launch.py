from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root=LaunchConfiguration('workspace_dir')
    def path(*parts):return PathJoinSubstitution([root,*parts])
    def flag(name):return ParameterValue(LaunchConfiguration(name),value_type=bool)
    args=[DeclareLaunchArgument('workspace_dir',description='Absolute path of this delivered workspace'),
          DeclareLaunchArgument('camera_enabled',default_value='false'),
          DeclareLaunchArgument('camera_device',default_value=''),
          DeclareLaunchArgument('capture_width',default_value='1280'),
          DeclareLaunchArgument('capture_height',default_value='480'),
          DeclareLaunchArgument('synchronization_verified',default_value='false'),
          DeclareLaunchArgument('detector_enabled',default_value='false'),
          DeclareLaunchArgument('ocr_enabled',default_value='false'),
          DeclareLaunchArgument('uart_dry_run',default_value='true'),
          DeclareLaunchArgument('uart_port',default_value=''),
          DeclareLaunchArgument('navigation_execute_enabled',default_value='false'),
          DeclareLaunchArgument('navigation_config',default_value=path('config','navigation.json')),
          DeclareLaunchArgument('voice_config',default_value=path('config','voice.json')),
          DeclareLaunchArgument('telemetry_config',default_value=path('config','telemetry.json')),
          DeclareLaunchArgument('stereo_calibration',default_value=path('config','stereo.template.json')),
          DeclareLaunchArgument('mount_config',default_value=path('config','mount.template.json')),
          DeclareLaunchArgument('servo_config',default_value=path('config','servo.template.json')),
          DeclareLaunchArgument('landing_calibration',default_value=''),
          DeclareLaunchArgument('facade_config',default_value=path('config','facade.template.json'))]
    def node(executable,params):return Node(package='rescue_vision',executable=executable,output='screen',parameters=[params])
    return LaunchDescription(args+[
        node('stereo_camera',dict(enabled=flag('camera_enabled'),device=LaunchConfiguration('camera_device'),
             capture_width=ParameterValue(LaunchConfiguration('capture_width'),value_type=int),
             capture_height=ParameterValue(LaunchConfiguration('capture_height'),value_type=int),
             synchronization_verified=flag('synchronization_verified'))),
        node('perception',dict(detector_enabled=flag('detector_enabled'),
             stereo_calibration=LaunchConfiguration('stereo_calibration'),
             model=path('models','yolo','wallpaper_yolo11n_3class_bayese_640_nv12.bin'))),
        node('fire_aim',dict(stereo_calibration=LaunchConfiguration('stereo_calibration'),servo_config=LaunchConfiguration('servo_config'),
             landing_calibration=ParameterValue(LaunchConfiguration('landing_calibration'),value_type=str))),
        node('mission',dict(traffic_config=path('config','traffic.json'),mount_config=LaunchConfiguration('mount_config'))),
        node('fpga_bridge',dict(dry_run=flag('uart_dry_run'),port=LaunchConfiguration('uart_port'))),
        node('scene',dict(ocr_enabled=flag('ocr_enabled'),ocr_models=path('models','ocr'),
             buildings_config=path('config','buildings.json'),facade_config=LaunchConfiguration('facade_config'))),
        node('interaction',dict(buildings_config=path('config','buildings.json'),navigation_execute_enabled=flag('navigation_execute_enabled'))),
        node('navigation',dict(navigation_map=path('config','navigation_map.json'),navigation_config=LaunchConfiguration('navigation_config'))),
        node('voice',dict(workspace_dir=root,voice_config=LaunchConfiguration('voice_config'))),
        node('telemetry',dict(telemetry_config=LaunchConfiguration('telemetry_config'),buildings_config=path('config','buildings.json')))
    ])
