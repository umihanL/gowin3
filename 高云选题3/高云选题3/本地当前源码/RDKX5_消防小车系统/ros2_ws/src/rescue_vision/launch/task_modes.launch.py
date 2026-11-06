"""Four task profiles reusing the running MIPI/fast YOLO pipeline."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = LaunchConfiguration('workspace_dir')
    path = lambda *parts: PathJoinSubstitution([root, *parts])
    flag = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    # Include propagates hardware/calibration arguments to the existing controller.
    include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        path('src','rescue_vision','launch','local_fire_control.launch.py')),
        launch_arguments={'workspace_dir': root, 'task_profiles_enabled':'true',
                          'navigation_owner':'fpga','initial_mode':LaunchConfiguration('task_initial_mode'),
                          'extension_enabled':'true','uart_dry_run':'true',
                          'maneuver_enabled':'false',
                          'manual_motion_enabled':LaunchConfiguration('manual_motion_enabled'),
                          'manual_pump_test_enabled':LaunchConfiguration('manual_pump_test_enabled'),
                          'arrival_policy':LaunchConfiguration('arrival_policy'),
                          'fixed_camera_observation':LaunchConfiguration('fixed_camera_observation'),
                          'traffic_config':path('config','traffic.fpga_navigation.json'),
                          'require_target_selection':'true'}.items())
    def node(name, params):
        # FPGA pose/STOP/OFF deadlines must not sit behind the continuous image
        # workers. Ordinary CFS priority only, matching the actuator strategy;
        # no real-time scheduling and no changes to timeouts or processing rates.
        priority = {'prefix': 'nice -n -5'} if name == 'extension_task' else {}
        return Node(package='rescue_vision', executable=name, output='screen', parameters=[params], **priority)
    return LaunchDescription([
        DeclareLaunchArgument('workspace_dir'),
        DeclareLaunchArgument('task_initial_mode', default_value='inspect'),
        DeclareLaunchArgument('extension_config', default_value=path('config','extension_uart.json')),
        DeclareLaunchArgument('fixed_camera_observation', default_value='false'),
        DeclareLaunchArgument('road_raw_input_topic', default_value='/image_right_raw'),
        DeclareLaunchArgument('road_process_on_receive', default_value='false'),
        DeclareLaunchArgument('navigation_execute_enabled', default_value='true'),
        DeclareLaunchArgument('manual_motion_enabled', default_value='false'),
        DeclareLaunchArgument('manual_pump_test_enabled', default_value='false'),
        DeclareLaunchArgument('arrival_policy', default_value='manual'),
        DeclareLaunchArgument('ocr_enabled', default_value='true'),
        DeclareLaunchArgument('ocr_corrected_input', default_value='true'),
        DeclareLaunchArgument('ocr_fixed_camera_preview', default_value='false'),
        DeclareLaunchArgument('voice_config', default_value=path('config','voice.local.json')),
        DeclareLaunchArgument('road_config', default_value=path('config','road_vision.template.json')),
        DeclareLaunchArgument('edge_dialogue_config', default_value=path('config','edge_dialogue.template.json')),
        include,
        node('extension_task',dict(extension_config=LaunchConfiguration('extension_config'),
             route_config=path('config','extension_map.json'))),
        node('road_vision', dict(road_config=LaunchConfiguration('road_config'),
             traffic_config=path('config','traffic.fpga_navigation.json'), source_topic='/image_right_raw',
             raw_input_topic=LaunchConfiguration('road_raw_input_topic'),
             process_on_receive=flag('road_process_on_receive'),
             fixed_camera_observation=flag('fixed_camera_observation'))),
        node('fire_targets', {}),
        node('auto_fire', dict(fixed_camera_observation=flag('fixed_camera_observation'))),
        # Read-only until an explicit bench Connect request. Does not enable
        # the production gimbal or open either motor UART during startup.
        node('pd42_test', {}),
        node('mipi_building_ocr', dict(buildings_config=path('config','buildings.json'),
             ocr_models=path('models','ocr'),lens_intrinsics=LaunchConfiguration('lens_intrinsics'),
             source_topic='/image_right_raw',ocr_enabled=flag('ocr_enabled'),ocr_threads=1,
             corrected_input=flag('ocr_corrected_input'),
             ocr_interval_s=1.0,max_result_age_s=3.0,require_settled_pose=True,
             fixed_camera_preview=flag('ocr_fixed_camera_preview'))),
        node('task_interaction', dict(buildings_config=path('config','buildings.json'),
             navigation_execute_enabled=flag('navigation_execute_enabled'), pump_lease_s=5.,
             manual_motion_enabled=flag('manual_motion_enabled'),extension_enabled=True)),
        node('task_scene', dict(edge_dialogue_config=LaunchConfiguration('edge_dialogue_config'),extension_enabled=True)),
        node('voice', dict(workspace_dir=root, task_commands=True, voice_config=LaunchConfiguration('voice_config'),extension_enabled=True)),
        # extension_task owns the real fixed-10-byte FPGA navigation state.
        # The older CRC/building navigator would overwrite /navigation/state.
    ])
