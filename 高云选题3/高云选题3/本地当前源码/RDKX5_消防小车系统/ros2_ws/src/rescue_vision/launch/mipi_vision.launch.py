"""Use an already-running raw MIPI stereo camera for 2D YOLO detection.

The official driver owns the cameras. No actuator, UART, or navigation node is
started here. Metric depth stays unavailable until calibration is validated.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = LaunchConfiguration('workspace_dir')
    value = lambda name, kind: ParameterValue(LaunchConfiguration(name), value_type=kind)
    return LaunchDescription([
        DeclareLaunchArgument('workspace_dir', description='Absolute project workspace path'),
        DeclareLaunchArgument('left_topic', default_value='/image_left_raw'),
        DeclareLaunchArgument('right_topic', default_value='/image_right_raw'),
        # 2026-10-10: user covered physical left eye; bottom (right topic) darkened.
        # This is specific to the currently connected channel 2/0 arrangement.
        DeclareLaunchArgument('swap_eyes', default_value='true'),
        DeclareLaunchArgument('process_fps', default_value='30.0'),
        DeclareLaunchArgument('observation_fps', default_value='2.0'),
        DeclareLaunchArgument('fast_mode', default_value='true'),
        DeclareLaunchArgument('pipeline_enabled', default_value='true'),
        DeclareLaunchArgument('raw_subscription', default_value='true'),
        DeclareLaunchArgument('fused_preprocess', default_value='true'),
        DeclareLaunchArgument('preprocess_threads', default_value='2'),
        DeclareLaunchArgument('remap_backend', default_value='native_or_opencv'),
        DeclareLaunchArgument('remap_threads', default_value='4'),
        DeclareLaunchArgument('backend', default_value='native_or_pyeasy'),
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='1088'),
        DeclareLaunchArgument('opencv_threads', default_value='2'),
        DeclareLaunchArgument('jpeg_quality', default_value='90'),
        DeclareLaunchArgument('fire_only', default_value='false'),
        DeclareLaunchArgument('mission_gating', default_value='false'),
        DeclareLaunchArgument('lens_enabled', default_value='true'),
        DeclareLaunchArgument('lens_intrinsics', default_value=PathJoinSubstitution([
            root, 'config', 'gs130w_intrinsics_20261010.json'])),
        DeclareLaunchArgument('lens_alpha', default_value='0.0'),
        DeclareLaunchArgument('detector_enabled', default_value='true'),
        DeclareLaunchArgument('model', default_value=PathJoinSubstitution([
            root, 'models', 'yolo', 'wallpaper_yolo11n_3class_bayese_640_nv12.bin'])),
        Node(package='rescue_vision', executable='mipi_camera', output='screen',
             condition=UnlessCondition(LaunchConfiguration('fast_mode')), parameters=[{
            'left_topic': LaunchConfiguration('left_topic'),
            'right_topic': LaunchConfiguration('right_topic'),
            'swap_eyes': value('swap_eyes', bool),
            'publish_fps': value('process_fps', float),
            'preserve_encoding': True,
            'max_skew_s': 0.012,
            'max_age_s': 0.5,
            'synchronization_verified': False,
            'input_rectified': False,
            'expected_width': 1280,
            'expected_height': 1088,
        }]),
        Node(package='rescue_vision', executable='perception', output='screen',
             condition=UnlessCondition(LaunchConfiguration('fast_mode')), parameters=[{
            'detector_enabled': value('detector_enabled', bool),
            'process_fps': value('process_fps', float),
            'process_on_receive': True,
            'model': LaunchConfiguration('model'),
            'backend': 'pyeasy',
            'fire_only': value('fire_only', bool),
            'lens_enabled': value('lens_enabled', bool),
            'lens_intrinsics': LaunchConfiguration('lens_intrinsics'),
            'lens_alpha': value('lens_alpha', float),
            'lens_swap_eyes': value('swap_eyes', bool),
            'left_source_topic': LaunchConfiguration('left_topic'),
            'right_source_topic': LaunchConfiguration('right_topic'),
            'input_rotation': 90.0,
            'image_width': 640,
            'image_height': 544,
            # Empty means explicitly uncalibrated. Never infer metric depth from
            # an example YAML or rectify an already-rectified input a second time.
            'stereo_calibration': '',
        }]),
        Node(package='rescue_vision', executable='fast_mipi_perception', output='screen',
             condition=IfCondition(LaunchConfiguration('fast_mode')), parameters=[{
            'pipeline_enabled': value('pipeline_enabled', bool),
            'mission_gating': value('mission_gating', bool),
            'raw_subscription': value('raw_subscription', bool),
            'fused_preprocess': value('fused_preprocess', bool),
            'preprocess_threads': value('preprocess_threads', int),
            'remap_backend': LaunchConfiguration('remap_backend'),
            'remap_threads': value('remap_threads', int),
            'detector_enabled': value('detector_enabled', bool),
            'process_fps': value('process_fps', float),
            'model': LaunchConfiguration('model'),
            'backend': LaunchConfiguration('backend'),
            'observation_fps': value('observation_fps', float),
            'fire_only': value('fire_only', bool),
            'lens_enabled': value('lens_enabled', bool),
            'lens_intrinsics': LaunchConfiguration('lens_intrinsics'),
            'lens_alpha': value('lens_alpha', float),
            'swap_eyes': value('swap_eyes', bool),
            'left_source_topic': LaunchConfiguration('left_topic'),
            'right_source_topic': LaunchConfiguration('right_topic'),
            'input_rotation': 90.0,
            'image_width': value('image_width', int),
            'image_height': value('image_height', int),
            'opencv_threads': value('opencv_threads', int),
            'jpeg_quality': value('jpeg_quality', int),
        }]),
    ])
