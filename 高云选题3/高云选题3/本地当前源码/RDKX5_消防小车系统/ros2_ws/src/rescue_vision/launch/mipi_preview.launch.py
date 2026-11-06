"""Preview the existing GS130W stream without opening another camera driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(package, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), "launch", filename)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    return LaunchDescription([
        include("hobot_codec", "hobot_codec_encode.launch.py", {
            "codec_in_mode": "ros",
            "codec_in_format": "nv12",
            "codec_out_mode": "ros",
            "codec_out_format": "jpeg",
            "codec_sub_topic": "/image_combine_raw",
            "codec_pub_topic": "/image_combine_jpeg",
            "codec_jpg_quality": "85.0",
        }),
        include("websocket", "websocket.launch.py", {
            "websocket_image_topic": "/image_combine_jpeg",
            "websocket_only_show_image": "True",
            "websocket_channel": "0",
        }),
    ])
