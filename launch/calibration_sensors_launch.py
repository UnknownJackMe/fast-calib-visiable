import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    fast_calib_share = get_package_share_directory("fast_calib")

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_calib_share, "launch", "mid360_pointcloud2_launch.py")
        )
    )

    camera = Node(
        package="fast_calib",
        executable="hikvision_image_publisher",
        name="hikvision_image_publisher",
        output="screen",
        parameters=[
            {"serial": LaunchConfiguration("camera_serial")},
            {
                "exposure_us": ParameterValue(
                    LaunchConfiguration("exposure_us"), value_type=float
                )
            },
            {"gain": ParameterValue(LaunchConfiguration("gain"), value_type=float)},
            {
                "publish_rate": ParameterValue(
                    LaunchConfiguration("publish_rate"), value_type=float
                )
            },
            {"topic": LaunchConfiguration("camera_topic")},
            {"frame_id": LaunchConfiguration("camera_frame_id")},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_serial", default_value="DA3217436"),
            DeclareLaunchArgument("exposure_us", default_value="30000.0"),
            DeclareLaunchArgument("gain", default_value="8.0"),
            DeclareLaunchArgument("publish_rate", default_value="1.0"),
            DeclareLaunchArgument("camera_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("camera_frame_id", default_value="hikvision_camera"),
            livox_launch,
            camera,
        ]
    )
