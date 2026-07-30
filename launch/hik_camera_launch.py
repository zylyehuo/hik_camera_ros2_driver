import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("hik_camera_ros2_driver")
    package_prefix = get_package_prefix("hik_camera_ros2_driver")

    gentl_path = os.path.join(package_prefix, "lib")
    third_party_path = os.path.join(gentl_path, "ThirdParty")

    current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    library_paths = [gentl_path, third_party_path]
    if current_ld_library_path:
        library_paths.append(current_ld_library_path)

    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            SetEnvironmentVariable("GENICAM_GENTL64_PATH", gentl_path),
            SetEnvironmentVariable("LD_LIBRARY_PATH", ":".join(library_paths)),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    package_share, "config", "camera_params.yaml"
                ),
                description="Camera parameter YAML path",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level",
            ),
            Node(
                name="hik_camera_ros2_driver",
                package="hik_camera_ros2_driver",
                executable="hik_camera_ros2_driver_node",
                parameters=[
                    params_file,
                    {"image.compressedDepth.disable_pub": True},
                    {"front/image.compressedDepth.disable_pub": True},
                ],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
        ]
    )
