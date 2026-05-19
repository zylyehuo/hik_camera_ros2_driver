import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory("hik_camera_ros2_driver")
    
    package_prefix = get_package_prefix("hik_camera_ros2_driver")
    gentl_path = os.path.join(package_prefix, "lib")
    
    # 增加对 ThirdParty 目录的路径追踪
    third_party_path = os.path.join(gentl_path, "ThirdParty")

    # 1. 注入 .cti 网口相机驱动探测路径
    gentl_envvar = SetEnvironmentVariable("GENICAM_GENTL64_PATH", gentl_path)
    
    # 2. 将 lib 目录和 lib/ThirdParty 目录一并注入 LD_LIBRARY_PATH
    current_ld_lib = os.environ.get("LD_LIBRARY_PATH", "")
    if current_ld_lib:
        new_ld_lib = f"{gentl_path}:{third_party_path}:{current_ld_lib}"
    else:
        new_ld_lib = f"{gentl_path}:{third_party_path}"
        
    ld_lib_envvar = SetEnvironmentVariable("LD_LIBRARY_PATH", new_ld_lib)

    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "camera_params.yaml"),
        description="The camera configuration file path",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    start_hik_camera_cmd = Node(
        name="hik_camera_ros2_driver",
        package="hik_camera_ros2_driver",
        executable="hik_camera_ros2_driver_node",
        parameters=[
            params_file,
            # 添加下面这两行配置，强行禁用深度插件
            {"image.compressedDepth.disable_pub": True},
            {"front/image.compressedDepth.disable_pub": True} 
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)
    
    # 加载环境变量
    ld.add_action(gentl_envvar)
    ld.add_action(ld_lib_envvar)

    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(start_hik_camera_cmd)

    return ld