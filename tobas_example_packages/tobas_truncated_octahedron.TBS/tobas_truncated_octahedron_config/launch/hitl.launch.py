from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Template parameters
DRONE_NAME = "truncated_octahedron"
CONFIG_PKG_NAME = "tobas_truncated_octahedron_config"

# Arguments
LOG_LEVEL = "log_level"
OUTPUT = "output"
USE_SIM_TIME = "use_sim_time"
GROUND_TRUTH = "ground_truth"


def generate_launch_description():
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument(LOG_LEVEL, default_value="info"))
    ld.add_action(DeclareLaunchArgument(OUTPUT, default_value="screen"))
    ld.add_action(DeclareLaunchArgument(USE_SIM_TIME, default_value="true"))
    ld.add_action(DeclareLaunchArgument(GROUND_TRUTH, default_value="false"))

    # Get arguments
    log_level = LaunchConfiguration(LOG_LEVEL)
    output = LaunchConfiguration(OUTPUT)
    use_sim_time = LaunchConfiguration(USE_SIM_TIME)
    ground_truth = LaunchConfiguration(GROUND_TRUTH)

    config_pkg_share = FindPackageShare(CONFIG_PKG_NAME)

    # Launch Tobas core software
    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([config_pkg_share, "launch", "common.launch.py"]),
        ),
        launch_arguments={
            "log_level": log_level,
            "output": output,
            "use_sim_time": use_sim_time,
            "ground_truth": ground_truth,
        }.items(),
    )
    # ld.add_action(common_launch)  # FIXME: 通信遅延と大量の通信に係る負荷が課題

    # Launch nodes for handling RC input
    ld.add_action(
        Node(
            package="tobas_sbus_driver",
            executable="sbus_driver",
            namespace=DRONE_NAME,
            parameters=[{"device": "/dev/ttyAMA0"}],  # TODO: FMUに応じてデバイスを変更
        )
    )
    ld.add_action(Node(package="tobas_real_ros", executable="rcin_handler", namespace=DRONE_NAME))

    return ld
