from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

# Template parameters
CONFIG_PKG_NAME = "tobas_voliro_like_config"

# Arguments
LOG_LEVEL = "log_level"
OUTPUT = "output"
USE_SIM_TIME = "use_sim_time"
GROUND_TRUTH = "ground_truth"
STANDALONE = "standalone"
MULTIPROCESS = "multiprocess"


def generate_launch_description():
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument(LOG_LEVEL, default_value="info"))
    ld.add_action(DeclareLaunchArgument(OUTPUT, default_value="screen"))
    ld.add_action(DeclareLaunchArgument(USE_SIM_TIME, default_value="false"))
    ld.add_action(DeclareLaunchArgument(GROUND_TRUTH, default_value="false"))
    ld.add_action(DeclareLaunchArgument(STANDALONE, default_value="false"))
    ld.add_action(DeclareLaunchArgument(MULTIPROCESS, default_value="false"))

    # Get arguments
    log_level = LaunchConfiguration(LOG_LEVEL)
    output = LaunchConfiguration(OUTPUT)
    use_sim_time = LaunchConfiguration(USE_SIM_TIME)
    ground_truth = LaunchConfiguration(GROUND_TRUTH)
    standalone = LaunchConfiguration(STANDALONE)
    multiprocess = LaunchConfiguration(MULTIPROCESS)

    config_pkg_share = FindPackageShare(CONFIG_PKG_NAME)

    # Launch realtime nodes
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([config_pkg_share, "launch", "common_realtime_standalone.launch.py"]),
            ),
            launch_arguments={
                "log_level": log_level,
                "output": output,
                "use_sim_time": use_sim_time,
                "ground_truth": ground_truth,
            }.items(),
            condition=IfCondition(standalone),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([config_pkg_share, "launch", "common_realtime_component.launch.py"]),
            ),
            launch_arguments={
                "log_level": log_level,
                "output": output,
                "use_sim_time": use_sim_time,
                "ground_truth": ground_truth,
                "multiprocess": multiprocess,
            }.items(),
            condition=UnlessCondition(standalone),
        )
    )

    # Launch interface nodes
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([config_pkg_share, "launch", "common_interface.launch.py"]),
            ),
            launch_arguments={
                "log_level": log_level,
                "output": output,
                "use_sim_time": use_sim_time,
                "ground_truth": ground_truth,
            }.items(),
        )
    )

    return ld
