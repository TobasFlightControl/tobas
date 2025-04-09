from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Template parameters
DRONE_NAME = "f450"
CONFIG_PKG_NAME = "tobas_f450_config"

# Arguments
LOG_LEVEL = "log_level"
OUTPUT = "output"
USE_SIM_TIME = "use_sim_time"
USER_DEBUG = "user_debug"


def generate_launch_description():
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument(LOG_LEVEL, default_value="info"))
    ld.add_action(DeclareLaunchArgument(OUTPUT, default_value="screen"))
    ld.add_action(DeclareLaunchArgument(USE_SIM_TIME, default_value="false"))
    ld.add_action(DeclareLaunchArgument(USER_DEBUG, default_value="false"))

    # Get arguments
    log_level = LaunchConfiguration(LOG_LEVEL)
    output = LaunchConfiguration(OUTPUT)
    use_sim_time = LaunchConfiguration(USE_SIM_TIME)
    user_debug = LaunchConfiguration(USER_DEBUG)

    urdf_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([FindPackageShare(CONFIG_PKG_NAME), "urdf", "drone.xacro"]),
            f" DEBUG:=",
            user_debug,
        ]
    )

    ld.add_action(
        Node(
            namespace=DRONE_NAME,
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf_content, "use_sim_time": use_sim_time}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ros_arguments=["--log-level", log_level],
            output=output,
        )
    )

    return ld
