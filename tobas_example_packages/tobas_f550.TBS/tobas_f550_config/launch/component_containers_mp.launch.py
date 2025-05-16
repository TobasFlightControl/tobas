from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Template parameters
DRONE_NAME = "f550"

# Arguments
LOG_LEVEL = "log_level"
OUTPUT = "output"


def generate_launch_description():
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument(LOG_LEVEL, default_value="info"))
    ld.add_action(DeclareLaunchArgument(OUTPUT, default_value="screen"))

    # Get arguments
    log_level = LaunchConfiguration(LOG_LEVEL)
    output = LaunchConfiguration(OUTPUT)

    # Launch component containers
    for i in range(1, 4):
        ld.add_action(
            Node(
                package="rclcpp_components",
                executable="component_container",
                name=f"component_manager_{i}",
                namespace=DRONE_NAME,
                ros_arguments=["--log-level", log_level],
                output=output,
            )
        )

    return ld
