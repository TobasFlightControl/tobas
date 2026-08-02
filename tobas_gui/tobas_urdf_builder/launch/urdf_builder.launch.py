import uuid

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Arguments
LOG_LEVEL = "log_level"
OUTPUT = "output"


def generate_launch_description():
    ld = LaunchDescription()

    # Set localhost only.
    set_discovery_range = SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    ld.add_action(set_discovery_range)

    # Declare arguments.
    ld.add_action(DeclareLaunchArgument(LOG_LEVEL, default_value="info"))
    ld.add_action(DeclareLaunchArgument(OUTPUT, default_value="screen"))

    # Get arguments.
    log_level = LaunchConfiguration(LOG_LEVEL)
    output = LaunchConfiguration(OUTPUT)

    # Create a namespace for this session.
    session_ns = f"session_{uuid.uuid4().hex[:8]}"

    # Set log level.
    ros_args = ["--log-level", log_level]

    # Launch setup assistant.
    setup_assistant = Node(
        package="tobas_urdf_builder",
        executable="TobasURDFBuilder",
        namespace=session_ns,
        ros_arguments=ros_args,
        output=output,
        on_exit=Shutdown(),
    )
    ld.add_action(setup_assistant)

    return ld
