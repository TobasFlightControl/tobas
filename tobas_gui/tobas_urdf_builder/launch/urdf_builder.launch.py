from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

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

    ros_args = ["--log-level", log_level]

    # Launch property server
    property_server = Node(
        package="tobas_property_server",
        executable="property_server",
        name="property_server",
        ros_arguments=ros_args,
        output=output,
    )
    ld.add_action(property_server)

    # Launch setup assistant
    setup_assistant = Node(
        package="tobas_urdf_builder",
        executable="main",
        ros_arguments=ros_args,
        output=output,
        on_exit=Shutdown(),
    )
    ld.add_action(setup_assistant)

    return ld
