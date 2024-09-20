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

    # Launch SSH server
    ssh_server = Node(
        package="tobas_ssh_server",
        executable="ssh_server",
        ros_arguments=ros_args,
        output=output,
    )
    ld.add_action(ssh_server)

    # Launch hardware setup
    hardware_setup = Node(
        package="tobas_hardware_setup",
        executable="main",
        ros_arguments=ros_args,
        output=output,
        on_exit=Shutdown(),
    )
    ld.add_action(hardware_setup)

    return ld
