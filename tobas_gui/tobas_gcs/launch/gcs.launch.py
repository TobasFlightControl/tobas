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

    # Set log level
    ros_args = ["--log-level", log_level]
    for node_name in [
        "rcl",
        "rclcpp",
        "tf2_ros_message_filter",
        "pluginlib.ClassLoader",
        "rmw_fastrtps_cpp",
        "rmw_cyclonedds_cpp",
    ]:
        ros_args += ["--log-level", f"{node_name}:=WARN"]

    # Launch property server
    ld.add_action(
        Node(
            package="tobas_property_server",
            executable="property_server",
            ros_arguments=ros_args,
            output=output,
            additional_env={"ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"},
        )
    )

    # Launch SSH server
    ld.add_action(
        Node(
            package="tobas_ssh_server",
            executable="ssh_server_node",
            ros_arguments=ros_args,
            output=output,
            additional_env={"ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"},
        )
    )

    # Launch main application
    ld.add_action(
        Node(
            package="tobas_gcs",
            executable="TobasGCS",
            ros_arguments=ros_args,
            output=output,
            on_exit=Shutdown(),
        )
    )

    return ld
