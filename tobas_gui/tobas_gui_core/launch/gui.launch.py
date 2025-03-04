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

    # Launch robot state publisher with minimul URDF
    minimul_urdf = '<robot name="empty"><link name="root"/></robot>'
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": minimul_urdf}],
            output=output,
        )
    )

    # Launch property server
    ld.add_action(
        Node(
            package="tobas_property_server",
            executable="property_server",
            ros_arguments=ros_args,
            output=output,
        )
    )

    # Launch SSH server
    ld.add_action(
        Node(
            package="tobas_ssh_server",
            executable="ssh_server_node",
            name="ssh_server",
            parameters=[{"host": "tobas.local", "port": 22, "user": "pi", "passwd": "raspberry"}],
            ros_arguments=ros_args,
            output=output,
        )
    )

    # Launch main application
    ld.add_action(
        Node(
            package="tobas_gui_core",
            executable="Tobas",
            ros_arguments=ros_args,
            output=output,
            on_exit=Shutdown(),
        )
    )

    return ld
