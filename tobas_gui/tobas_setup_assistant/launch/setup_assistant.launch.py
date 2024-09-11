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

    # Launch robot state publisher with minimul URDF
    minimul_urdf = '<robot name="empty"><link name="root"/></robot>'
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": minimul_urdf}],
        ros_arguments=ros_args,
        output=output,
    )
    ld.add_action(rsp)

    # Launch property server
    property_server = Node(
        package="tobas_property_tools",
        executable="property_server",
        name="property_server_gcs",
        ros_arguments=ros_args,
        output=output,
    )
    ld.add_action(property_server)

    # Launch setup assistant
    setup_assistant = Node(
        package="tobas_setup_assistant",
        executable="main",
        ros_arguments=ros_args,
        output=output,
        on_exit=Shutdown(),
    )
    ld.add_action(setup_assistant)

    return ld
