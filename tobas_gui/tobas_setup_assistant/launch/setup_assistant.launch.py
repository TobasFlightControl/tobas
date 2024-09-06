from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

# Arguments
LOG_LEVEL = "log_level"


def generate_launch_description():
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument(LOG_LEVEL, default_value="info"))

    # Get arguments
    log_level = LaunchConfiguration(LOG_LEVEL)

    # Launch robot state publisher with minimul URDF
    minimul_urdf = '<robot name="empty"><link name="root"/></robot>'
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": minimul_urdf}],
    )
    ld.add_action(rsp)

    # Launch property server
    property_server = Node(
        package="tobas_property_tools",
        executable="property_server",
        name="property_server_gcs",
        arguments=["--ros-args", "--log-level", log_level],
    )
    ld.add_action(property_server)

    # Launch setup assistant
    setup_assistant = Node(
        package="tobas_setup_assistant",
        executable="main",
        arguments=["--ros-args", "--log-level", log_level],
    )
    ld.add_action(setup_assistant)

    # Require setup assistant
    ld.add_action(
        RegisterEventHandler(event_handler=OnProcessExit(target_action=setup_assistant, on_exit=[Shutdown()]))
    )

    return ld
