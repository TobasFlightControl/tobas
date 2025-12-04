from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, Shutdown
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, TextSubstitution, LaunchConfiguration

from launch_ros.actions import Node

# Arguments
LOG_LEVEL = "log_level"
OUTPUT = "output"


def generate_launch_description():
    ld = LaunchDescription()

    # Add ament prefix path
    new_ament_prefix_path = PathJoinSubstitution([EnvironmentVariable("HOME"), ".local/share/tobas/colcon_ws/install"])
    set_ament_prefix_path = SetEnvironmentVariable(
        name="AMENT_PREFIX_PATH",
        value=[
            new_ament_prefix_path,
            TextSubstitution(text=":"),
            EnvironmentVariable("AMENT_PREFIX_PATH", default_value=""),
        ],
    )
    ld.add_action(set_ament_prefix_path)

    # Declare arguments
    ld.add_action(DeclareLaunchArgument(LOG_LEVEL, default_value="info"))
    ld.add_action(DeclareLaunchArgument(OUTPUT, default_value="screen"))

    # Get arguments
    log_level = LaunchConfiguration(LOG_LEVEL)
    output = LaunchConfiguration(OUTPUT)

    ros_args = ["--log-level", log_level]

    # Launch robot state publisher with minimul URDF
    minimul_urdf = '<robot name="empty"><link name="root"/></robot>'
    run_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": minimul_urdf}],
        ros_arguments=ros_args,
        output=output,
    )
    ld.add_action(run_rsp)

    # Launch property server
    run_property_server = Node(
        package="tobas_property_server",
        executable="property_server",
        ros_arguments=ros_args,
        output=output,
        additional_env={"ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"},
    )
    ld.add_action(run_property_server)

    # Launch setup assistant
    run_setup_assistant = Node(
        package="tobas_setup_assistant",
        executable="TobasSetupAssistant",
        ros_arguments=ros_args,
        output=output,
        on_exit=Shutdown(),
    )
    ld.add_action(run_setup_assistant)

    return ld
