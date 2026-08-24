import sys
import uuid

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, Shutdown
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node

# Arguments
LOG_LEVEL = "log_level"
OUTPUT = "output"


def generate_launch_description():
    ld = LaunchDescription()

    # Set localhost only.
    set_discovery_range = SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    ld.add_action(set_discovery_range)

    # Add ament prefix path.
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

    # Add python path.
    python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    new_python_path = PathJoinSubstitution([new_ament_prefix_path, "lib", python_version, "site-packages"])
    set_ament_prefix_path = SetEnvironmentVariable(
        name="PYTHONPATH",
        value=[
            new_python_path,
            TextSubstitution(text=":"),
            EnvironmentVariable("PYTHONPATH", default_value=""),
        ],
    )
    ld.add_action(set_ament_prefix_path)

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
    for node_name in [
        "rcl",
        "rclcpp",
        "tf2_ros_message_filter",
        "pluginlib.ClassLoader",
        "rmw_fastrtps_cpp",
        "rmw_cyclonedds_cpp",
    ]:
        ros_args += ["--log-level", f"{node_name}:=WARN"]

    # Launch SSH server.
    run_ssh_server = Node(
        package="tobas_ssh_server",
        executable="ssh_server_node",
        namespace=session_ns,
        ros_arguments=ros_args,
        output=output,
    )
    ld.add_action(run_ssh_server)

    # Launch Tile proxy.
    run_tile_server = Node(
        package="tobas_tile_proxy",
        executable="tile_proxy_node",
        namespace=session_ns,
        ros_arguments=ros_args,
        output=output,
    )
    ld.add_action(run_tile_server)

    # Launch heartbeat sender to monitor network connectivity.
    run_heartbeat_sender = Node(
        package="tobas_connection_monitor",
        executable="heartbeat_sender",
        namespace=session_ns,
        ros_arguments=ros_args,
        output=output,
    )
    ld.add_action(run_heartbeat_sender)

    # Launch ground control station.
    run_gcs = Node(
        package="tobas_gcs",
        executable="TobasGCS",
        namespace=session_ns,
        ros_arguments=ros_args,
        output=output,
        on_exit=Shutdown(),
    )
    ld.add_action(run_gcs)

    return ld
