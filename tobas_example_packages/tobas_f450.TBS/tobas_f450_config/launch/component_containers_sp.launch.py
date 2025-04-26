import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessIO

from launch_ros.actions import Node

# Template parameters
DRONE_NAME = "f450"

# Arguments
LOG_LEVEL = "log_level"
OUTPUT = "output"


def handle_output(event):
    # コンポーネントのロードに失敗したらシャットダウン
    output = event.text.decode().strip()
    if re.search(r"Failed to find class with the requested plugin name", output):
        return EmitEvent(event=Shutdown())


def generate_launch_description():
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument(LOG_LEVEL, default_value="info"))
    ld.add_action(DeclareLaunchArgument(OUTPUT, default_value="screen"))

    # Get arguments
    log_level = LaunchConfiguration(LOG_LEVEL)
    output = LaunchConfiguration(OUTPUT)

    # Launch component containers
    component_containers = Node(
        package="tobas_components_rt",
        executable="component_containers",
        namespace=DRONE_NAME,
        ros_arguments=["--log-level", log_level],
        output=output,
    )
    ld.add_action(component_containers)

    # If failed to load a component, shutdown the entire system
    ld.add_action(
        RegisterEventHandler(
            OnProcessIO(
                target_action=component_containers,
                on_stdout=lambda event: handle_output(event),
                on_stderr=lambda event: handle_output(event),
            )
        )
    )

    return ld
