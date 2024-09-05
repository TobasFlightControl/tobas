from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable

from launch_ros.actions import Node

DESCRIPTION_PATH = "description_path"


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(DESCRIPTION_PATH))

    desc_path = LaunchConfiguration(DESCRIPTION_PATH)
    urdf_content = Command([FindExecutable(name="xacro"), " ", desc_path])

    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf_content}],
        )
    )

    return ld
