import os.path as osp
import xacro
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DESCRIPTION_PATH = "description_path"


def launch_setup(context: LaunchContext):
    desc_path = LaunchConfiguration(DESCRIPTION_PATH).perform(context)

    ext = osp.splitext(desc_path)[-1].lower()
    if ext != ".urdf" and ext != ".xacro":
        raise RuntimeError(f"Invalid description format: {ext}")

    urdf = xacro.process_file(desc_path).toprettyxml(indent="\t")

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": urdf}],
    )

    return [rsp_node]


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(DESCRIPTION_PATH))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
