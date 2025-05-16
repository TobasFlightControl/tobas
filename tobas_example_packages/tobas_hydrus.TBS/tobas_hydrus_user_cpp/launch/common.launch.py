# Do not delete or rename this file because it is executed in tobas_hydrus_config/common.launch.py.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Please add the nodes that run on both the actual machine and the Gazebo simulation.

    ld.add_action(
        Node(
            package="tobas_hydrus_user_cpp",
            executable="user_node",
            namespace="hydrus",
        )
    )

    return ld
