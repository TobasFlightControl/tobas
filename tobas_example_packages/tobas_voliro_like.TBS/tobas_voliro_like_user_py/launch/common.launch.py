# Do not delete or rename this file because it is executed in tobas_voliro_like_config/common.launch.py.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Please add the nodes that run on both the actual machine and the Gazebo simulation.

    ld.add_action(
        Node(
            package="tobas_voliro_like_user_py",
            executable="user_node",
            namespace="voliro_like",
        )
    )

    return ld
