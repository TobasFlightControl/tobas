from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="tobas_coding_style_example",
            executable="my_node",
            output="screen",
        )
    )

    return ld
