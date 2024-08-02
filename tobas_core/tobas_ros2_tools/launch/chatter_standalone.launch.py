from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(Node(package="tobas_ros2_tools", executable="talker", name="talker"))
    ld.add_action(Node(package="tobas_ros2_tools", executable="listener", name="listener"))
    return ld
