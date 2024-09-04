from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(Node(package="tobas_aso_ros", executable="sbus_driver"))
    return ld
