from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Launch setup assistant
    ld.add_action(Node(package="tobas_setup_assistant", executable="main"))

    # Launch robot state publisher with minimul URDF
    minimul_urdf = '<robot name="empty"><link name="root"/></robot>'
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": minimul_urdf}],
        )
    )

    return ld
