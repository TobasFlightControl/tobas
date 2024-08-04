from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    node_name = DeclareLaunchArgument("node_name", default_value=TextSubstitution(text="dummy"))

    return LaunchDescription([node_name])
