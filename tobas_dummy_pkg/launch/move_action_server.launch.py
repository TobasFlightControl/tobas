from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    ground_truth = DeclareLaunchArgument("ground_truth", default_value=TextSubstitution(text="false"))

    return LaunchDescription([ground_truth])
