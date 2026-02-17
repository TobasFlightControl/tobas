from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    timeout = LaunchConfiguration("timeout")

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="8080"),
            DeclareLaunchArgument("timeout", default_value="5.0"),
            Node(
                package="tobas_tile_proxy",
                executable="tile_proxy_node",
                output="screen",
                additional_env={
                    "TILE_PROXY_HOST": host,
                    "TILE_PROXY_PORT": port,
                    "TILE_PROXY_TIMEOUT": timeout,
                },
            ),
        ]
    )
