from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = Node(
        package="rclcpp_components",
        executable="component_container",
        name="component_manager",
        output="screen",
    )

    load_composable_nodes = LoadComposableNodes(
        target_container="component_manager",
        composable_node_descriptions=[
            ComposableNode(
                package="tobas_ros2_tools",
                plugin="ros2::Talker",
                name="talker",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="tobas_ros2_tools",
                plugin="ros2::Listener",
                name="listener",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    return LaunchDescription([container, load_composable_nodes])
