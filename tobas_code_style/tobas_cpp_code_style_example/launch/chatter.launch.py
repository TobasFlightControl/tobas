from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="rclcpp_components",
            executable="component_container",
            name=f"component_manager",
            output="screen",
        )
    )

    ld.add_action(
        LoadComposableNodes(
            target_container="component_manager",
            composable_node_descriptions=[
                ComposableNode(
                    package="tobas_cpp_code_style_example",
                    plugin="tobas::Talker",
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
        )
    )

    ld.add_action(
        LoadComposableNodes(
            target_container="component_manager",
            composable_node_descriptions=[
                ComposableNode(
                    package="tobas_cpp_code_style_example",
                    plugin="tobas::Listener",
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
        )
    )

    return ld
