from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_name = "tobas_aso_ros"
    extra_arguments = [{"use_intra_process_comms": True}]

    load_composable_nodes_high = LoadComposableNodes(
        target_container="component_manager_high",
        composable_node_descriptions=[
            ComposableNode(package=pkg_name, plugin="IMUDriverNode", extra_arguments=extra_arguments),
            ComposableNode(package=pkg_name, plugin="SBUSDriverNode", extra_arguments=extra_arguments),
            ComposableNode(package=pkg_name, plugin="DShotDriverNode", extra_arguments=extra_arguments),
            ComposableNode(package=pkg_name, plugin="PWMDriverNode", extra_arguments=extra_arguments),
        ],
    )

    load_composable_nodes_medium = LoadComposableNodes(
        target_container="component_manager_medium",
        composable_node_descriptions=[
            ComposableNode(package=pkg_name, plugin="MagDriverNode", extra_arguments=extra_arguments),
            ComposableNode(package=pkg_name, plugin="BaroDriverNode", extra_arguments=extra_arguments),
            ComposableNode(package=pkg_name, plugin="GNSSDriverNode", extra_arguments=extra_arguments),
            ComposableNode(package=pkg_name, plugin="ADCDriverNode", extra_arguments=extra_arguments),
        ],
    )

    return LaunchDescription([load_composable_nodes_high, load_composable_nodes_medium])
