from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

NAMESPACE = "namespace"


def generate_launch_description():
    pkg_name = "tobas_aso_ros"
    extra_arguments = [{"use_intra_process_comms": True}]

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(NAMESPACE))
    ns = LaunchConfiguration(NAMESPACE)

    ld.add_action(
        LoadComposableNodes(
            target_container=PathJoinSubstitution([ns, "component_manager_high"]),  # target_containerはフルパスのみ
            composable_node_descriptions=[
                ComposableNode(
                    package=pkg_name,
                    plugin="IMUDriverNode",
                    namespace=ns,
                    extra_arguments=extra_arguments,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="SBUSDriverNode",
                    namespace=ns,
                    extra_arguments=extra_arguments,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="DShotDriverNode",
                    namespace=ns,
                    extra_arguments=extra_arguments,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="PWMDriverNode",
                    namespace=ns,
                    extra_arguments=extra_arguments,
                ),
            ],
        )
    )

    ld.add_action(
        LoadComposableNodes(
            target_container=PathJoinSubstitution([ns, "component_manager_medium"]),
            composable_node_descriptions=[
                ComposableNode(
                    package=pkg_name,
                    plugin="MagDriverNode",
                    namespace=ns,
                    extra_arguments=extra_arguments,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="BaroDriverNode",
                    namespace=ns,
                    extra_arguments=extra_arguments,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="GNSSDriverNode",
                    namespace=ns,
                    extra_arguments=extra_arguments,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="ADCDriverNode",
                    namespace=ns,
                    extra_arguments=extra_arguments,
                ),
            ],
        )
    )

    return ld
