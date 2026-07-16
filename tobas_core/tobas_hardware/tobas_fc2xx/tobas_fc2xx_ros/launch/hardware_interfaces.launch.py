from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

NAMESPACE = "namespace"


def generate_launch_description():
    pkg_name = "tobas_fc2xx_ros"

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(NAMESPACE))
    ns = LaunchConfiguration(NAMESPACE)

    # Launch 1st priority nodes (Twist Control).
    ld.add_action(
        LoadComposableNodes(
            target_container=PathJoinSubstitution([ns, "component_manager_1"]),
            composable_node_descriptions=[
                ComposableNode(
                    package=pkg_name,
                    plugin="tobas::fc2xx::PwmBattImuDriverNode",
                    namespace=ns,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="tobas::fc2xx::DShotDriverNode",
                    namespace=ns,
                ),
            ],
        )
    )

    # Launch 2nd priority nodes (Pose Control & Navigation & Manipulation).
    ld.add_action(
        LoadComposableNodes(
            target_container=PathJoinSubstitution([ns, "component_manager_2"]),
            composable_node_descriptions=[
                ComposableNode(
                    package=pkg_name,
                    plugin="tobas::fc2xx::MagDriverNode",
                    namespace=ns,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="tobas::fc2xx::BaroDriverNode",
                    namespace=ns,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="tobas::fc2xx::GnssDriverNode",
                    namespace=ns,
                ),
            ],
        )
    )

    return ld
