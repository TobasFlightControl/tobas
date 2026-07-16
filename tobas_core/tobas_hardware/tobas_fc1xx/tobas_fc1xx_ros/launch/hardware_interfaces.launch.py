from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

NAMESPACE = "namespace"


def generate_launch_description():
    pkg_name = "tobas_fc1xx_ros"

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
                    plugin="tobas::fc1xx::ImuDriverNode",
                    namespace=ns,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="tobas::fc1xx::DShotDriverNode",
                    namespace=ns,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="tobas::fc1xx::PwmDriverNode",
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
                    plugin="tobas::fc1xx::MagDriverNode",
                    namespace=ns,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="tobas::fc1xx::BaroDriverNode",
                    namespace=ns,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="tobas::fc1xx::GnssDriverNode",
                    namespace=ns,
                ),
                ComposableNode(
                    package=pkg_name,
                    plugin="tobas::fc1xx::BatteryDriverNode",
                    namespace=ns,
                ),
            ],
        )
    )

    return ld
