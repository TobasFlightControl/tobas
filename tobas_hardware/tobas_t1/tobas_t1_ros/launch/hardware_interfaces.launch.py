from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnExecutionComplete
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

from tobas_launch_events.events import HardwareInterfacesReady

NAMESPACE = "namespace"


def generate_launch_description():
    pkg_name = "tobas_t1_ros"
    extra_arguments = [{"use_intra_process_comms": True}]

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(NAMESPACE))
    ns = LaunchConfiguration(NAMESPACE)

    # Launch 1st priority nodes (Twist Control)
    nodes_1 = LoadComposableNodes(
        target_container=PathJoinSubstitution([ns, "component_manager_1"]),
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin="ImuDriverNode",
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
                plugin="PwmDriverNode",
                namespace=ns,
                extra_arguments=extra_arguments,
            ),
        ],
    )
    ld.add_action(nodes_1)

    # Launch 2nd priority nodes (Pose Control & Navigation & Manipulation)
    nodes_2 = LoadComposableNodes(
        target_container=PathJoinSubstitution([ns, "component_manager_2"]),
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
                plugin="GnssDriverNode",
                namespace=ns,
                extra_arguments=extra_arguments,
            ),
            ComposableNode(
                package=pkg_name,
                plugin="BatteryDriverNode",
                namespace=ns,
                extra_arguments=extra_arguments,
            ),
        ],
    )
    ld.add_action(RegisterEventHandler(OnExecutionComplete(target_action=nodes_1, on_completion=[nodes_2])))

    # Launch 3th priority nodes (Others)
    nodes_3 = LoadComposableNodes(
        target_container=PathJoinSubstitution([ns, "component_manager_3"]),
        composable_node_descriptions=[],
    )
    ld.add_action(RegisterEventHandler(OnExecutionComplete(target_action=nodes_2, on_completion=[nodes_3])))

    # Emit ready event
    ready_event = EmitEvent(event=HardwareInterfacesReady())
    ld.add_action(RegisterEventHandler(OnExecutionComplete(target_action=nodes_3, on_start=[ready_event])))

    return ld
