from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# Template parameters
DRONE_NAME = "f450"
CONFIG_PKG_NAME = "tobas_f450_config"
HARDWARE_PKG = "tobas_t1_ros"

# Arguments
LOG_LEVEL = "log_level"
OUTPUT = "output"
MULTIPROCESS = "multiprocess"


def generate_launch_description():
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument(LOG_LEVEL, default_value="info"))
    ld.add_action(DeclareLaunchArgument(OUTPUT, default_value="screen"))
    ld.add_action(DeclareLaunchArgument(MULTIPROCESS, default_value="false"))

    # Get arguments
    log_level = LaunchConfiguration(LOG_LEVEL)
    output = LaunchConfiguration(OUTPUT)
    multiprocess = LaunchConfiguration(MULTIPROCESS)

    config_pkg_share = FindPackageShare(CONFIG_PKG_NAME)
    hw_pkg_share = FindPackageShare(HARDWARE_PKG)
    extra_arguments = [{"use_intra_process_comms": True}]

    # Parse URDF
    xacro_path = PathJoinSubstitution([FindPackageShare(CONFIG_PKG_NAME), "urdf", "drone.xacro"])
    urdf_content = Command([FindExecutable(name="xacro"), " ", xacro_path, f" DEBUG:=false"])

    # Launch Tobas core software
    # XXX: To ensure stable registration of components, the component managers are launched first.
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([config_pkg_share, "launch", "common_realtime_component.launch.py"])
            ),
            launch_arguments={
                "log_level": log_level,
                "output": output,
                "use_sim_time": "false",
                "ground_truth": "false",
                "multiprocess": multiprocess,
            }.items(),
        )
    )

    # Launch hardware interfaces
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([hw_pkg_share, "launch", "hardware_interfaces.launch.py"])
            ),
            launch_arguments={"namespace": DRONE_NAME}.items(),
        )
    )

    # Launch 1st priority nodes (Twist Control)
    ld.add_action(
        LoadComposableNodes(
            target_container=f"{DRONE_NAME}/component_manager_1",
            composable_node_descriptions=[
                ComposableNode(
                    package="tobas_real_ros",
                    plugin="ImuHandlerNode",
                    namespace=DRONE_NAME,
                    extra_arguments=extra_arguments,
                ),
                ComposableNode(
                    package="tobas_real_ros",
                    plugin="ICEPropulsionSystemHandlerNode",
                    namespace=DRONE_NAME,
                    extra_arguments=extra_arguments,
                ),
            ],
        )
    )

    # Launch 2nd priority nodes (Pose Control & Navigation & Manipulation)
    ld.add_action(
        LoadComposableNodes(
            target_container=f"{DRONE_NAME}/component_manager_2",
            composable_node_descriptions=[
                ComposableNode(
                    package="robot_state_publisher",
                    plugin="robot_state_publisher::RobotStatePublisher",
                    namespace=DRONE_NAME,
                    parameters=[{"robot_description": urdf_content}],
                    remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    extra_arguments=extra_arguments,
                ),
                ComposableNode(
                    package="tobas_sbus_driver",
                    plugin="SbusDriverNode",
                    namespace=DRONE_NAME,
                    parameters=[{"device": "/dev/ttyAMA0"}],  # TODO: FMUに応じてデバイスを変更
                    extra_arguments=extra_arguments,
                ),
                ComposableNode(
                    package="tobas_real_ros",
                    plugin="RCInputHandlerNode",
                    namespace=DRONE_NAME,
                    extra_arguments=extra_arguments,
                ),
                ComposableNode(
                    package="tobas_real_ros",
                    plugin="MagnetometerHandlerNode",
                    namespace=DRONE_NAME,
                    extra_arguments=extra_arguments,
                ),
                ComposableNode(
                    package="tobas_real_ros",
                    plugin="BarometerHandlerNode",
                    namespace=DRONE_NAME,
                    extra_arguments=extra_arguments,
                ),
                ComposableNode(
                    package="tobas_real_ros",
                    plugin="JointsHandlerNode",
                    namespace=DRONE_NAME,
                    extra_arguments=extra_arguments,
                ),
            ],
        )
    )

    # Launch 3th priority nodes (Others)
    ld.add_action(
        LoadComposableNodes(
            target_container=f"{DRONE_NAME}/component_manager_3",
            composable_node_descriptions=[
                ComposableNode(
                    package="tobas_real_ros",
                    plugin="CpuHandlerNode",
                    namespace=DRONE_NAME,
                    extra_arguments=extra_arguments,
                ),
            ],
        )
    )

    return ld
