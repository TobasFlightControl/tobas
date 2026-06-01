# User Code (C++)

This section assumes that you have a basic understanding of C++ and ROS 2.
To learn ROS 2, please refer to
<a href=https://docs.ros.org/en/jazzy/Tutorials.html target="_blank">Tutorials | ROS 2 Documentation</a>.

Among the ROS packages included in a Tobas project created with Setup Assistant (example: tobas_f450.TBS),
the user C++ package (example: tobas_f450_user_cpp) is a C++ package that the user can edit freely.
It includes the following 5 launch files.

- `common_(interface/realtime).launch.py`: Launched in both real hardware and simulation.
- `real_(interface/realtime).launch.py`: Launched only on real hardware.
- `gazebo.launch.py`: Launched only in simulation.

`common` and `real` each have two types of launch files, `interface` and `realtime`.
Nodes launched by the former can perform ROS communication across networks, whereas nodes launched by the latter cannot.
Therefore, for a node launched by `*_realtime.launch` to communicate across networks,
it must go through the interface node launched by `*_interface.launch`.
This mechanism is used to avoid negative effects on real-time performance caused by the DDS node discovery algorithm, which is the backend of ROS 2.

As an example, let's create a C++ node that checks the GNSS status and publishes a message every second when 3D positioning is available.
Please edit `tobas_f450_user_cpp/tobas_f450_user_cpp/user_node.cpp` as follows.

```cpp
#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs_adapter/gnss.hpp>

class GnssStateCheckerNode : public tobas::BaseNode
{
  using self = GnssStateCheckerNode;
  using super = tobas::BaseNode;

public:
  explicit GnssStateCheckerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::ros2::SubscriberPtr<tobas_msgs::Gnss> gnss_sub_;

  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss);
};

GnssStateCheckerNode::GnssStateCheckerNode(const rclcpp::NodeOptions& options)
  : super("gnss_state_checker", nodeOptions_Default(options))
{
  gnss_sub_ = createSubscriber<tobas_msgs::Gnss>(tobas::topic::kGnss, &self::gnssCb, this);
}

void GnssStateCheckerNode::gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss)
{
  if (gnss->fix_type == tobas_msgs::msg::Gnss::FIX_3D) {
    TOBAS_INFO_THROTTLE(1., "GNSS 3D Fix");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(GnssStateCheckerNode)
```

Add this node to the build target.
Add the dependent packages to `tobas_f450_user_cpp/CMakeLists.txt` and uncomment the build process.

```cmake
cmake_minimum_required(VERSION 3.25)
project(tobas_f450_user_cpp)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Wswitch-enum -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(AMENT_DEPENDENCIES
  rclcpp
  rclcpp_components
  tobas_constants
  tobas_node
  tobas_msgs_adapter
)

find_package(ament_cmake REQUIRED)

foreach(dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

add_library(${PROJECT_NAME} SHARED nodes/user_node.cpp)
ament_target_dependencies(${PROJECT_NAME} ${AMENT_DEPENDENCIES})
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "UserNode" EXECUTABLE user_node)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
```

Configure the node to start automatically.
You could launch it as a regular ROS node, but this time we will launch it as a component to eliminate copies in topic communication.

In Tobas, three component managers (`component_manager_x (x = 1, 2, 3)`) run on separate CPUs,
and topic communication between their components is also performed with zero copy.
Lower-numbered managers handle higher-speed processing with stricter latency requirements, and their rough roles are as follows.

- `component_manager_1`: IMU filtering, state estimation, attitude/position control, etc. (400-800Hz)
- `component_manager_2`: Path planning, mission execution, etc. (30-100Hz)
- `component_manager_3`: Parameter server, logging, failsafe, etc. (1-10Hz)

Since the processing performed by the node launched this time is not latency-critical, plug it into `component_manager_3`, which handles the slowest processing.
In `tobas_f450_user_cpp/launch/common_realtime.launch.py`, uncomment the `add_action` section.

```python
# Do not delete or rename this file because it is executed in tobas_f450_config/common_realtime.launch.py.

from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription()

    # Please add the nodes that run both on real hardware and in simulation, with real-time requirements.

    ld.add_action(
        LoadComposableNodes(
            target_container=f"f450/component_manager_3",
            composable_node_descriptions=[
                ComposableNode(
                    package="tobas_f450_user_cpp",
                    plugin="UserNode",
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
        )
    )

    return ld
```

When you start the simulation from the GCS, a message will be displayed in the console of `Control System`.

![console](../../assets/user_code_cpp/console.png)

For details on the API, see [ROS API](./ros_api.md).
