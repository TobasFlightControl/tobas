#pragma once

#include <rviz_common/ros_integration/ros_client_abstraction.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace qt
{
/**
 * @brief RvizのROSノードを作成，管理する．
 */
class RvizNodeManager
{
public:
  explicit RvizNodeManager(int argc, char** argv, const std::string& node_name);

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node();

private:
  std::unique_ptr<rviz_common::ros_integration::RosClientAbstraction> client_;
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node_;
};
}  // namespace qt
