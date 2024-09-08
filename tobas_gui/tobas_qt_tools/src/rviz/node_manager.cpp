#include "tobas_qt_tools/rviz/node_manager.hpp"

namespace qt
{
RvizNodeManager::RvizNodeManager(int argc, char** argv, const std::string& node_name)
{
  client_ = std::make_unique<rviz_common::ros_integration::RosClientAbstraction>();
  node_ = client_->init(argc, argv, node_name, false);
}

rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr RvizNodeManager::node()
{
  return node_;
}
}  // namespace qt
