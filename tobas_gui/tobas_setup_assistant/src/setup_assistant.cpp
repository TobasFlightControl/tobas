#include "tobas_setup_assistant/setup_assistant.hpp"

namespace gui
{
namespace setup_assistant
{
SetupAssistant::SetupAssistant() : tree_info_(tree_)
{
  // TODO
}

const rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr& SetupAssistant::rvizRosNode()
{
  return rviz_ros_node_;
}

const rclcpp::Node::SharedPtr& SetupAssistant::node()
{
  return node_;
}

const TreeInformation& SetupAssistant::treeInfo()
{
  return tree_info_;
}
}  // namespace setup_assistant
}  // namespace gui
