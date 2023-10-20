#include <pluginlib/class_list_macros.hpp>

#include "./bridge_nodelet.hpp"

namespace tobas_mavros_bridge
{
void TobasMavrosBridgeNodelet::onInit()
{
  NODELET_INFO("Initializing Tobas Mavros Bridge Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new TobasMavrosBridge(nh, pnh, name));
}
}  // namespace tobas_mavros_bridge

PLUGINLIB_EXPORT_CLASS(tobas_mavros_bridge::TobasMavrosBridgeNodelet, nodelet::Nodelet);
