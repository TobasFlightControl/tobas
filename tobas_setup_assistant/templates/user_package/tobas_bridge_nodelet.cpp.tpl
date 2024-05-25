#include <pluginlib/class_list_macros.hpp>

#include "./tobas_bridge_nodelet.hpp"

void TobasBridgeNodelet::onInit()
{
  NODELET_INFO("Initializing Tobas Bridge Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();

  node_.reset(new TobasBridge(nh, pnh));
}

PLUGINLIB_EXPORT_CLASS(TobasBridgeNodelet, nodelet::Nodelet);
