#include <pluginlib/class_list_macros.hpp>

#include "./bridge_nodelet.hpp"

namespace tobas_mavros_bridge
{
void TobasMavrosBridgeNodelet::onInit()
{
  node_.reset(new TobasMavrosBridge(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_mavros_bridge

PLUGINLIB_EXPORT_CLASS(tobas_mavros_bridge::TobasMavrosBridgeNodelet, nodelet::Nodelet);
