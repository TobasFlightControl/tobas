#include <pluginlib/class_list_macros.hpp>

#include "./tobas_bridge_nodelet.hpp"

void TobasBridgeNodelet::onInit()
{
  node_.reset(new TobasBridge(getNodeHandle(), getPrivateNodeHandle()));
}

PLUGINLIB_EXPORT_CLASS(TobasBridgeNodelet, nodelet::Nodelet);
