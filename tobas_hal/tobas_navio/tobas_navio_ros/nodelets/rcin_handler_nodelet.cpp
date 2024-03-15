#include <pluginlib/class_list_macros.hpp>

#include "./rcin_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void RCInputHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing RC Input Handler Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new RCInputHandler(nh, pnh, name));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::RCInputHandlerNodelet, nodelet::Nodelet);
