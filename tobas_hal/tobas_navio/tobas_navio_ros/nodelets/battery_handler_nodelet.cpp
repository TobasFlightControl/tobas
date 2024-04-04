#include <pluginlib/class_list_macros.hpp>

#include "./battery_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void BatteryHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing Battery Handler Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new BatteryHandler(nh, pnh, name));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::BatteryHandlerNodelet, nodelet::Nodelet);
