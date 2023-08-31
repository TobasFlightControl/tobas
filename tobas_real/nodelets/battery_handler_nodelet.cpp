#include <pluginlib/class_list_macros.hpp>

#include "./battery_handler_nodelet.hpp"

namespace tobas_real
{
void BatteryHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing Battery Handler Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new BatteryHandler(nh, pnh));
}
}  // namespace tobas_real

PLUGINLIB_EXPORT_CLASS(tobas_real::BatteryHandlerNodelet, nodelet::Nodelet);
