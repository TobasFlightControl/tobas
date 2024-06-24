#include <pluginlib/class_list_macros.hpp>

#include "./battery_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void BatteryHandlerNodelet::onInit()
{
  node_.reset(new BatteryHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::BatteryHandlerNodelet, nodelet::Nodelet);
