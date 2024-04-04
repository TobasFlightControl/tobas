#include <pluginlib/class_list_macros.hpp>

#include "./gps_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void GpsHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing GPS Handler Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new GpsHandler(nh, pnh, name));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::GpsHandlerNodelet, nodelet::Nodelet);
