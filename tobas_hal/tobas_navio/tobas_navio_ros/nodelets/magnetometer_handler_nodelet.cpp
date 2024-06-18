#include <pluginlib/class_list_macros.hpp>

#include "./magnetometer_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void MagnetometerHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing Magnetometer Handler Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new MagnetometerHandler(nh, pnh, name));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::MagnetometerHandlerNodelet, nodelet::Nodelet);
