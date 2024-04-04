#include <pluginlib/class_list_macros.hpp>

#include "./barometer_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void BarometerHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing Barometer Handler Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new BarometerHandler(nh, pnh, name));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::BarometerHandlerNodelet, nodelet::Nodelet);
