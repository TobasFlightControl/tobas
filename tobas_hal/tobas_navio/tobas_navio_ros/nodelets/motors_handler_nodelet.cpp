#include <pluginlib/class_list_macros.hpp>

#include "./motors_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void MotorsHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing Motors Handler Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new MotorsHandler(nh, pnh, name));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::MotorsHandlerNodelet, nodelet::Nodelet);
