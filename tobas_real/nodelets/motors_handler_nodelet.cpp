#include <pluginlib/class_list_macros.hpp>

#include "./motors_handler_nodelet.hpp"

namespace tobas_real
{
void MotorsHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing Motors Handler Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new MotorsHandler(nh, pnh));
}
}  // namespace tobas_real

PLUGINLIB_EXPORT_CLASS(tobas_real::MotorsHandlerNodelet, nodelet::Nodelet);
