#include <pluginlib/class_list_macros.hpp>

#include "./rcin_handler_nodelet.hpp"

namespace tobas_real
{
void RCInputHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing RC Input Handler Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new RCInputHandler(nh, pnh));
}
}  // namespace tobas_real

PLUGINLIB_EXPORT_CLASS(tobas_real::RCInputHandlerNodelet, nodelet::Nodelet);
