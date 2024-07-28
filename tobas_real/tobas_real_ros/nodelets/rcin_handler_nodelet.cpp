#include <pluginlib/class_list_macros.hpp>

#include "./rcin_handler_nodelet.hpp"

namespace tobas_real_ros
{
void RCInputHandlerNodelet::onInit()
{
  node_.reset(new RCInputHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_real_ros

PLUGINLIB_EXPORT_CLASS(tobas_real_ros::RCInputHandlerNodelet, nodelet::Nodelet);
