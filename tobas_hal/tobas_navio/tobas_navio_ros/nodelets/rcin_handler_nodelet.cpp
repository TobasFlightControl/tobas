#include <pluginlib/class_list_macros.hpp>

#include "./rcin_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void RCInputHandlerNodelet::onInit()
{
  node_.reset(new RCInputHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::RCInputHandlerNodelet, nodelet::Nodelet);
