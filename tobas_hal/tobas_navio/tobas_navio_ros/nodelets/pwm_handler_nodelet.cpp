#include <pluginlib/class_list_macros.hpp>

#include "./pwm_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void PwmHandlerNodelet::onInit()
{
  node_.reset(new PwmHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::PwmHandlerNodelet, nodelet::Nodelet);
