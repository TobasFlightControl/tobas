#include <pluginlib/class_list_macros.hpp>

#include "./motors_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void MotorsHandlerNodelet::onInit()
{
  node_.reset(new MotorsHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::MotorsHandlerNodelet, nodelet::Nodelet);
