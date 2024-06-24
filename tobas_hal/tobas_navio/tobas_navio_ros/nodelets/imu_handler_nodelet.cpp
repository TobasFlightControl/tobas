#include <pluginlib/class_list_macros.hpp>

#include "./imu_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void ImuHandlerNodelet::onInit()
{
  node_.reset(new ImuHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::ImuHandlerNodelet, nodelet::Nodelet);
