#include <pluginlib/class_list_macros.hpp>

#include "./imu_handler_nodelet.hpp"

namespace tobas_real_ros
{
void ImuHandlerNodelet::onInit()
{
  node_.reset(new ImuHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_real_ros

PLUGINLIB_EXPORT_CLASS(tobas_real_ros::ImuHandlerNodelet, nodelet::Nodelet);
