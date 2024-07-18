#include <pluginlib/class_list_macros.hpp>

#include "./sbus_driver_nodelet.hpp"

namespace tobas_a1_ros
{
void SBUSDriverNodelet::onInit()
{
  node_.reset(new SBUSDriver(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_a1_ros

PLUGINLIB_EXPORT_CLASS(tobas_a1_ros::SBUSDriverNodelet, nodelet::Nodelet);
