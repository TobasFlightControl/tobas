#include <pluginlib/class_list_macros.hpp>

#include "./sbus_handler_nodelet.hpp"

namespace tobas_a1_ros
{
void SBUSHandlerNodelet::onInit()
{
  node_.reset(new SBUSHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_a1_ros

PLUGINLIB_EXPORT_CLASS(tobas_a1_ros::SBUSHandlerNodelet, nodelet::Nodelet);
