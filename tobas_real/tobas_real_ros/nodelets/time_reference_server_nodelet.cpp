#include <pluginlib/class_list_macros.hpp>

#include "./time_reference_server_nodelet.hpp"

namespace tobas_real_ros
{
void TimeReferenceServerNodelet::onInit()
{
  node_.reset(new TimeReferenceServer(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_real_ros

PLUGINLIB_EXPORT_CLASS(tobas_real_ros::TimeReferenceServerNodelet, nodelet::Nodelet);
