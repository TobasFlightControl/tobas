#include <pluginlib/class_list_macros.hpp>

#include "./time_reference_server_nodelet.hpp"

namespace tobas_real_ros
{
void TimeReferenceServerNodelet::onInit()
{
  NODELET_INFO("Initializing Time Reference Server Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new TimeReferenceServer(nh, pnh, name));
}
}  // namespace tobas_real_ros

PLUGINLIB_EXPORT_CLASS(tobas_real_ros::TimeReferenceServerNodelet, nodelet::Nodelet);
