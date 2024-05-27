#include <pluginlib/class_list_macros.hpp>

#include "./position_yaw_nodelet.hpp"

namespace tobas_trajectory_commander
{
void FollowPositionYawTrajectoryServerNodelet::onInit()
{
  NODELET_INFO("Initializing Follow Trajectory Server Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new FollowPositionYawTrajectoryServer(nh, pnh, name));
}
}  // namespace tobas_trajectory_commander

PLUGINLIB_EXPORT_CLASS(tobas_trajectory_commander::FollowPositionYawTrajectoryServerNodelet, nodelet::Nodelet);
