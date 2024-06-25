#include <pluginlib/class_list_macros.hpp>

#include "./position_yaw_nodelet.hpp"

namespace tobas_trajectory_commander
{
void FollowPositionYawTrajectoryServerNodelet::onInit()
{
  node_.reset(new FollowPositionYawTrajectoryServer(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_trajectory_commander

PLUGINLIB_EXPORT_CLASS(tobas_trajectory_commander::FollowPositionYawTrajectoryServerNodelet, nodelet::Nodelet);
