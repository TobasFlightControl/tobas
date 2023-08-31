#include <pluginlib/class_list_macros.hpp>

#include "./position_yaw_nodelet.hpp"

namespace tobas_trajectory_commander
{
void FollowPositionYawTrajectoryServerNodelet::onInit()
{
  NODELET_INFO("Initializing Follow Trajectory Server Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new FollowPositionYawTrajectoryServer(nh, pnh));
}
}  // namespace tobas_trajectory_commander

PLUGINLIB_EXPORT_CLASS(
  tobas_trajectory_commander::FollowPositionYawTrajectoryServerNodelet,
  nodelet::Nodelet);
