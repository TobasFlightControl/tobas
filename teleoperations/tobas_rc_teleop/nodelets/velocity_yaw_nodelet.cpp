#include <pluginlib/class_list_macros.hpp>

#include "./velocity_yaw_nodelet.hpp"

namespace tobas_rc_teleop
{
void RcinToVelocityYawNodelet::onInit()
{
  NODELET_INFO("Initializing RC Teleoperation Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new RcinToVelocityYaw(nh, pnh));
}
}  // namespace tobas_rc_teleop

PLUGINLIB_EXPORT_CLASS(tobas_rc_teleop::RcinToVelocityYawNodelet, nodelet::Nodelet);
