#include <pluginlib/class_list_macros.hpp>

#include "./velocity_yaw_nodelet.hpp"

namespace tobas_rc_teleop
{
void RcinToVelocityYawNodelet::onInit()
{
  NODELET_INFO("Initializing RC Teleoperation Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new RcinToVelocityYaw(nh, pnh, name));
}
}  // namespace tobas_rc_teleop

PLUGINLIB_EXPORT_CLASS(tobas_rc_teleop::RcinToVelocityYawNodelet, nodelet::Nodelet);
