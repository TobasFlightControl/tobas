#include <pluginlib/class_list_macros.hpp>

#include "./roll_pitch_yawrate_thrust_nodelet.hpp"

namespace tobas_rc_teleop
{
void RcinToRollPitchYawrateThrustNodelet::onInit()
{
  NODELET_INFO("Initializing RC Teleoperation Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new RcinToRollPitchYawrateThrust(nh, pnh, name));
}
}  // namespace tobas_rc_teleop

PLUGINLIB_EXPORT_CLASS(tobas_rc_teleop::RcinToRollPitchYawrateThrustNodelet, nodelet::Nodelet);
