#include <pluginlib/class_list_macros.hpp>

#include "./rc_teleop_nodelet.hpp"

namespace tobas_rc_teleop
{
void RCTeleopNodelet::onInit()
{
  NODELET_INFO("Initializing RC Teleoperation Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new RCTeleop(nh, pnh, name));
}
}  // namespace tobas_rc_teleop

PLUGINLIB_EXPORT_CLASS(tobas_rc_teleop::RCTeleopNodelet, nodelet::Nodelet);
