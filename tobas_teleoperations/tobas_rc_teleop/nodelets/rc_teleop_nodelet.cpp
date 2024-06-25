#include <pluginlib/class_list_macros.hpp>

#include "./rc_teleop_nodelet.hpp"

namespace tobas_rc_teleop
{
void RCTeleopNodelet::onInit()
{
  node_.reset(new RCTeleop(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_rc_teleop

PLUGINLIB_EXPORT_CLASS(tobas_rc_teleop::RCTeleopNodelet, nodelet::Nodelet);
