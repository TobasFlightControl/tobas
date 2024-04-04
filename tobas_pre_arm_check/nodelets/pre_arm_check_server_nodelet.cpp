#include <pluginlib/class_list_macros.hpp>

#include "./pre_arm_check_server_nodelet.hpp"

namespace tobas_pre_arm_check
{
void PreArmCheckServerNodelet::onInit()
{
  NODELET_INFO("Initializing Pre-Arm Check Server Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new PreArmCheckServer(nh, pnh, name));
}
}  // namespace tobas_pre_arm_check

PLUGINLIB_EXPORT_CLASS(tobas_pre_arm_check::PreArmCheckServerNodelet, nodelet::Nodelet);
