#include <pluginlib/class_list_macros.hpp>

#include "./pre_arm_check_server_nodelet.hpp"

namespace tobas_common_actions
{
void PreArmCheckServerNodelet::onInit()
{
  NODELET_INFO("Initializing Pre-Arm Check Server Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new PreArmCheckServer(nh, pnh, name));
}
}  // namespace tobas_common_actions

PLUGINLIB_EXPORT_CLASS(tobas_common_actions::PreArmCheckServerNodelet, nodelet::Nodelet);
