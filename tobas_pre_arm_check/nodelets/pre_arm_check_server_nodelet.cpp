#include <pluginlib/class_list_macros.hpp>

#include "./pre_arm_check_server_nodelet.hpp"

namespace tobas_pre_arm_check
{
void PreArmCheckServerNodelet::onInit()
{
  node_.reset(new PreArmCheckServer(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_pre_arm_check

PLUGINLIB_EXPORT_CLASS(tobas_pre_arm_check::PreArmCheckServerNodelet, nodelet::Nodelet);
