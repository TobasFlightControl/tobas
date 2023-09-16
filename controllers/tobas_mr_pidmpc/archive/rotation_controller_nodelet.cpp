#include <pluginlib/class_list_macros.hpp>

#include "./rotation_controller_nodelet.hpp"

namespace tobas_mr_pidmpc
{
void RotationControllerNodelet::onInit()
{
  NODELET_INFO("Initializing Rotation Controller Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new RotationControllerRos(nh, pnh, name));
}
}  // namespace tobas_mr_pidmpc

PLUGINLIB_EXPORT_CLASS(tobas_mr_pidmpc::RotationControllerNodelet, nodelet::Nodelet);
