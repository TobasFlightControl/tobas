#include <pluginlib/class_list_macros.hpp>

#include "./controller_nodelet.hpp"

namespace tobas_mr_pidmpc
{
void ControllerNodelet::onInit()
{
  NODELET_INFO("Initializing Position Controller Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new ControllerRos(nh, pnh, name));
}
}  // namespace tobas_mr_pidmpc

PLUGINLIB_EXPORT_CLASS(tobas_mr_pidmpc::ControllerNodelet, nodelet::Nodelet);
