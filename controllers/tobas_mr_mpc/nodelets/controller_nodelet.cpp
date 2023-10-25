#include <pluginlib/class_list_macros.hpp>

#include "./controller_nodelet.hpp"

namespace tobas_mr_mpc
{
void ControllerNodelet::onInit()
{
  NODELET_INFO("Initializing Multirotor MPC Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new ControllerRos(nh, pnh, name));
}
}  // namespace tobas_mr_mpc

PLUGINLIB_EXPORT_CLASS(tobas_mr_mpc::ControllerNodelet, nodelet::Nodelet);
