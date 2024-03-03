#include <pluginlib/class_list_macros.hpp>

#include "./controller_nodelet.hpp"

namespace tobas_np_pid
{
void ControllerNodelet::onInit()
{
  NODELET_INFO("Initializing Multirotor PID Controller Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new ControllerRos(nh, pnh, name));
}
}  // namespace tobas_np_pid

PLUGINLIB_EXPORT_CLASS(tobas_np_pid::ControllerNodelet, nodelet::Nodelet);
