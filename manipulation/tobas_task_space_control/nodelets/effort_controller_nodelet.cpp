#include <pluginlib/class_list_macros.hpp>

#include "./effort_controller_nodelet.hpp"

namespace tobas_task_space_control
{
void EffortControllerNodelet::onInit()
{
  NODELET_INFO("Initializing Cartesian Manipulation Effort Controller Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new EffortControllerRos(nh, pnh, name));
}
}  // namespace tobas_task_space_control

PLUGINLIB_EXPORT_CLASS(tobas_task_space_control::EffortControllerNodelet, nodelet::Nodelet);
