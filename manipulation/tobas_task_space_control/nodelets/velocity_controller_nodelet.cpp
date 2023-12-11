#include <pluginlib/class_list_macros.hpp>

#include "./velocity_controller_nodelet.hpp"

namespace tobas_task_space_control
{
void VelocityControllerNodelet::onInit()
{
  NODELET_INFO("Initializing Cartesian Manipulation Velocity Controller Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new VelocityControllerRos(nh, pnh, name));
}
}  // namespace tobas_task_space_control

PLUGINLIB_EXPORT_CLASS(tobas_task_space_control::VelocityControllerNodelet, nodelet::Nodelet);
