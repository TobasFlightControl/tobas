#include <pluginlib/class_list_macros.hpp>

#include "./velocity_controller_nodelet.hpp"

namespace tobas_multirotor_controller
{
void VelocityControllerNodelet::onInit()
{
  NODELET_INFO("Initializing Velocity Controller Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new VelocityControllerRos(nh, pnh, name));
}
}  // namespace tobas_multirotor_controller

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_controller::VelocityControllerNodelet, nodelet::Nodelet);
