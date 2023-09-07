#include <pluginlib/class_list_macros.hpp>

#include "./position_controller_nodelet.hpp"

namespace tobas_multirotor_controller
{
void PositionControllerNodelet::onInit()
{
  NODELET_INFO("Initializing Position Controller Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new PositionControllerRos(nh, pnh, name));
}
}  // namespace tobas_multirotor_controller

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_controller::PositionControllerNodelet, nodelet::Nodelet);
