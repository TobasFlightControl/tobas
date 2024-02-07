#include <pluginlib/class_list_macros.hpp>

#include "./position_controller_nodelet.hpp"

namespace tobas_manipulation
{
void PositionControllerNodelet::onInit()
{
  NODELET_INFO("Initializing Cartesian Manipulation Position Controller Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new PositionControllerRos(nh, pnh, name));
}
}  // namespace tobas_manipulation

PLUGINLIB_EXPORT_CLASS(tobas_manipulation::PositionControllerNodelet, nodelet::Nodelet);
