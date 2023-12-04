#include <pluginlib/class_list_macros.hpp>

#include "./cartesian_manipulation_nodelet.hpp"

namespace tobas_cartesian_manipulation
{
void CartesianManipulationNodelet::onInit()
{
  NODELET_INFO("Initializing Cartesian Manipulation Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new CartesianManipulationRos(nh, pnh, name));
}
}  // namespace tobas_cartesian_manipulation

PLUGINLIB_EXPORT_CLASS(
  tobas_cartesian_manipulation::CartesianManipulationNodelet,
  nodelet::Nodelet);
