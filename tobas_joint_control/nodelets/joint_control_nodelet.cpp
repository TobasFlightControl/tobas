#include <pluginlib/class_list_macros.hpp>

#include "./joint_control_nodelet.hpp"

namespace tobas_joint_control
{
void JointControlNodelet::onInit()
{
  NODELET_INFO("Initializing Cartesian Manipulation Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new JointControlRos(nh, pnh, name));
}
}  // namespace tobas_joint_control

PLUGINLIB_EXPORT_CLASS(
  tobas_joint_control::JointControlNodelet,
  nodelet::Nodelet);
