#include <pluginlib/class_list_macros.hpp>

#include "./controller_nodelet.hpp"

namespace tobas_fixed_wing_lqd
{
void ControllerNodelet::onInit()
{
  NODELET_INFO("Initializing Tobas Fixed Wing LQD Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new Controller(nh, pnh, name));
}
}  // namespace tobas_fixed_wing_lqd

PLUGINLIB_EXPORT_CLASS(tobas_fixed_wing_lqd::ControllerNodelet, nodelet::Nodelet);
