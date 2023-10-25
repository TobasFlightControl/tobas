#include <pluginlib/class_list_macros.hpp>

#include "./state_checker_nodelet.hpp"

namespace tobas_state_checker
{
void StateCheckerNodelet::onInit()
{
  NODELET_INFO("Initializing State Checker Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new StateChecker(nh, pnh, name));
}
}  // namespace tobas_state_checker

PLUGINLIB_EXPORT_CLASS(tobas_state_checker::StateCheckerNodelet, nodelet::Nodelet);
