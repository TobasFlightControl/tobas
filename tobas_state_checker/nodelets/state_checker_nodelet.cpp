#include <pluginlib/class_list_macros.hpp>

#include "./state_checker_nodelet.hpp"

namespace tobas_state_checker
{
void StateCheckerNodelet::onInit()
{
  node_.reset(new StateChecker(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_state_checker

PLUGINLIB_EXPORT_CLASS(tobas_state_checker::StateCheckerNodelet, nodelet::Nodelet);
