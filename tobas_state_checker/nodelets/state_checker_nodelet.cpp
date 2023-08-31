#include <pluginlib/class_list_macros.hpp>

#include "./state_checker_nodelet.hpp"

namespace tobas_state_checker
{
void StateCheckerNodelet::onInit()
{
  NODELET_INFO("Initializing State Checker Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new StateChecker(nh, pnh));
}
}  // namespace tobas_state_checker

PLUGINLIB_EXPORT_CLASS(tobas_state_checker::StateCheckerNodelet, nodelet::Nodelet);
