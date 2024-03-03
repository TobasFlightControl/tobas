#include <pluginlib/class_list_macros.hpp>

#include "./takeoff_action_server_nodelet.hpp"

namespace tobas_mr_arducopter
{
void TakeoffActionServerNodelet::onInit()
{
  NODELET_INFO("Initializing Multirotor Takeoff Action Server Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new TakeoffActionServer(nh, pnh, name));
}
}  // namespace tobas_mr_arducopter

PLUGINLIB_EXPORT_CLASS(tobas_mr_arducopter::TakeoffActionServerNodelet, nodelet::Nodelet);
