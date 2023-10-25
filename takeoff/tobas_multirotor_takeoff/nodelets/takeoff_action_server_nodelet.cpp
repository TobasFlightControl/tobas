#include <pluginlib/class_list_macros.hpp>

#include "./takeoff_action_server_nodelet.hpp"

namespace tobas_multirotor_takeoff
{
void TakeoffActionServerNodelet::onInit()
{
  NODELET_INFO("Initializing Multirotor Takeoff Action Server Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new TakeoffActionServer(nh, pnh, name));
}
}  // namespace tobas_multirotor_takeoff

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_takeoff::TakeoffActionServerNodelet, nodelet::Nodelet);
