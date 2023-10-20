#include <pluginlib/class_list_macros.hpp>

#include "./takeoff_action_server_nodelet.hpp"

namespace tobas_multirotor_takeoff
{
void MultirotorTakeoffServerNodelet::onInit()
{
  NODELET_INFO("Initializing Multirotor Takeoff Action Server Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new MultirotorTakeoffServer(nh, pnh, name));
}
}  // namespace tobas_multirotor_takeoff

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_takeoff::MultirotorTakeoffServerNodelet, nodelet::Nodelet);
