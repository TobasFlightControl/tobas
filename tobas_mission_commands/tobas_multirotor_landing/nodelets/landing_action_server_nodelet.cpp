#include <pluginlib/class_list_macros.hpp>

#include "./landing_action_server_nodelet.hpp"

namespace tobas_multirotor_landing
{
void LandActionServerNodelet::onInit()
{
  NODELET_INFO("Initializing Multirotor Landing Action Server Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new LandActionServer(nh, pnh, name));
}
}  // namespace tobas_multirotor_landing

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_landing::LandActionServerNodelet, nodelet::Nodelet);
