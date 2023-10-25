#include <pluginlib/class_list_macros.hpp>

#include "./wait_for_stillness_server_nodelet.hpp"

namespace tobas_common_actions
{
void WaitForStillnessServerNodelet::onInit()
{
  NODELET_INFO("Initializing Wait For Stillness Server Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new WaitForStillnessServer(nh, pnh, name));
}
}  // namespace tobas_common_actions

PLUGINLIB_EXPORT_CLASS(tobas_common_actions::WaitForStillnessServerNodelet, nodelet::Nodelet);
