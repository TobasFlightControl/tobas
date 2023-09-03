#include <pluginlib/class_list_macros.hpp>

#include "./static_state_determination_server_nodelet.hpp"

namespace tobas_common_actions
{
void StaticStateDeterminationServerNodelet::onInit()
{
  NODELET_INFO("Initializing Static State Determination Server Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new StaticStateDeterminationServer(nh, pnh, name));
}
}  // namespace tobas_common_actions

PLUGINLIB_EXPORT_CLASS(
  tobas_common_actions::StaticStateDeterminationServerNodelet,
  nodelet::Nodelet);
