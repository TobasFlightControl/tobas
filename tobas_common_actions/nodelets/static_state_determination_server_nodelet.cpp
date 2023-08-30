#include <pluginlib/class_list_macros.hpp>

#include "./static_state_determination_server_nodelet.hpp"

namespace tobas_common_actions
{
void StaticStateDeterminationServerNodelet::onInit()
{
  node_.reset(new StaticStateDeterminationServer());
}
}  // namespace tobas_common_actions

PLUGINLIB_EXPORT_CLASS(
  tobas_common_actions::StaticStateDeterminationServerNodelet,
  nodelet::Nodelet);
