#include <pluginlib/class_list_macros.hpp>

#include "./param_server_nodelet.hpp"

namespace tobas_mr_arducopter
{
void ParamServerNodelet::onInit()
{
  NODELET_INFO("Initializing ArduCopter Parameter Server Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new ParamServerRos(nh, pnh, name));
}
}  // namespace tobas_mr_arducopter

PLUGINLIB_EXPORT_CLASS(tobas_mr_arducopter::ParamServerNodelet, nodelet::Nodelet);
