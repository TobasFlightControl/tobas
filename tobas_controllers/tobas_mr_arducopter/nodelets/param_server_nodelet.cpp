#include <pluginlib/class_list_macros.hpp>

#include "./param_server_nodelet.hpp"

namespace tobas_mr_arducopter
{
void ParamServerNodelet::onInit()
{
  node_.reset(new ParamServerRos(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_mr_arducopter

PLUGINLIB_EXPORT_CLASS(tobas_mr_arducopter::ParamServerNodelet, nodelet::Nodelet);
