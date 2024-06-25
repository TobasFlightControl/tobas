#include <pluginlib/class_list_macros.hpp>

#include "./controller_nodelet.hpp"

namespace tobas_mr_mpc
{
void ControllerNodelet::onInit()
{
  node_.reset(new ControllerRos(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_mr_mpc

PLUGINLIB_EXPORT_CLASS(tobas_mr_mpc::ControllerNodelet, nodelet::Nodelet);
