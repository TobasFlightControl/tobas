#include <pluginlib/class_list_macros.hpp>

#include "./controller_nodelet.hpp"

namespace tobas_mr_arducopter
{
void ControllerNodelet::onInit()
{
  node_.reset(new ControllerRos(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_mr_arducopter

PLUGINLIB_EXPORT_CLASS(tobas_mr_arducopter::ControllerNodelet, nodelet::Nodelet);
