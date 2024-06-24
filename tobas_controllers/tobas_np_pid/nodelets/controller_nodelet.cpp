#include <pluginlib/class_list_macros.hpp>

#include "./controller_nodelet.hpp"

namespace tobas_np_pid
{
void ControllerNodelet::onInit()
{
  node_.reset(new ControllerRos(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_np_pid

PLUGINLIB_EXPORT_CLASS(tobas_np_pid::ControllerNodelet, nodelet::Nodelet);
