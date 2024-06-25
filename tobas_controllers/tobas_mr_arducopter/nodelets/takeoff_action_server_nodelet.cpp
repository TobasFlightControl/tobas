#include <pluginlib/class_list_macros.hpp>

#include "./takeoff_action_server_nodelet.hpp"

namespace tobas_mr_arducopter
{
void TakeoffActionServerNodelet::onInit()
{
  node_.reset(new TakeoffActionServer(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_mr_arducopter

PLUGINLIB_EXPORT_CLASS(tobas_mr_arducopter::TakeoffActionServerNodelet, nodelet::Nodelet);
