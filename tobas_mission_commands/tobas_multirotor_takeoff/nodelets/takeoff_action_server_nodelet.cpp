#include <pluginlib/class_list_macros.hpp>

#include "./takeoff_action_server_nodelet.hpp"

namespace tobas_multirotor_takeoff
{
void TakeoffActionServerNodelet::onInit()
{
  node_.reset(new TakeoffActionServer(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_multirotor_takeoff

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_takeoff::TakeoffActionServerNodelet, nodelet::Nodelet);
