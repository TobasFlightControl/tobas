#include <pluginlib/class_list_macros.hpp>

#include "./landing_action_server_nodelet.hpp"

namespace tobas_multirotor_landing
{
void LandActionServerNodelet::onInit()
{
  node_.reset(new LandActionServer(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_multirotor_landing

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_landing::LandActionServerNodelet, nodelet::Nodelet);
