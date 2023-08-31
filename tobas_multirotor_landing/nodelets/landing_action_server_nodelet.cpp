#include <pluginlib/class_list_macros.hpp>

#include "./landing_action_server_nodelet.hpp"

namespace tobas_multirotor_landing
{
void MultirotorLandServerNodelet::onInit()
{
  NODELET_INFO("Initializing Multirotor Landing Action Server Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new MultirotorLandServer(nh, pnh));
}
}  // namespace tobas_multirotor_landing

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_landing::MultirotorLandServerNodelet, nodelet::Nodelet);
