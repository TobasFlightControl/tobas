#include <pluginlib/class_list_macros.hpp>

#include "./rotation_controller_nodelet.hpp"

namespace tobas_multirotor_controller
{
void RotationControllerNodelet::onInit()
{
  NODELET_INFO("Initializing Rotation Controller Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new RotationControllerRos(nh, pnh));
}
}  // namespace tobas_multirotor_controller

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_controller::RotationControllerNodelet, nodelet::Nodelet);
