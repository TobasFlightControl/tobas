#include <pluginlib/class_list_macros.hpp>

#include "./velocity_controller_nodelet.hpp"

namespace tobas_multirotor_controller
{
void VelocityControllerNodelet::onInit()
{
  NODELET_INFO("Initializing Velocity Controller Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new VelocityControllerRos(nh, pnh));
}
}  // namespace tobas_multirotor_controller

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_controller::VelocityControllerNodelet, nodelet::Nodelet);
