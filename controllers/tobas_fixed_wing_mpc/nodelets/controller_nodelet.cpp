#include <pluginlib/class_list_macros.hpp>

#include "./controller_nodelet.hpp"

namespace tobas_fixed_wing_mpc
{
void ControllerNodelet::onInit()
{
  NODELET_INFO("Initializing Tobas Fixed Wing MPC Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new Controller(nh, pnh));
}
}  // namespace tobas_fixed_wing_mpc

PLUGINLIB_EXPORT_CLASS(tobas_fixed_wing_mpc::ControllerNodelet, nodelet::Nodelet);
