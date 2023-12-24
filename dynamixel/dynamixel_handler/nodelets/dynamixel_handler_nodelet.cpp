#include <pluginlib/class_list_macros.hpp>

#include "./dynamixel_handler_nodelet.hpp"

namespace dynamixel_handler
{
void DynamixelHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing Multirotor PID Controller Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new DynamixelHandler(nh, pnh, name));
}
}  // namespace dynamixel_handler

PLUGINLIB_EXPORT_CLASS(dynamixel_handler::DynamixelHandlerNodelet, nodelet::Nodelet);
