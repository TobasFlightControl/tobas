#include <pluginlib/class_list_macros.hpp>

#include "./dynamixel_handler_nodelet.hpp"

namespace tobas_dynamixel_handler
{
void DynamixelHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing Dynamixel Handler Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new DynamixelHandler(nh, pnh, name));
}
}  // namespace tobas_dynamixel_handler

PLUGINLIB_EXPORT_CLASS(tobas_dynamixel_handler::DynamixelHandlerNodelet, nodelet::Nodelet);
