#include <pluginlib/class_list_macros.hpp>

#include "./dynamixel_handler_nodelet.hpp"

namespace tobas_dynamixel_handler
{
void DynamixelHandlerNodelet::onInit()
{
  node_.reset(new DynamixelHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_dynamixel_handler

PLUGINLIB_EXPORT_CLASS(tobas_dynamixel_handler::DynamixelHandlerNodelet, nodelet::Nodelet);
