#include <pluginlib/class_list_macros.hpp>

#include "./velocity_controller_nodelet.hpp"

namespace tobas_manipulation
{
void VelocityControllerNodelet::onInit()
{
  node_.reset(new VelocityControllerRos(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_manipulation

PLUGINLIB_EXPORT_CLASS(tobas_manipulation::VelocityControllerNodelet, nodelet::Nodelet);
