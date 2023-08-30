#include <pluginlib/class_list_macros.hpp>

#include "./velocity_controller_nodelet.hpp"

namespace tobas_multirotor_controller
{
void VelocityControllerNodelet::onInit()
{
  node_.reset(new VelocityControllerRos());
}
}  // namespace tobas_multirotor_controller

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_controller::VelocityControllerNodelet, nodelet::Nodelet);
