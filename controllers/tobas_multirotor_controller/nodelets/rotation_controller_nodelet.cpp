#include <pluginlib/class_list_macros.hpp>

#include "./rotation_controller_nodelet.hpp"

namespace tobas_multirotor_controller
{
void RotationControllerNodelet::onInit()
{
  node_.reset(new RotationControllerRos());
}
}  // namespace tobas_multirotor_controller

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_controller::RotationControllerNodelet, nodelet::Nodelet);
