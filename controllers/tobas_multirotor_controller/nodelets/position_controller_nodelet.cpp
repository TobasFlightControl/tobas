#include <pluginlib/class_list_macros.hpp>

#include "./position_controller_nodelet.hpp"

namespace tobas_multirotor_controller
{
void PositionControllerNodelet::onInit()
{
  node_.reset(new PositionControllerRos());
}
}  // namespace tobas_multirotor_controller

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_controller::PositionControllerNodelet, nodelet::Nodelet);
