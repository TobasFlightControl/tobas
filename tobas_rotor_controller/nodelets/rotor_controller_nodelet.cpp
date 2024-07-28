#include <pluginlib/class_list_macros.hpp>

#include "./rotor_controller_nodelet.hpp"

namespace tobas_rotor_controller
{
void RotorControllerNodelet::onInit()
{
  node_.reset(new RotorController(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_rotor_controller

PLUGINLIB_EXPORT_CLASS(tobas_rotor_controller::RotorControllerNodelet, nodelet::Nodelet);
