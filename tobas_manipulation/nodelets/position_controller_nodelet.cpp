#include <pluginlib/class_list_macros.hpp>

#include "./position_controller_nodelet.hpp"

namespace tobas_manipulation
{
void PositionControllerNodelet::onInit()
{
  node_.reset(new PositionControllerRos(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_manipulation

PLUGINLIB_EXPORT_CLASS(tobas_manipulation::PositionControllerNodelet, nodelet::Nodelet);
