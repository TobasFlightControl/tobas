#include <pluginlib/class_list_macros.hpp>

#include "./effort_controller_nodelet.hpp"

namespace tobas_manipulation
{
void EffortControllerNodelet::onInit()
{
  node_.reset(new EffortControllerRos(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_manipulation

PLUGINLIB_EXPORT_CLASS(tobas_manipulation::EffortControllerNodelet, nodelet::Nodelet);
