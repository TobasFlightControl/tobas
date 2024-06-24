#include <pluginlib/class_list_macros.hpp>

#include "./controller_nodelet.hpp"

namespace tobas_fixed_wing_lqd
{
void ControllerNodelet::onInit()
{
  node_.reset(new Controller(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_fixed_wing_lqd

PLUGINLIB_EXPORT_CLASS(tobas_fixed_wing_lqd::ControllerNodelet, nodelet::Nodelet);
