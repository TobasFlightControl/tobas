#include <pluginlib/class_list_macros.hpp>

#include "./controller_nodelet.hpp"

namespace tobas_fixed_wing_mpc
{
void ControllerNodelet::onInit()
{
  node_.reset(new Controller(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_fixed_wing_mpc

PLUGINLIB_EXPORT_CLASS(tobas_fixed_wing_mpc::ControllerNodelet, nodelet::Nodelet);
