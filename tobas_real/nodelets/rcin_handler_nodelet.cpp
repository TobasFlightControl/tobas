#include <pluginlib/class_list_macros.hpp>

#include "./rcin_handler_nodelet.hpp"

namespace tobas_real
{
void RCInputHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing RC Input Handler Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new RCInputHandler(nh, pnh, name));
}
}  // namespace tobas_real

PLUGINLIB_EXPORT_CLASS(tobas_real::RCInputHandlerNodelet, nodelet::Nodelet);
