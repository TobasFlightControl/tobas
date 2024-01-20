#include <pluginlib/class_list_macros.hpp>

#include "./pwm_handler_nodelet.hpp"

namespace tobas_real
{
void PwmHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing PWM Handler Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new PwmHandler(nh, pnh, name));
}
}  // namespace tobas_real

PLUGINLIB_EXPORT_CLASS(tobas_real::PwmHandlerNodelet, nodelet::Nodelet);
