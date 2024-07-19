#include <pluginlib/class_list_macros.hpp>

#include "./pwm_driver_nodelet.hpp"

namespace a1
{
void PWMDriverNodelet::onInit()
{
  node_.reset(new PWMDriver(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace a1

PLUGINLIB_EXPORT_CLASS(a1::PWMDriverNodelet, nodelet::Nodelet);
