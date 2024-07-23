#include <pluginlib/class_list_macros.hpp>

#include "./adc_driver_nodelet.hpp"

namespace a1
{
void ADCDriverNodelet::onInit()
{
  node_.reset(new ADCDriver(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace a1

PLUGINLIB_EXPORT_CLASS(a1::ADCDriverNodelet, nodelet::Nodelet);
