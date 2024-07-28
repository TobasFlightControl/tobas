#include <pluginlib/class_list_macros.hpp>

#include "./baro_driver_nodelet.hpp"

namespace a1
{
void BaroDriverNodelet::onInit()
{
  node_.reset(new BaroDriver(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace a1

PLUGINLIB_EXPORT_CLASS(a1::BaroDriverNodelet, nodelet::Nodelet);
