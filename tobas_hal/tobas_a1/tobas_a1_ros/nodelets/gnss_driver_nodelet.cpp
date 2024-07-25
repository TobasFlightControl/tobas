#include <pluginlib/class_list_macros.hpp>

#include "./gnss_driver_nodelet.hpp"

namespace a1
{
void GNSSDriverNodelet::onInit()
{
  node_.reset(new GNSSDriver(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace a1

PLUGINLIB_EXPORT_CLASS(a1::GNSSDriverNodelet, nodelet::Nodelet);
