#include <pluginlib/class_list_macros.hpp>

#include "./sbus_driver_nodelet.hpp"

namespace a1
{
void SBUSDriverNodelet::onInit()
{
  node_.reset(new SBUSDriver(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace a1

PLUGINLIB_EXPORT_CLASS(a1::SBUSDriverNodelet, nodelet::Nodelet);
