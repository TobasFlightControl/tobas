#include <pluginlib/class_list_macros.hpp>

#include "./mag_driver_nodelet.hpp"

namespace a1
{
void MagDriverNodelet::onInit()
{
  node_.reset(new MagDriver(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace a1

PLUGINLIB_EXPORT_CLASS(a1::MagDriverNodelet, nodelet::Nodelet);
