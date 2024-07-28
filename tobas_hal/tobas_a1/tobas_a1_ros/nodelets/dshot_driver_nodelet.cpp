#include <pluginlib/class_list_macros.hpp>

#include "./dshot_driver_nodelet.hpp"

namespace a1
{
void DShotDriverNodelet::onInit()
{
  node_.reset(new DShotDriver(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace a1

PLUGINLIB_EXPORT_CLASS(a1::DShotDriverNodelet, nodelet::Nodelet);
