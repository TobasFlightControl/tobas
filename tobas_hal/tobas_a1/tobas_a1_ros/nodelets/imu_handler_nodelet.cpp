#include <pluginlib/class_list_macros.hpp>

#include "./imu_handler_nodelet.hpp"

namespace a1
{
void IMUHandlerNodelet::onInit()
{
  node_.reset(new IMUHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace a1

PLUGINLIB_EXPORT_CLASS(a1::IMUHandlerNodelet, nodelet::Nodelet);
