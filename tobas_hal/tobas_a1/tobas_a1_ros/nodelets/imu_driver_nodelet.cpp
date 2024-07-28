#include <pluginlib/class_list_macros.hpp>

#include "./imu_driver_nodelet.hpp"

namespace a1
{
void IMUDriverNodelet::onInit()
{
  node_.reset(new IMUDriver(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace a1

PLUGINLIB_EXPORT_CLASS(a1::IMUDriverNodelet, nodelet::Nodelet);
