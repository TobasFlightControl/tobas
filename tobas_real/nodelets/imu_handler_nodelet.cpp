#include <pluginlib/class_list_macros.hpp>

#include "./imu_handler_nodelet.hpp"

namespace tobas_real
{
void ImuHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing IMU Handler Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new ImuHandler(nh, pnh, name));
}
}  // namespace tobas_real

PLUGINLIB_EXPORT_CLASS(tobas_real::ImuHandlerNodelet, nodelet::Nodelet);
