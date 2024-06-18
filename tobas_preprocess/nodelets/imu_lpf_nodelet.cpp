#include <pluginlib/class_list_macros.hpp>

#include "./imu_lpf_nodelet.hpp"

namespace tobas_preprocess
{
void ImuLpfNodelet::onInit()
{
  NODELET_INFO("Initializing IMU LPF Nodelet.");
  node_.reset(new ImuLpf(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_preprocess

PLUGINLIB_EXPORT_CLASS(tobas_preprocess::ImuLpfNodelet, nodelet::Nodelet);
