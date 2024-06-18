#include <pluginlib/class_list_macros.hpp>

#include "./battery_lpf_nodelet.hpp"

namespace tobas_preprocess
{
void BatteryLpfNodelet::onInit()
{
  NODELET_INFO("Initializing Battery LPF Nodelet.");
  node_.reset(new BatteryLpf(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_preprocess

PLUGINLIB_EXPORT_CLASS(tobas_preprocess::BatteryLpfNodelet, nodelet::Nodelet);
