#include <pluginlib/class_list_macros.hpp>

#include "./battery_lpf_nodelet.hpp"

namespace tobas_preprocess
{
void BatteryLpfNodelet::onInit()
{
  NODELET_INFO("Initializing Battery LPF Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new BatteryLpf(nh, pnh, name));
}
}  // namespace tobas_preprocess

PLUGINLIB_EXPORT_CLASS(tobas_preprocess::BatteryLpfNodelet, nodelet::Nodelet);
