#include <pluginlib/class_list_macros.hpp>

#include "./wind_estimator_nodelet.hpp"

namespace tobas_mr_wind_estimation
{
void WindEstimatorNodelet::onInit()
{
  NODELET_INFO("Initializing Wind Estimator Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new WindEstimator(nh, pnh, name));
}
}  // namespace tobas_mr_wind_estimation

PLUGINLIB_EXPORT_CLASS(tobas_mr_wind_estimation::WindEstimatorNodelet, nodelet::Nodelet);
