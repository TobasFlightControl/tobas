#include <pluginlib/class_list_macros.hpp>

#include "./wind_estimator_nodelet.hpp"

namespace tobas_mr_wind_estimation
{
void WindEstimatorNodelet::onInit()
{
  node_.reset(new WindEstimator(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_mr_wind_estimation

PLUGINLIB_EXPORT_CLASS(tobas_mr_wind_estimation::WindEstimatorNodelet, nodelet::Nodelet);
