#include <pluginlib/class_list_macros.hpp>

#include "./orientation_estimator_nodelet.hpp"

namespace orientation_estimation_complement
{
void OrientationEstimatorNodelet::onInit()
{
  NODELET_INFO("Initializing Complementary Orientation Estimator Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new OrientationEstimatorRos(nh, pnh, name));
}
}  // namespace orientation_estimation_complement

PLUGINLIB_EXPORT_CLASS(
  orientation_estimation_complement::OrientationEstimatorNodelet,
  nodelet::Nodelet);
