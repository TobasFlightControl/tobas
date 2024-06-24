#include <pluginlib/class_list_macros.hpp>

#include "./orientation_estimator_nodelet.hpp"

namespace orientation_estimation_complement
{
void OrientationEstimatorNodelet::onInit()
{
  node_.reset(new OrientationEstimatorRos(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace orientation_estimation_complement

PLUGINLIB_EXPORT_CLASS(orientation_estimation_complement::OrientationEstimatorNodelet, nodelet::Nodelet);
