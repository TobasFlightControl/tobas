#include <pluginlib/class_list_macros.hpp>

#include "./eskf_nodelet.hpp"

namespace state_estimation_eskf
{
void ErrorStateKalmanFilterNodelet::onInit()
{
  node_.reset(new ErrorStateKalmanFilterRos());
}
}  // namespace state_estimation_eskf

PLUGINLIB_EXPORT_CLASS(state_estimation_eskf::ErrorStateKalmanFilterNodelet, nodelet::Nodelet);
