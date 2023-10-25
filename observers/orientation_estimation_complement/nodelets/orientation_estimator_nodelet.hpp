#pragma once

#include <nodelet/nodelet.h>

#include "../include/orientation_estimation_complement/orientation_estimator_ros.hpp"

namespace orientation_estimation_complement
{
class OrientationEstimatorNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<OrientationEstimatorRos> node_;
};
}  // namespace orientation_estimation_complement
