#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_mr_thrust_estimation/thrust_estimator.hpp"

namespace tobas_mr_thrust_estimation
{
class ThrustEstimatorNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<ThrustEstimator> node_;
};
}  // namespace tobas_mr_thrust_estimation
