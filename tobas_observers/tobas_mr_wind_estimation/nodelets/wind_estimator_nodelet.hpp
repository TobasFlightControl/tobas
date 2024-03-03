#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_mr_wind_estimation/wind_estimator.hpp"

namespace tobas_mr_wind_estimation
{
class WindEstimatorNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<WindEstimator> node_;
};
}  // namespace tobas_mr_wind_estimation
