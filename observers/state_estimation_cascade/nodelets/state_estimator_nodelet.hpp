#pragma once

#include <nodelet/nodelet.h>

#include "../include/state_estimation_cascade/state_estimator.hpp"

namespace state_estimation_cascade
{
class StateEstimatorNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<StateEstimator> node_;
};
}  // namespace state_estimation_cascade
