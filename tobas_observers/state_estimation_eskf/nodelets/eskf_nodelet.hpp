#pragma once

#include <nodelet/nodelet.h>

#include "../include/state_estimation_eskf/eskf_ros.hpp"

namespace state_estimation_eskf
{
class ErrorStateKalmanFilterNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<ErrorStateKalmanFilterRos> node_;
};
}  // namespace state_estimation_eskf
