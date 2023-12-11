#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_cartesian_manipulation/effort_controller_ros.hpp"

namespace tobas_cartesian_manipulation
{
class EffortControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<EffortControllerRos> node_;
};
}  // namespace tobas_cartesian_manipulation
