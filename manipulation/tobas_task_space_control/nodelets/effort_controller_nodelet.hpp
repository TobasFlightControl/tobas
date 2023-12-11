#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_task_space_control/effort_controller_ros.hpp"

namespace tobas_task_space_control
{
class EffortControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<EffortControllerRos> node_;
};
}  // namespace tobas_task_space_control
