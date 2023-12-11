#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_task_space_control/velocity_controller_ros.hpp"

namespace tobas_task_space_control
{
class VelocityControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<VelocityControllerRos> node_;
};
}  // namespace tobas_task_space_control
