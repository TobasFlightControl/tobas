#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_joint_control/velocity_controller_ros.hpp"

namespace tobas_joint_control
{
class VelocityControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<VelocityControllerRos> node_;
};
}  // namespace tobas_joint_control
