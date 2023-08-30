#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_multirotor_controller/velocity_controller_ros.hpp"

namespace tobas_multirotor_controller
{
class VelocityControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<VelocityControllerRos> node_;
};
}  // namespace tobas_multirotor_controller
