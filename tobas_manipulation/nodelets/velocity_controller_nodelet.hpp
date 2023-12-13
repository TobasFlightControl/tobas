#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_manipulation/velocity_controller_ros.hpp"

namespace tobas_manipulation
{
class VelocityControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<VelocityControllerRos> node_;
};
}  // namespace tobas_manipulation
