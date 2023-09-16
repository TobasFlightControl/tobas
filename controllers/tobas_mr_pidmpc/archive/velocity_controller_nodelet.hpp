#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_mr_pidmpc/velocity_controller_ros.hpp"

namespace tobas_mr_pidmpc
{
class VelocityControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<VelocityControllerRos> node_;
};
}  // namespace tobas_mr_pidmpc
