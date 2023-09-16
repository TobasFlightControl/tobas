#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_mr_pidmpc/rotation_controller_ros.hpp"

namespace tobas_mr_pidmpc
{
class RotationControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<RotationControllerRos> node_;
};
}  // namespace tobas_mr_pidmpc
