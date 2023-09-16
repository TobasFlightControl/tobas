#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_mr_pidmpc/controller_ros.hpp"

namespace tobas_mr_pidmpc
{
class ControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<ControllerRos> node_;
};
}  // namespace tobas_mr_pidmpc
