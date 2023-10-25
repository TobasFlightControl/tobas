#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_mr_mpc/controller_ros.hpp"

namespace tobas_mr_mpc
{
class ControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<ControllerRos> node_;
};
}  // namespace tobas_mr_mpc
