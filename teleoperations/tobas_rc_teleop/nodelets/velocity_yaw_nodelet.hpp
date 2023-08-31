#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_rc_teleop/velocity_yaw.hpp"

namespace tobas_rc_teleop
{
class RcinToVelocityYawNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<RcinToVelocityYaw> node_;
};
}  // namespace tobas_rc_teleop
