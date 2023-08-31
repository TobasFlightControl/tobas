#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_rc_teleop/roll_pitch_yawrate_thrust.hpp"

namespace tobas_rc_teleop
{
class RcinToRollPitchYawrateThrustNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<RcinToRollPitchYawrateThrust> node_;
};
}  // namespace tobas_rc_teleop
