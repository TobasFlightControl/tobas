// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/speed_roll_dpitch.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
SpeedRollDeltaPitchController::SpeedRollDeltaPitchController()
{
}

bool SpeedRollDeltaPitchController::requirePosition()
{
  return false;
}

bool SpeedRollDeltaPitchController::requireVelocity()
{
  return true;
}

bool SpeedRollDeltaPitchController::requireAttitude()
{
  return true;
}

bool SpeedRollDeltaPitchController::requireHeading()
{
  return false;
}

void SpeedRollDeltaPitchController::initialize(BaseNode* node, FlightMode mode)
{
  node->addDynamicDoubleParam(addMode("min_speed", mode), &self::minSpeedCb, this, 0.5, 10, 1, 20, " m/s");
  node->addDynamicDoubleParam(addMode("max_speed", mode), &self::maxSpeedCb, this, 0.5, 40, 1, 80, " m/s");
  node->addDynamicDoubleParam(addMode("max_roll", mode), &self::maxRollCb, this, 15.0, 6, 1, 12, " deg");
  node->addDynamicDoubleParam(addMode("max_delta_pitch", mode), &self::maxDeltaPitchCb, this, 5.0, 9, 1, 18, " deg");
  node->addDynamicDoubleParam(addMode("speed_expo", mode), &self::speedExpoCb, this, 5.0, 0, 0, 20);
  node->addDynamicDoubleParam(addMode("roll_expo", mode), &self::rollExpoCb, this, 5.0, 0, -20, 20);
  node->addDynamicDoubleParam(addMode("pitch_expo", mode), &self::pitchExpoCb, this, 5.0, 0, -20, 20);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::msg::SpeedRollDeltaPitch>(topic::kSpeedRollDpitchCmd);
}

void SpeedRollDeltaPitchController::reset(const builtin_interfaces::msg::Time&, const tobas_msgs::Odometry&, bool)
{
}

void SpeedRollDeltaPitchController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&, bool)
{
  // Create command
  auto cmd = std::make_unique<tobas_command_msgs::msg::SpeedRollDeltaPitch>();
  cmd->header = rcin.header;

  // TODO: Consider the airframe speed limit.
  const auto throttle = expo(remap(rcin.throttle, kMinThrot, kMaxThrot), speed_expo_);
  cmd->speed = math::remap(throttle, kMinThrot, kMaxThrot, min_speed_, max_speed_);

  cmd->roll = remapDead(rcin.roll, -max_roll_, max_roll_);
  cmd->delta_pitch = remapDead(rcin.pitch, -max_dpitch_, max_dpitch_);

  // Publish command
  cmd_pub_->publish(std::move(cmd));
}

bool SpeedRollDeltaPitchController::minSpeedCb(const double& p)
{
  if (p >= max_speed_) {
    std::cerr << "Minimum speed must be lower than maximum speed." << std::endl;
    return false;
  }

  min_speed_ = p;
  return true;
}

bool SpeedRollDeltaPitchController::maxSpeedCb(const double& p)
{
  if (p <= min_speed_) {
    std::cerr << "Maximum speed must be greater than minimum speed." << std::endl;
    return false;
  }

  max_speed_ = p;
  return true;
}

bool SpeedRollDeltaPitchController::maxRollCb(const double& p)
{
  max_roll_ = st::deg2rad(p);
  return true;
}

bool SpeedRollDeltaPitchController::maxDeltaPitchCb(const double& p)
{
  max_dpitch_ = st::deg2rad(p);
  return true;
}

bool SpeedRollDeltaPitchController::speedExpoCb(const double& p)
{
  speed_expo_ = p / kExpoScale;
  return true;
}

bool SpeedRollDeltaPitchController::rollExpoCb(const double& p)
{
  roll_expo_ = p / kExpoScale;
  return true;
}

bool SpeedRollDeltaPitchController::pitchExpoCb(const double& p)
{
  pitch_expo_ = p / kExpoScale;
  return true;
}
}  // namespace rc
}  // namespace tobas
