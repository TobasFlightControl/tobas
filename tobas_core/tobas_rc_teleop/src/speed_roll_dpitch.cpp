#include "tobas_rc_teleop/speed_roll_dpitch.hpp"

#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
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

void SpeedRollDeltaPitchController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicDoubleParam(addMode("min_speed", mode), &self::minSpeedCb, this, 0.5, 10, 0, 20, " m/s");
  node->addDynamicDoubleParam(addMode("max_speed", mode), &self::maxSpeedCb, this, 0.5, 40, 0, 80, " m/s");
  node->addDynamicIntParam(addMode("max_roll", mode), &self::maxRollCb, this, 90, 0, 180, " deg");
  node->addDynamicIntParam(addMode("max_delta_pitch", mode), &self::maxDeltaPitchCb, this, 45, 0, 90, " deg");
  node->addDynamicIntParam(addMode("speed_expo", mode), &self::speedExpoCb, this, 0, 0, kExpoScale);
  node->addDynamicIntParam(addMode("roll_expo", mode), &self::rollExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("pitch_expo", mode), &self::pitchExpoCb, this, 0, -kExpoScale, kExpoScale);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::msg::SpeedRollDeltaPitch>(tobas::kSpeedRollDpitchCmdTopic);
}

void SpeedRollDeltaPitchController::reset(const tobas_msgs::Odometry&)
{
}

void SpeedRollDeltaPitchController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // コマンドを作成
  auto cmd = std::make_unique<tobas_command_msgs::msg::SpeedRollDeltaPitch>();
  cmd->header = rcin.header;

  // TODO: 機体の制限速度を考慮
  const auto throttle = expo(remap(rcin.throttle, tobas::kMinThrot, tobas::kMaxThrot), speed_expo_);
  cmd->speed = math::remap(throttle, tobas::kMinThrot, tobas::kMaxThrot, min_speed_, max_speed_);

  cmd->roll = remapDead(rcin.roll, -max_roll_, max_roll_);
  cmd->delta_pitch = remapDead(rcin.pitch, -max_dpitch_, max_dpitch_);

  // コマンドを発行
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

bool SpeedRollDeltaPitchController::maxRollCb(const long& p)
{
  max_roll_ = tbs::deg2rad(p);
  return true;
}

bool SpeedRollDeltaPitchController::maxDeltaPitchCb(const long& p)
{
  max_dpitch_ = tbs::deg2rad(p);
  return true;
}

bool SpeedRollDeltaPitchController::speedExpoCb(const long& p)
{
  speed_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool SpeedRollDeltaPitchController::rollExpoCb(const long& p)
{
  roll_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool SpeedRollDeltaPitchController::pitchExpoCb(const long& p)
{
  pitch_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop
