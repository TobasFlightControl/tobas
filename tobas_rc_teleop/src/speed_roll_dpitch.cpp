#include "../include/tobas_rc_teleop/speed_roll_dpitch.hpp"

#include <tobas_std_tools/check.hpp>

using namespace std;

namespace tobas_rc_teleop
{
SpeedRollDeltaPitchController::SpeedRollDeltaPitchController()
{
}

bool SpeedRollDeltaPitchController::requirePosition()
{
  return false;
}

bool SpeedRollDeltaPitchController::requireOrientation()
{
  return true;
}

bool SpeedRollDeltaPitchController::requireLinearVelocity()
{
  return true;
}

bool SpeedRollDeltaPitchController::requireAngularVelocity()
{
  return false;
}

void SpeedRollDeltaPitchController::initialize(tobas::BaseNode* node, tobas::flight_mode_t mode)
{
  node->addDynamicDoubleParam(addMode("min_speed", mode), &self::minSpeedCb, this, 5., 0., 10.);
  node->addDynamicDoubleParam(addMode("max_speed", mode), &self::maxSpeedCb, this, 20., 0., 40.);
  node->addDynamicDoubleParam(addMode("max_roll", mode), &self::maxRollCb, this, M_PI_2, 0., M_PI);
  node->addDynamicDoubleParam(addMode("max_delta_pitch", mode), &self::maxDeltaPitchCb, this, M_PI_4, 0., M_PI_2);
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
  auto cmd = make_unique<tobas_command_msgs::msg::SpeedRollDeltaPitch>();
  cmd->header = rcin.header;

  const auto throttle = expo(remap(rcin.throttle, 0., 1.), speed_expo_);  // [-1, 1] -> [0, 1] -> [0, 1]
  cmd->speed = math::remap(throttle, 0., 1., min_speed_, max_speed_);     // TODO: 機体の制限速度を考慮

  cmd->roll = remapDead(rcin.roll, -max_roll_, max_roll_);
  cmd->delta_pitch = remapDead(rcin.pitch, -max_dpitch_, max_dpitch_);

  // コマンドを発行
  cmd_pub_->publish(move(cmd));
}

bool SpeedRollDeltaPitchController::minSpeedCb(const double& p)
{
  if (p >= max_speed_) {
    cerr << "Minimum speed must be lower than maximum speed." << endl;
    return false;
  }

  min_speed_ = p;
  return true;
}

bool SpeedRollDeltaPitchController::maxSpeedCb(const double& p)
{
  if (p <= min_speed_) {
    cerr << "Maximum speed must be greater than minimum speed." << endl;
    return false;
  }

  max_speed_ = p;
  return true;
}

bool SpeedRollDeltaPitchController::maxRollCb(const double& p)
{
  max_roll_ = p;
  return true;
}

bool SpeedRollDeltaPitchController::maxDeltaPitchCb(const double& p)
{
  max_dpitch_ = p;
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
