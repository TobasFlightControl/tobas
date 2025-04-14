#include <tobas_std_tools/check.hpp>

#include "../include/tobas_rc_teleop/speed_roll_dpitch.hpp"

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
  cmd->speed = remap(rcin.throttle, min_speed_, max_speed_);  // TODO: 機体の制限速度を考慮
  cmd->roll = remapDead(rcin.roll, -max_roll_, max_roll_);
  cmd->delta_pitch = remapDead(rcin.pitch, -max_dpitch_, max_dpitch_);

  // コマンドを発行
  cmd_pub_->publish(move(cmd));
}

bool SpeedRollDeltaPitchController::minSpeedCb(const double& p)
{
  if (p >= max_speed_)
  {
    cerr << "Minimum speed must be lower than maximum speed." << endl;
    return false;
  }

  min_speed_ = p;
  return true;
}

bool SpeedRollDeltaPitchController::maxSpeedCb(const double& p)
{
  if (p <= min_speed_)
  {
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
}  // namespace tobas_rc_teleop
