#include <tobas_std_tools/check.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_rc_teleop/speed_roll_dpitch.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;

namespace tobas_rc_teleop
{
SpeedRollDeltaPitchController::SpeedRollDeltaPitchController()
{
}

void SpeedRollDeltaPitchController::initialize(tobas::BaseNode* node)
{
  getStaticRosParams(node);

  cmd_pub_ = node->createPublisher<tobas_msgs::msg::SpeedRollDeltaPitch>(tobas::kSpeedRollDpitchCmdTopic);
}

void SpeedRollDeltaPitchController::reset(const tobas_msgs::Odometry&)
{
}

void SpeedRollDeltaPitchController::update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // コマンドを作成
  auto cmd = std::make_unique<tobas_msgs::msg::SpeedRollDeltaPitch>();
  cmd->speed = remap(rcin.throttle, min_speed_, max_speed_);  // TODO: 機体の制限速度を考慮
  cmd->roll = remapDead(rcin.roll, -max_roll_, max_roll_);
  cmd->delta_pitch = remapDead(rcin.pitch, -max_dpitch_, max_dpitch_);

  // コマンドを発行
  cmd_pub_->publish(move(cmd));
}

void SpeedRollDeltaPitchController::getStaticRosParams(tobas::BaseNode* node)
{
  min_speed_ = node->getDoubleParam("min_speed", kDefaultMinSpeed);
  max_speed_ = node->getDoubleParam("max_speed", kDefaultMaxSpeed);
  if (min_speed_ <= 0 || max_speed_ < min_speed_)
  {
    node->error("Invalid speed limit.");
    min_speed_ = kDefaultMinSpeed;
    max_speed_ = kDefaultMaxSpeed;
  }

  max_roll_ = node->getDoubleParam("max_roll", kDefaultMaxRoll);
  if (max_roll_ < 0)
  {
    node->error("Maximum roll angle must be positive.");
    max_roll_ = kDefaultMaxRoll;
  }

  max_dpitch_ = node->getDoubleParam("max_dpitch", kDefaultMaxDeltaPitch);
  if (max_dpitch_ < 0)
  {
    node->error("Maximum delta pitch angle must be positive.");
    max_dpitch_ = kDefaultMaxDeltaPitch;
  }
}
}  // namespace tobas_rc_teleop
