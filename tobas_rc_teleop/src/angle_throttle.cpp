#include <tobas_kdl/euler.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_rc_teleop/angle_throttle.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;

namespace tobas_rc_teleop
{
AngleThrottleController::AngleThrottleController()
{
}

void AngleThrottleController::initialize(tobas::BaseNode* node)
{
  getStaticRosParams(node);

  rpyt_pub_ = node->createPublisher<tobas_command_msgs::msg::AngleThrottle>(tobas::kAngleThrottleCmdTopic);
}

void AngleThrottleController::reset(const tobas_msgs::Odometry& odom)
{
  yaw_ = kdl::Euler(odom.frame.M).yaw;
  t_last_rcin_ = odom.header.stamp;
}

void AngleThrottleController::update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Yawの目標値を更新
  const auto yawrate = remapDead(rcin.yaw, -max_yawrate_, max_yawrate_);
  yaw_ += yawrate * dt;

  // コマンドを作成
  auto cmd = std::make_unique<tobas_command_msgs::msg::AngleThrottle>();
  cmd->header = rcin.header;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;

  // 姿勢とスロットルを埋める
  cmd->roll = remapDead(rcin.roll, -max_attitude_, max_attitude_);
  cmd->pitch = remapDead(rcin.pitch, -max_attitude_, max_attitude_);
  cmd->yaw = yaw_;
  cmd->throttle = remap(rcin.throttle, tobas::kMinThrot, tobas::kMaxThrot);

  // コマンドを発行
  rpyt_pub_->publish(move(cmd));
}

void AngleThrottleController::getStaticRosParams(tobas::BaseNode* node)
{
  max_attitude_ = node->getDoubleParam("max_attitude", kDefaultMaxAttitude);
  if (max_attitude_ < 0)
  {
    node->error("Maximum attitude angle must be positive.");
    max_attitude_ = kDefaultMaxAttitude;
  }

  max_yawrate_ = node->getDoubleParam("max_yawrate", kDefaultMaxYawrate);
  if (max_yawrate_ < 0)
  {
    node->error("Maximum yawrate must be positive.");
    max_yawrate_ = kDefaultMaxYawrate;
  }
}
}  // namespace tobas_rc_teleop
