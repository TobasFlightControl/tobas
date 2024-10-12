#include <tobas_kdl/euler.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_rc_teleop/rpy_throttle.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;

namespace tobas_rc_teleop
{
RollPitchYawThrottleController::RollPitchYawThrottleController()
{
}

void RollPitchYawThrottleController::initialize(tobas::BaseNode* node)
{
  getStaticRosParams(node);

  rpyt_pub_ = node->createPublisher<tobas_msgs::RollPitchYawThrottle>(tobas::kRPYThrotCmdTopic);
}

void RollPitchYawThrottleController::reset(const tobas_msgs::Odometry& odom)
{
  yaw_ = kdl::Euler(odom.frame.M).yaw;
  t_last_rcin_ = odom.header.stamp;
}

void RollPitchYawThrottleController::update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Yawの目標値を更新
  const auto yawrate = remapDead(rcin.yaw, -max_yawrate_, max_yawrate_);
  yaw_ += yawrate * dt;

  // コマンドを作成
  auto rpyt = std::make_unique<tobas_msgs::RollPitchYawThrottle>();
  rpyt->level.data = tobas_msgs::msg::CommandLevel::MANUAL;

  // 姿勢とスロットルを埋める
  rpyt->rpy.roll = remapDead(rcin.roll, -max_attitude_, max_attitude_);
  rpyt->rpy.pitch = remapDead(rcin.pitch, -max_attitude_, max_attitude_);
  rpyt->rpy.yaw = yaw_;
  rpyt->throttle = rcin.throttle;

  // コマンドを発行
  rpyt_pub_->publish(move(rpyt));
}

void RollPitchYawThrottleController::getStaticRosParams(tobas::BaseNode* node)
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
