#include "tobas_rc_teleop/angle_throttle_vector.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
{
AngleThrottleVectorController::AngleThrottleVectorController()
{
}

bool AngleThrottleVectorController::requirePosition()
{
  return false;
}

bool AngleThrottleVectorController::requireVelocity()
{
  return false;
}

bool AngleThrottleVectorController::requireAttitude()
{
  return true;
}

bool AngleThrottleVectorController::requireHeading()
{
  return true;
}

void AngleThrottleVectorController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicIntParam(addMode("max_roll", mode), &self::maxRollCb, this, 45, 0, 80, " deg");
  node->addDynamicIntParam(addMode("max_roll_rate", mode), &self::maxRollRateCb, this, 360, 0, 720, " dps");
  node->addDynamicIntParam(addMode("max_pitch", mode), &self::maxPitchCb, this, 90, 0, 180, " deg");
  node->addDynamicIntParam(addMode("max_pitch_rate", mode), &self::maxPitchRateCb, this, 180, 0, 360, " dps");
  node->addDynamicIntParam(addMode("max_yaw_rate", mode), &self::maxYawRateCb, this, 180, 0, 360, " dps");
  node->addDynamicIntParam(addMode("max_thrust_angle", mode), &self::maxThrustAngleCb, this, 90, 0, 180, " deg");
  node->addDynamicIntParam(
    addMode("max_thrust_angle_rate", mode), &self::maxThrustAngleRateCb, this, 360, 0, 720, " dps");
  node->addDynamicIntParam(addMode("roll_expo", mode), &self::rollExpoCb, this, -30, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("yaw_expo", mode), &self::yawExpoCb, this, -15, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("throttle_expo", mode), &self::throttleExpoCb, this, 0, 0, kExpoScale);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AngleThrottleVector>(tobas::topic::kAngleThrotVectorCmd);
}

void AngleThrottleVectorController::reset(const tobas_msgs::Odometry& odom, bool)
{
  t_last_rcin_ = odom.header.stamp;

  const auto [roll, pitch, yaw] = odom.frame.M.getRPY();
  roll_filt_.resetCurrentTrajectoryPoint(roll);
  pitch_filt_.resetCurrentTrajectoryPoint(pitch);
  thrust_angle_filt_.resetCurrentTrajectoryPoint(-pitch);
  tar_yaw_ = yaw;
}

void AngleThrottleVectorController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&, bool)
{
  // Update timestamp
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Create a command
  auto cmd = std::make_unique<tobas_command_msgs::AngleThrottleVector>();
  cmd->header = rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;

  // Roll
  roll_filt_.setTargetPosition(expoRemapDead(rcin.roll, roll_expo_, -max_roll_, max_roll_));
  roll_filt_.update(dt);
  cmd->angle.roll = roll_filt_.getTrajectoryPosition();

  // Yaw
  const auto yawrate = expoRemapDead(rcin.yaw, yaw_expo_, -max_yaw_rate_, max_yaw_rate_);
  tar_yaw_ += yawrate * dt;
  cmd->angle.yaw = tar_yaw_;

  // Throttle
  cmd->throttle = expo(remap(rcin.throttle, tobas::kMinThrot, tobas::kMaxThrot), throt_expo_);

  // Pitch & Thrust Angle
  if (rcin.sub_mode)  // Translation mode
  {
    pitch_filt_.setTargetPosition(0.);
    thrust_angle_filt_.setTargetPosition(remapDead(rcin.pitch, -max_thrust_angle_, max_thrust_angle_));
  }
  else  // Rotation mode
  {
    const auto tar_pitch = remapDead(rcin.pitch, -max_pitch_, max_pitch_);
    pitch_filt_.setTargetPosition(tar_pitch);
    thrust_angle_filt_.setTargetPosition(-tar_pitch);
  }
  pitch_filt_.update(dt);
  thrust_angle_filt_.update(dt);
  cmd->angle.pitch = pitch_filt_.getTrajectoryPosition();
  cmd->thrust_angle = thrust_angle_filt_.getTrajectoryPosition();

  // Publish the command
  cmd_pub_->publish(std::move(cmd));
}

bool AngleThrottleVectorController::maxRollCb(const long& p)
{
  max_roll_ = tbs::deg2rad(p);
  return true;
}

bool AngleThrottleVectorController::maxRollRateCb(const long& p)
{
  roll_filt_.setMaxVelocity(tbs::deg2rad(p));
  return true;
}

bool AngleThrottleVectorController::maxPitchCb(const long& p)
{
  max_pitch_ = tbs::deg2rad(p);
  return true;
}

bool AngleThrottleVectorController::maxPitchRateCb(const long& p)
{
  pitch_filt_.setMaxVelocity(tbs::deg2rad(p));
  return true;
}

bool AngleThrottleVectorController::maxYawRateCb(const long& p)
{
  max_yaw_rate_ = tbs::deg2rad(p);
  return true;
}

bool AngleThrottleVectorController::maxThrustAngleCb(const long& p)
{
  max_thrust_angle_ = tbs::deg2rad(p);
  return true;
}

bool AngleThrottleVectorController::maxThrustAngleRateCb(const long& p)
{
  thrust_angle_filt_.setMaxVelocity(tbs::deg2rad(p));
  return true;
}

bool AngleThrottleVectorController::rollExpoCb(const long& p)
{
  roll_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AngleThrottleVectorController::yawExpoCb(const long& p)
{
  yaw_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AngleThrottleVectorController::throttleExpoCb(const long& p)
{
  throt_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop
