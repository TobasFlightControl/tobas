#include "tobas_constants/rc_command.hpp"

#include <iostream>

#define RATE_THROTTLE_TEXT "rate_throttle"
#define RATE_THROTTLE_VECTOR_TEXT "rate_throttle_vector"
#define ANGLE_THROTTLE_TEXT "angle_throttle"
#define ANGLE_THROTTLE_VECTOR_TEXT "angle_throttle_vector"
#define ACCEL_YAW_TEXT "accel_yaw"
#define ACCEL_PITCH_YAW_TEXT "accel_pitch_yaw"
#define POS_VEL_ACC_YAW_TEXT "pos_vel_acc_yaw"
#define POS_VEL_ACC_PITCH_YAW_TEXT "pos_vel_acc_pitch_yaw"
#define ACCEL_RATE_TEXT "accel_rate"
#define ACCEL_ANGLE_TEXT "accel_angle"
#define POS_VEL_ACC_ANGLE_TEXT "pos_vel_acc_angle"
#define SPEED_ROLL_DPITCH_TEXT "speed_roll_dpitch"

using namespace std;

namespace tobas
{
string textFromEnum(RcCommand cmd)
{
  switch (cmd) {
    case RcCommand::kRateThrottle:
      return RATE_THROTTLE_TEXT;
    case RcCommand::kRateThrottleVector:
      return RATE_THROTTLE_VECTOR_TEXT;
    case RcCommand::kAngleThrottle:
      return ANGLE_THROTTLE_TEXT;
    case RcCommand::kAngleThrottleVector:
      return ANGLE_THROTTLE_VECTOR_TEXT;
    case RcCommand::kAccelYaw:
      return ACCEL_YAW_TEXT;
    case RcCommand::kAccelPitchYaw:
      return ACCEL_PITCH_YAW_TEXT;
    case RcCommand::kPosVelAccYaw:
      return POS_VEL_ACC_YAW_TEXT;
    case RcCommand::kPosVelAccPitchYaw:
      return POS_VEL_ACC_PITCH_YAW_TEXT;
    case RcCommand::kAccelRate:
      return ACCEL_RATE_TEXT;
    case RcCommand::kAccelAngle:
      return ACCEL_ANGLE_TEXT;
    case RcCommand::kPosVelAccAngle:
      return POS_VEL_ACC_ANGLE_TEXT;
    case RcCommand::kSpeedRollDPitch:
      return SPEED_ROLL_DPITCH_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, RcCommand& dst)
{
  if (text == RATE_THROTTLE_TEXT) {
    dst = RcCommand::kRateThrottle;
    return true;
  }
  else if (text == RATE_THROTTLE_VECTOR_TEXT) {
    dst = RcCommand::kRateThrottleVector;
    return true;
  }
  else if (text == ANGLE_THROTTLE_TEXT) {
    dst = RcCommand::kAngleThrottle;
    return true;
  }
  else if (text == ANGLE_THROTTLE_VECTOR_TEXT) {
    dst = RcCommand::kAngleThrottleVector;
    return true;
  }
  else if (text == ACCEL_YAW_TEXT) {
    dst = RcCommand::kAccelYaw;
    return true;
  }
  else if (text == ACCEL_PITCH_YAW_TEXT) {
    dst = RcCommand::kAccelPitchYaw;
    return true;
  }
  else if (text == POS_VEL_ACC_YAW_TEXT) {
    dst = RcCommand::kPosVelAccYaw;
    return true;
  }
  else if (text == POS_VEL_ACC_PITCH_YAW_TEXT) {
    dst = RcCommand::kPosVelAccPitchYaw;
    return true;
  }
  else if (text == ACCEL_RATE_TEXT) {
    dst = RcCommand::kAccelRate;
    return true;
  }
  else if (text == ACCEL_ANGLE_TEXT) {
    dst = RcCommand::kAccelAngle;
    return true;
  }
  else if (text == POS_VEL_ACC_ANGLE_TEXT) {
    dst = RcCommand::kPosVelAccAngle;
    return true;
  }
  else if (text == SPEED_ROLL_DPITCH_TEXT) {
    dst = RcCommand::kSpeedRollDPitch;
    return true;
  }
  else {
    cerr << "Invalid RC command: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::RcCommand>::encode(const tobas::RcCommand& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::RcCommand>::decode(const Node& node, tobas::RcCommand& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
