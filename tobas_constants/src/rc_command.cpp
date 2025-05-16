#include "tobas_constants/rc_command.hpp"

#include <iostream>

#define RATE_THROTTLE_TEXT "rate_throttle"
#define ANGLE_THROTTLE_TEXT "angle_throttle"
#define ACCEL_YAW_TEXT "accel_yaw"
#define POS_VEL_YAW_TEXT "pos_vel_yaw"
#define ACCEL_RATE_TEXT "accel_rate"
#define ACCEL_ANGLE_TEXT "accel_angle"
#define POS_VEL_ANGLE_TEXT "pos_vel_angle"
#define SPEED_ROLL_DPITCH_TEXT "speed_roll_dpitch"

using namespace std;

namespace tobas
{
string textFromEnum(rc_command_t role)
{
  switch (role) {
    case rc_command_t::RATE_THROTTLE:
      return RATE_THROTTLE_TEXT;
    case rc_command_t::ANGLE_THROTTLE:
      return ANGLE_THROTTLE_TEXT;
    case rc_command_t::ACCEL_YAW:
      return ACCEL_YAW_TEXT;
    case rc_command_t::POS_VEL_YAW:
      return POS_VEL_YAW_TEXT;
    case rc_command_t::ACCEL_RATE:
      return ACCEL_RATE_TEXT;
    case rc_command_t::ACCEL_ANGLE:
      return ACCEL_ANGLE_TEXT;
    case rc_command_t::POS_VEL_ANGLE:
      return POS_VEL_ANGLE_TEXT;
    case rc_command_t::SPEED_ROLL_DPITCH:
      return SPEED_ROLL_DPITCH_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, rc_command_t& dst)
{
  if (text == RATE_THROTTLE_TEXT) {
    dst = rc_command_t::RATE_THROTTLE;
    return true;
  }
  else if (text == ANGLE_THROTTLE_TEXT) {
    dst = rc_command_t::ANGLE_THROTTLE;
    return true;
  }
  else if (text == ACCEL_YAW_TEXT) {
    dst = rc_command_t::ACCEL_YAW;
    return true;
  }
  else if (text == POS_VEL_YAW_TEXT) {
    dst = rc_command_t::POS_VEL_YAW;
    return true;
  }
  else if (text == ACCEL_RATE_TEXT) {
    dst = rc_command_t::ACCEL_RATE;
    return true;
  }
  else if (text == ACCEL_ANGLE_TEXT) {
    dst = rc_command_t::ACCEL_ANGLE;
    return true;
  }
  else if (text == POS_VEL_ANGLE_TEXT) {
    dst = rc_command_t::POS_VEL_ANGLE;
    return true;
  }
  else if (text == SPEED_ROLL_DPITCH_TEXT) {
    dst = rc_command_t::SPEED_ROLL_DPITCH;
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
Node convert<tobas::rc_command_t>::encode(const tobas::rc_command_t& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::rc_command_t>::decode(const Node& node, tobas::rc_command_t& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
