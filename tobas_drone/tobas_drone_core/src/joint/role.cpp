#include "tobas_drone_core/joint/role.hpp"

#include <iostream>

#define ROTOR_TEXT "rotor"
#define TILT_JOINT_TEXT "tilt_joint"
#define CONTROL_SURFACE_TEXT "control_surface"
#define MANIPULATION_TEXT "manipulation"
#define PASSIVE_WHEEL_TEXT "passive_wheel"
#define OTHER_TEXT "other"

using namespace std;

namespace tobas
{
string textFromEnum(jnt_role_t role)
{
  switch (role) {
    case jnt_role_t::ROTOR:
      return ROTOR_TEXT;
    case jnt_role_t::TILT_JOINT:
      return TILT_JOINT_TEXT;
    case jnt_role_t::CONTROL_SURFACE:
      return CONTROL_SURFACE_TEXT;
    case jnt_role_t::MANIPULATION:
      return MANIPULATION_TEXT;
    case jnt_role_t::PASSIVE_WHEEL:
      return PASSIVE_WHEEL_TEXT;
    case jnt_role_t::OTHER:
      return OTHER_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, jnt_role_t& dst)
{
  if (text == ROTOR_TEXT) {
    dst = jnt_role_t::ROTOR;
    return true;
  }
  else if (text == TILT_JOINT_TEXT) {
    dst = jnt_role_t::TILT_JOINT;
    return true;
  }
  else if (text == CONTROL_SURFACE_TEXT) {
    dst = jnt_role_t::CONTROL_SURFACE;
    return true;
  }
  else if (text == MANIPULATION_TEXT) {
    dst = jnt_role_t::MANIPULATION;
    return true;
  }
  else if (text == PASSIVE_WHEEL_TEXT) {
    dst = jnt_role_t::PASSIVE_WHEEL;
    return true;
  }
  else if (text == OTHER_TEXT) {
    dst = jnt_role_t::OTHER;
    return true;
  }
  else {
    cerr << "Invalid joint role: " << text << endl;
    return false;
  }
}

bool isServoJoint(jnt_role_t role)
{
  switch (role) {
    case jnt_role_t::ROTOR:
      return false;
    case jnt_role_t::TILT_JOINT:
      return true;
    case jnt_role_t::CONTROL_SURFACE:
      return true;
    case jnt_role_t::MANIPULATION:
      return true;
    case jnt_role_t::PASSIVE_WHEEL:
      return false;
    case jnt_role_t::OTHER:
      return false;
    default:
      throw;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::jnt_role_t>::encode(const tobas::jnt_role_t& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::jnt_role_t>::decode(const Node& node, tobas::jnt_role_t& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
