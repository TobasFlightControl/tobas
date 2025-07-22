#include "tobas_drone_core/joint/role.hpp"

#include <iostream>

#define TILT_JOINT_TEXT "tilt_joint"
#define CONTROL_SURFACE_TEXT "control_surface"
#define LANDING_GEAR_TEXT "landing_gear"
#define PASSIVE_WHEEL_TEXT "passive_wheel"
#define MANIPULATION_TEXT "manipulation"
#define OTHER_TEXT "other"

using namespace std;

namespace tobas
{
string textFromEnum(JointRole role)
{
  switch (role) {
    case JointRole::kTiltJoint:
      return TILT_JOINT_TEXT;
    case JointRole::kControlSurface:
      return CONTROL_SURFACE_TEXT;
    case JointRole::kLandingGear:
      return LANDING_GEAR_TEXT;
    case JointRole::kPassiveWheel:
      return PASSIVE_WHEEL_TEXT;
    case JointRole::kManipulation:
      return MANIPULATION_TEXT;
    case JointRole::kOther:
      return OTHER_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, JointRole& dst)
{
  if (text == TILT_JOINT_TEXT) {
    dst = JointRole::kTiltJoint;
    return true;
  }
  else if (text == CONTROL_SURFACE_TEXT) {
    dst = JointRole::kControlSurface;
    return true;
  }
  else if (text == LANDING_GEAR_TEXT) {
    dst = JointRole::kLandingGear;
    return true;
  }
  else if (text == PASSIVE_WHEEL_TEXT) {
    dst = JointRole::kPassiveWheel;
    return true;
  }
  else if (text == MANIPULATION_TEXT) {
    dst = JointRole::kManipulation;
    return true;
  }
  else if (text == OTHER_TEXT) {
    dst = JointRole::kOther;
    return true;
  }
  else {
    cerr << "Invalid joint role: " << text << endl;
    return false;
  }
}

bool isServoJoint(JointRole role)
{
  switch (role) {
    case JointRole::kTiltJoint:
      return true;
    case JointRole::kControlSurface:
      return true;
    case JointRole::kLandingGear:
      return true;
    case JointRole::kPassiveWheel:
      return false;
    case JointRole::kManipulation:
      return true;
    case JointRole::kOther:
      return false;
    default:
      throw;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::JointRole>::encode(const tobas::JointRole& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::JointRole>::decode(const Node& node, tobas::JointRole& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
