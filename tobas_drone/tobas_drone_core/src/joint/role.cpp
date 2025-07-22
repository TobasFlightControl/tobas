#include "tobas_drone_core/joint/role.hpp"

#include <iostream>

#define TILT_JOINT "tilt_joint"
#define CONTROL_SURFACE "control_surface"
#define LANDING_GEAR "landing_gear"
#define PASSIVE_WHEEL "passive_wheel"
#define MANIPULATION "manipulation"
#define OTHER "other"  

using namespace std;

namespace tobas
{
string textFromEnum(JointRole role)
{
  switch (role) {
    case JointRole::kTiltJoint:
      return TILT_JOINT;
    case JointRole::kControlSurface:
      return CONTROL_SURFACE;
    case JointRole::kLandingGear:
      return LANDING_GEAR;
    case JointRole::kPassiveWheel:
      return PASSIVE_WHEEL;
    case JointRole::kManipulation:
      return MANIPULATION;
    case JointRole::kOther:
      return OTHER;
    default:
      throw;
  }
}

bool enumFromText(const string& text, JointRole& dst)
{
  if (text == TILT_JOINT) {
    dst = JointRole::kTiltJoint;
    return true;
  }
  else if (text == CONTROL_SURFACE) {
    dst = JointRole::kControlSurface;
    return true;
  }
  else if (text == LANDING_GEAR) {
    dst = JointRole::kLandingGear;
    return true;
  }
  else if (text == PASSIVE_WHEEL) {
    dst = JointRole::kPassiveWheel;
    return true;
  }
  else if (text == MANIPULATION) {
    dst = JointRole::kManipulation;
    return true;
  }
  else if (text == OTHER) {
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
