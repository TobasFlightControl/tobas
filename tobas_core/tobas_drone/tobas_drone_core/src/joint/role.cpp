#include "tobas_drone_core/joint/role.hpp"

#include <iostream>

#define TILT_JOINT "tilt_joint"
#define CONTROL_SURFACE "control_surface"
#define MANIPULATION "manipulation"
#define USER_ACTIVE "user_active"
#define USER_PASSIVE "user_passive"

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
    case JointRole::kManipulation:
      return MANIPULATION;
    case JointRole::kUserActive:
      return USER_ACTIVE;
    case JointRole::kUserPassive:
      return USER_PASSIVE;
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
  else if (text == MANIPULATION) {
    dst = JointRole::kManipulation;
    return true;
  }
  else if (text == USER_ACTIVE) {
    dst = JointRole::kUserActive;
    return true;
  }
  else if (text == USER_PASSIVE) {
    dst = JointRole::kUserPassive;
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
    case JointRole::kManipulation:
      return true;
    case JointRole::kUserActive:
      return true;
    case JointRole::kUserPassive:
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
