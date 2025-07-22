#include "tobas_drone_core/joint/command_interface.hpp"

#include <iostream>

#define POSITION_TEXT "position"
#define VELOCITY_TEXT "velocity"
#define EFFORT_TEXT "effort"
#define NONE_TEXT "none"

using namespace std;

namespace tobas
{
string textFromEnum(JointCommandInterface value)
{
  switch (value) {
    case JointCommandInterface::kPosition:
      return POSITION_TEXT;
    case JointCommandInterface::kVelocity:
      return VELOCITY_TEXT;
    case JointCommandInterface::kEffort:
      return EFFORT_TEXT;
    case JointCommandInterface::kNone:
      return NONE_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, JointCommandInterface& dst)
{
  if (text == POSITION_TEXT) {
    dst = tobas::JointCommandInterface::kPosition;
    return true;
  }
  else if (text == VELOCITY_TEXT) {
    dst = tobas::JointCommandInterface::kVelocity;
    return true;
  }
  else if (text == EFFORT_TEXT) {
    dst = tobas::JointCommandInterface::kEffort;
    return true;
  }
  else if (text == NONE_TEXT) {
    dst = tobas::JointCommandInterface::kNone;
    return true;
  }
  else {
    cerr << "Invalid joint command interface: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::JointCommandInterface>::encode(const tobas::JointCommandInterface& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::JointCommandInterface>::decode(const Node& node, tobas::JointCommandInterface& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
