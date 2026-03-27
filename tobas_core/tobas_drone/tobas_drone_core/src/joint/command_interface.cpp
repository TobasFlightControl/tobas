#include "tobas_drone_core/joint/command_interface.hpp"

#include <iostream>

#define POSITION "position"
#define VELOCITY "velocity"
#define EFFORT "effort"
#define NONE "none"

using namespace std;

namespace tobas
{
string textFromEnum(JointCommandInterface value)
{
  switch (value) {
    case JointCommandInterface::kPosition:
      return POSITION;
    case JointCommandInterface::kVelocity:
      return VELOCITY;
    case JointCommandInterface::kEffort:
      return EFFORT;
    case JointCommandInterface::kNone:
      return NONE;
    default:
      throw;
  }
}

bool enumFromText(const string& text, JointCommandInterface& dst)
{
  if (text == POSITION) {
    dst = JointCommandInterface::kPosition;
    return true;
  }
  else if (text == VELOCITY) {
    dst = JointCommandInterface::kVelocity;
    return true;
  }
  else if (text == EFFORT) {
    dst = JointCommandInterface::kEffort;
    return true;
  }
  else if (text == NONE) {
    dst = JointCommandInterface::kNone;
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
