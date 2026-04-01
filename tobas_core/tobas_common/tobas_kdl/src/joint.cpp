// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/joint.hpp"

using namespace std;

namespace tobas
{
namespace kdl
{
bool Joint::isValid(string& error_msg) const
{
  if (name.empty()) {
    error_msg = "Joint name is empty.";
    return false;
  }

  return true;
}

const char* Joint::typeToText(JointType type)
{
  switch (type) {
    case kRotation:
      return "Rotation";
    case kTranslation:
      return "Translation";
    case kFixed:
      return "Fixed";
    default:
      throw;
  }
}

ostream& operator<<(ostream& os, const Joint& arg)
{
  os << "Name: " << arg.name << endl;
  os << "Type: " << Joint::typeToText(arg.type) << endl;
  os << "Origin: " << arg.origin << endl;
  os << "Axis: " << arg.axis() << endl;
  os << "Damping: " << arg.damping << endl;
  os << "Friction: " << arg.friction << endl;
  os << "Lower Limit: " << arg.lower_limit << endl;
  os << "Upper Limit: " << arg.upper_limit << endl;
  os << "Max Effort: " << arg.max_effort << endl;
  os << "Max Velocity: " << arg.max_velocity;
  return os;
}
}  // namespace kdl
}  // namespace tobas
