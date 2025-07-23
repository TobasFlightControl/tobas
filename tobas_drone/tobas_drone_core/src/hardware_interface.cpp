#include "tobas_drone_core/hardware_interface.hpp"

#include <iostream>

#define PWM_TEXT "pwm"
#define OTHER_TEXT "other"

using namespace std;

namespace tobas
{
string textFromEnum(HardwareInterface value)
{
  switch (value) {
    case HardwareInterface::kPwm:
      return PWM_TEXT;
    case HardwareInterface::kOther:
      return OTHER_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, HardwareInterface& dst)
{
  if (text == PWM_TEXT) {
    dst = tobas::HardwareInterface::kPwm;
    return true;
  }
  else if (text == OTHER_TEXT) {
    dst = tobas::HardwareInterface::kOther;
    return true;
  }
  else {
    cerr << "Invalid joint hardware interface: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::HardwareInterface>::encode(const tobas::HardwareInterface& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::HardwareInterface>::decode(const Node& node, tobas::HardwareInterface& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
