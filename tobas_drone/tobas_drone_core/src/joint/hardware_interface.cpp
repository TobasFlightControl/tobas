#include <iostream>

#include "../../include/tobas_drone_core/joint/hardware_interface.hpp"

#define PWM_TEXT "pwm"
#define OTHER_TEXT "other"

using namespace std;

namespace tobas
{
string textFromEnum(jnt_hw_iface_t cmd_iface)
{
  switch (cmd_iface)
  {
    case jnt_hw_iface_t::PWM:
      return PWM_TEXT;
    case jnt_hw_iface_t::OTHER:
      return OTHER_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, jnt_hw_iface_t& dst)
{
  if (text == PWM_TEXT)
  {
    dst = tobas::jnt_hw_iface_t::PWM;
    return true;
  }
  else if (text == OTHER_TEXT)
  {
    dst = tobas::jnt_hw_iface_t::OTHER;
    return true;
  }
  else
  {
    cerr << "Invalid joint hardware interface: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::jnt_hw_iface_t>::encode(const tobas::jnt_hw_iface_t& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::jnt_hw_iface_t>::decode(const Node& node, tobas::jnt_hw_iface_t& rhs)
{
  if (!node.IsScalar())
    return false;

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
