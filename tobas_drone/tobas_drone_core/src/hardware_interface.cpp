#include <iostream>

#include "tobas_drone_core/hardware_interface.hpp"

#define PWM_TEXT "pwm"
#define OTHER_TEXT "other"

using namespace std;

namespace tobas
{
string textFromEnum(hw_iface_t value)
{
  switch (value)
  {
    case hw_iface_t::PWM:
      return PWM_TEXT;
    case hw_iface_t::OTHER:
      return OTHER_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, hw_iface_t& dst)
{
  if (text == PWM_TEXT)
  {
    dst = tobas::hw_iface_t::PWM;
    return true;
  }
  else if (text == OTHER_TEXT)
  {
    dst = tobas::hw_iface_t::OTHER;
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
Node convert<tobas::hw_iface_t>::encode(const tobas::hw_iface_t& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::hw_iface_t>::decode(const Node& node, tobas::hw_iface_t& rhs)
{
  if (!node.IsScalar())
    return false;

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
