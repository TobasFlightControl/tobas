#include <iostream>

#include "tobas_drone_core/propulsion_system/turning_direction.hpp"

#define CCW_TEXT "ccw"
#define CW_TEXT "cw"

using namespace std;

namespace tobas
{
string textFromEnum(turning_direction_t cmd_iface)
{
  switch (cmd_iface) {
    case turning_direction_t::CCW:
      return CCW_TEXT;
    case turning_direction_t::CW:
      return CW_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, turning_direction_t& dst)
{
  if (text == CCW_TEXT) {
    dst = tobas::turning_direction_t::CCW;
    return true;
  }
  else if (text == CW_TEXT) {
    dst = tobas::turning_direction_t::CW;
    return true;
  }
  else {
    cerr << "Invalid rotor turning direction: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::turning_direction_t>::encode(const tobas::turning_direction_t& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::turning_direction_t>::decode(const Node& node, tobas::turning_direction_t& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
