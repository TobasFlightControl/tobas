#include "tobas_drone_core/propulsion_system/type.hpp"

#include <iostream>

#define ELECTRIC_TEXT "electric"
#define ICE_TEXT "ice"

using namespace std;

namespace tobas
{
string textFromEnum(propulsion_system_t cmd_iface)
{
  switch (cmd_iface) {
    case propulsion_system_t::ELECTRIC:
      return ELECTRIC_TEXT;
    case propulsion_system_t::ICE:
      return ICE_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, propulsion_system_t& dst)
{
  if (text == ELECTRIC_TEXT) {
    dst = tobas::propulsion_system_t::ELECTRIC;
    return true;
  }
  else if (text == ICE_TEXT) {
    dst = tobas::propulsion_system_t::ICE;
    return true;
  }
  else {
    cerr << "Invalid propulsion system type: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::propulsion_system_t>::encode(const tobas::propulsion_system_t& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::propulsion_system_t>::decode(const Node& node, tobas::propulsion_system_t& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
