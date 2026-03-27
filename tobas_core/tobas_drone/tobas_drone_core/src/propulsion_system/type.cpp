#include "tobas_drone_core/propulsion_system/type.hpp"

#include <iostream>

#define ELECTRIC_TEXT "electric"
#define ICE_TEXT "ice"

using namespace std;

namespace tobas
{
string textFromEnum(PropulsionSystem cmd_iface)
{
  switch (cmd_iface) {
    case PropulsionSystem::kElectric:
      return ELECTRIC_TEXT;
    case PropulsionSystem::kIce:
      return ICE_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, PropulsionSystem& dst)
{
  if (text == ELECTRIC_TEXT) {
    dst = PropulsionSystem::kElectric;
    return true;
  }
  else if (text == ICE_TEXT) {
    dst = PropulsionSystem::kIce;
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
Node convert<tobas::PropulsionSystem>::encode(const tobas::PropulsionSystem& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::PropulsionSystem>::decode(const Node& node, tobas::PropulsionSystem& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
