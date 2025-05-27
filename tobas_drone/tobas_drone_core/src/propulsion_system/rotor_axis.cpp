#include "tobas_drone_core/propulsion_system/rotor_axis.hpp"

#include <iostream>

#define X_POSITIVE_TEXT "x_positive"
#define Z_POSITIVE_TEXT "z_positive"
#define UNKNOWN_TEXT "unknown"

using namespace std;

namespace tobas
{
string textFromEnum(rotor_axis_t axis)
{
  switch (axis) {
    case rotor_axis_t::X_POSITIVE:
      return X_POSITIVE_TEXT;
    case rotor_axis_t::Z_POSITIVE:
      return Z_POSITIVE_TEXT;
    case rotor_axis_t::UNKNOWN:
      return UNKNOWN_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, rotor_axis_t& dst)
{
  if (text == X_POSITIVE_TEXT) {
    dst = tobas::rotor_axis_t::X_POSITIVE;
    return true;
  }
  else if (text == Z_POSITIVE_TEXT) {
    dst = tobas::rotor_axis_t::Z_POSITIVE;
    return true;
  }
  else if (text == UNKNOWN_TEXT) {
    dst = tobas::rotor_axis_t::UNKNOWN;
    return true;
  }
  else {
    cerr << "Invalid rotor axis: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::rotor_axis_t>::encode(const tobas::rotor_axis_t& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::rotor_axis_t>::decode(const Node& node, tobas::rotor_axis_t& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
