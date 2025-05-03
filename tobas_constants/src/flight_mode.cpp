#include <iostream>

#include "../include/tobas_constants/flight_mode.hpp"

#define ACROBAT_TEXT "acrobat"
#define STABILIZE_TEXT "stabilize"
#define LOITER_TEXT "loiter"

using namespace std;

namespace tobas
{
string textFromEnum(flight_mode_t role)
{
  switch (role) {
    case flight_mode_t::ACROBAT:
      return ACROBAT_TEXT;
    case flight_mode_t::STABILIZE:
      return STABILIZE_TEXT;
    case flight_mode_t::LOITER:
      return LOITER_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, flight_mode_t& dst)
{
  if (text == ACROBAT_TEXT) {
    dst = flight_mode_t::ACROBAT;
    return true;
  }
  else if (text == STABILIZE_TEXT) {
    dst = flight_mode_t::STABILIZE;
    return true;
  }
  else if (text == LOITER_TEXT) {
    dst = flight_mode_t::LOITER;
    return true;
  }
  else {
    cerr << "Invalid flight mode: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::flight_mode_t>::encode(const tobas::flight_mode_t& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::flight_mode_t>::decode(const Node& node, tobas::flight_mode_t& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
