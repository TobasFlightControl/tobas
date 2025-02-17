#include <iostream>

#include "../../include/tobas_drone_core/joint/command_interface.hpp"

#define POSITION_TEXT "position"
#define VELOCITY_TEXT "velocity"
#define EFFORT_TEXT "effort"
#define NONE_TEXT "none"

using namespace std;

namespace tobas
{
string textFromEnum(jnt_cmd_iface_t cmd_iface)
{
  switch (cmd_iface)
  {
    case jnt_cmd_iface_t::POSITION:
      return POSITION_TEXT;
    case jnt_cmd_iface_t::VELOCITY:
      return VELOCITY_TEXT;
    case jnt_cmd_iface_t::EFFORT:
      return EFFORT_TEXT;
    case jnt_cmd_iface_t::NONE:
      return NONE_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, jnt_cmd_iface_t& dst)
{
  if (text == POSITION_TEXT)
  {
    dst = tobas::jnt_cmd_iface_t::POSITION;
    return true;
  }
  else if (text == VELOCITY_TEXT)
  {
    dst = tobas::jnt_cmd_iface_t::VELOCITY;
    return true;
  }
  else if (text == EFFORT_TEXT)
  {
    dst = tobas::jnt_cmd_iface_t::EFFORT;
    return true;
  }
  else if (text == NONE_TEXT)
  {
    dst = tobas::jnt_cmd_iface_t::NONE;
    return true;
  }
  else
  {
    cerr << "Invalid joint command interface: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::jnt_cmd_iface_t>::encode(const tobas::jnt_cmd_iface_t& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::jnt_cmd_iface_t>::decode(const Node& node, tobas::jnt_cmd_iface_t& rhs)
{
  if (!node.IsScalar())
    return false;

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
