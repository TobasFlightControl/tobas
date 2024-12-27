#include "../../include/tobas_drone_core/joint/command_interface.hpp"

namespace tobas
{
std::string jntCmdIfaceEnumToText(jnt_cmd_iface_t cmd_iface)
{
  switch (cmd_iface)
  {
    case jnt_cmd_iface_t::POSITION:
      return "position";
    case jnt_cmd_iface_t::VELOCITY:
      return "velocity";
    case jnt_cmd_iface_t::EFFORT:
      return "effort";
    default:
      throw;
  }
}

jnt_cmd_iface_t jntCmdIfaceTextToEnum(const std::string& text)
{
  if (text == "position")
    return tobas::jnt_cmd_iface_t::POSITION;
  else if (text == "velocity")
    return tobas::jnt_cmd_iface_t::VELOCITY;
  else if (text == "effort")
    return tobas::jnt_cmd_iface_t::EFFORT;
  else
    throw std::runtime_error("Invalid joint command interface: " + text);
}
}  // namespace tobas
