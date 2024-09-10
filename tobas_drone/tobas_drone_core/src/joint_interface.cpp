#include "../include/tobas_drone_core/joint_interface.hpp"

namespace tobas
{
std::string jointIFEnumToText(joint_interface_t interface)
{
  switch (interface)
  {
    case joint_interface_t::POSITION:
      return "position";
    case joint_interface_t::VELOCITY:
      return "velocity";
    case joint_interface_t::EFFORT:
      return "effort";
    default:
      throw;
  }
}

joint_interface_t jointIFTextToEnum(const std::string& text)
{
  if (text == "position")
    return tobas::joint_interface_t::POSITION;
  else if (text == "velocity")
    return tobas::joint_interface_t::VELOCITY;
  else if (text == "effort")
    return tobas::joint_interface_t::EFFORT;
  else
    throw std::runtime_error("Invalid joint interface: " + text);
}
}  // namespace tobas
