#include "../../include/tobas_drone_core/joint/hardware_interface.hpp"

namespace tobas
{
std::string jntHwIfaceEnumToText(jnt_hw_iface_t cmd_iface)
{
  switch (cmd_iface)
  {
    case jnt_hw_iface_t::PWM:
      return "pwm";
    case jnt_hw_iface_t::OTHER:
      return "other";
    default:
      throw;
  }
}

jnt_hw_iface_t jntHwIfaceTextToEnum(const std::string& text)
{
  if (text == "pwm")
    return tobas::jnt_hw_iface_t::PWM;
  else if (text == "other")
    return tobas::jnt_hw_iface_t::OTHER;
  else
    throw std::runtime_error("Invalid joint hardware interface: " + text);
}
}  // namespace tobas
