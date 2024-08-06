#pragma once

#include <cinttypes>
#include <yaml-cpp/yaml.h>

namespace tobas
{
enum esc_mode_t : uint8_t
{
  BLHELI_OPEN_LOOP,
  BLHELI_CLOSED_LOOP_LOW_RANGE,
  BLHELI_CLOSED_LOOP_MID_RANGE,
  BLHELI_CLOSED_LOOP_HIGH_RANGE,
};

static constexpr double kBLHeliCLLowMaxERPM = 50000;    // The maximum ERPM in BLHeli closed loop low range mode
static constexpr double kBLHeliCLMidMaxERPM = 100000;   // The maximum ERPM in BLHeli closed loop middle range mode
static constexpr double kBLHeliCLHighMaxERPM = 200000;  // The maximum ERPM in BLHeli closed loop high range mode
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::esc_mode_t>
{
  static Node encode(const tobas::esc_mode_t& rhs)
  {
    Node node;

    switch (rhs)
    {
      case tobas::esc_mode_t::BLHELI_OPEN_LOOP:
        node = "BLHELI_OPEN_LOOP";
        break;
      case tobas::esc_mode_t::BLHELI_CLOSED_LOOP_LOW_RANGE:
        node = "BLHELI_CLOSED_LOOP_LOW_RANGE";
        break;
      case tobas::esc_mode_t::BLHELI_CLOSED_LOOP_MID_RANGE:
        node = "BLHELI_CLOSED_LOOP_MID_RANGE";
        break;
      case tobas::esc_mode_t::BLHELI_CLOSED_LOOP_HIGH_RANGE:
        node = "BLHELI_CLOSED_LOOP_HIGH_RANGE";
        break;
    }

    return node;
  }

  static bool decode(const Node& node, tobas::esc_mode_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    const auto value = node.as<std::string>();
    if (value == "BLHELI_OPEN_LOOP")
      rhs = tobas::esc_mode_t::BLHELI_OPEN_LOOP;
    else if (value == "BLHELI_CLOSED_LOOP_LOW_RANGE")
      rhs = tobas::esc_mode_t::BLHELI_CLOSED_LOOP_LOW_RANGE;
    else if (value == "BLHELI_CLOSED_LOOP_MID_RANGE")
      rhs = tobas::esc_mode_t::BLHELI_CLOSED_LOOP_MID_RANGE;
    else if (value == "BLHELI_CLOSED_LOOP_HIGH_RANGE")
      rhs = tobas::esc_mode_t::BLHELI_CLOSED_LOOP_HIGH_RANGE;
    else
      return false;

    return true;
  }
};
}  // namespace YAML
