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

namespace esc
{
static constexpr double kBLHeliCLLowMaxERPM = 50000;    // The maximum ERPM in BLHeli closed loop low range mode
static constexpr double kBLHeliCLMidMaxERPM = 100000;   // The maximum ERPM in BLHeli closed loop middle range mode
static constexpr double kBLHeliCLHighMaxERPM = 200000;  // The maximum ERPM in BLHeli closed loop high range mode
}  // namespace esc
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::esc_mode_t>
{
  static Node encode(const tobas::esc_mode_t& rhs)
  {
    return Node(static_cast<int>(rhs));
  }

  static bool decode(const Node& node, tobas::esc_mode_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    rhs = static_cast<tobas::esc_mode_t>(node.as<int>());
    return true;
  }
};
}  // namespace YAML
