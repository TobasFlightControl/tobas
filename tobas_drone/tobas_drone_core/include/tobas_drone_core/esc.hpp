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
static constexpr char kBLHeliOpenLoopName[] = "BLHELI_OPEN_LOOP";
static constexpr char kBLHeliCloseLoopLowName[] = "BLHELI_CLOSED_LOOP_LOW_RANGE";
static constexpr char kBLHeliCloseLoopMidName[] = "BLHELI_CLOSED_LOOP_MID_RANGE";
static constexpr char kBLHeliCloseLoopHighName[] = "BLHELI_CLOSED_LOOP_HIGH_RANGE";

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
    Node node;

    switch (rhs)
    {
      case tobas::esc_mode_t::BLHELI_OPEN_LOOP:
        node = tobas::esc::kBLHeliOpenLoopName;
        break;
      case tobas::esc_mode_t::BLHELI_CLOSED_LOOP_LOW_RANGE:
        node = tobas::esc::kBLHeliCloseLoopLowName;
        break;
      case tobas::esc_mode_t::BLHELI_CLOSED_LOOP_MID_RANGE:
        node = tobas::esc::kBLHeliCloseLoopMidName;
        break;
      case tobas::esc_mode_t::BLHELI_CLOSED_LOOP_HIGH_RANGE:
        node = tobas::esc::kBLHeliCloseLoopHighName;
        break;
    }

    return node;
  }

  static bool decode(const Node& node, tobas::esc_mode_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    const auto value = node.as<std::string>();
    if (value == tobas::esc::kBLHeliOpenLoopName)
      rhs = tobas::esc_mode_t::BLHELI_OPEN_LOOP;
    else if (value == tobas::esc::kBLHeliCloseLoopLowName)
      rhs = tobas::esc_mode_t::BLHELI_CLOSED_LOOP_LOW_RANGE;
    else if (value == tobas::esc::kBLHeliCloseLoopMidName)
      rhs = tobas::esc_mode_t::BLHELI_CLOSED_LOOP_MID_RANGE;
    else if (value == tobas::esc::kBLHeliCloseLoopHighName)
      rhs = tobas::esc_mode_t::BLHELI_CLOSED_LOOP_HIGH_RANGE;
    else
      return false;

    return true;
  }
};
}  // namespace YAML
