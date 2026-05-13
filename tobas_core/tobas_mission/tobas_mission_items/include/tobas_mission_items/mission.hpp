#pragma once

#include <cinttypes>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace tobas
{
namespace mission
{
enum Type : uint8_t
{
  kWaypoint,
  kTakeoff,
  kLand,
  kReturnToLaunch,
};

struct MissionItem
{
  uint8_t type;
  std::vector<uint8_t> data;
};

struct Mission
{
public:
  std::vector<MissionItem> items;
};
}  // namespace mission
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::mission::Mission>
{
  static Node encode(const tobas::mission::Mission& rhs);
  static bool decode(const Node& node, tobas::mission::Mission& rhs);
};
}  // namespace YAML
