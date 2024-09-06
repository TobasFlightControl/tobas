#pragma once

#include <cinttypes>
#include <yaml-cpp/yaml.h>

namespace tobas
{
/**
 * @brief ロータの回転軸タイプ．
 * kdl::Treeからランタイムに判定することもできるが，SetupAssistantとControllerで判定基準が異なると困るため，
 * Droneの設定の時点でタイプを確定させておく．
 */
enum rotor_axis_t : uint8_t
{
  X_POSITIVE,
  Z_POSITIVE,
  UNKNOWN,
};
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::rotor_axis_t>
{
  static Node encode(const tobas::rotor_axis_t& rhs)
  {
    return Node(static_cast<int>(rhs));
  }

  static bool decode(const Node& node, tobas::rotor_axis_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    rhs = static_cast<tobas::rotor_axis_t>(node.as<int>());
    return true;
  }
};
}  // namespace YAML
