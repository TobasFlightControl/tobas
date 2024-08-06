#pragma once

#include <array>
#include <yaml-cpp/yaml.h>

namespace YAML
{
template <typename T, size_t N>
struct convert<std::array<T, N>>
{
  static Node encode(const std::array<T, N>& rhs)
  {
    Node node(NodeType::Sequence);

    for (auto& value : rhs)
      node.push_back(value);

    return node;
  }

  static bool decode(const Node& node, std::array<T, N>& rhs)
  {
    if (!node.IsSequence())
      return false;
    if (node.size() != N)
      return false;

    for (size_t i = 0; i < N; ++i)
      rhs[i] = node[i].as<T>();

    return true;
  }
};
}  // namespace YAML
