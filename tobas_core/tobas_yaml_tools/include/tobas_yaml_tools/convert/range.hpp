#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_std_tools/range.hpp>

namespace YAML
{
template <typename T>
struct convert<tobas_std::Range<T>>
{
  static Node encode(const tobas_std::Range<T>& rhs)
  {
    Node node(NodeType::Sequence);

    node.push_back(rhs.lower);
    node.push_back(rhs.upper);

    return node;
  }

  static bool decode(const Node& node, tobas_std::Range<T>& rhs)
  {
    if (!node.IsSequence()) {
      return false;
    }
    if (node.size() != 2) {
      return false;
    }

    rhs.lower = node[0].as<T>();
    rhs.upper = node[1].as<T>();

    return true;
  }
};
}  // namespace YAML
