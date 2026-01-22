#pragma once

#include <yaml-cpp/yaml.h>

#include "../format.hpp"

namespace YAML
{
template <>
struct convert<std::pair<double, double>>
{
  static Node encode(const std::pair<double, double>& rhs)
  {
    Node node(NodeType::Sequence);

    node.push_back(yaml::format(rhs.first));
    node.push_back(yaml::format(rhs.second));

    return node;
  }

  static bool decode(const Node& node, std::pair<double, double>& rhs)
  {
    if (!node.IsSequence()) {
      return false;
    }
    if (node.size() != 2) {
      return false;
    }

    rhs.first = node[0].as<double>();
    rhs.second = node[1].as<double>();

    return true;
  }
};
}  // namespace YAML
