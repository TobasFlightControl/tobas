#pragma once

#include <yaml-cpp/yaml.h>
#include <QString>

namespace YAML
{
template <>
struct convert<QString>
{
  static Node encode(const QString& rhs)
  {
    return Node(rhs.toStdString());
  }

  static bool decode(const Node& node, QString& rhs)
  {
    if (!node.IsScalar()) {
      return false;
    }

    rhs = QString::fromStdString(node.as<std::string>());
    return true;
  }
};
}  // namespace YAML
