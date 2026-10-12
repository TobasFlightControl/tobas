// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>
#include <QString>

namespace YAML
{
template <>
struct convert<QString>
{
  static Node encode(const QString& _rhs)
  {
    return Node(_rhs.toStdString());
  }

  static bool decode(const Node& _node, QString& _rhs)
  {
    if (!_node.IsScalar()) {
      return false;
    }

    _rhs = QString::fromStdString(_node.as<std::string>());
    return true;
  }
};
}  // namespace YAML
