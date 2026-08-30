// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/network_config.hpp"

#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_yaml_tools/core.hpp>

namespace tobas
{
namespace gui
{
namespace cmn
{
bool NetworkConfig::load(const QString& path)
{
  const auto node = yaml::load(path.toStdString());
  if (!node) {
    std::cerr << node.error() << std::endl;
    return false;
  }

  if (!yaml::load(kInterfaceKey, node.value(), interface)) {
    return false;
  }

  return true;
}

bool NetworkConfig::save(const QString& path) const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kInterfaceKey] = interface;

  if (!yaml::save(path.toStdString(), node)) {
    return false;
  }

  return true;
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
