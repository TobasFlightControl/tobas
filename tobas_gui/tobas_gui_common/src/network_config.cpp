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
namespace
{
constexpr char kInterfaceKey[] = "interface";
}  // namespace

std::expected<NetworkConfig, QString> loadNetworkConfig(const QString& path)
{
  const auto node = yaml::load(path.toStdString());
  if (!node) {
    return std::unexpected(QString::fromStdString(node.error()));
  }

  NetworkConfig config;
  if (const auto result = yaml::load(kInterfaceKey, *node, config.interface); !result) {
    return std::unexpected(QString::fromStdString(result.error()));
  }

  return config;
}

std::expected<void, QString> saveNetworkConfig(const QString& path, const NetworkConfig& config)
{
  YAML::Node node(YAML::NodeType::Map);

  node[kInterfaceKey] = config.interface;

  if (const auto result = yaml::save(path.toStdString(), node); !result) {
    return std::unexpected(QString::fromStdString(result.error()));
  }

  return {};
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
