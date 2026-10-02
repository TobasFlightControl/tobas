// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/ssh_config.hpp"

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
constexpr char kHostKey[] = "host";
constexpr char kUserKey[] = "user";
}  // namespace

std::expected<SshConfig, QString> loadSshConfig(const QString& path)
{
  const auto node = yaml::load(path.toStdString());
  if (!node) {
    return std::unexpected(QString::fromStdString(node.error()));
  }

  SshConfig config;
  if (const auto result = yaml::load(kHostKey, *node, config.host); !result) {
    return std::unexpected(QString::fromStdString(result.error()));
  }
  if (const auto result = yaml::load(kUserKey, *node, config.user); !result) {
    return std::unexpected(QString::fromStdString(result.error()));
  }

  return config;
}

std::expected<void, QString> saveSshConfig(const QString& path, const SshConfig& config)
{
  YAML::Node node(YAML::NodeType::Map);

  node[kHostKey] = config.host;
  node[kUserKey] = config.user;

  if (const auto result = yaml::save(path.toStdString(), node); !result) {
    return std::unexpected(QString::fromStdString(result.error()));
  }

  return {};
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
