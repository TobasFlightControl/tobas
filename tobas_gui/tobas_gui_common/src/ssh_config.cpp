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
bool SshConfig::load(const QString& path)
{
  const auto node = yaml::load(path.toStdString());
  if (!node) {
    std::cerr << node.error() << std::endl;
    return false;
  }

  if (!yaml::load(kHostKey, *node, host)) {
    return false;
  }
  if (!yaml::load(kUserKey, *node, user)) {
    return false;
  }

  return true;
}

bool SshConfig::save(const QString& path) const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kHostKey] = host;
  node[kUserKey] = user;

  if (!yaml::save(path.toStdString(), node)) {
    return false;
  }

  return true;
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
