// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/ssh_config.hpp"

#include <tobas_yaml_tools/core.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace cmn
{
bool SshConfig::load(const fs::path& path)
{
  const auto node = yaml::load(path);
  if (!node) {
    std::cerr << node.error() << std::endl;
    return false;
  }

  if (!yaml::load(kHostKey, node.value(), host)) {
    return false;
  }
  if (!yaml::load(kUserKey, node.value(), user)) {
    return false;
  }

  return true;
}

bool SshConfig::save(const fs::path& path) const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kHostKey] = host;
  node[kUserKey] = user;

  if (!yaml::save(path, node)) {
    return false;
  }

  return true;
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
