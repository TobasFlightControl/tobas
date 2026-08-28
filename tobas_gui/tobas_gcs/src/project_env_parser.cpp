// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/project_env_parser.hpp"

namespace tobas
{
namespace gui
{
namespace gcs
{
ProjectEnvParser::ProjectEnvParser()
{
}

bool ProjectEnvParser::parseFromText(const QString& text)
{
  for (auto line : text.split('\n')) {
    // Trim whitespaces.
    line = line.trimmed();

    // Skip blank lines and comments.
    if (line.isEmpty() || line.startsWith('#')) {
      continue;
    }

    // Get elements.
    if (line.starts_with(kConfigPkgPrefix)) {
      config_pkg = line.sliced(sizeof(kConfigPkgPrefix) - 1);
      continue;
    }
    if (line.starts_with(kNetworkIfacePrefix)) {
      nic = line.sliced(sizeof(kNetworkIfacePrefix) - 1);
      continue;
    }
    if (line.starts_with(kIdPrefix)) {
      id = line.sliced(sizeof(kIdPrefix) - 1);
      continue;
    }
  }

  return true;
}

QString ProjectEnvParser::exportText() const
{
  return QString(kConfigPkgPrefix) + config_pkg + '\n' + kNetworkIfacePrefix + nic + '\n' + kIdPrefix + id + '\n';
}
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
