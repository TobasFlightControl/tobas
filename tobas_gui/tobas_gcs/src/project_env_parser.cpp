// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/project_env_parser.hpp"

#include <QStringList>

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
    if (line.startsWith(kConfigPkgPrefix)) {
      config_pkg = line.mid(sizeof(kConfigPkgPrefix) - 1);
      continue;
    }
    if (line.startsWith(kNetworkIfacePrefix)) {
      nic = line.mid(sizeof(kNetworkIfacePrefix) - 1);
      continue;
    }
    if (line.startsWith(kIdPrefix)) {
      id = line.mid(sizeof(kIdPrefix) - 1);
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
