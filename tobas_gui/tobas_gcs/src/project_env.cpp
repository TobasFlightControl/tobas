// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/project_env.hpp"

#include <QStringList>

namespace tobas::gui::gcs
{
namespace
{
constexpr char kConfigPkgPrefix[] = "TOBAS_CONFIG_PKG=";
constexpr char kNetworkIfacePrefix[] = "TOBAS_NIC=";
constexpr char kDronePrefix[] = "TOBAS_DRONE=";
constexpr char kIdPrefix[] = "TOBAS_ID=";
}  // namespace

ProjectEnv parseProjectEnv(const QString& _text)
{
  ProjectEnv env;
  for (auto line : _text.split('\n')) {
    // Trim whitespaces.
    line = line.trimmed();

    // Skip blank lines and comments.
    if (line.isEmpty() || line.startsWith('#')) {
      continue;
    }

    // Get elements.
    if (line.startsWith(kConfigPkgPrefix)) {
      env.config_pkg = line.mid(sizeof(kConfigPkgPrefix) - 1);
      continue;
    }
    if (line.startsWith(kNetworkIfacePrefix)) {
      env.nic = line.mid(sizeof(kNetworkIfacePrefix) - 1);
      continue;
    }
    if (line.startsWith(kDronePrefix)) {
      env.drone = line.mid(sizeof(kDronePrefix) - 1);
      continue;
    }
    if (line.startsWith(kIdPrefix)) {
      env.id = line.mid(sizeof(kIdPrefix) - 1);
      continue;
    }
  }

  return env;
}

QString exportProjectEnv(const ProjectEnv& _env)
{
  QString res;
  res += kConfigPkgPrefix + _env.config_pkg + '\n';
  res += kNetworkIfacePrefix + _env.nic + '\n';
  res += kDronePrefix + _env.drone + '\n';
  res += kIdPrefix + _env.id + '\n';
  return res;
}
}  // namespace tobas::gui::gcs
