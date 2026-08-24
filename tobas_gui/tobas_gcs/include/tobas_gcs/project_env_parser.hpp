// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace gui
{
namespace gcs
{
class ProjectEnvParser
{
  static constexpr char kConfigPkgPrefix[] = "TOBAS_CONFIG_PKG=";
  static constexpr char kNetworkIfacePrefix[] = "TOBAS_NIC=";
  static constexpr char kIdPrefix[] = "TOBAS_ID=";

public:
  std::string config_pkg;
  std::string nic;
  std::string id;

  explicit ProjectEnvParser();

  bool parseFromText(const std::string& text);

  std::string exportText() const;
};
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
