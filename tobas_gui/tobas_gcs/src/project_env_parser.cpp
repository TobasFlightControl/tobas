// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/project_env_parser.hpp"

#include <sstream>

#include <tobas_string_tools/core.hpp>

namespace tobas
{
namespace gui
{
namespace gcs
{
ProjectEnvParser::ProjectEnvParser()
{
}

bool ProjectEnvParser::parseFromText(const std::string& text)
{
  const auto lines = str::splitLines(text);

  for (auto line : lines) {
    // Trim whitespaces.
    line = str::trim(line);

    // Skip blank lines and comments.
    if (line.empty() || line.starts_with('#')) {
      continue;
    }

    // Get elements.
    if (line.starts_with(kConfigPkgPrefix)) {
      config_pkg = line.substr(sizeof(kConfigPkgPrefix) - 1);
      continue;
    }
    if (line.starts_with(kNetworkIfacePrefix)) {
      nif = line.substr(sizeof(kNetworkIfacePrefix) - 1);
      continue;
    }
  }

  return true;
}

std::string ProjectEnvParser::exportText() const
{
  std::ostringstream oss;

  oss << kConfigPkgPrefix << config_pkg << std::endl;
  oss << kNetworkIfacePrefix << nif << std::endl;

  return oss.str();
}
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
