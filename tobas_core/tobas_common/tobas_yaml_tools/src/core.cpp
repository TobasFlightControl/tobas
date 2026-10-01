// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_yaml_tools/core.hpp"

#include <fstream>

namespace fs = std::filesystem;

namespace tobas
{
namespace yaml
{
std::string dump(const YAML::Node& node) noexcept
{
  std::ostringstream res;
  YAML::Emitter emitter(res);
  emitter << node;
  res << std::endl;
  return res.str();
}

std::expected<YAML::Node, std::string> load(const fs::path& path) noexcept
{
  if (!fs::exists(path)) {
    return std::unexpected(path.string() + " does not exist.");
  }

  try {
    return YAML::LoadFile(path);
  }
  catch (const std::exception& e) {
    return std::unexpected("Failed to load " + path.string() + ": " + e.what());
  }
}

std::expected<void, std::string> save(const fs::path& path, const YAML::Node& node) noexcept
{
  std::ofstream fout(path);
  if (!fout.is_open()) {
    return std::unexpected("Failed to open '" + path.string() + "' for writing.");
  }

  fout << dump(node);
  fout.close();

  if (fout.fail()) {
    return std::unexpected("Failed to write YAML to '" + path.string() + "'.");
  }

  return {};
}
}  // namespace yaml
}  // namespace tobas
