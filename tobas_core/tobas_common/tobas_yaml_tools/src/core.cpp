// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_yaml_tools/core.hpp"

#include <fstream>

namespace fs = std::filesystem;

namespace tobas
{
namespace yaml
{
std::string dump(const YAML::Node& _node) noexcept
{
  std::ostringstream res;
  YAML::Emitter emitter(res);
  emitter << _node;
  res << '\n';
  return res.str();
}

std::expected<YAML::Node, std::string> load(const fs::path& _path) noexcept
{
  if (!fs::exists(_path)) {
    return std::unexpected(_path.string() + " does not exist.");
  }

  try {
    return YAML::LoadFile(_path);
  }
  catch (const std::exception& e) {
    return std::unexpected("Failed to load " + _path.string() + ": " + e.what());
  }
}

std::expected<void, std::string> save(const fs::path& _path, const YAML::Node& _node) noexcept
{
  std::ofstream fout(_path);
  if (!fout.is_open()) {
    return std::unexpected("Failed to open '" + _path.string() + "' for writing.");
  }

  fout << dump(_node);
  fout.close();

  if (fout.fail()) {
    return std::unexpected("Failed to write YAML to '" + _path.string() + "'.");
  }

  return {};
}
}  // namespace yaml
}  // namespace tobas
