// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <filesystem>
#include <iostream>

#include <yaml-cpp/yaml.h>

namespace tobas
{
namespace yaml
{
template <typename T>
bool load(const std::string& key, const YAML::Node& parent, T& value)
{
  if (!parent.IsMap()) {
    std::cerr << "The type of the parent node of key \"" << key << "\" is not map." << std::endl;
    return false;
  }

  try {
    value = parent[key].as<T>();
  }
  catch (...) {
    std::cerr << "Key \"" << key << "\" type mismatch." << std::endl;
    return false;
  }

  return true;
}

/* YAML::Nodeをテキストに変換する． */
std::string dump(const YAML::Node& node);

std::expected<YAML::Node, std::string> load(const std::filesystem::path& path);
bool save(const std::filesystem::path& path, const YAML::Node& node);
}  // namespace yaml
}  // namespace tobas
