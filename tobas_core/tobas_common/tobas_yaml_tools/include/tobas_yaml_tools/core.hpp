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
std::expected<T, std::string> load(const std::string& key, const YAML::Node& parent) noexcept
{
  if (!parent.IsMap()) {
    return std::unexpected("The type of the parent node of key \"" + key + "\" is not map.");
  }

  try {
    return parent[key].as<T>();
  }
  catch (...) {
    return std::unexpected("Key \"" + key + "\" type mismatch.");
  }
}

template <typename T>
bool load(const std::string& key, const YAML::Node& parent, T& value) noexcept
{
  const auto res = load<T>(key, parent);
  if (!res) {
    std::cerr << res.error();
    return false;
  }

  value = res.value();
  return true;
}

/* YAML::Nodeをテキストに変換する． */
std::string dump(const YAML::Node& node) noexcept;

std::expected<YAML::Node, std::string> load(const std::filesystem::path& path) noexcept;
bool save(const std::filesystem::path& path, const YAML::Node& node) noexcept;
}  // namespace yaml
}  // namespace tobas
