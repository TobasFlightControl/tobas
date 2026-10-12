// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <filesystem>
#include <string>

#include <yaml-cpp/yaml.h>

namespace tobas
{
namespace yaml
{
template <typename T>
std::expected<T, std::string> load(const std::string& _key, const YAML::Node& _parent) noexcept
{
  if (!_parent.IsDefined()) {
    return std::unexpected("The parent node is not defined.");
  }
  if (!_parent.IsMap()) {
    return std::unexpected("The type of the parent node is not map.");
  }

  try {
    return _parent[_key].as<T>();
  }
  catch (...) {
    return std::unexpected("Type mismatch.");
  }
}

template <typename T>
std::expected<void, std::string> load(const std::string& _key, const YAML::Node& _parent, T& _value) noexcept
{
  const auto res = load<T>(_key, _parent);
  if (!res) {
    return std::unexpected(res.error());
  }

  _value = *res;
  return {};
}

/** Convert `YAML::Node` to text. */
std::string dump(const YAML::Node& _node) noexcept;

std::expected<YAML::Node, std::string> load(const std::filesystem::path& _path) noexcept;
std::expected<void, std::string> save(const std::filesystem::path& _path, const YAML::Node& _node) noexcept;
}  // namespace yaml
}  // namespace tobas
